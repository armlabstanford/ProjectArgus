//
// This is the main entry point for the Argus server, which is a ROS 2 node
// that performs inverse kinematics (IK) on a given OpenSim model.
//
// Use Instructions:
// 1. Launch the Argus server node with the required parameters.
//    - `model_folder_path`: The absolute path to the folder containing the
//      OpenSim model file (`.osim`) and its associated geometry files.
//    - `topic_names`: A list of ROS 2 topics publishing `nav_msgs/msg/Odometry`
//      messages for each tracked body segment. The last part of the topic name
//      should match the body name in the OpenSim model.
//    - `joint_outputs`: A list of joint names from the OpenSim model for which
//      the joint angles should be published.
//    - `body_calibrations`: A list of body calibration strings, each in the
//      format "body_name:euler_angles", where euler_angles is a comma-separated
//      list of three numbers representing the Euler angles in radians.
//
// 2. The server subscribes to the specified odometry topics and uses the
//    orientation data to perform IK tracking.
//
// 3. The server publishes the resulting joint angles to the `/joint_states`
//    topic as `sensor_msgs/msg/JointState` messages.
//
// 4. To start tracking, you must first "zero" the system by publishing a
//    `std_msgs/msg/Bool` message with `data: true` to the `/reset_tracking`
//    topic. This calibrates the system by computing the offset between the
//    initial orientation of the tracked segments and the corresponding bodies
//    in the OpenSim model.

#include "rclcpp/rclcpp.hpp"
#include <OpenSim/OpenSim.h>
#include <memory>      
#include "nav_msgs/msg/odometry.hpp"
#include "std_msgs/msg/bool.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "geometry_msgs/msg/quaternion.hpp"
#include <string>
#include <unordered_map>
#include <string>
#include <mutex>
#include <chrono>


//#include <OpenSim/Common/ModelVisualizer.h>

using namespace OpenSim;


class ArgusServer : public rclcpp::Node
{
public:
  /**
   * @brief Construct a new Argus Server object
   * 
   * This constructor initializes the ROS 2 node, declares and retrieves
   * parameters, and calls the system initialization function.
   */
  ArgusServer() : Node("argus_server")
  {
    //OpenSim::Logger::setLevel(OpenSim::Logger::Level::Debug);
    RCLCPP_INFO(this->get_logger(), "ArgusServer node started");

    // Declare parameters for the node
    this->declare_parameter<std::string>("model_folder_path", "");
    this->declare_parameter<std::vector<std::string>>(
      "topic_names",
      std::vector<std::string>{}
    );
    this->declare_parameter<std::vector<std::string>>(
      "joint_outputs",
      std::vector<std::string>{}
    );
    this->declare_parameter<std::vector<std::string>>(
      "body_calibrations",
      std::vector<std::string>{}
    );

    // Get the parameter values
    std::string model_folder_path;
    this->get_parameter("model_folder_path", model_folder_path);

    std::vector<std::string> topic_names;
    this->get_parameter("topic_names", topic_names);

    std::vector<std::string> joint_outputs;
    this->get_parameter("joint_outputs", joint_outputs);

    std::vector<std::string> body_calibrations;
    this->get_parameter("body_calibrations", body_calibrations);
    
    // run system initialization (creation of all objects)
    // later we will run the zero-ing process, when user requests it.
    init_system(model_folder_path, topic_names, joint_outputs, body_calibrations);
  }

private:
  /**
   * @brief Initializes the ArgusServer system.
   * 
   * This function sets up the OpenSim model, creates ROS 2 subscribers and
   * publishers, and initializes the Inverse Kinematics (IK) solver.
   * 
   * @param model_folder_path Path to the folder containing the OpenSim model.
   * @param topic_names List of odometry topics to subscribe to.
   * @param joint_outputs List of joint names to publish.
   */
  void init_system(const std::string model_folder_path, const std::vector<std::string> topic_names, const std::vector<std::string> joint_outputs, const std::vector<std::string>& body_calibrations)
  {
    RCLCPP_INFO(this->get_logger(), "Initializing ArgusServer system...");

    // Parse body calibrations
    for (const auto& calib_str : body_calibrations) {
        size_t delimiter_pos = calib_str.find(':');
        if (delimiter_pos == std::string::npos) {
            RCLCPP_WARN(this->get_logger(), "Invalid body calibration string format: %s", calib_str.c_str());
            continue;
        }

        std::string body_name = calib_str.substr(0, delimiter_pos);
        std::string values_str = calib_str.substr(delimiter_pos + 1);

        std::stringstream ss(values_str);
        std::string value;
        std::vector<double> euler_angles;
        while (std::getline(ss, value, ',')) {
            try {
                euler_angles.push_back(std::stod(value));
            } catch (const std::invalid_argument& ia) {
                RCLCPP_WARN(this->get_logger(), "Invalid number in body calibration for %s: %s", body_name.c_str(), value.c_str());
                euler_angles.clear();
                break;
            }
        }

        if (euler_angles.size() == 3) {
            SimTK::Rotation rot(SimTK::BodyRotationSequence, euler_angles[0], SimTK::XAxis, euler_angles[1], SimTK::YAxis, euler_angles[2], SimTK::ZAxis);
            body_calibration_rotations[body_name] = rot;
            RCLCPP_INFO(this->get_logger(), "Loaded calibration for %s", body_name.c_str());
        } else {
            RCLCPP_WARN(this->get_logger(), "Incorrect number of euler angles for %s", body_name.c_str());
        }
    }


    auto now = this->get_clock()->now();
    double timestamp = now.seconds();

    // Create a subscription to the reset_tracking topic to allow for zeroing the tracking.
    reset_sub = this->create_subscription<std_msgs::msg::Bool>(
      "reset_tracking",
      10,
      [this](const std_msgs::msg::Bool::SharedPtr msg) {
        RCLCPP_INFO(this->get_logger(), "Received reset publish");
        reset_tracking(msg->data);
      }
    );

    tracked_segment_count = topic_names.size();
    joint_output_ref = joint_outputs;



    for (size_t i = 0; i < joint_outputs.size(); i++) {
      RCLCPP_INFO(this->get_logger(), "allowed joint: %s", joint_outputs[i].c_str());
    }
  
    RCLCPP_INFO(this->get_logger(), "Found segments: %d", tracked_segment_count);

    // establish references to the data supply topics (odom messages)
    for (const auto& topic_name : topic_names) {
      RCLCPP_INFO(this->get_logger(), "Subscribing to topic: %s", topic_name.c_str());
      
      // Create a subscription for each odometry topic.
      auto subscription = this->create_subscription<nav_msgs::msg::Odometry>(
        topic_name,
        10,
        [this, topic_name](const nav_msgs::msg::Odometry::SharedPtr msg) {
          this->odom_cb(msg, topic_name);
        }
      );
      std::string segment_name = extract_segment_name(topic_name);
      segment_ref.push_back(segment_name);
      odom_subs[segment_name] = subscription;
    }

    // Create an initial table of orientations for the IK solver.
    // This is required to initialize the solver, but the values are arbitrary
    // as they will be updated with real data.
    OpenSim::TimeSeriesTable_<SimTK::Rotation> initial_table;

    initial_table.setColumnLabels(segment_ref);

    SimTK::RowVector_<SimTK::Rotation> dummy;
    dummy.resize(tracked_segment_count);

    // given quaternion
    SimTK::Quaternion q(1, 0, 0, 0);
    q.normalize();

    // convert to rotation
    SimTK::Rotation R(q);

    // fill row
    for (int i = 0; i < tracked_segment_count; ++i) {
      dummy[i] = R;
    }

    initial_table.appendRow(timestamp, dummy);

    // Create a timer to periodically run the IK tracking.
    // This is the goal frequency, which will not be met if computationally constrained.
    // New data overwrites old data, IK is always run on the most recent pose data.
    solve_timer = this->create_wall_timer(
      std::chrono::milliseconds(50),
      std::bind(&ArgusServer::track_motion, this)
    );

    // Create a publisher for the joint angles.
    joint_angle_pub = this->create_publisher<sensor_msgs::msg::JointState>(
      "/joint_states", 10);

    // Create or load the model. If you pass a path in `model_path`, we'll
    // attempt to load that model; otherwise we construct an empty Model.
    OpenSim::ModelVisualizer::addDirToGeometrySearchPaths(
        model_folder_path + "/Geometry"
    );
    model_ptr = std::make_unique<Model>(model_folder_path + "/model.osim");



    state = model_ptr->initSystem();

    const auto& bodySet = model_ptr->getBodySet();

    RCLCPP_INFO(this->get_logger(), "=== VALID BODY NAMES ===");
    for (int i = 0; i < bodySet.getSize(); ++i) {
      const auto& body = bodySet.get(i);
      RCLCPP_INFO(this->get_logger(), "Body: %s", body.getName().c_str());
    }

    // this is marker constraints, we have no markers, so leave it empty
    markers_ref_ = std::make_shared<OpenSim::MarkersReference>();

    // we also don't want to add any coordinate constraints on specific joint angles,
    // they should already be in the model
    coordinate_refs_.clear();

    // this is orientation constraints, which is the only thing we actually will constrain
    orientations_ref_ = std::make_shared<OpenSim::BufferedOrientationsReference>(initial_table);

    // Construct the IK solver and keep it on the class so other methods can use it.
    ik_solver_ = std::make_unique<OpenSim::InverseKinematicsSolver>(
      *model_ptr,
      markers_ref_,
      orientations_ref_,
      coordinate_refs_
    );

    // assemble() works for any initial state, but it takes longer than track().
    // so, assemble for the first pose, track for subsequent poses.
    state.setTime(timestamp);

    int num_refs = orientations_ref_->getNumRefs();
    RCLCPP_INFO(this->get_logger(), "Number of references: %d", num_refs);

    RCLCPP_INFO(this->get_logger(), "Assembling IK solver...");
    ik_solver_->assemble(state);
    RCLCPP_INFO(this->get_logger(), "IK solver assembled.");
  }

  /**
   * @brief Callback function for odometry messages.
   * 
   * This function is called whenever a new odometry message is received. It
   * stores the orientation from the message in a map, keyed by the segment name.
   * 
   * @param msg The received odometry message.
   * @param segment_name The name of the body segment associated with the message.
   */
  void odom_cb(const nav_msgs::msg::Odometry::SharedPtr msg, const std::string& topic_name)
  {
    // recieves odom messages from a specific topic, and stores them in the class.
    auto q = msg->pose.pose.orientation;

    std::string segment_name = extract_segment_name(topic_name);
    std::lock_guard<std::mutex> lock(odom_mutex);
    cur_odom_data[segment_name] = q;

  }

  /**
   * @brief Performs IK tracking.
   * 
   * This function is called periodically by a timer. It adds the latest
   * orientation data to the IK solver, runs the tracking algorithm, and
   * publishes the resulting joint angles.
   */
  void track_motion()
  {


    if(!is_zeroed) {
      return;
    }

    add_orientation_data();

    state.setTime(current_timestamp);
      // ---- start timing ----
    auto t0 = std::chrono::high_resolution_clock::now();

    ik_solver_->track(state);

    auto t1 = std::chrono::high_resolution_clock::now();
    // ---- end timing ----
    model_ptr->realizeReport(state);

    publish_joint_angles();

    // compute duration (microseconds → milliseconds)
    auto duration_us = std::chrono::duration_cast<std::chrono::microseconds>(t1 - t0).count();
    double duration_ms = duration_us / 1000.0;

    RCLCPP_INFO(this->get_logger(), "IK track() took %.3f ms", duration_ms);

  }

  /**
   * @brief Resets the tracking references.
   * 
   * This function is called when a message is received on the `reset_tracking`
   * topic. It computes the offset between the current orientation of the
   * tracked segments and the corresponding bodies in the OpenSim model. This
   * offset is then used to correct the orientation data before it is passed
   * to the IK solver.
   * 
   * @param reset_command A boolean indicating whether to reset the tracking.
   */
  void reset_tracking(bool reset_command)
  {
    std::lock_guard<std::mutex> lock(odom_mutex);


    if(reset_command == true) {

      RCLCPP_INFO(this->get_logger(), "Resetting tracking references...");

      std::unordered_map<std::string, SimTK::Quaternion> cur_body_quaternions = get_body_orientations_as_quaternions();

      for (std::string segment_name : segment_ref) {
        // convert to rotation matrices for math

        // Body rotation
        SimTK::Rotation cur_body_R(cur_body_quaternions[segment_name]);

        // Odom rotation (convert from ROS msg)
        const auto& q_msg = cur_odom_data[segment_name];
        SimTK::Quaternion q_odom(q_msg.w, q_msg.x, q_msg.y, q_msg.z);
        SimTK::Rotation cur_odom_R(q_odom);

        // Compute reference offset
        SimTK::Rotation ref_odom_R = cur_body_R * body_calibration_rotations[segment_name] * ~cur_odom_R;
        SimTK::Quaternion ref_quat(ref_odom_R);

        // Convert back to ROS msg
        geometry_msgs::msg::Quaternion ref_msg;
        ref_msg.w = ref_quat[0];
        ref_msg.x = ref_quat[1];
        ref_msg.y = ref_quat[2];
        ref_msg.z = ref_quat[3];

        ref_odom_data[segment_name] = ref_msg;// add in the constant offset matrix here, later!
      }
    }
    is_zeroed = true;
  }

  /**
   * @brief Adds orientation data to the IK solver.
   * 
   * This function retrieves the latest orientation data for each tracked
   * segment, applies the reference offset, and adds the resulting orientation
   * to the IK solver's orientation reference.
   */
  void add_orientation_data()
  {

    // need to add orientation data for most recent tracked body segments.
    auto now = this->get_clock()->now();
    double timestamp = now.seconds();

    SimTK::RowVector_<SimTK::Rotation> dataRow;

    dataRow.resize(tracked_segment_count);
    geometry_msgs::msg::Quaternion cur_quat;
    geometry_msgs::msg::Quaternion ref_quat;

    // RCLCPP_INFO(this->get_logger(), "Adding orientation data for IK at time %.3f", timestamp);
    // RCLCPP_INFO(this->get_logger(), "Current odometry data for humerus_l: w=%.3f, x=%.3f, y=%.3f, z=%.3f",
    //   cur_odom_data["humerus_l"].w, cur_odom_data["humerus_l"].x, cur_odom_data["humerus_l"].y, cur_odom_data["humerus_l"].z);

    std::lock_guard<std::mutex> lock(odom_mutex);
    if (!is_zeroed) {
        RCLCPP_WARN(this->get_logger(), "Tracking not zeroed, can't add orientation data for IK.");
        return;
    }

    for (int i = 0; i < tracked_segment_count; i++) {
      std::string segment_name = segment_ref[i];
      
      if(cur_odom_data.find(segment_name) == cur_odom_data.end() || ref_odom_data.find(segment_name) == ref_odom_data.end()) {
        RCLCPP_WARN(this->get_logger(), "Missing odometry data for segment '%s', skipping IK update.", segment_name.c_str());
        return;
      }

      cur_quat = cur_odom_data[segment_name];
      ref_quat = ref_odom_data[segment_name];

      SimTK::Quaternion q_cur(cur_quat.w, cur_quat.x, cur_quat.y, cur_quat.z);
      SimTK::Quaternion q_ref(ref_quat.w, ref_quat.x, ref_quat.y, ref_quat.z);

      SimTK::Rotation R_ref(q_ref);
      SimTK::Rotation R_cur(q_cur);

      SimTK::Rotation R_rel = R_ref * R_cur;

      dataRow[i] = R_rel;

    }
    
    orientations_ref_->putValues(timestamp, dataRow);

    const SimTK::Array_<std::string> names = orientations_ref_->getNames();

    current_timestamp = timestamp;
  }

  /**
   * @brief Publishes the joint angles.
   * 
   * This function retrieves the current joint angles from the OpenSim model
   * and publishes them as a `sensor_msgs/msg/JointState` message.
   */
  void publish_joint_angles()
  {
      // publish the current joint angles as a JointState message
      sensor_msgs::msg::JointState joint_state_msg;
      joint_state_msg.header.stamp = this->get_clock()->now();
  
      const OpenSim::CoordinateSet& coords = model_ptr->getCoordinateSet();



      for (int i = 0; i < coords.getSize(); ++i) {
        const OpenSim::Coordinate& c = coords.get(i);

        if (std::find(joint_output_ref.begin(), joint_output_ref.end(), c.getName()) != joint_output_ref.end()) {
          double value = c.getValue(state);
    
          joint_state_msg.name.push_back(c.getName());
          joint_state_msg.position.push_back(value);
        }
      }
  
      joint_angle_pub->publish(joint_state_msg);
  }

  /**
   * @brief Extracts the segment name from a topic name.
   *
   * Assumes topic format: /<segment>/<something>
   * Returns the first path element after the leading '/'.
   *
   * Example:
   *   /humerus/odomimu -> humerus
   */
  std::string extract_segment_name(const std::string& topic_name)
  {
      // Find first non-leading slash
      size_t start = topic_name.find_first_not_of('/');

      if (start == std::string::npos)
          return "";  // empty or invalid string

      // Find next slash after segment start
      size_t end = topic_name.find('/', start);

      if (end == std::string::npos)
      {
          // Only one token after slash
          return topic_name.substr(start);
      }

     // RCLCPP_INFO(this->get_logger(), "Extracted segment name '%s' from topic '%s'", topic_name.substr(start, end - start).c_str(), topic_name.c_str());

      return topic_name.substr(start, end - start);
  }

  /**
   * @brief Gets the body orientations as quaternions.
   * 
   * This function retrieves the orientation of each tracked body segment from
   * the OpenSim model and returns them as a map of SimTK::Quaternion objects.
   * 
   * @return A map of body names to quaternions.
   */
  std::unordered_map<std::string, SimTK::Quaternion> get_body_orientations_as_quaternions()
  {
    std::unordered_map<std::string, SimTK::Quaternion> result;

    // Make sure transforms are up to date
    model_ptr->realizePosition(state);

    const auto& bodySet = model_ptr->getBodySet();

    for (const auto& segment_name : segment_ref) {

      if (!bodySet.contains(segment_name)) {
        RCLCPP_WARN(this->get_logger(),
          "Body not found in model: %s", segment_name.c_str());
        continue;
      }

      const OpenSim::Body& body = bodySet.get(segment_name);

      // Transform of body in Ground
      const SimTK::Transform& X_GB = body.getTransformInGround(state);

      // Extract rotation
      const SimTK::Rotation& R_GB = X_GB.R();

      // Convert to quaternion
      SimTK::Quaternion q(R_GB);

      result[segment_name] = q;
    }

    return result;
  }

  // --- Member Variables ---

  // OpenSim objects
  std::shared_ptr<OpenSim::MarkersReference> markers_ref_; ///< Marker references for IK (unused).
  std::shared_ptr<OpenSim::BufferedOrientationsReference> orientations_ref_; ///< Orientation references for IK.
  SimTK::Array_<OpenSim::CoordinateReference> coordinate_refs_; ///< Coordinate references for IK (unused).
  std::unique_ptr<OpenSim::InverseKinematicsSolver> ik_solver_; ///< The IK solver.
  SimTK::State state; ///< The state of the OpenSim model.
  std::unique_ptr<Model> model_ptr; ///< The OpenSim model.

  // ROS 2 objects
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_angle_pub; ///< Publisher for joint angles.
  std::unordered_map<std::string, rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr> odom_subs; ///< Map of odometry subscribers.
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr reset_sub; ///< Subscriber for reset commands.
  
  // Data storage
  std::unordered_map<std::string, geometry_msgs::msg::Quaternion> cur_odom_data; ///< Current odometry data.
  std::unordered_map<std::string, geometry_msgs::msg::Quaternion> ref_odom_data; ///< Reference odometry data for zeroing.
  std::unordered_map<std::string, SimTK::Rotation> body_calibration_rotations; ///< Calibration rotations for each body.
  std::mutex odom_mutex; ///< Mutex to protect access to odometry data.

  // Timers and tracking variables
  rclcpp::TimerBase::SharedPtr solve_timer; ///< Timer to run the IK solver.
  std::vector<std::string> joint_output_ref; ///< List of joint names to publish.
  std::vector<std::string> segment_ref; ///< List of tracked segment names.
  bool is_zeroed = false; ///< Flag indicating if the tracking has been zeroed.
  int tracked_segment_count; ///< Number of tracked segments.
  double current_timestamp; ///< Timestamp of the current IK solution.
};


// --------------------
// Main entry point
// --------------------
int main(int argc, char *argv[])
{
  // Initialize the ROS 2 client library.
  rclcpp::init(argc, argv);

  // Create an instance of the ArgusServer node.
  auto node = std::make_shared<ArgusServer>(); // example topic names

  // Spin the node to process callbacks.
  rclcpp::spin(node);

  // Shutdown the ROS 2 client library.
  rclcpp::shutdown();
  return 0;
}
