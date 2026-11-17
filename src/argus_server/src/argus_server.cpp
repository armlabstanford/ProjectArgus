//
// This is the main entry point for the Argus server, which is a ROS 2 node
// that performs inverse kinematics (IK) on a given OpenSim model.
//
// Use Instructions:
// 1. Launch the Argus server node with the required parameters.
//    - `model_folder_path`: The absolute path to the folder containing the
//      OpenSim model file (`.osim`) and its associated geometry files.
//    - `topic_names`: A list of ROS 2 topics publishing `nav_msgs/msg/Odometry`
//      messages for each tracked body segment.
//    - `topic_translations`: Optional list of "alias:body" strings. The alias
//      is the first element of the topic path (or the full topic). The body
//      is the OpenSim body name used for IK. Example: "argus1:thorax".
//      Topics with no entry still use that first path element as the body name.
//    - `joint_outputs`: A list of joint names from the OpenSim model for which
//      the joint angles should be published.
//    - `body_calibrations`: A list of body calibration strings, each in the
//      format "body_name:euler_angles", where euler_angles is a comma-separated
//      list of three extrinsic xyz Euler angles in degrees. This matches
//      scipy Rotation.from_euler('xyz', angles, degrees=True): rotations about
//      fixed axes, x then y then z. It is the transform from bone to sensor.
//      These are the defaults. If calibration_dir contains calibration_*.json
//      files written by pose_capture.py, the newest file replaces the offsets
//      for the bodies it lists.
//    - `calibration_dir`: Folder of calibration_*.json files. Empty uses
//      <repo>/poses/calibrations next to the model folder.
//    - `elv_angle_prior_weight`, `elv_angle_prior_deg`: weak IK target that
//      pulls elv_angle_r/l (plane of elevation) toward elv_angle_prior_deg.
//      With the arm hanging the plane is unobserved and otherwise drifts to
//      its limit, from where a raise in another plane cannot be reached.
//      Weight 0 disables it. Defaults 0.002 and 0.
//
// 2. The server subscribes to the specified odometry topics and uses the
//    orientation data to perform IK tracking.
//
// 3. The server publishes the resulting joint angles to the `/joint_states`
//    topic as `sensor_msgs/msg/JointState` messages.
//
// 4. To start tracking, you must first "zero" the system by publishing a
//    `std_msgs/msg/Bool` message with `data: true` to the `/reset_tracking`
//    topic. This pairs the current sensor orientations with the zero pose,
//    which the subject must be holding at that moment.
//    - `zero_pose_file`: pose JSON in the pose_capture.py format (a "poses"
//      list of {"name", "joint_angles"}). Empty uses <repo>/poses/poses.json.
//    - `zero_pose`: name of the pose in that file. Default "neutral".
//    Coordinates the pose leaves out keep the model's default value; coupled
//    coordinates (shoulder girdle) are solved to match.
//    Publish a pose name as `std_msgs/msg/String` on `/load_zero_pose` to
//    re-read the file and switch poses without a restart (an empty string
//    loads "neutral"). The next `/reset_tracking` uses it.
//
// 5. Optional drift correction (argus_drift node). After each zero the server
//    publishes, latched on `drift_reference_topic` (tf2_msgs/TFMessage, all
//    stamped with the zero time), each module's world-to-ground rotation
//    (child_frame_id "<module>") and the correction it currently applies
//    (child_frame_id "<module>/drift"). Corrections arrive on
//    `drift_correction_topic` as rotations in ground (child_frame_id
//    "<module>") and are applied as R_ground_bone = W * R_ref * R_cur * C.
//    Only corrections stamped with the current zero time are used; each zero
//    clears them. Without the node nothing changes. `drift_correction`
//    false ignores corrections altogether.

#include "rclcpp/rclcpp.hpp"
#include <OpenSim/OpenSim.h>
#include <memory>      
#include "nav_msgs/msg/odometry.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/string.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "geometry_msgs/msg/quaternion.hpp"
#include "tf2_msgs/msg/tf_message.hpp"
#include "builtin_interfaces/msg/time.hpp"
#include <string>
#include <unordered_map>
#include <string>
#include <mutex>
#include <chrono>
#include <deque>
#include <algorithm>
#include <cmath>
#include <cstdio>
#include <filesystem>
#include <fstream>
#include <regex>
#include <sstream>
#include <cmath>


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
    this->declare_parameter<std::string>("calibration_dir", "");
    this->declare_parameter<std::string>("zero_pose_file", "");
    this->declare_parameter<std::string>("zero_pose", "neutral");
    this->declare_parameter<double>("elv_angle_prior_weight", 0.002);
    this->declare_parameter<double>("elv_angle_prior_deg", 0.0);
    this->declare_parameter<std::vector<std::string>>("lock_coordinates", std::vector<std::string>{});
    this->declare_parameter<bool>("drift_correction", true);
    this->declare_parameter<std::string>("drift_correction_topic", "/argus/drift_correction");
    this->declare_parameter<std::string>("drift_reference_topic", "/argus/drift_reference");

    this->declare_parameter<std::vector<std::string>>(
      "sensor_weights",
      std::vector<std::string>{}
    );

    this->declare_parameter<std::vector<std::string>>(
      "topic_translations",
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

    std::vector<std::string> sensor_weights;
    this->get_parameter("sensor_weights", sensor_weights);
    RCLCPP_INFO(this->get_logger(), "Sensor weights parameter size: %d", static_cast<int>(sensor_weights.size()));

    std::vector<std::string> topic_translations;
    this->get_parameter("topic_translations", topic_translations);
    // run system initialization (creation of all objects)
    // later we will run the zero-ing process, when user requests it.
    init_system(model_folder_path, topic_names, joint_outputs, body_calibrations, sensor_weights, topic_translations);
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
   * @param topic_translations Optional "alias:body" entries. See resolve_segment_name().
   */
  void init_system(const std::string model_folder_path, const std::vector<std::string> topic_names, 
    const std::vector<std::string> joint_outputs, const std::vector<std::string>& body_calibrations,
    const std::vector<std::string>& sensor_weights, const std::vector<std::string>& topic_translations)
  {
    ParseBodyCalibrations(body_calibrations);
    LoadNewestCalibration(model_folder_path);
    ParseOrientationWeights(sensor_weights); // Call the new function
    ParseTopicTranslations(topic_translations);

    auto now = this->get_clock()->now();
    double timestamp = now.seconds();

    // Sensor callbacks stay responsive while IK runs on another thread.
    sensor_cb_group_ = this->create_callback_group(
      rclcpp::CallbackGroupType::Reentrant);
    // Timer and reset share OpenSim state; keep them exclusive of each other.
    solve_cb_group_ = this->create_callback_group(
      rclcpp::CallbackGroupType::MutuallyExclusive);

    rclcpp::SubscriptionOptions sensor_options;
    sensor_options.callback_group = sensor_cb_group_;
    rclcpp::SubscriptionOptions solve_options;
    solve_options.callback_group = solve_cb_group_;

    // Create a subscription to the reset_tracking topic to allow for zeroing the tracking.
    reset_sub = this->create_subscription<std_msgs::msg::Bool>(
      "reset_tracking",
      10,
      [this](const std_msgs::msg::Bool::SharedPtr msg) {
        RCLCPP_INFO(this->get_logger(), "Received reset publish");
        reset_tracking(msg->data);
      },
      solve_options
    );

    // Switch the zero pose by name; same group as reset so they never overlap.
    load_zero_pose_sub_ = this->create_subscription<std_msgs::msg::String>(
      "load_zero_pose",
      10,
      [this](const std_msgs::msg::String::SharedPtr msg) {
        std::string name = msg->data;
        name.erase(0, name.find_first_not_of(" \t\n"));
        name.erase(name.find_last_not_of(" \t\n") + 1);
        LoadZeroPose(name.empty() ? "neutral" : name);
      },
      solve_options
    );

    tracked_segment_count = topic_names.size();
    joint_output_ref = joint_outputs;



    for (size_t i = 0; i < joint_outputs.size(); i++) {
      RCLCPP_INFO(this->get_logger(), "allowed joint: %s", joint_outputs[i].c_str());
    }
  
    RCLCPP_INFO(this->get_logger(), "Found segments: %d", tracked_segment_count);

    // Best-effort, keep-last-1: IK only needs the newest pose. A Reliable
    // subscription forces ACK/retransmission on the Pis and stalls them when
    // this machine or the hotspot falls behind.
    auto sensor_qos = rclcpp::SensorDataQoS().keep_last(1);

    // establish references to the data supply topics (odom messages)
    for (const auto& topic_name : topic_names) {
      std::string segment_name = resolve_segment_name(topic_name);
      RCLCPP_INFO(this->get_logger(), "Subscribing to topic: %s as body %s", topic_name.c_str(), segment_name.c_str());
      topic_stats_[topic_name].segment = segment_name;
      debug_topic_order_.push_back(topic_name);
      
      // Create a subscription for each odometry topic.
      auto subscription = this->create_subscription<nav_msgs::msg::Odometry>(
        topic_name,
        sensor_qos,
        [this, topic_name](const nav_msgs::msg::Odometry::SharedPtr msg) {
          this->odom_cb(msg, topic_name);
        },
        sensor_options
      );
      segment_ref.push_back(segment_name);
      odom_subs[segment_name] = subscription;
      const std::string module = extract_segment_name(topic_name);
      segment_module_[segment_name] = module;
      module_segment_[module] = segment_name;
    }

    // Drift correction: the reference is latched so a node started later
    // still gets the current zero. Corrections share the solve group, so they
    // never land in the middle of a zero.
    std::string drift_reference_topic, drift_correction_topic;
    this->get_parameter("drift_reference_topic", drift_reference_topic);
    this->get_parameter("drift_correction_topic", drift_correction_topic);
    drift_reference_pub_ = this->create_publisher<tf2_msgs::msg::TFMessage>(
      drift_reference_topic, rclcpp::QoS(1).reliable().transient_local());
    bool drift_correction = true;
    this->get_parameter("drift_correction", drift_correction);
    if (drift_correction) {
      drift_correction_sub_ = this->create_subscription<tf2_msgs::msg::TFMessage>(
        drift_correction_topic, 10,
        [this](const tf2_msgs::msg::TFMessage::SharedPtr msg) { on_drift_correction(*msg); },
        solve_options);
      RCLCPP_INFO(this->get_logger(), "Drift corrections accepted on %s (optional; none applied until received)",
        drift_correction_topic.c_str());
    } else {
      RCLCPP_INFO(this->get_logger(), "Drift corrections disabled");
    }

    // Create a timer to periodically run the IK tracking.
    // This is the goal frequency, which will not be met if computationally constrained.
    // New data overwrites old data, IK is always run on the most recent pose data.
    solve_timer = this->create_wall_timer(
      std::chrono::milliseconds(20),
      std::bind(&ArgusServer::track_motion, this),
      solve_cb_group_
    );

    // Create a publisher for the joint angles.
    joint_angle_pub = this->create_publisher<sensor_msgs::msg::JointState>(
      "/joint_states", 10);

    // Timing summary for argus_debug. Same group as the solve so the solve
    // statistics need no extra locking.
    debug_pub_ = this->create_publisher<std_msgs::msg::String>("/argus_debug", 10);
    debug_timer_ = this->create_wall_timer(
      std::chrono::milliseconds(250),
      std::bind(&ArgusServer::publish_debug, this),
      solve_cb_group_
    );

    // Create or load the model. If you pass a path in `model_path`, we'll
    // attempt to load that model; otherwise we construct an empty Model.
    OpenSim::ModelVisualizer::addDirToGeometrySearchPaths(
        model_folder_path + "/Geometry"
    );
    model_ptr = std::make_unique<Model>(model_folder_path + "/model.osim");



    state = model_ptr->initSystem();
    model_ptr->realizePosition(state);
    {
      auto & cs = model_ptr->updCoordinateSet();
      std::vector<std::string> lock_coordinates;
      this->get_parameter("lock_coordinates", lock_coordinates);
      for (const auto & name : lock_coordinates) {
        if (!cs.contains(name)) {
          RCLCPP_WARN(this->get_logger(), "lock_coordinates: no coordinate %s", name.c_str());
          continue;
        }
        auto & coord = cs.get(name);
        coord.setValue(state, coord.getDefaultValue(), false);
        coord.setLocked(state, true);
        RCLCPP_INFO(this->get_logger(), "Locked unused joint %s", name.c_str());
      }
    }
    {
      auto & cs = model_ptr->updCoordinateSet();
      const char * names[][3] = {{"r_x", "r_y", "r_z"}, {"pelvis_tilt", "pelvis_list", "pelvis_rotation"}};
      root_rx_ = root_ry_ = root_rz_ = nullptr;
      for (const auto & row : names) {
        if (cs.contains(row[0]) && cs.contains(row[1]) && cs.contains(row[2])) {
          root_rx_ = &cs.get(row[0]);
          root_ry_ = &cs.get(row[1]);
          root_rz_ = &cs.get(row[2]);
          break;
        }
      }
      n_shoulders_ = 0;
      for (const char * side : {"r", "l"}) {
        const std::string elv = std::string("elv_angle_") + side;
        const std::string elev = std::string("shoulder_elv_") + side;
        const std::string rot = std::string("shoulder_rot_") + side;
        if (!cs.contains(elv) || !cs.contains(elev) || !cs.contains(rot)) {
          continue;
        }
        auto & sh = shoulders_[n_shoulders_++];
        sh.elv = &cs.get(elv);
        sh.elev = &cs.get(elev);
        sh.rot = &cs.get(rot);
        const std::string unrot = std::string("shoulder1_r2_") + side;
        sh.unrot = cs.contains(unrot) ? &cs.get(unrot) : nullptr;
        sh.have = false;
        sh.elv->setClamped(state, false);
        sh.rot->setClamped(state, false);
      }
    }
    model_default_q_ = state.getQ();
    // The zero pose: the model default until a pose file entry replaces it.
    default_q_ = model_default_q_;
    default_body_orientations_ = get_body_orientations_as_quaternions(state);
    zero_pose_dir_model_ = model_folder_path;
    {
      std::string zero_pose_name;
      this->get_parameter("zero_pose", zero_pose_name);
      LoadZeroPose(zero_pose_name.empty() ? "neutral" : zero_pose_name);
    }
    // Start IK from the zero pose, the same pose offline uses for its zero sample.
    state.setQ(default_q_);
    model_ptr->realizePosition(state);

    const auto& bodySet = model_ptr->getBodySet();

    RCLCPP_INFO(this->get_logger(), "=== VALID BODY NAMES ===");
    for (int i = 0; i < bodySet.getSize(); ++i) {
      const auto& body = bodySet.get(i);
      RCLCPP_INFO(this->get_logger(), "Body: %s", body.getName().c_str());
    }

    // The solver needs an initial orientation table. Use the default pose so
    // assemble() stays on the same pose offline uses for its zero sample.
    OpenSim::TimeSeriesTable_<SimTK::Rotation> initial_table;
    initial_table.setColumnLabels(segment_ref);
    SimTK::RowVector_<SimTK::Rotation> default_row;
    default_row.resize(tracked_segment_count);
    for (int i = 0; i < tracked_segment_count; ++i) {
      const std::string& segment_name = segment_ref[i];
      auto pose = default_body_orientations_.find(segment_name);
      if (pose == default_body_orientations_.end()) {
        default_row[i] = SimTK::Rotation();
        continue;
      }
      default_row[i] = SimTK::Rotation(pose->second);
    }
    initial_table.appendRow(timestamp, default_row);

    // this is marker constraints, we have no markers, so leave it empty
    markers_ref_ = std::make_shared<OpenSim::MarkersReference>();

    // The only coordinate goal is a weak pull on the plane of elevation.
    // Unlike the orientation goal it is not divided by the total sensor
    // weight, so 0.002 is about 1% of one arm's pull: with the arm raised
    // the orientations outweigh it, with the arm hanging it keeps the plane
    // off its limit.
    coordinate_refs_.clear();
    double elv_prior_weight = 0.0;
    double elv_prior_deg = 0.0;
    this->get_parameter("elv_angle_prior_weight", elv_prior_weight);
    this->get_parameter("elv_angle_prior_deg", elv_prior_deg);
    if (elv_prior_weight > 0.0) {
      const OpenSim::Constant target(SimTK::convertDegreesToRadians(elv_prior_deg));
      for (const std::string name : {"elv_angle_r", "elv_angle_l"}) {
        if (!model_ptr->getCoordinateSet().contains(name)) {
          RCLCPP_WARN(this->get_logger(), "No coordinate %s, no plane of elevation target", name.c_str());
          continue;
        }
        OpenSim::CoordinateReference ref(name, target);
        ref.setWeight(elv_prior_weight);
        coordinate_refs_.push_back(ref);
      }
      RCLCPP_INFO(this->get_logger(), "Plane of elevation target %.1f deg, weight %g",
                  elv_prior_deg, elv_prior_weight);
    }

    // this is orientation constraints, which is the only thing we actually will constrain
    OpenSim::Set<OpenSim::OrientationWeight> sensor_weights_set = construct_orientation_weights(sensor_weights_map);
    orientations_ref_ = std::make_shared<OpenSim::BufferedOrientationsReference>(initial_table, &sensor_weights_set);

    // Construct the IK solver and keep it on the class so other methods can use it.
    // A finite constraint weight turns the shoulder coupler constraints into a
    // penalty instead of enforcing them exactly; this makes track() ~20x
    // faster on this model. Same value as RealTimeKin.
    ik_solver_ = std::make_unique<OpenSim::InverseKinematicsSolver>(
      *model_ptr,
      markers_ref_,
      orientations_ref_,
      coordinate_refs_,
      10.0
    );

    // Looser than the OpenSim default of 1e-4. Same value as RealTimeKin.
    ik_solver_->setAccuracy(0.001);

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

    std::string segment_name = resolve_segment_name(topic_name);
    const auto now = std::chrono::steady_clock::now();
    std::lock_guard<std::mutex> lock(odom_mutex);
    cur_odom_data[segment_name] = q;
    latest_odom_stamp_ = msg->header.stamp;

    auto& stats = topic_stats_[topic_name];
    if (stats.last != std::chrono::steady_clock::time_point{}) {
      stats.gaps.emplace_back(now, std::chrono::duration<double, std::milli>(now - stats.last).count());
    }
    stats.last = now;
    while (!stats.gaps.empty() && now - stats.gaps.front().first > std::chrono::seconds(20)) {
      stats.gaps.pop_front();
    }
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

    // track() pops one queued row and blocks forever if none was queued.
    if (!add_orientation_data()) {
      return;
    }

    state.setTime(current_timestamp);
      // ---- start timing ----
    auto t0 = std::chrono::high_resolution_clock::now();

    ik_solver_->track(state);
    snap_root_euler();
    snap_shoulder_euler();

    auto t1 = std::chrono::high_resolution_clock::now();
    // ---- end timing ----
    model_ptr->realizeReport(state);

    publish_joint_angles();

    // compute duration (microseconds → milliseconds)
    auto duration_us = std::chrono::duration_cast<std::chrono::microseconds>(t1 - t0).count();
    double duration_ms = duration_us / 1000.0;

    const auto now = std::chrono::steady_clock::now();
    solve_times_.emplace_back(now, duration_ms);
    while (!solve_times_.empty() && now - solve_times_.front().first > std::chrono::seconds(1)) {
      solve_times_.pop_front();
    }
  }

  /**
   * @brief Publishes a timing summary on /argus_debug.
   *
   * - mean IK solve time over the last 1 s
   * - per odometry topic, the mean gap between messages over the last 2 s
   *   and the worst gap over the last 20 s. A gap still open (no message
   *   since) counts toward the worst gap.
   */
  void publish_debug()
  {
    const auto now = std::chrono::steady_clock::now();
    std::string out;
    char line[256];

    while (!solve_times_.empty() && now - solve_times_.front().first > std::chrono::seconds(1)) {
      solve_times_.pop_front();
    }
    if (solve_times_.empty()) {
      std::snprintf(line, sizeof(line), "solve: no solves in the last 1 s%s\n",
        is_zeroed ? "" : " (not zeroed; publish /reset_tracking)");
    } else {
      double sum = 0.0;
      for (const auto& s : solve_times_) sum += s.second;
      std::snprintf(line, sizeof(line), "solve: mean %.2f ms over %zu solves (last 1 s)\n",
        sum / solve_times_.size(), solve_times_.size());
    }
    out += line;

    std::lock_guard<std::mutex> lock(odom_mutex);
    for (const auto& topic : debug_topic_order_) {
      auto& stats = topic_stats_[topic];
      while (!stats.gaps.empty() && now - stats.gaps.front().first > std::chrono::seconds(20)) {
        stats.gaps.pop_front();
      }
      if (stats.last == std::chrono::steady_clock::time_point{}) {
        std::snprintf(line, sizeof(line), "%s (%s): no messages yet\n",
          topic.c_str(), stats.segment.c_str());
        out += line;
        continue;
      }

      const double open_gap_ms = std::chrono::duration<double, std::milli>(now - stats.last).count();
      double worst_ms = open_gap_ms;
      double sum_2s = 0.0;
      size_t n_2s = 0;
      for (const auto& g : stats.gaps) {
        worst_ms = std::max(worst_ms, g.second);
        if (now - g.first <= std::chrono::seconds(2)) {
          sum_2s += g.second;
          ++n_2s;
        }
      }
      char mean_txt[48];
      if (n_2s > 0) {
        std::snprintf(mean_txt, sizeof(mean_txt), "%.1f ms", sum_2s / n_2s);
      } else {
        std::snprintf(mean_txt, sizeof(mean_txt), "none (last msg %.0f ms ago)", open_gap_ms);
      }
      std::snprintf(line, sizeof(line), "%s (%s): mean gap %s (last 2 s), worst gap %.0f ms (last 20 s)\n",
        topic.c_str(), stats.segment.c_str(), mean_txt, worst_ms);
      out += line;
    }
    if (!drift_rotations_.empty()) {
      out += "drift correction:";
      for (const auto& segment_name : segment_ref) {
        const auto drift = drift_rotations_.find(segment_name);
        if (drift == drift_rotations_.end()) {
          continue;
        }
        std::snprintf(line, sizeof(line), " %s %.2f deg", segment_module_[segment_name].c_str(),
          SimTK::convertRadiansToDegrees(drift->second.convertRotationToAngleAxis()[0]));
        out += line;
      }
      out += "\n";
    }

    std_msgs::msg::String msg;
    msg.data = out;
    debug_pub_->publish(msg);
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


    if(reset_command != true) {
      return;
    }

    for (const std::string& segment_name : segment_ref) {
      if (cur_odom_data.find(segment_name) == cur_odom_data.end()) {
        RCLCPP_WARN(this->get_logger(),
          "No odometry yet for '%s'. Zero was not applied.", segment_name.c_str());
        return;
      }
      if (default_body_orientations_.find(segment_name) == default_body_orientations_.end()) {
        RCLCPP_WARN(this->get_logger(),
          "No default pose for '%s'. Zero was not applied.", segment_name.c_str());
        return;
      }
    }

    RCLCPP_INFO(this->get_logger(), "Resetting tracking references to zero pose '%s'...",
      zero_pose_name_.c_str());

    // Start the next solve from the same coordinates offline uses.
    state.setQ(default_q_);
    model_ptr->realizePosition(state);

    for (const std::string& segment_name : segment_ref) {
      SimTK::Rotation cur_body_R(default_body_orientations_.at(segment_name));

      const auto& q_msg = cur_odom_data.at(segment_name);
      SimTK::Quaternion q_odom(q_msg.w, q_msg.x, q_msg.y, q_msg.z);
      SimTK::Rotation cur_odom_R(q_odom);

      // Same product as offline: R_GB * C^T * R_zero^T
      SimTK::Rotation ref_odom_R = cur_body_R * ~body_calibration_rotations[segment_name] * ~cur_odom_R;
      SimTK::Quaternion ref_quat(ref_odom_R);

      geometry_msgs::msg::Quaternion ref_msg;
      ref_msg.w = ref_quat[0];
      ref_msg.x = ref_quat[1];
      ref_msg.y = ref_quat[2];
      ref_msg.z = ref_quat[3];

      ref_odom_data[segment_name] = ref_msg;
    }
    is_zeroed = true;
    root_have_prev_ = false;
    for (auto & sh : shoulders_) {
      sh.have = false;
    }
    drift_rotations_.clear();
    zero_stamp_ = this->get_clock()->now();
    publish_drift_reference();
  }

  /**
   * @brief Publishes each module's world-to-ground rotation and its drift
   * correction, stamped with the zero time. Caller holds odom_mutex.
   */
  void publish_drift_reference()
  {
    tf2_msgs::msg::TFMessage out;
    for (const std::string& segment_name : segment_ref) {
      const auto ref = ref_odom_data.find(segment_name);
      if (ref == ref_odom_data.end()) {
        continue;
      }
      geometry_msgs::msg::TransformStamped item;
      item.header.stamp = zero_stamp_;
      item.header.frame_id = "ground";
      item.child_frame_id = segment_module_[segment_name];
      item.transform.rotation = ref->second;
      out.transforms.push_back(item);

      SimTK::Quaternion w;
      const auto drift = drift_rotations_.find(segment_name);
      if (drift != drift_rotations_.end()) {
        w = SimTK::Quaternion(drift->second);
      }
      item.child_frame_id = segment_module_[segment_name] + "/drift";
      item.transform.rotation.w = w[0];
      item.transform.rotation.x = w[1];
      item.transform.rotation.y = w[2];
      item.transform.rotation.z = w[3];
      out.transforms.push_back(item);
    }
    drift_reference_pub_->publish(out);
  }

  /**
   * @brief Takes drift corrections (rotations in ground, one per module)
   * from the argus_drift node. Corrections made for an earlier zero are
   * ignored.
   */
  void on_drift_correction(const tf2_msgs::msg::TFMessage& msg)
  {
    std::lock_guard<std::mutex> lock(odom_mutex);
    if (!is_zeroed) {
      return;
    }
    std::string applied;
    int stale = 0;
    for (const auto& item : msg.transforms) {
      if (rclcpp::Time(item.header.stamp) != rclcpp::Time(zero_stamp_)) {
        ++stale;
        continue;
      }
      const auto segment = module_segment_.find(item.child_frame_id);
      if (segment == module_segment_.end()) {
        continue;
      }
      const auto& q = item.transform.rotation;
      const double norm = std::sqrt(q.w * q.w + q.x * q.x + q.y * q.y + q.z * q.z);
      if (!std::isfinite(norm) || norm < 0.5) {
        RCLCPP_WARN(this->get_logger(), "Drift correction for %s is not a rotation; ignored",
          item.child_frame_id.c_str());
        continue;
      }
      const SimTK::Rotation w(SimTK::Quaternion(q.w / norm, q.x / norm, q.y / norm, q.z / norm));
      drift_rotations_[segment->second] = w;
      char text[64];
      std::snprintf(text, sizeof(text), " %s %.2f deg", item.child_frame_id.c_str(),
        SimTK::convertRadiansToDegrees(w.convertRotationToAngleAxis()[0]));
      applied += text;
    }
    if (stale > 0) {
      RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
        "Ignored %d drift correction(s) made for an earlier zero", stale);
    }
    if (!applied.empty()) {
      RCLCPP_INFO(this->get_logger(), "Drift correction:%s", applied.c_str());
      publish_drift_reference();
    }
  }

  /**
   * @brief Adds orientation data to the IK solver.
   * 
   * This function retrieves the latest orientation data for each tracked
   * segment, applies the reference offset, and adds the resulting orientation
   * to the IK solver's orientation reference.
   *
   * @return true if a row was queued for the next track() call.
   */
  bool add_orientation_data()
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
        return false;
    }

    for (int i = 0; i < tracked_segment_count; i++) {
      std::string segment_name = segment_ref[i];
      
      if(cur_odom_data.find(segment_name) == cur_odom_data.end() || ref_odom_data.find(segment_name) == ref_odom_data.end()) {
        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
          "Missing odometry data for segment '%s', skipping IK update.", segment_name.c_str());
        return false;
      }

      cur_quat = cur_odom_data[segment_name];
      ref_quat = ref_odom_data[segment_name];

      SimTK::Quaternion q_cur(cur_quat.w, cur_quat.x, cur_quat.y, cur_quat.z);
      SimTK::Quaternion q_ref(ref_quat.w, ref_quat.x, ref_quat.y, ref_quat.z);

      SimTK::Rotation R_ref(q_ref);
      SimTK::Rotation R_cur(q_cur);

      SimTK::Rotation R_rel = R_ref * R_cur * body_calibration_rotations[segment_name];
      const auto drift = drift_rotations_.find(segment_name);
      if (drift != drift_rotations_.end()) {
        R_rel = drift->second * R_rel;
      }

      dataRow[i] = R_rel;

    }
    
    orientations_ref_->putValues(timestamp, dataRow);

    const SimTK::Array_<std::string> names = orientations_ref_->getNames();

    current_timestamp = timestamp;
    solve_input_stamp_ = latest_odom_stamp_;
    return true;
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
      // Header stamp of the newest odometry message behind this solve, so the
      // angles carry the module sample time. Unstamped input falls back to now.
      if (solve_input_stamp_.sec != 0 || solve_input_stamp_.nanosec != 0) {
        joint_state_msg.header.stamp = solve_input_stamp_;
      } else {
        joint_state_msg.header.stamp = this->get_clock()->now();
      }
  
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

  // (a, b, c) and (a+π, π-b, c+π) are the same orientation. Stay on the chart
  // of the previous frame so the upright trunk does not flash Euler wells.
  void snap_root_euler()
  {
    if (root_rx_ == nullptr) {
      return;
    }
    double a = root_rx_->getValue(state), b = root_ry_->getValue(state), c = root_rz_->getValue(state);
    if (root_have_prev_) {
      const double pi = SimTK::Pi;
      const double cand[2][3] = {{a, b, c}, {a + pi, pi - b, c + pi}};
      double best_d = 1e300, best[3] = {a, b, c};
      for (int k = 0; k < 2; ++k) {
        double f[3];
        for (int i = 0; i < 3; ++i) {
          f[i] = cand[k][i] + 2 * pi * std::round((root_prev_[i] - cand[k][i]) / (2 * pi));
        }
        const double d = (f[0] - root_prev_[0]) * (f[0] - root_prev_[0]) +
                         (f[1] - root_prev_[1]) * (f[1] - root_prev_[1]) +
                         (f[2] - root_prev_[2]) * (f[2] - root_prev_[2]);
        if (d < best_d) {
          best_d = d;
          best[0] = f[0];
          best[1] = f[1];
          best[2] = f[2];
        }
      }
      root_rx_->setValue(state, best[0], false);
      root_ry_->setValue(state, best[1], false);
      root_rz_->setValue(state, best[2], false);
      model_ptr->realizePosition(state);
      a = best[0];
      b = best[1];
      c = best[2];
    }
    root_prev_[0] = a;
    root_prev_[1] = b;
    root_prev_[2] = c;
    root_have_prev_ = true;
  }

  // Glenohumeral YXY with coupler unrot = -elv. Stay on the previous chart
  // so plane of elevation / axial rotation do not wrap onto the other well.
  void snap_shoulder_euler()
  {
    const double pi = SimTK::Pi;
    const double lock = 20.0 * pi / 180.0;
    for (int s = 0; s < n_shoulders_; ++s) {
      auto & sh = shoulders_[s];
      double a = sh.elv->getValue(state), b = sh.elev->getValue(state), c = sh.rot->getValue(state);
      if (sh.have) {
        const double cand[2][3] = {{a, b, c}, {a + pi, -b, c}};
        double best_d = 1e300, best[3] = {a, b, c};
        for (int k = 0; k < 2; ++k) {
          double f[3] = {cand[k][0], cand[k][1], cand[k][2]};
          for (int i = 0; i < 3; ++i) {
            f[i] += 2 * pi * std::round((sh.prev[i] - f[i]) / (2 * pi));
          }
          const double d = (f[0] - sh.prev[0]) * (f[0] - sh.prev[0]) +
                           (f[1] - sh.prev[1]) * (f[1] - sh.prev[1]) +
                           (f[2] - sh.prev[2]) * (f[2] - sh.prev[2]);
          if (d < best_d) {
            best_d = d;
            best[0] = f[0];
            best[1] = f[1];
            best[2] = f[2];
          }
        }
        if (std::abs(best[1]) < lock || std::abs(std::abs(best[1]) - pi) < lock) {
          const double gamma = best[2] - best[0];
          best[0] = sh.prev[0];
          best[2] = gamma + best[0];
          best[2] += 2 * pi * std::round((sh.prev[2] - best[2]) / (2 * pi));
        }
        sh.elv->setValue(state, best[0], false);
        sh.elev->setValue(state, best[1], false);
        sh.rot->setValue(state, best[2], false);
        if (sh.unrot != nullptr) {
          sh.unrot->setValue(state, -best[0], false);
        }
        model_ptr->realizePosition(state);
        a = best[0];
        b = best[1];
        c = best[2];
      }
      sh.prev[0] = a;
      sh.prev[1] = b;
      sh.prev[2] = c;
      sh.have = true;
    }
  }

  /**
   * @brief Maps a subscribed topic onto an OpenSim body name.
   *
   * Looks up topic_translations by the first path element, then by the topic
   * with and without a leading slash. Without a match, the first path element
   * is the body name.
   *
   * Examples:
   *   /argus1/odomimu with "argus1:thorax" -> thorax
   *   /humerus_r/odomimu with no entry     -> humerus_r
   */
  std::string resolve_segment_name(const std::string& topic_name)
  {
      std::string token = extract_segment_name(topic_name);
      auto it = topic_translations_.find(token);
      if (it != topic_translations_.end()) {
          return it->second;
      }

      std::string stripped = topic_name;
      if (!stripped.empty() && stripped.front() == '/') {
          stripped.erase(stripped.begin());
      }
      it = topic_translations_.find(stripped);
      if (it != topic_translations_.end()) {
          return it->second;
      }

      it = topic_translations_.find(topic_name);
      if (it != topic_translations_.end()) {
          return it->second;
      }

      return token;
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
   * @param pose_state State to read; its positions are realized here.
   * @return A map of body names to quaternions.
   */
  std::unordered_map<std::string, SimTK::Quaternion> get_body_orientations_as_quaternions(SimTK::State& pose_state)
  {
    std::unordered_map<std::string, SimTK::Quaternion> result;

    // Make sure transforms are up to date
    model_ptr->realizePosition(pose_state);

    const auto& bodySet = model_ptr->getBodySet();

    for (const auto& segment_name : segment_ref) {

      if (!bodySet.contains(segment_name)) {
        RCLCPP_WARN(this->get_logger(),
          "Body not found in model: %s", segment_name.c_str());
        continue;
      }

      const OpenSim::Body& body = bodySet.get(segment_name);

      // Transform of body in Ground
      const SimTK::Transform& X_GB = body.getTransformInGround(pose_state);

      // Extract rotation
      const SimTK::Rotation& R_GB = X_GB.R();

      // Convert to quaternion
      SimTK::Quaternion q(R_GB);

      result[segment_name] = q;
    }

    return result;
  }


  OpenSim::Set<OpenSim::OrientationWeight> construct_orientation_weights(std::unordered_map<std::string, double>& sensor_weights) 
  {
    RCLCPP_INFO(this->get_logger(), "Constructing orientation weights for IK solver...");
    OpenSim::Set<OpenSim::OrientationWeight> weights;
    for (const auto& pair : sensor_weights) {
      RCLCPP_INFO(this->get_logger(), "Sensor: %s, Weight: %f", pair.first.c_str(), pair.second);
    }

    for (int i = 0; i < tracked_segment_count; ++i) {
      if (sensor_weights.find(segment_ref[i]) == sensor_weights.end()) {
        RCLCPP_WARN(this->get_logger(), "No weight found for segment '%s', defaulting to 1.0", segment_ref[i].c_str());
        weights.cloneAndAppend(OpenSim::OrientationWeight(segment_ref[i], 1.0));
        continue;
      }
      RCLCPP_INFO(this->get_logger(), "Setting orientation weight for segment %s: %f", segment_ref[i].c_str(), sensor_weights[segment_ref[i]]);
      weights.cloneAndAppend(OpenSim::OrientationWeight(segment_ref[i], sensor_weights.at(segment_ref[i])));
    }

    return weights;
  }

  void ParseBodyCalibrations(std::vector<std::string> body_calibrations)
  {
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
          // Degrees, extrinsic xyz. Matches scipy from_euler('xyz', ..., degrees=True).
          const double deg_to_rad = SimTK::Pi / 180.0;
          SimTK::Rotation rot(SimTK::SpaceRotationSequence,
            euler_angles[0] * deg_to_rad, SimTK::XAxis,
            euler_angles[1] * deg_to_rad, SimTK::YAxis,
            euler_angles[2] * deg_to_rad, SimTK::ZAxis);
          body_calibration_rotations[body_name] = rot;
          RCLCPP_INFO(this->get_logger(),
            "Loaded calibration for %s: xyz degrees [%.3f, %.3f, %.3f]",
            body_name.c_str(), euler_angles[0], euler_angles[1], euler_angles[2]);
      } else {
          RCLCPP_WARN(this->get_logger(), "Incorrect number of euler angles for %s", body_name.c_str());
      }
    }
  }

  // Brace-matched object starting at text[open] == '{'. Empty when unbalanced.
  static std::string JsonObjectAt(const std::string& text, size_t open)
  {
    if (open >= text.size() || text[open] != '{') {
      return "";
    }
    int depth = 0;
    for (size_t i = open; i < text.size(); ++i) {
      if (text[i] == '{') {
        ++depth;
      } else if (text[i] == '}' && --depth == 0) {
        return text.substr(open, i - open + 1);
      }
    }
    return "";
  }

  // Make a named pose from the pose file the zero pose. On any problem the
  // current zero pose stays in place.
  bool LoadZeroPose(const std::string& pose_name)
  {
    std::string file_param;
    this->get_parameter("zero_pose_file", file_param);
    std::filesystem::path path = file_param;
    if (path.empty()) {
      path = std::filesystem::path(zero_pose_dir_model_).parent_path().parent_path()
        / "poses" / "poses.json";
    }
    std::ifstream input(path);
    if (!input) {
      RCLCPP_WARN(this->get_logger(), "Cannot read zero pose file %s; zero pose stays '%s'",
        path.c_str(), zero_pose_name_.c_str());
      return false;
    }
    std::stringstream buffer;
    buffer << input.rdbuf();
    const std::string text = buffer.str();

    // Find the pose object whose "name" matches.
    const std::regex name_re("\"name\"\\s*:\\s*\"([^\"]*)\"");
    std::string pose_object;
    for (auto it = std::sregex_iterator(text.begin(), text.end(), name_re);
         it != std::sregex_iterator(); ++it) {
      if ((*it)[1].str() != pose_name) {
        continue;
      }
      int depth = 0;
      for (size_t i = static_cast<size_t>(it->position()); i-- > 0;) {
        if (text[i] == '}') {
          ++depth;
        } else if (text[i] == '{') {
          if (depth == 0) {
            pose_object = JsonObjectAt(text, i);
            break;
          }
          --depth;
        }
      }
      break;
    }
    if (pose_object.empty()) {
      RCLCPP_WARN(this->get_logger(), "No pose named '%s' in %s; zero pose stays '%s'",
        pose_name.c_str(), path.c_str(), zero_pose_name_.c_str());
      return false;
    }
    const auto angles_key = pose_object.find("\"joint_angles\"");
    const std::string angles_object = angles_key == std::string::npos
      ? "" : JsonObjectAt(pose_object, pose_object.find('{', angles_key));
    if (angles_object.empty()) {
      RCLCPP_WARN(this->get_logger(), "Pose '%s' in %s has no joint_angles; zero pose stays '%s'",
        pose_name.c_str(), path.c_str(), zero_pose_name_.c_str());
      return false;
    }

    // Angles are degrees unless the file says "units": "radians".
    std::smatch units;
    const bool radians = std::regex_search(text, units, std::regex("\"units\"\\s*:\\s*\"([A-Za-z]+)\"")) &&
      units[1].str().rfind("rad", 0) == 0;
    const double angle_scale = radians ? 1.0 : SimTK::Pi / 180.0;

    // Work on a copy so tracking keeps its state. Unlisted coordinates keep
    // the model default; listed ones are locked while the coupler
    // constraints (shoulder girdle) are solved around them.
    SimTK::State pose_state = state;
    pose_state.updQ() = model_default_q_;
    auto& coords = model_ptr->updCoordinateSet();
    std::vector<std::string> applied;
    const std::regex entry_re("\"([A-Za-z0-9_]+)\"\\s*:\\s*([-+0-9.eE]+)");
    for (auto it = std::sregex_iterator(angles_object.begin(), angles_object.end(), entry_re);
         it != std::sregex_iterator(); ++it) {
      const std::string name = (*it)[1].str();
      if (!coords.contains(name)) {
        RCLCPP_WARN(this->get_logger(), "Zero pose '%s': %s is not a model coordinate, skipped",
          pose_name.c_str(), name.c_str());
        continue;
      }
      auto& coord = coords.get(name);
      if (coord.getLocked(pose_state)) {
        RCLCPP_WARN(this->get_logger(), "Zero pose '%s': %s is locked in the model, skipped",
          pose_name.c_str(), name.c_str());
        continue;
      }
      double value = std::stod((*it)[2].str());
      if (coord.getMotionType() != OpenSim::Coordinate::Translational) {
        value *= angle_scale;
      }
      coord.setValue(pose_state, value, false);
      applied.push_back(name);
    }
    for (const auto& name : applied) {
      coords.get(name).setLocked(pose_state, true);
    }
    try {
      model_ptr->assemble(pose_state);
    } catch (const std::exception& error) {
      RCLCPP_WARN(this->get_logger(), "Zero pose '%s': coupled coordinates not solved (%s)",
        pose_name.c_str(), error.what());
    }
    for (const auto& name : applied) {
      coords.get(name).setLocked(pose_state, false);
    }

    default_q_ = pose_state.getQ();
    default_body_orientations_ = get_body_orientations_as_quaternions(pose_state);
    zero_pose_name_ = pose_name;
    RCLCPP_INFO(this->get_logger(), "Zero pose '%s' from %s (%zu coordinates set)",
      pose_name.c_str(), path.c_str(), applied.size());
    if (is_zeroed) {
      RCLCPP_INFO(this->get_logger(),
        "Publish /reset_tracking while holding '%s' to zero against it", pose_name.c_str());
    }
    return true;
  }

  // Newest calibration_*.json wins. Its offsets replace the launch-file
  // values for the bodies it lists. Other bodies keep those defaults.
  void LoadNewestCalibration(const std::string& model_folder_path)
  {
    std::string dir_param;
    this->get_parameter("calibration_dir", dir_param);
    std::filesystem::path dir = dir_param;
    if (dir.empty()) {
      dir = std::filesystem::path(model_folder_path).parent_path().parent_path()
        / "poses" / "calibrations";
    }

    std::error_code exists_error;
    if (!std::filesystem::is_directory(dir, exists_error)) {
      RCLCPP_INFO(this->get_logger(),
        "No calibration directory %s; using launch-file offsets", dir.c_str());
      return;
    }

    std::filesystem::path newest;
    std::string newest_name;
    try {
      for (const auto& entry : std::filesystem::directory_iterator(dir)) {
        if (!entry.is_regular_file()) {
          continue;
        }
        const std::string name = entry.path().filename().string();
        if (name.rfind("calibration_", 0) != 0 || entry.path().extension() != ".json") {
          continue;
        }
        if (name > newest_name) {
          newest_name = name;
          newest = entry.path();
        }
      }
    } catch (const std::filesystem::filesystem_error& error) {
      RCLCPP_WARN(this->get_logger(),
        "Could not read %s (%s); using launch-file offsets", dir.c_str(), error.what());
      return;
    }

    if (newest.empty()) {
      RCLCPP_INFO(this->get_logger(),
        "No calibration_*.json in %s; using launch-file offsets", dir.c_str());
      return;
    }

    std::ifstream input(newest);
    std::stringstream buffer;
    buffer << input.rdbuf();
    const std::string text = buffer.str();
    const auto offsets_at = text.find("\"offsets\"");
    const auto brace = offsets_at == std::string::npos ? std::string::npos : text.find('{', offsets_at);
    if (brace == std::string::npos) {
      RCLCPP_WARN(this->get_logger(),
        "Calibration file %s has no offsets object; using launch-file offsets", newest.c_str());
      return;
    }
    int depth = 0;
    size_t end = brace;
    for (; end < text.size(); ++end) {
      if (text[end] == '{') {
        ++depth;
      } else if (text[end] == '}') {
        --depth;
        if (depth == 0) {
          break;
        }
      }
    }
    if (depth != 0) {
      RCLCPP_WARN(this->get_logger(),
        "Calibration file %s has an unfinished offsets object; using launch-file offsets",
        newest.c_str());
      return;
    }

    const std::string block = text.substr(brace, end - brace + 1);
    const std::regex entry(
      "\"([A-Za-z0-9_]+)\"\\s*:\\s*\\[\\s*([-+0-9.eE]+)\\s*,\\s*([-+0-9.eE]+)\\s*,\\s*([-+0-9.eE]+)\\s*\\]");
    const auto begin = std::sregex_iterator(block.begin(), block.end(), entry);
    const auto finish = std::sregex_iterator();
    int loaded = 0;
    const double deg_to_rad = SimTK::Pi / 180.0;
    for (auto match = begin; match != finish; ++match) {
      const std::string body_name = (*match)[1].str();
      try {
        const double x = std::stod((*match)[2].str());
        const double y = std::stod((*match)[3].str());
        const double z = std::stod((*match)[4].str());
        body_calibration_rotations[body_name] = SimTK::Rotation(
          SimTK::SpaceRotationSequence,
          x * deg_to_rad, SimTK::XAxis,
          y * deg_to_rad, SimTK::YAxis,
          z * deg_to_rad, SimTK::ZAxis);
        RCLCPP_INFO(this->get_logger(),
          "Calibration file offset for %s: xyz degrees [%.3f, %.3f, %.3f]",
          body_name.c_str(), x, y, z);
        ++loaded;
      } catch (const std::exception& error) {
        RCLCPP_WARN(this->get_logger(),
          "Skipping offset for %s in %s (%s)", body_name.c_str(), newest.c_str(), error.what());
      }
    }
    if (loaded == 0) {
      RCLCPP_WARN(this->get_logger(),
        "Calibration file %s listed no sensor offsets; using launch-file offsets", newest.c_str());
      return;
    }
    RCLCPP_INFO(this->get_logger(),
      "Loaded %d sensor offset(s) from %s", loaded, newest.c_str());
  }
  
  void ParseOrientationWeights(std::vector<std::string> sensor_weights)
  {
            // Parse sensor weights
    for (const auto& weight_str : sensor_weights) {
      size_t delimiter_pos = weight_str.find(':');
      if (delimiter_pos == std::string::npos) {
          RCLCPP_WARN(this->get_logger(), "Invalid sensor weight string format: %s", weight_str.c_str());
          continue;
      }

      std::string body_name = weight_str.substr(0, delimiter_pos);
      std::string value = weight_str.substr(delimiter_pos + 1);

      RCLCPP_INFO(this->get_logger(), "Parsing sensor weight for body '%s' with value string '%s'", body_name.c_str(), value.c_str());

      double sensor_weight;

      try {
          sensor_weight = std::stod(value);
      } catch (const std::invalid_argument& ia) {
          RCLCPP_WARN(this->get_logger(), "Invalid number in sensor weight for %s: %s", body_name.c_str(), value.c_str());
          continue;
      }
      

      sensor_weights_map[body_name] = sensor_weight;
    }
  }

  void ParseTopicTranslations(const std::vector<std::string>& topic_translations)
  {
    for (const auto& entry : topic_translations) {
      size_t delimiter_pos = entry.find(':');
      if (delimiter_pos == std::string::npos || delimiter_pos == 0 || delimiter_pos + 1 >= entry.size()) {
          RCLCPP_WARN(this->get_logger(), "Invalid topic translation '%s'. Expected alias:body", entry.c_str());
          continue;
      }

      std::string alias = entry.substr(0, delimiter_pos);
      std::string body_name = entry.substr(delimiter_pos + 1);
      topic_translations_[alias] = body_name;
      RCLCPP_INFO(this->get_logger(), "Topic alias '%s' maps to body '%s'", alias.c_str(), body_name.c_str());
    }
  }

  // --- Member Variables ---

  // OpenSim objects
  std::shared_ptr<OpenSim::MarkersReference> markers_ref_; ///< Marker references for IK (unused).
  std::shared_ptr<OpenSim::BufferedOrientationsReference> orientations_ref_; ///< Orientation references for IK.
  SimTK::Array_<OpenSim::CoordinateReference> coordinate_refs_; ///< Coordinate references for IK (unused).
  std::unique_ptr<OpenSim::InverseKinematicsSolver> ik_solver_; ///< The IK solver.
  SimTK::State state; ///< The state of the OpenSim model.
  std::unique_ptr<Model> model_ptr; ///< The OpenSim model.
  OpenSim::Coordinate * root_rx_ = nullptr;
  OpenSim::Coordinate * root_ry_ = nullptr;
  OpenSim::Coordinate * root_rz_ = nullptr;
  double root_prev_[3] = {0.0, 0.0, 0.0};
  bool root_have_prev_ = false;
  struct ShoulderEuler {
    OpenSim::Coordinate *elv = nullptr, *elev = nullptr, *rot = nullptr, *unrot = nullptr;
    double prev[3] = {0.0, 0.0, 0.0};
    bool have = false;
  };
  ShoulderEuler shoulders_[2];
  int n_shoulders_ = 0;

  // ROS 2 objects
  rclcpp::CallbackGroup::SharedPtr sensor_cb_group_; ///< Odometry callbacks; can overlap the IK solve.
  rclcpp::CallbackGroup::SharedPtr solve_cb_group_; ///< Timer and reset; exclusive of each other.
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_angle_pub; ///< Publisher for joint angles.
  std::unordered_map<std::string, rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr> odom_subs; ///< Map of odometry subscribers.
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr reset_sub; ///< Subscriber for reset commands.
  
  // Data storage
  std::unordered_map<std::string, geometry_msgs::msg::Quaternion> cur_odom_data; ///< Current odometry data.
  builtin_interfaces::msg::Time latest_odom_stamp_; ///< Header stamp of the most recently received odometry message.
  builtin_interfaces::msg::Time solve_input_stamp_; ///< latest_odom_stamp_ when the current solve's input was taken.
  std::unordered_map<std::string, geometry_msgs::msg::Quaternion> ref_odom_data; ///< Reference odometry data for zeroing.
  std::unordered_map<std::string, SimTK::Rotation> body_calibration_rotations; ///< Calibration rotations for each body.
  std::unordered_map<std::string, SimTK::Quaternion> default_body_orientations_; ///< Body orientations in ground at the default pose.
  SimTK::Vector default_q_; ///< Generalized coordinates of the zero pose.
  SimTK::Vector model_default_q_; ///< Coordinates straight from model.osim, before any pose file.
  std::string zero_pose_name_ = "model default"; ///< Name of the loaded zero pose.
  std::string zero_pose_dir_model_; ///< Model folder, for the default pose file path.
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr load_zero_pose_sub_; ///< /load_zero_pose.
  std::unordered_map<std::string, double> sensor_weights_map; ///< Weights for each sensor in the IK solver.
  std::unordered_map<std::string, std::string> topic_translations_; ///< Sensor topic alias to OpenSim body name.
  std::mutex odom_mutex; ///< Mutex to protect access to odometry data and topic_stats_.

  // Timing statistics for /argus_debug
  struct TopicStats {
    std::string segment;
    std::chrono::steady_clock::time_point last{};
    std::deque<std::pair<std::chrono::steady_clock::time_point, double>> gaps; ///< (arrival, gap ms), last 20 s
  };
  std::unordered_map<std::string, TopicStats> topic_stats_; ///< Keyed by topic name.
  std::vector<std::string> debug_topic_order_; ///< Topics in parameter order.
  std::deque<std::pair<std::chrono::steady_clock::time_point, double>> solve_times_; ///< (finish, ms), last 1 s
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr debug_pub_; ///< Publishes /argus_debug.
  rclcpp::TimerBase::SharedPtr debug_timer_; ///< Drives publish_debug().

  // Optional drift correction (argus_drift node)
  std::unordered_map<std::string, SimTK::Rotation> drift_rotations_; ///< Per segment, in ground; empty = none.
  std::unordered_map<std::string, std::string> segment_module_; ///< Segment -> module (topic alias, e.g. argus1).
  std::unordered_map<std::string, std::string> module_segment_; ///< Module -> segment.
  builtin_interfaces::msg::Time zero_stamp_; ///< Time of the last zero; corrections must carry it.
  rclcpp::Publisher<tf2_msgs::msg::TFMessage>::SharedPtr drift_reference_pub_; ///< Latched zero reference.
  rclcpp::Subscription<tf2_msgs::msg::TFMessage>::SharedPtr drift_correction_sub_; ///< Corrections, if enabled.

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

  // Multi-threaded so odometry keeps landing while track() is busy.
  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(node);
  executor.spin();

  // Shutdown the ROS 2 client library.
  rclcpp::shutdown();
  return 0;
}
