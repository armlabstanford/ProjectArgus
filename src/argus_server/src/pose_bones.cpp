// Apply a JSON map of OpenSim coordinate values and print each body's pose in ground.
// Usage: pose_bones <model-folder> <angles.json> [--pose]
// angles.json is {"coordinate_name": radians, ...}. Names that are not in the model
// are reported on stderr and skipped. stdout is one JSON object. --pose adds each
// body's origin in ground.
//
// pose_bones --server <model-folder>
// Reads one JSON angle map per stdin line and writes one JSON pose object per line.
//
// Batch mode for a whole motion:
//   pose_bones --mot <model.osim | model-folder> <motion.mot> [every-nth-row] [body,body,...]
// Every coordinate column in the .mot (degrees when inDegrees=yes) is applied
// frame by frame. stdout has one line per frame and body:
//   time body qw qx qy qz        (body orientation in ground)
// --mot-pose adds the body origin's position in ground: time body qw qx qy qz px py pz
// Locked coordinates keep their model value. A nan coordinate makes the body
// it moves, and every body below it, "nan nan nan nan" in that frame.
// --mot-coupled does the same, but coordinates that a coordinate coupler drives
// are computed from their coupler instead of read from the .mot; stderr then
// gets how far the .mot values were from their couplers.

#include <OpenSim/OpenSim.h>

#include <algorithm>
#include <cctype>
#include <cmath>
#include <fstream>
#include <map>
#include <iostream>
#include <sstream>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

namespace {

void skip_ws(const std::string & text, size_t & i) {
  while (i < text.size() && std::isspace(static_cast<unsigned char>(text[i]))) ++i;
}

std::string parse_string(const std::string & text, size_t & i) {
  if (i >= text.size() || text[i] != '"') {
    throw std::runtime_error("expected a string in the angles file");
  }
  ++i;
  std::string out;
  while (i < text.size() && text[i] != '"') {
    if (text[i] == '\\' && i + 1 < text.size()) {
      ++i;
    }
    out.push_back(text[i]);
    ++i;
  }
  if (i >= text.size() || text[i] != '"') {
    throw std::runtime_error("unterminated string in the angles file");
  }
  ++i;
  return out;
}

double parse_number(const std::string & text, size_t & i) {
  skip_ws(text, i);
  const size_t start = i;
  if (i < text.size() && (text[i] == '-' || text[i] == '+')) ++i;
  while (i < text.size() && (std::isdigit(static_cast<unsigned char>(text[i])) || text[i] == '.' ||
                             text[i] == 'e' || text[i] == 'E' || text[i] == '+' || text[i] == '-')) {
    ++i;
  }
  if (start == i) throw std::runtime_error("expected a number in the angles file");
  return std::stod(text.substr(start, i - start));
}

std::vector<std::pair<std::string, double>> parse_angles(const std::string & text) {
  size_t i = 0;
  skip_ws(text, i);
  if (i >= text.size() || text[i] != '{') {
    throw std::runtime_error("angles file must be a JSON object");
  }
  ++i;
  std::vector<std::pair<std::string, double>> angles;
  skip_ws(text, i);
  if (i < text.size() && text[i] == '}') return angles;
  while (i < text.size()) {
    skip_ws(text, i);
    const std::string name = parse_string(text, i);
    skip_ws(text, i);
    if (i >= text.size() || text[i] != ':') throw std::runtime_error("expected ':' after " + name);
    ++i;
    angles.emplace_back(name, parse_number(text, i));
    skip_ws(text, i);
    if (i < text.size() && text[i] == ',') {
      ++i;
      continue;
    }
    if (i < text.size() && text[i] == '}') return angles;
    throw std::runtime_error("expected ',' or '}' in the angles file");
  }
  throw std::runtime_error("angles file ended inside the object");
}

std::string json_escape(const std::string & s) {
  std::string out;
  for (char c : s) {
    if (c == '\\' || c == '"') out.push_back('\\');
    out.push_back(c);
  }
  return out;
}

std::string read_file(const std::string & path) {
  std::ifstream in(path);
  if (!in) throw std::runtime_error("could not read " + path);
  std::ostringstream ss;
  ss << in.rdbuf();
  return ss.str();
}

std::vector<std::string> split(const std::string & text, char sep) {
  std::vector<std::string> parts;
  std::string part;
  std::istringstream in(text);
  while (std::getline(in, part, sep)) {
    if (!part.empty() && part.back() == '\r') part.pop_back();
    parts.push_back(part);
  }
  return parts;
}

int run_motion(int argc, char ** argv, bool coupled, bool with_position = false) {
  std::string model_path = argv[2];
  if (model_path.size() < 5 || model_path.substr(model_path.size() - 5) != ".osim") {
    model_path += "/model.osim";
  }
  const int every = argc > 4 ? std::max(1, std::stoi(argv[4])) : 1;
  std::vector<std::string> wanted = argc > 5 ? split(argv[5], ',') : std::vector<std::string>{};

  std::ifstream in(argv[3]);
  if (!in) throw std::runtime_error(std::string("could not read ") + argv[3]);
  std::string line;
  bool in_degrees = false;
  while (std::getline(in, line)) {
    if (line.find("inDegrees=yes") != std::string::npos) in_degrees = true;
    if (line.rfind("endheader", 0) == 0) break;
  }
  if (!std::getline(in, line)) throw std::runtime_error("no column names after endheader");
  const auto columns = split(line, '\t');

  OpenSim::Logger::setLevelString("error");
  OpenSim::Model model(model_path);
  SimTK::State & state = model.initSystem();
  OpenSim::CoordinateSet & coords = model.updCoordinateSet();
  std::vector<std::pair<size_t, OpenSim::Coordinate *>> applied;
  for (size_t c = 1; c < columns.size(); ++c) {
    if (!coords.contains(columns[c])) continue;
    OpenSim::Coordinate & coord = coords.get(columns[c]);
    if (coord.getLocked(state)) continue;
    applied.emplace_back(c, &coord);
  }
  if (applied.empty()) throw std::runtime_error("no .mot column matches an unlocked model coordinate");

  struct Coupler {
    const OpenSim::CoordinateCouplerConstraint * constraint;
    std::vector<OpenSim::Coordinate *> independent;
    OpenSim::Coordinate * dependent;
    int column = -1;
    double sum_sq = 0.0, worst = 0.0;
    long count = 0;
  };
  std::vector<Coupler> couplers;
  if (coupled) {
    const OpenSim::ConstraintSet & constraints = model.getConstraintSet();
    for (int i = 0; i < constraints.getSize(); ++i) {
      const auto * cc = dynamic_cast<const OpenSim::CoordinateCouplerConstraint *>(&constraints.get(i));
      if (!cc || !cc->isEnforced(state)) continue;
      Coupler c{cc, {}, &coords.get(cc->getDependentCoordinateName())};
      const auto & names = cc->getIndependentCoordinateNames();
      for (int k = 0; k < names.size(); ++k) c.independent.push_back(&coords.get(names[k]));
      for (const auto & [col, coord] : applied) {
        if (coord == c.dependent) c.column = static_cast<int>(col);
      }
      couplers.push_back(c);
    }
    applied.erase(std::remove_if(applied.begin(), applied.end(), [&](const auto & a) {
      return std::any_of(couplers.begin(), couplers.end(), [&](const Coupler & c) { return c.dependent == a.second; });
    }), applied.end());
  }

  const OpenSim::BodySet & bodies = model.getBodySet();
  std::vector<const OpenSim::Body *> out_bodies;
  if (wanted.empty()) {
    for (int i = 0; i < bodies.getSize(); ++i) out_bodies.push_back(&bodies.get(i));
  } else {
    for (const auto & name : wanted) {
      if (!bodies.contains(name)) throw std::runtime_error("no body named " + name);
      out_bodies.push_back(&bodies.get(name));
    }
  }

  // A nan coordinate leaves the body its joint moves, and everything below it, unknown.
  std::map<std::string, std::string> parent_of;
  const OpenSim::JointSet & joints = model.getJointSet();
  for (int j = 0; j < joints.getSize(); ++j) {
    parent_of[joints.get(j).getChildFrame().findBaseFrame().getName()] =
        joints.get(j).getParentFrame().findBaseFrame().getName();
  }
  auto moved_body = [](const OpenSim::Coordinate * coord) {
    return coord->getJoint().getChildFrame().findBaseFrame().getName();
  };
  std::vector<std::vector<std::string>> lineage;
  for (const OpenSim::Body * body : out_bodies) {
    std::vector<std::string> chain{body->getName()};
    while (parent_of.count(chain.back()) && chain.size() < 64) chain.push_back(parent_of[chain.back()]);
    lineage.push_back(chain);
  }

  std::cout.precision(9);
  long row = -1;
  while (std::getline(in, line)) {
    if (line.empty()) continue;
    if (++row % every != 0) continue;
    const auto cells = split(line, '\t');
    if (cells.size() < columns.size()) continue;
    std::vector<double> values(cells.size());
    for (size_t c = 0; c < cells.size(); ++c) values[c] = std::stod(cells[c]);
    if (!std::isfinite(values[0])) continue;
    std::vector<std::string> unknown;
    for (const auto & [c, coord] : applied) {
      double v = values[c];
      if (!std::isfinite(v)) {
        unknown.push_back(moved_body(coord));
        continue;
      }
      if (in_degrees && coord->getMotionType() != OpenSim::Coordinate::Translational) {
        v *= SimTK::Pi / 180.0;
      }
      coord->setValue(state, v, false);
    }
    for (Coupler & c : couplers) {
      bool known = true;
      for (const auto * ind : c.independent) {
        for (const auto & [col, coord] : applied) {
          if (coord == ind && !std::isfinite(values[col])) known = false;
        }
      }
      if (!known) {
        unknown.push_back(moved_body(c.dependent));
        continue;
      }
      SimTK::Vector x(static_cast<int>(c.independent.size()));
      for (size_t k = 0; k < c.independent.size(); ++k) x[static_cast<int>(k)] = c.independent[k]->getValue(state);
      const double v = c.constraint->getFunction().calcValue(x) *
                       c.constraint->getPropertyByName("scale_factor").getValue<double>();
      c.dependent->setValue(state, v, false);
      if (c.column >= 0 && std::isfinite(values[c.column])) {
        double from_mot = values[c.column];
        if (in_degrees && c.dependent->getMotionType() != OpenSim::Coordinate::Translational) {
          from_mot *= SimTK::Pi / 180.0;
        }
        const double d = (from_mot - v) * 180.0 / SimTK::Pi;
        c.sum_sq += d * d;
        c.worst = std::max(c.worst, std::abs(d));
        ++c.count;
      }
    }
    model.realizePosition(state);
    for (size_t b = 0; b < out_bodies.size(); ++b) {
      const bool is_unknown = std::any_of(lineage[b].begin(), lineage[b].end(), [&](const std::string & name) {
        return std::find(unknown.begin(), unknown.end(), name) != unknown.end();
      });
      std::cout << values[0] << ' ' << out_bodies[b]->getName() << ' ';
      if (is_unknown) {
        std::cout << (with_position ? "nan nan nan nan nan nan nan\n" : "nan nan nan nan\n");
        continue;
      }
      const SimTK::Transform pose = out_bodies[b]->getTransformInGround(state);
      const SimTK::Quaternion q(pose.R());
      std::cout << q[0] << ' ' << q[1] << ' ' << q[2] << ' ' << q[3];
      if (with_position) {
        const SimTK::Vec3 & p = pose.p();
        std::cout << ' ' << p[0] << ' ' << p[1] << ' ' << p[2];
      }
      std::cout << '\n';
    }
  }
  for (const Coupler & c : couplers) {
    if (c.count == 0) continue;
    std::cerr << c.dependent->getName() << " .mot minus coupler: rms " << std::sqrt(c.sum_sq / c.count)
              << " deg, worst " << c.worst << " deg\n";
  }
  return 0;
}

}  // namespace

void write_bones_json(const OpenSim::Model & model, const SimTK::State & state, bool with_pose) {
  const OpenSim::BodySet & bodies = model.getBodySet();
  std::cout << "{\"bones\":{";
  for (int i = 0; i < bodies.getSize(); ++i) {
    const OpenSim::Body & body = bodies.get(i);
    const SimTK::Transform & X = body.getTransformInGround(state);
    const SimTK::Quaternion q(X.R());
    if (i) std::cout << ',';
    std::cout << '"' << json_escape(body.getName()) << "\":{"
              << "\"orientation_wxyz\":[" << q[0] << ',' << q[1] << ',' << q[2] << ',' << q[3] << "]";
    if (with_pose) {
      const SimTK::Vec3 & p = X.p();
      std::cout << ",\"position\":[" << p[0] << ',' << p[1] << ',' << p[2] << "]";
    }
    std::cout << "}";
  }
  std::cout << "}}\n" << std::flush;
}

int run_server(const std::string & model_folder) {
  std::string model_path = model_folder;
  if (model_path.size() < 5 || model_path.substr(model_path.size() - 5) != ".osim") {
    model_path += "/model.osim";
  }
  OpenSim::Logger::setLevelString("error");
  OpenSim::Model model(model_path);
  SimTK::State & state = model.initSystem();
  OpenSim::CoordinateSet & coords = model.updCoordinateSet();
  std::string line;
  while (std::getline(std::cin, line)) {
    if (line.empty()) continue;
    for (const auto & entry : parse_angles(line)) {
      if (!coords.contains(entry.first)) continue;
      try {
        coords.get(entry.first).setValue(state, entry.second, false);
      } catch (const std::exception & ex) {
        std::cerr << "could not set " << entry.first << ": " << ex.what() << "\n";
      }
    }
    model.realizePosition(state);
    write_bones_json(model, state, true);
  }
  return 0;
}

int main(int argc, char ** argv) {
  if (argc >= 3 && std::string(argv[1]) == "--server") {
    try {
      return run_server(argv[2]);
    } catch (const std::exception & ex) {
      std::cerr << ex.what() << '\n';
      return 1;
    }
  }
  if (argc >= 4 && (std::string(argv[1]) == "--mot" || std::string(argv[1]) == "--mot-coupled" ||
                    std::string(argv[1]) == "--mot-pose")) {
    try {
      return run_motion(argc, argv, std::string(argv[1]) == "--mot-coupled", std::string(argv[1]) == "--mot-pose");
    } catch (const std::exception & ex) {
      std::cerr << ex.what() << '\n';
      return 1;
    }
  }
  if (argc != 3 && !(argc == 4 && std::string(argv[3]) == "--pose")) {
    std::cerr << "Usage: pose_bones <model-folder> <angles.json> [--pose]\n"
                 "       pose_bones --server <model-folder>\n"
                 "       pose_bones --mot <model.osim|model-folder> <motion.mot> [every] [body,...]\n";
    return 2;
  }
  try {
    const std::string model_path = std::string(argv[1]) + "/model.osim";
    const auto angles = parse_angles(read_file(argv[2]));

    // Model loading writes status lines to stdout. The JSON result has to be the only stdout.
    OpenSim::Logger::setLevelString("error");
    OpenSim::Model model(model_path);
    SimTK::State & state = model.initSystem();
    OpenSim::CoordinateSet & coords = model.updCoordinateSet();

    for (const auto & entry : angles) {
      if (!coords.contains(entry.first)) {
        std::cerr << "unknown coordinate: " << entry.first << "\n";
        continue;
      }
      try {
        coords.get(entry.first).setValue(state, entry.second, false);
      } catch (const std::exception & ex) {
        std::cerr << "could not set " << entry.first << ": " << ex.what() << "\n";
      }
    }
    model.realizePosition(state);
    write_bones_json(model, state, argc > 3 && std::string(argv[3]) == "--pose");
    return 0;
  } catch (const std::exception & ex) {
    std::cerr << ex.what() << '\n';
    return 1;
  }
}
