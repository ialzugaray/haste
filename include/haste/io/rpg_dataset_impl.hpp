// Copyright (c) 2021 by Ignacio Alzugaray <alzugaray dot ign at gmail dot com>
// ETH Zurich, Vision for Robotics Lab.

#include <glog/logging.h>

namespace haste {
auto RpgDataset::countLinesInFile(std::ifstream& file) -> size_t{
  file.clear();
  file.seekg(0, std::ios::beg);  // Reset reading point
  size_t n_lines = 0;
  for (std::string line;!file.eof(); ++n_lines) {std::getline(file, line);}
  file.clear();
  file.seekg(0, std::ios::beg);  // Reset reading point
  return n_lines;
}

template<typename Event>
auto RpgDataset::loadEvents(const std::string& file_path, std::vector<Event>& events, size_t num_events) -> bool {
  LOG(INFO) << "Loading Events File (Format: t x y p) from file " << file_path << " ...";
  std::ifstream file(file_path);
  if (file.fail()) {
    LOG(ERROR) << "Error opening file. " << file_path;
    return false;
  }
  // Preallocation.
  num_events = std::min(countLinesInFile(file), num_events);
  events.clear();
  events.reserve(num_events);

  // Loop over lines parsing them as events.
  for (size_t i_event = 0; i_event < num_events && (!file.eof()); ++i_event) {
    // RPG event txt file: t: double, x: uint16_t, y: uint16_t, bool: p
    double t;
    uint16_t x, y;
    bool p;
    file >> t >> x >> y >> p;

    events.template emplace_back(Event{.t = t, .x = x, .y = y, .p = p});
  }

  file.close();
  LOG(INFO) << "Successfully loaded  " << events.size() << " events.";
  return true;
}

template<typename Camera>
auto RpgDataset::loadCalibration(const std::string& file_path, Camera& camera) -> bool {
  LOG(INFO) << "Loading camera calibration (Format: fx fy cx cy k1 k2 p1 p2 k3) from file " << file_path << " ...";
  std::ifstream file(file_path);
  if (file.fail()) {
    LOG(ERROR) << "Error opening file. ";
    return false;
  }

  double fx, fy, cx, cy;
  double k1, k2, p1, p2, k3;
  file >> fx >> fy >> cx >> cy >> k1 >> k2 >> p1 >> p2 >> k3;// Single line expected
  file.close();

  camera.fx = fx;
  camera.fy = fy;
  camera.cx = cx;
  camera.cy = cy;
  camera.k1 = k1;
  camera.k2 = k2;
  camera.p1 = p1;
  camera.p2 = p2;
  camera.k3 = k3;

  LOG(INFO) << "Successfully loaded camera calibration parameters.";
  return true;
}

}// namespace haste