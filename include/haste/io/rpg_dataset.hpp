// Copyright (c) 2021 by Ignacio Alzugaray <alzugaray dot ign at gmail dot com>
// ETH Zurich, Vision for Robotics Lab.

#pragma once

#include <fstream>
#include <vector>

namespace haste {
class RpgDataset {
 public:
  template<typename Event>
  static inline auto loadEvents(const std::string& file_path, std::vector<Event>& events,
                                size_t num_events = std::numeric_limits<size_t>::max()) -> bool;
  template<typename Camera>
  static inline auto loadCalibration(const std::string& file_path, Camera& camera) -> bool;

 protected:
  static auto countLinesInFile(std::ifstream& file) -> size_t;
};
}// namespace haste

#include "rpg_dataset_impl.hpp"
