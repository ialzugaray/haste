// Copyright (c) 2021 by Ignacio Alzugaray <alzugaray dot ign at gmail dot com>
// ETH Zurich, Vision for Robotics Lab.

#pragma once
#include <gflags/gflags.h>

#include "haste/app/tracking.hpp"

// Define GFLAGS.
// Experiment Setup.
DEFINE_string(events_file, "path/to/file",
              "File of events. Multiline file (one line per event) with format: t x y p. Event time \"t\" specified as "
              "a floating point.");

DEFINE_uint64(num_events, 0, "Number of events loaded from file. ");

DEFINE_string(camera_size, "240x180", "Camera sensor size. Format: WIDTHxHEIGHT.");

DEFINE_string(
    camera_params_file, "path/to/file",
    "File with camera parameters and rad-tan distortion. Single-line file with format: fx fy cx cy k1 k2 p1 p2 k3.");

// Tracking Setup.
DEFINE_string(tracker_type, "unspecified_tracker",
              "Type of tracker: correlation, haste_correlation, haste_correlation_star, haste_difference, "
              "haste_difference_star");// TODO: default , haste_difference_star. Maybe default to not valid.

DEFINE_bool(centered_initialization, true, "Force tracker to be initialized at specified seed (See documentation).");

DEFINE_string(seed, "5.0,120,90,0", "Initial seed of an individual tracker with format: t,x,y,theta,id");

DEFINE_string(seeds_file, "path/to/file",
              "File of tracker seeds. Multiline file (one line per tracker seed) with format: t,x,y,theta,id");

// Results Setup.
DEFINE_bool(visualize, true, "Generate simple views of the tracker's internal states.");
DEFINE_string(output_file, "path/to/file", "Dump tracking states into a specified files.");

// Help Setup.
DEFINE_bool(usage, false, "Show usage.");

namespace haste {
namespace app {
namespace parser {

const auto kWelcomeMessage =
    "\n"
    "**********************************************************************************************\n"
    "Thanks for using our code. Please cite our works Alzugaray & Chli [3DV'19] and [BMVC'20].\n"
    "Check for updates and documentation in the repo: https://github.com/ialzugaray/haste\n"
    "ETH Zurich. Vision For Robotics Lab (V4RL). MIT License, 2021.\n"
    "Contact: Ignacio Alzugaray <ialzugaray @ mavt . ethz . ch> OR <alzugaray . ign @ gmail . com>\n"
    "**********************************************************************************************\n\n";

/// Parse command line and load events from specified file.
auto parseEvents() -> std::vector<Event> {
  std::vector<Event> events;
  if (!google::GetCommandLineFlagInfoOrDie("events_file").is_default) {
    if (!google::GetCommandLineFlagInfoOrDie("num_events").is_default) {
      haste::RpgDataset::loadEvents(FLAGS_events_file, events, FLAGS_num_events);
    } else {
      haste::RpgDataset::loadEvents(FLAGS_events_file, events);
    }
  } else {
    LOG(ERROR) << "Required event file path is not specified (--events_file=path/to/file).";
  }
  return events;
}

/// Parse command line and create camera with specified resolution.
auto parseCamera() -> Camera {
  auto camera_size_str = splitString(FLAGS_camera_size, 'x');
  if (!google::GetCommandLineFlagInfoOrDie("camera_size").is_default) {
    LOG(WARNING) << "Camera size is not specified (--camera_size=WIDTHxHEIGHT). Using default values "
                 << FLAGS_camera_size << ".";
  }
  if (camera_size_str.size() != 2) {
    LOG(FATAL) << "Camera size is specified, but incorrectly formatted. Please use format WIDTHxHEIGHT.";
  }
  size_t camera_width = std::stoul(camera_size_str[0]), camera_height = std::stoul(camera_size_str[1]);
  return {.width = camera_width, .height = camera_height};
}

/// Parse command line and load distorted pinhole camera parameters from specified file.
auto parseCameraCalibration(Camera& camera) -> bool {
  // Load Calibration.
  if (!google::GetCommandLineFlagInfoOrDie("camera_params_file").is_default) {
    haste::RpgDataset::loadCalibration(FLAGS_camera_params_file, camera);
    return true;
  } else {
    LOG(INFO) << "Camera parameters file path is not specified (--camera_params_file=path/to/file) and thus no "
                 "undistortion is applied."
              << std::endl;
    return false;
  }
}

/// Parse command line and generate tracker seed(s) from file or directly from command line.
auto parseTrackerSeeds() -> std::vector<TrackerState> {
  std::vector<TrackerState> seeds;
  if (!((google::GetCommandLineFlagInfoOrDie("seed").is_default)
        ^ (google::GetCommandLineFlagInfoOrDie("seeds_file").is_default))) {// Check incompatibilities.
    LOG(FATAL) << "You must define either a single tracker seed (--seed=t,x,y,theta) or a file of tracker seeds "
                  "(--seeds_file=path/to/file).";
  } else if (!google::GetCommandLineFlagInfoOrDie("seed").is_default) {// Single seed.
    seeds.push_back(getTrackerStateFromString(FLAGS_seed));
    LOG(INFO) << "Single seed loaded from command line --seed=" << FLAGS_seed;
  } else if (!google::GetCommandLineFlagInfoOrDie("seeds_file").is_default) {// Multiple seeds from file.
    seeds = getTrackerStatesFromFile(FLAGS_seeds_file);
  }
  return seeds;
}

}// namespace parser
}// namespace app
}// namespace haste