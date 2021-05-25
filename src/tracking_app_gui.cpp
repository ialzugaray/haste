// Copyright (c) 2021 by Ignacio Alzugaray <alzugaray dot ign at gmail dot com>
// ETH Zurich, Vision for Robotics Lab.

#include "haste/app_gui.hpp"

using namespace haste::app;

// clang-format off
const auto kHelpMessage = "usage: ./tracking_app_gui \n"
                          "                           -events_file=path/to/file            # Plain text file with events. [Required]\n"
                          "                           -num_events=n                        # Load only the first n-events in the event file.\n"
                          "                           -camera_params_file=path/to/file     # Load pinhole camera calibration model\n"
                          "                           -camera_size=WIDTHxHEIGHT            # Set image sensor resolution.\n"
                          "\n"
                          "\n"
                          "\n"
                          "Detailed explanation:\n"
                          "\n"
                          "-events_file=path/to/file\n"
                          "* Description: Specifies a path to a plain text file from which are loaded. Each line in file represents an event with format: t x y polarity. \n"
                          "* Note: This flag must be specified.\n"
                          "\n"
                          "-num_events=n\n"
                          "* Description: Only the first n-events from the event file are loaded.\n"
                          "* Default: inf\n"
                          "\n"
                          "-camera_params_file=path/to/file     \n"
                          "* Description: Specifies the path to a plain text file from which the pinhole camera with rad-tan distortion model will be loaded. The file contains a single line with format: fx fy cx cy k1 k2 p1 p2 k3. Specifying this file, loaded events would be undistorted as pre-processing step.\n"
                          "\n"
                          "-camera_size=WIDTHxHEIGHT            \n"
                          "* Description: Specifies image resolution.\n"
                          "* Default: 240x180 (DAVIS240c resolution)\n";
// clang-format on

int main(int argc, char** argv) {
  google::SetVersionString("1.0.0");
  google::InitGoogleLogging(argv[0]);
  FLAGS_alsologtostderr = true;

  google::ParseCommandLineFlags(&argc, &argv, false);
  if (FLAGS_usage || argc == 1) {
    LOG(INFO) << kHelpMessage;
    return EXIT_SUCCESS;
  }
  LOG(INFO) << parser::kWelcomeMessage;

  // Events
  auto events = parser::parseEvents();
  CHECK(!events.empty()) << "No events are loaded.";

  // Camera
  auto camera = parser::parseCamera();
  if (parser::parseCameraCalibration(camera)) {
    // Preprocessing: undistort events' location.
    LOG(INFO) << "Applying undistortion to events as a preprocessing step.";
    auto undistortion_map =
        camera.createUndistortionMap();// This undistort mapping could alternatively be used during an online process
    std::for_each(events.begin(), events.end(), [&undistortion_map](Event& event) {
      std::tie(event.x, event.y) = undistortion_map(event.x, event.y);
    });
  }

  gui::HasteGui gui_app(camera.width, camera.height, &events);
  gui_app.start();

  return EXIT_SUCCESS;
}
