// Copyright (c) 2021 by Ignacio Alzugaray <alzugaray dot ign at gmail dot com>
// ETH Zurich, Vision for Robotics Lab.

#include "haste/app_file.hpp"

using namespace haste::app;

// clang-format off
const auto kHelpMessage = "usage: ./tracking_app_file \n"
    "                           -events_file=path/to/file            # Plain text file with events. [Required]\n"
    "                           -seed=t,x,y,theta,id                 # Initial seed for tracker. [Required if -seeds_file is not set]\n"
    "                           -seeds_file=path/to/file             # Plain text file with several initial tracking seeds. [Required if -seed is not set]\n"
    "                           -num_events=n                        # Load only the first n-events in the event file.\n"
    "                           -tracker_type=correlation|haste_correlation|haste_correlation_star|haste_difference|haste_difference_star # Tracker type.\n"
    "                           -centered_initialization=false|true  # Force tracker to be centered/non-centered initialized (Check docs).\n"
    "                           -camera_params_file=path/to/file     # Load pinhole camera calibration model\n"
    "                           -camera_size=WIDTHxHEIGHT            # Set image sensor resolution.\n"
    "                           -visualize=true|false                # Visualize internal tracker state.\n"
    "                           -output_file=path/to/file            # Write tracking results to file.\n"
    "\n"
    "\n"
    "\n"
    "Detailed explanation:\n"
    "\n"
    "-events_file=path/to/file\n"
    "* Description: Specifies a path to a plain text file from which are loaded. Each line in file represents an event with format: t x y polarity. \n"
    "* Note: This flag must be specified.\n"
    "\n"
    "-tracker_type=correlation|haste_correlation|haste_difference|haste_difference_star\n"
    "* Description: Specifies the employed tracking algorithm. All the algorithms are described in [BMVC'20] except Correlation, described in [3DV'19].\n"
    "* Note: This flag must be specified. \n"
    "\n"
    "-seed=t,x,y,theta,id\n"
    "* Description: Specifies the seed from which a single tracker will be generated.\n"
    "* Note: Either this flag or -seeds_file must be specified. \n"
    "\n"
    "-seeds_file=path/to/file \n"
    "* Description: Specifies the path to a plain text file from which multiple tracking seeds are loaded. These seeds will generate multiple trackers that are processed sequentially. Each line in the file represents a tracking seed with format: t,x,y,theta,id. \n"
    "* Note: Either this flag or -seeds must be specified. \n"
    "\n"
    "-num_events=n\n"
    "* Description: Only the first n-events from the event file are loaded.\n"
    "* Default: inf\n"
    "\n"
    "-centered_initialization=false|true\n"
    "* Description: Specifies whether the tracker is centered initialized or not. Centered initialized trackers would attempt to be fully initialized as close in time as possible to the originating seed by exahustively searching backward and forward in the event-stream. Non-centered initialized trackers would only collect events after the originating seed, initializing the tracker arbitrarily later in time. \n"
    "\n"
    "-camera_params_file=path/to/file     \n"
    "* Description: Specifies the path to a plain text file from which the pinhole camera with rad-tan distorion model will be loaded. The file contains a single line with format: fx fy cx cy k1 k2 p1 p2 k3. Specifying this file, loaded events would be undistorted as pre-processing step.\n"
    "\n"
    "-camera_size=WIDTHxHEIGHT            \n"
    "* Description: Specifies image resolution.\n"
    "* Default: 240x180 (DAVIS240c resolution)\n"
    "\n"
    "-visualize=true|false                \n"
    "* Description: Visualize the internal states of the currently processed tracker. \n"
    "* Note: The resulting timing benchmark might be inaccurate due to the overhead.\n"
    "\n"
    "-output_file=path/to/file            \n"
    "* Description: Specifies the path to a plain text file where the tracking states will be recorded. Each line in the file respresents a tracking state with format: t,x,y,theta,id\n"
    "* Note: The resulting timing benchmark might be inaccurate due to the overhead.";
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

  // Load tracker seed(s).
  auto seeds = parser::parseTrackerSeeds();

  // Stopping condition: track until feature leave the FOV.
  constexpr auto kRemovalMargin = Tracker::kPatchSizeHalf;
  const auto stoppingCondition = [&](const Tracker& tracker) -> bool {
    return !((tracker.x() >= kRemovalMargin) && (tracker.y() >= kRemovalMargin)
             && ((tracker.x() + kRemovalMargin) < camera.width) && ((tracker.y() + kRemovalMargin) < camera.height));
  };
  if (FLAGS_centered_initialization) {
    LOG(INFO) << "Centered tracker initialization enforced (--centered_initialization=true). Tracker will initialize "
                 "as close as spatio-temporally possible to the originating seed.";
  } else {
    LOG(INFO) << "Centered tracker initialization is NOT enforced (--centered_initialization=false). Tracker will "
                 "initialize arbitrarily later after the originating seed.";
  }

  // Initialize benchmarking counters.
  benchmark::TimingBenchmark benchmark;
  // Set recording vector.
  std::vector<TrackerState> states_recorded;
  const bool is_recording = !google::GetCommandLineFlagInfoOrDie("output_file").is_default;

  // Tracking from seed(s).
  for (const auto& seed : seeds) {
    LOG(INFO) << "Starting Tracker #" << seed.id << " with seed "
              << "{t = " << seed.t << ", x = " << seed.x << ", y = " << seed.y << ", theta = " << seed.theta << "}.";

    // Initialize tracker.
    auto tracker = createTracker(FLAGS_tracker_type, seed);

    auto it = events.begin();
    if (!FLAGS_centered_initialization) {
      it = initializeTrackerCentered(events, *tracker);
    } else {
      it = initializeTrackerRegular(events, *tracker);
    }

    if (tracker->status() != Tracker::kUninitialized) {
      LOG(INFO) << "Tracker #" << seed.id << " is initialized with state {t = " << tracker->t()
                << ", x = " << tracker->x() << ", y = " << tracker->y() << ", theta = " << tracker->theta() << "}.";
    } else {
      LOG(INFO) << "Tracker #" << seed.id << " could not be initialized. Skipping to the next seed.";
      continue;
    }

    if (is_recording)  { appendTrackerState(seed.id, *tracker, states_recorded); }// Record first state
    benchmark::Stopwatch timer;

    // Process events starting just after initialization.
    for (auto it_end = events.end(); it != it_end; ++it) {
      const auto& event = *it;
      const auto& [et, ex, ey, ep] = event;

      timer.tic();
      const auto& update_type = tracker->pushEvent(et, ex, ey);
      auto t_elapsed = timer.toc();

      if (update_type == Tracker::EventUpdate::kRegularEvent) { benchmark.registerRegular(t_elapsed); }

      if (update_type == Tracker::EventUpdate::kStateEvent) {
        benchmark.registerState(t_elapsed);
        if (is_recording) { appendTrackerState(seed.id, *tracker, states_recorded); }
        if (stoppingCondition(*tracker)) { break; }
      }

      // Visualize internal state change after each event.
      if (FLAGS_visualize && update_type != Tracker::EventUpdate::kOutOfRange) {
        haste::ImshowEigenArrayNormalized(
            "Feature Event Window Projection",
            tracker->eventWindowToModel(tracker->event_window(), tracker->state()).transpose());
        haste::ImshowEigenArrayNormalized("Feature Template", tracker->tracker_template().transpose());
        cv::waitKey(1);
      }
    }
  }

  if (is_recording) { writeTrackerStates(states_recorded, FLAGS_output_file); }

  if (is_recording) {
    LOG(WARNING)
        << "** WARNING ** Intermediate tracking states have been recorded to file (--output_file=path/to/file). "
           "Following timing results might be inaccurate.";
  }
  if (FLAGS_visualize) {
    LOG(WARNING) << "** WARNING ** Internal tracking states have been visualized (--visualize=true). "
                    "Following timing results might be inaccurate.";
  }

  LOG(INFO) << benchmark.fullReport();
  return EXIT_SUCCESS;
}