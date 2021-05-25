// Copyright (c) 2021 by Ignacio Alzugaray <alzugaray dot ign at gmail dot com>
// ETH Zurich, Vision for Robotics Lab.

#include "haste/tracking.hpp"

namespace haste {
namespace app {
namespace gui {

HasteGui::HasteGui(const Scalar& camera_width, const Scalar& camera_height, const Events* events_ptr)
    : quickgui::Gui(),
      camera_width_{camera_width},
      camera_height_{camera_height},
      events_ptr_(events_ptr) {
  CHECK(events_ptr_);
  CHECK(!events_ptr->empty());
  resetCamera();
}

auto HasteGui::drawEvents(const EventsConstIterator& it_begin, const EventsConstIterator& it_end) -> void {
  glBegin(GL_POINTS);
  for (auto it = it_begin; it != it_end; it++) {
    const auto& event = *it;
    glVertex3f(event.x, event.y, event.t);
  }
  glEnd();
}

auto HasteGui::drawSeedWithCrosshair(const Vector3& seed) -> void {

  glPushMatrix();
  glTranslatef(0, 0, seed.z());
  glBegin(GL_LINES);
  glVertex2f(seed.x(), 0);
  glVertex2f(seed.x(), camera_height_ - 1);
  glVertex2f(0, seed.y());
  glVertex2f(camera_width_ - 1, seed.y());
  glEnd();

  constexpr auto kSeedSize = 5;
  glPointSize(kSeedSize);
  glBegin(GL_POINTS);
  glVertex2f(seed.x(), seed.y());
  glEnd();
  glPointSize(1.0);

  DrawPlanarRectangle({camera_width_, camera_height_, 0.0}, {camera_width_ / 2.0, camera_height_ / 2.0, 0});
  glPopMatrix();
}

auto HasteGui::resetCamera() -> void {
  camera_pos_ = Vector3{-0.75, 0.50, 0.75};
  camera_quat_ = lookAt(camera_pos_, Vector3::Zero(), Vector3::UnitY());
}

auto HasteGui::run_() -> void {
  glEnable(GL_DEPTH_TEST);

  ImVec4 clear_color = ImVec4(1.00f, 1.00f, 1.00f, 1.00f);

  // Visualization Options
  float time_scale = 1.0f;
  float time_window = 0.15f;
  int step_per_render = 10;
  float time_per_render = 1;
  bool play_flag = true;

  const char* kPlayCycleLabel[] = {"Time Per Cycle", "Events Per Cycle"};
  enum PlayCycleType : int { kTimeCycle, kEventCycle } play_cycle_type;
  play_cycle_type = kTimeCycle;

  // Data variables - Event stream
  const auto events_begin = events_ptr_->begin();
  const auto events_end = events_ptr_->end();

  auto window_begin = events_begin;
  auto window_end = window_begin;

  const auto time_origin = events_begin->t;
  auto current_time = time_origin;
  constexpr auto lower_bound_events_pred = [](const Event& event, const Scalar& time) { return event.t < time; };

  // Data from tracking
  constexpr auto kRemovalMargin = Tracker::kPatchSizeHalf;
  const auto stoppingCondition = [&](const Tracker& tracker) -> bool {
    return !((tracker.x() >= kRemovalMargin) && (tracker.y() >= kRemovalMargin)
             && ((tracker.x() + kRemovalMargin) < camera_width_) && ((tracker.y() + kRemovalMargin) < camera_height_));
  };

  std::vector<Vector3, Eigen::aligned_allocator<Vector3>> states;
  Vector3 seed_selector = {camera_width_ / 2.0, camera_height_ / 2.0, current_time};
  Vector3 initial_seed = seed_selector;

  TrackerPtr tracker = nullptr;
  constexpr auto kPatchWindowName = "Feature Event Window Projection";
  constexpr auto kTemplateWindowName = "Feature Template";
  std::vector<Event> initial_event_window;

  const char* kTrackerTypeLabel[] = {"Correlation", "HasteCorrelation", "HasteCorrelationStar", "HasteDifference",
                                     "HasteDifferenceStar"};
  enum TrackerType : int {
    kCorrelation,
    kHasteCorrelation,
    kHasteCorrelationStar,
    kHasteDifference,
    kHasteDifferenceStar
  } tracker_type;

  bool view_current_event_window_flag = true;
  bool view_initial_event_window_flag = true;
  bool view_track_projection = true;
  bool initial_popup = true;

  // Main loop
  while (!glfwWindowShouldClose(window_)) {
    glfwPollEvents();
    int display_w, display_h;
    glfwGetFramebufferSize(window_, &display_w, &display_h);
    glViewport(0, 0, display_w, display_h);

    glClearColor(clear_color.x, clear_color.y, clear_color.z, clear_color.w);
    glClear(GL_DEPTH_BUFFER_BIT | GL_COLOR_BUFFER_BIT);

    // start the Dear ImGui frame
    ImGui_ImplOpenGL3_NewFrame();
    ImGui_ImplGlfw_NewFrame();
    ImGui::NewFrame();

    // Navigation information & welcome info popup.
    if (initial_popup) { ImGui::OpenPopup("Initial Popup"); }
    if (ImGui::BeginPopup("Initial Popup", ImGuiWindowFlags_::ImGuiWindowFlags_AlwaysAutoResize)) {
      ImGui::Text(
          "Welcome to HASTE Gui!\n\n"
          "This is a minimal GUI playground to visualize the algorithms described in Alzugaray & Chli [3DV'19] and "
          "[BMVC'20].\nPlease cite both works if you use this code base. \n\n"
          "Keyboard shortcuts:\n"
          "\t WASDQE: translation.\n"
          "\t Left-click & Drag: rotation.\n"
          "\t R: Reset camera.\n"
          "\t Space: Pause/Unpause event stream.");

      if (ImGui::Button("Begin")) {
        initial_popup = false;
        ImGui::CloseCurrentPopup();
      }
      ImGui::EndPopup();
    }
    // Pull next events and window
    if (play_flag) {
      auto previous_window_end = window_end;
      if (play_cycle_type == kEventCycle) {
        window_end = std::next(window_end, std::min(std::distance(window_end, events_end), (long) step_per_render));
        current_time = std::prev(window_end)->t;
      } else if (play_cycle_type == kTimeCycle) {
        auto new_current_time = current_time + 1e-3 * time_per_render;
        window_end =
            std::find_if(window_end, events_end, [=](const auto& event) { return event.t > new_current_time; });
        current_time = new_current_time;
      }

      // Feed events to the tracker
      if (tracker) {
        for (auto it = previous_window_end; it != window_end; it++) {
          const auto& event = *it;
          const auto& update_type = tracker->pushEvent(event.t, event.x, event.y);

          if (update_type == Tracker::kInitializingEvent) {
            LOG(INFO) << "Registered event in initial window at time t=" << event.t << "(  " << tracker->event_counter()
                      << " / " << tracker->kEventWindowSize << " events).";
            initial_event_window.push_back(event);
          }

          if (update_type == Tracker::EventUpdate::kStateEvent) {
            LOG(INFO) << "Tracker state updated to: {t=" << tracker->t() << ",\t x=" << tracker->x()
                      << ",\t y=" << tracker->y() << ",\t theta=" << tracker->theta() << "}";
            states.emplace_back(tracker->x(), tracker->y(), tracker->t());
            seed_selector.x() = tracker->x();
            seed_selector.y() = tracker->y();
          }

          if (stoppingCondition(*tracker)) {
            tracker = nullptr;
            play_flag = false;
            break;
          }
        }
      }
      if (tracker) {
        // View tracker inner data
        ImshowEigenArrayNormalized(kPatchWindowName,
                                   tracker->eventWindowToModel(tracker->event_window(), tracker->state()).transpose());
        ImshowEigenArrayNormalized(kTemplateWindowName, tracker->tracker_template().transpose());
        cv::waitKey(1);
      }

      // Find the last event in the window. (Search in the stream).
      window_begin = std::lower_bound(window_begin, window_end, current_time - time_window,
                                      lower_bound_events_pred);// TODO use linear search instead.

      // End of the stream is reached.
      if (window_end == events_end) {
        // Regenerate event-stream slice.
        window_begin = events_begin;
        window_end = events_begin;
        current_time = time_origin;
        // Tracker reset.
        tracker = nullptr;
        states.clear();
        initial_event_window.clear();
        cv::destroyWindow(kPatchWindowName);
        cv::destroyWindow(kTemplateWindowName);
      }
    }

    // Advance seed selector with event slice front.
    seed_selector.z() = current_time;

    // Main gui control window.
    ImGui::SetNextTreeNodeOpen(true);
    if (ImGui::Begin("GUI Control")) {
      ImGui::Checkbox("Play", &play_flag);

      if (ImGui::CollapsingHeader("Play Options")) {
        ImGui::SliderFloat("Time Scale", &time_scale, 0.01, 10.0);
        ImGui::SliderFloat("Time Window", &time_window, 0.01, 10);

        static int play_cycle_type_selection = 0;
        ImGui::Combo("Render Update", &play_cycle_type_selection, kPlayCycleLabel, IM_ARRAYSIZE(kPlayCycleLabel));
        play_cycle_type = static_cast<PlayCycleType>(play_cycle_type_selection);
        switch (play_cycle_type) {
          case kEventCycle:
            ImGui::SliderInt("Events per cycle", &step_per_render, 1, 10000, "%d ev/cycle",
                             ImGuiSliderFlags_Logarithmic);
            break;
          case kTimeCycle:
            ImGui::SliderFloat("Time per cycle", &time_per_render, 0.01, 100, "%.2f ms/cycle",
                               ImGuiSliderFlags_Logarithmic);
            break;
          default: LOG(WARNING) << "Unknown cycle type: " << play_cycle_type;
        }
      }

      ImGui::SetNextTreeNodeOpen(true);
      if (ImGui::CollapsingHeader("Tracking Options")) {
        ImGui::Checkbox("View Initializing Event Window", &view_initial_event_window_flag);
        ImGui::Checkbox("View Current Event Window", &view_current_event_window_flag);
        ImGui::Checkbox("View Track Projection In Plane", &view_track_projection);

        static int tracker_type_selection = 4;
        ImGui::Combo("Tracker Type", &tracker_type_selection, kTrackerTypeLabel, IM_ARRAYSIZE(kTrackerTypeLabel));
        tracker_type = static_cast<TrackerType>(tracker_type_selection);

        ImGui::Text("Click and drag point to modify the initial tracker seed_selector.");
        ImPlot::SetNextPlotLimits(0, camera_width_ - 1, 0, camera_height_ - 1);
        if (ImPlot::BeginPlot("##SeedSelector", 0, 0,
                              ImVec2(-1, ImGui::GetColumnWidth() * (float) camera_height_ / (float) camera_width_),
                              ImPlotFlags_CanvasOnly, ImPlotAxisFlags_Lock, ImPlotAxisFlags_Lock)) {
          ImPlot::DragPoint("seed_selector", &seed_selector.x(), &seed_selector.y(), true, ImVec4(1.0f, 0.0f, 0, 1));
          ImPlot::EndPlot();
        }

        if (ImGui::Button("Track")) {
          tracker = nullptr;
          if (tracker_type == kCorrelation) {
            tracker = std::make_shared<CorrelationTracker>(seed_selector.z(), seed_selector.x(), seed_selector.y(), 0);
          } else if (tracker_type == kHasteCorrelation) {
            tracker =
                std::make_shared<HasteCorrelationTracker>(seed_selector.z(), seed_selector.x(), seed_selector.y(), 0);
          } else if (tracker_type == kHasteCorrelationStar) {
            tracker = std::make_shared<HasteCorrelationStarTracker>(seed_selector.z(), seed_selector.x(),
                                                                    seed_selector.y(), 0);
          } else if (tracker_type == kHasteDifference) {
            tracker =
                std::make_shared<HasteDifferenceTracker>(seed_selector.z(), seed_selector.x(), seed_selector.y(), 0);
          } else if (tracker_type == kHasteDifferenceStar) {
            tracker = std::make_shared<HasteDifferenceStarTracker>(seed_selector.z(), seed_selector.x(),
                                                                   seed_selector.y(), 0);
          } else {
            LOG(ERROR) << "Unknown tracker type selected.";
          }

          initial_seed = seed_selector;
          states.clear();
          initial_event_window.clear();
          play_flag = true;
        }
      }
    }

    ImGui::End();
    // Rendering gui.
    ImGui::Render();

    // Handle camera via keyboard / mouse.
    handleDeviceInput_();
    if (ImGui::IsKeyPressed(GLFW_KEY_SPACE)) { play_flag = !play_flag; }

    loadProjection_();

    glMatrixMode(GL_MODELVIEW);
    glLoadIdentity();

    glPushMatrix();
    glScalef(1.0 / camera_width_, 1.0 / camera_width_, time_scale);
    glTranslatef(-camera_width_ / 2.0, -camera_height_ / 2.0, 0.0);
    glTranslatef(0.0, 0.0, -current_time);

    // Draw event stream slice.
    glColor3fv(kColorBlack);
    drawEvents(window_begin, window_end);

    // Draw image border.
    glColor3fv(kColorBlack);
    DrawPlanarRectangle({camera_width_, camera_height_, 0.0},
                        {camera_width_ / 2.0, camera_height_ / 2.0, current_time});
    // Draw margin
    DrawPlanarRectangle({camera_width_ - 2 * kRemovalMargin, camera_height_ - 2 * kRemovalMargin, 0.0},
                        {camera_width_ / 2.0, camera_height_ / 2.0, current_time});

    if (view_initial_event_window_flag && !initial_event_window.empty()) {
      // Draw event initial event window
      glColor3fv(kColorBlue);
      glPointSize(3.0);
      drawEvents(initial_event_window.begin(), initial_event_window.end());

      // Draw initial seed.
      glColor3fv(kColorGreen);
      glBegin(GL_POINTS);
      glPointSize(5.0);
      glVertex3f(initial_seed.x(), initial_seed.y(), initial_seed.z());
      glEnd();
      glPointSize(1.0);
    }

    if (view_current_event_window_flag) {
      if (tracker) {
        // Draw tracker event window.
        glColor3fv(kColorMagenta);
        glPointSize(3.0);
        glBegin(GL_POINTS);
        for (size_t i = 0; i < Tracker::EventWindow::kSize; ++i) {
          auto [et, ex, ey] = tracker->event_window().getEvent(i);
          glVertex3f(ex, ey, et);
        }
        glPointSize(1.0);
        glEnd();
        // Draw tracker range.
        DrawPlanarRectangle({Tracker::kPatchSize, Tracker::kPatchSize, 0.0},
                            {tracker->x(), tracker->y(), current_time});
      }
    }

    // Draw seed selector.
    glColor3fv(kColorRed);
    drawSeedWithCrosshair(seed_selector);

    // Feature track. Recorded states.
    // Consecutive states connected by lines.
    static constexpr auto kStateSize = 3.0f;
    glColor3fv(kColorRed);
    glLineWidth(kStateSize);
    glBegin(GL_LINE_STRIP);
    for (const auto& state : states) { glVertex3f(state.x(), state.y(), state.z()); }
    glEnd();
    if (view_track_projection) {
      glBegin(GL_LINE_STRIP);
      for (const auto& state : states) { glVertex3f(state.x(), state.y(), current_time); }
      glEnd();
    }
    glLineWidth(1.0f);
    // States drawn as points.
    glColor3fv(kColorRed);
    glPointSize(kStateSize);
    glBegin(GL_POINTS);
    for (const auto& state : states) { glVertex3f(state.x(), state.y(), state.z()); }
    glEnd();
    if (view_track_projection) {
      glBegin(GL_POINTS);
      for (const auto& state : states) { glVertex3f(state.x(), state.y(), current_time); }
      glEnd();
    }
    glPointSize(1.0);

    glPopMatrix();

    ImGui_ImplOpenGL3_RenderDrawData(ImGui::GetDrawData());
    glfwSwapBuffers(window_);
  }
}
}// namespace gui
}// namespace app
}// namespace haste