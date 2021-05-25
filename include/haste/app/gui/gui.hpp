// Copyright (c) 2021 by Ignacio Alzugaray <alzugaray dot ign at gmail dot com>
// ETH Zurich, Vision for Robotics Lab.
#pragma once

#include "haste/app/tracking.hpp"
#include "quickgui/gui.hpp"

namespace haste {
namespace app {
namespace gui {

class HasteGui : public quickgui::Gui {
 public:
  using Events = std::vector<Event>;
  using EventsIterator = Events::iterator;
  using EventsConstIterator = Events::const_iterator;

  static auto drawEvents(const EventsConstIterator& it_begin, const EventsConstIterator& it_end) -> void;
  auto drawSeedWithCrosshair(const Vector3& seed) -> void;

  HasteGui(const Scalar& camera_width, const Scalar& camera_height, const Events* events_ptr);

  auto resetCamera() -> void override;
  auto run_() -> void override;

 private:
  const Events* events_ptr_;
  Scalar camera_width_ = 1.0f, camera_height_ = 1.0f;
};

}// namespace gui
}// namespace app
}// namespace haste

#include "gui_impl.hpp"