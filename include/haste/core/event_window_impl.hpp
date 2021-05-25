// Copyright (c) 2021 by Ignacio Alzugaray <alzugaray dot ign at gmail dot com>
// ETH Zurich, Vision for Robotics Lab.

namespace haste {

template<typename E, size_t S>
FixedSizeLocationEventWindowType<E, S>::FixedSizeLocationEventWindowType() {
  et_vec_.setZero();
  ex_vec_.setZero();
  ey_vec_.setZero();
}

template<typename E, size_t S>
auto FixedSizeLocationEventWindowType<E, S>::getEvent(const size_t &idx) const -> EventTuple {
  return {et_vec_(idx), ex_vec_(idx), ey_vec_(idx)};
}

template<typename E, size_t S>
auto FixedSizeLocationEventWindowType<E, S>::oldestEvent() const -> EventTuple {
  return getEvent(kOldestEventIdx);
}

template<typename E, size_t S>
auto FixedSizeLocationEventWindowType<E, S>::newestEvent() const -> EventTuple {
  return getEvent(kNewestEventIdx);
}

template<typename E, size_t S>
auto FixedSizeLocationEventWindowType<E, S>::middleEvent() const -> EventTuple {
  return getEvent(kMiddleEventIdx);
}

template<typename E, size_t S>
auto FixedSizeLocationEventWindowType<E, S>::setEvent(const Time &et, const Location &ex, const Location &ey,
                                                      const size_t &idx) -> void {
  et_vec_(idx) = et;
  ex_vec_(idx) = ex;
  ey_vec_(idx) = ey;
}

template<typename E, size_t S>
auto FixedSizeLocationEventWindowType<E, S>::setEvent(const EventTuple &event, const size_t &idx) -> void {
  const auto &[et, ex, ey] = event;// Unpack.
  setEvent(et, ex, ey, idx);
}

template<typename E, size_t S>
auto FixedSizeLocationEventWindowType<E, S>::appendEvent(const Time &et, const Location &ex, const Location &ey)
    -> EventTuple {
  auto oldest_event = oldestEvent();// Event to be removed from the window and returned

  // Roll the data array. TODO: To be substituted by a circular index
  et_vec_.template topRows<kSize - 1>() = et_vec_.template bottomRows<kSize - 1>();
  ex_vec_.template topRows<kSize - 1>() = ex_vec_.template bottomRows<kSize - 1>();
  ey_vec_.template topRows<kSize - 1>() = ey_vec_.template bottomRows<kSize - 1>();

  setEvent(et, ex, ey, kNewestEventIdx);// Edit the newest event

  return oldest_event;
}

template<typename E, size_t S>
auto FixedSizeLocationEventWindowType<E, S>::appendEvent(const EventTuple &new_event) -> EventTuple {
  const auto &[et, ex, ey] = new_event;
  return appendEvent(et, ex, ey);
}

}// namespace haste
