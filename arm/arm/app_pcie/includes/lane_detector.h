#pragma once
#include "adas_types.h"
#include <cstddef>

namespace adas {
class LaneScanner {
public:
  std::pair<LaneEdge,LaneEdge> detect(const uint8_t *mask,int width,int height,float horizon_y=-1000.f,
      float center=.5f,const std::vector<LaneEdge> &seeds={},float vehicle_x=.5f,
      const uint8_t *weak=nullptr,const uint8_t *paint=nullptr);
  void reset();
private:
  LaneEdge previous_[2];
  LaneEdge pending_geometry_[2];
  int reacquire_[2]{0,0};
  LineKind pending_[2]{LineKind::Unknown,LineKind::Unknown};
  int streak_[2]{0,0};
};
} // namespace adas
