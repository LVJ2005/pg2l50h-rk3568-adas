#pragma once
#include "adas_types.h"
#include <array>

namespace adas {
struct DetectorHead {
  const float *boxes=nullptr,*scores=nullptr;
  int height=0,width=0;
  bool boxes_nhwc=false,scores_nhwc=false;
};
std::vector<Detection> decode_heads(const std::array<DetectorHead,3> &,const Settings &);
} // namespace adas
