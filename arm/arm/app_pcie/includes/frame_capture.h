#pragma once
#include "latest_mailbox.h"
#include "recognition_control.h"
#include <opencv2/core.hpp>

namespace adas {
struct Frame {
  cv::Mat bgr;
  uint64_t sequence=0;
  double captured_ms=0;
};
void capture_frames(LatestMailbox<Frame> &,Runtime &,std::atomic<bool> &stop,const std::string &video,bool use_mmap);
} // namespace adas
