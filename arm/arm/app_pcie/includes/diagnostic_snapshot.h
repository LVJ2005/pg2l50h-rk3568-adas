#pragma once
#include "frame_capture.h"
#include <string>

namespace adas {
void diagnostic_worker(LatestMailbox<Frame> &,Runtime &,std::atomic<bool> &,const std::string &directory);
}
