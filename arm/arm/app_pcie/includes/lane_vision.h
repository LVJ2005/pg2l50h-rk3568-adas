#pragma once
#include "adas_types.h"
#include "lane_detector.h"
#include <opencv2/core.hpp>
#include <vector>
#include <cstdint>

namespace adas {
const char *lane_execution_backend();
cv::Rect lane_roi(const Settings &settings);
// 0: background, 1: paint, 2: occlusion, 5: structurally ambiguous, 6: outside road.
// Debug only: 3 weak, 4 yellow. Ambiguous samples cannot prove a dash gap.
void make_lane_mask(const cv::Mat &bgr,std::vector<uint8_t> &mask,int contrast=12,
    std::vector<uint8_t> *weak=nullptr,std::vector<uint8_t> *paint=nullptr);
std::pair<int,int> lane_road_bounds(const Settings &,int y,int height,int width);
cv::Vec3b lane_debug_rgb(uint8_t value);
LaneResult analyze_lane_image(const cv::Mat &bgr,const Settings &,LaneScanner &,
    const std::vector<Box> &occluders={});
}
