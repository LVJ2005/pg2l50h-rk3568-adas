#pragma once
#include "adas_types.h"
#include <memory>
#include <opencv2/core.hpp>

namespace adas {
class RknnModel;
cv::Mat make_detector_input(const cv::Mat &,const std::string &camera_order="rbg");
cv::Mat make_classifier_input(const cv::Mat &,const Box &,const std::string &camera_order="rbg");
class TrafficPipeline {
public:
  explicit TrafficPipeline(std::string directory);
  ~TrafficPipeline();
  void ensure_models(const Settings &);
  Perception infer(const cv::Mat &,const Settings &,uint64_t seq,double captured_ms);
  const std::string &det_profile() const {return det_profile_;}
  const std::string &cls_profile() const {return cls_profile_;}
private:
  std::string directory_,det_profile_="none",cls_profile_="none";
  std::unique_ptr<RknnModel> detector_,classifier_;
};
} // namespace adas
