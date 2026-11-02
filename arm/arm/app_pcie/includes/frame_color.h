#pragma once
#include "adas_types.h"
#include <opencv2/imgproc.hpp>
#include <string>

namespace adas {
// Only remap a consumer-owned image. This does not change the DMA or model contract.
inline cv::Mat camera_display_bgr(const cv::Mat &bytes,const std::string &order) {
  CV_Assert(bytes.type()==CV_8UC3 && valid_camera_order(order));
  if(order=="bgr") return bytes.clone();
  cv::Mat result(bytes.size(),bytes.type());
  const int mapping[]={static_cast<int>(order.find('b')),0,static_cast<int>(order.find('g')),1,
      static_cast<int>(order.find('r')),2};
  cv::mixChannels(&bytes,1,&result,1,mapping,3);return result;
}
inline cv::Mat camera_model_rgb(const cv::Mat &bytes,const std::string &order) {
  CV_Assert(bytes.type()==CV_8UC3 && valid_camera_order(order));
  cv::Mat result(bytes.size(),bytes.type());
  const int mapping[]={static_cast<int>(order.find('r')),0,static_cast<int>(order.find('g')),1,
      static_cast<int>(order.find('b')),2};
  cv::mixChannels(&bytes,1,&result,1,mapping,3);return result;
}
// Display-only copy. The DMA snapshot and NPU input source remain BGR.
inline cv::Mat make_display_rgb(const cv::Mat &bgr) {
  CV_Assert(bgr.type()==CV_8UC3);
  cv::Mat rgb;cv::cvtColor(bgr,rgb,cv::COLOR_BGR2RGB);return rgb;
}
inline cv::Mat color_comparison(const cv::Mat &camera) {
  cv::Mat small;cv::resize(camera,small,cv::Size(320,180),0,0,cv::INTER_AREA);
  const char *orders[]={"bgr","rgb","brg","rbg","gbr","grb"};
  cv::Mat comparison(2*208,3*320,CV_8UC3,cv::Scalar(20,20,20));
  for(int n=0;n<6;++n) {
    auto tile=camera_display_bgr(small,orders[n]);
    tile.copyTo(comparison(cv::Rect((n%3)*320,(n/3)*208+28,320,180)));
    cv::putText(comparison,orders[n],cv::Point((n%3)*320+8,(n/3)*208+20),
        cv::FONT_HERSHEY_SIMPLEX,.6,cv::Scalar(255,255,255),1,cv::LINE_AA);
  }
  return comparison;
}
} // namespace adas
