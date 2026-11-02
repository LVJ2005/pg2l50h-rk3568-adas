#include "lane_vision.h"
#include <opencv2/imgproc.hpp>
#include <cassert>
#include <iostream>

namespace {
cv::Mat road(bool clutter) {
  cv::Mat image(100,320,CV_8UC3,cv::Scalar(65,65,65));
  for(int y=0;y<image.rows;++y) {
    const float t=y/99.f;
    const int left=std::lround(115-65*t),right=std::lround(195+65*t);
    if(clutter) {
      const int reflection=std::lround(142-28*t);
      cv::line(image,{reflection-9,y},{reflection+9,y},cv::Scalar(240,240,240));
    }
    cv::line(image,{left-2,y},{left+2,y},cv::Scalar(22,125,150));
    if(y%24<14) cv::line(image,{right-2,y},{right+2,y},cv::Scalar(220,220,220));
  }
  return image;
}
void stripe_and_color_tests() {
  for(int threshold:{12,40,60}) {
    auto image=road(true);
    std::vector<uint8_t> strong,weak,paint;
    adas::make_lane_mask(image,strong,threshold,&weak,&paint);
    int rejected_pixels=0,yellow_pixels=0;
    for(size_t k=0;k<strong.size();++k) {
      if(strong[k]==5) {++rejected_pixels;assert(!weak[k]);}
      if(strong[k]==1 && paint[k]==2) ++yellow_pixels;
    }
    assert(rejected_pixels>500 && yellow_pixels>250);
    adas::LaneScanner scanner;std::pair<adas::LaneEdge,adas::LaneEdge> result;
    for(int n=0;n<3;++n) result=scanner.detect(strong.data(),320,100,-20,.5f,{},.5f,weak.data(),paint.data());
    assert(result.first.fitted && result.first.paint==adas::PaintColor::Yellow);
    assert(result.first.kind==adas::LineKind::Solid);
    assert(result.second.kind==adas::LineKind::Dashed);
    for(int y:{20,50,90}) assert(std::abs(result.first.x_at(y)-(115-65*y/99.f))<4);
  }
  // Color fragments may not be pooled to invent one continuous solid marking.
  constexpr int w=192,h=96;
  std::vector<uint8_t> mask(w*h,0),paint(w*h,0);
  for(int y=0;y<h;++y) {
    const int x=std::lround(73-42*y/95.f);
    for(int dx=-2;dx<=2;++dx) {mask[y*w+x+dx]=1;paint[y*w+x+dx]=y<48 ? 1:2;}
  }
  adas::LaneScanner scanner;std::pair<adas::LaneEdge,adas::LaneEdge> result;
  for(int n=0;n<3;++n) result=scanner.detect(mask.data(),w,h,-20,.5f,{},.5f,nullptr,paint.data());
  assert(result.first.kind!=adas::LineKind::Solid);
}
void road_exclusion_tests() {
  adas::Settings s;s.infer_order="bgr";s.lane_debug=true;
  s.lane_trap_tl=.32f;s.lane_trap_tr=.68f;s.lane_trap_bl=.18f;s.lane_trap_br=.82f;
  const auto roi=adas::lane_roi(s);
  cv::Mat small(100,320,CV_8UC3,cv::Scalar(65,65,65));
  for(int y=0;y<100;++y) {
    const auto bounds=adas::lane_road_bounds(s,y,100,320);
    for(int x:{bounds.first-12,bounds.second+8})
      cv::line(small,{x-2,y},{x+2,y},cv::Scalar(240,240,240));
  }
  cv::Mat frame(720,1280,CV_8UC3,cv::Scalar(65,65,65));
  cv::resize(small,frame(roi),roi.size(),0,0,cv::INTER_NEAREST);
  const auto original=frame.clone();
  adas::LaneScanner scanner;
  for(int n=0;n<3;++n) {
    const auto result=adas::analyze_lane_image(frame,s,scanner);
    assert(!result.left.fitted && !result.right.fitted);
    assert(result.paint_ratio<.001 && result.occlusion_ratio==0);
    assert(std::count(result.mask.begin(),result.mask.end(),uint8_t{6})>5000);
  }
  assert(cv::norm(frame,original,cv::NORM_INF)==0);
}
}
int main() {
  cv::setNumThreads(1);
  stripe_and_color_tests();road_exclusion_tests();
  std::cout<<"PASS stripe structure, separate yellow/white support, threshold independence, trapezoid and no input mutation\n";
}
