#include "traffic_detector.h"
#include "frame_color.h"
#include "lane_detector.h"
#include "lane_vision.h"
#include <cassert>
#include <iostream>
#ifdef ADAS_HOST_TEST
extern "C" const uint8_t *adas_test_input(int detector,size_t *size);
#endif

namespace {
void lane_texture_regression() {
  cv::Mat image(180,320,CV_8UC3,cv::Scalar(55,55,55));
  // Screen texture / repeated bright objects must not erase an entire scan row.
  for(int x=12;x<310;x+=38)
    cv::rectangle(image,{x,0,10,92},cv::Scalar(190,190,190),cv::FILLED);
  for(int y=0;y<image.rows;++y) {
    const float t=static_cast<float>(y)/(image.rows-1);
    const int left=std::lround(115-65*t),right=std::lround(195+67*t);
    cv::line(image,{left-2,y},{left+2,y},cv::Scalar(230,230,230));
    if(y%40<23) cv::line(image,{right-2,y},{right+2,y},cv::Scalar(230,230,230));
  }
  std::vector<uint8_t> mask;adas::make_lane_mask(image,mask);
  const auto erased=std::count(mask.begin(),mask.end(),uint8_t{2});
  std::cout<<"texture regression: invented occlusion pixels="<<erased<<std::endl;
  assert(erased==0);
  adas::LaneScanner scanner;std::pair<adas::LaneEdge,adas::LaneEdge> result;
  for(int n=0;n<3;++n) result=scanner.detect(mask.data(),320,180,-30);
  std::cout<<"texture fit L="<<adas::line_name(result.first.kind)<<"/"<<result.first.fitted
    <<"/"<<result.first.coverage<<"/"<<result.first.confidence
    <<" R="<<adas::line_name(result.second.kind)<<"/"<<result.second.fitted
    <<"/"<<result.second.coverage<<"/"<<result.second.confidence<<std::endl;
  assert(result.first.kind==adas::LineKind::Solid && result.second.kind==adas::LineKind::Dashed);
  for(int y:{100,140,170}) assert(std::abs(result.first.x_at(y)-(115-65*y/179.f))<4);
}
void lane_scan_rows_regression() {
  cv::Mat frame(720,1280,CV_8UC3,cv::Scalar(55,55,55));
  for(int y=162;y<353;++y) {
    const float t=(y-129.6f)/(352-129.6f);
    const int left=std::lround(384+(140-384)*t);
    const int right=std::lround(384+(580-384)*t);
    cv::line(frame,{left-4,y},{left+4,y},cv::Scalar(230,230,230));
    if((y-162)%44<26) cv::line(frame,{right-4,y},{right+4,y},cv::Scalar(230,230,230));
  }
  const cv::Mat before=frame.clone();
  for(int count:{100,180,60}) {
    adas::Settings config;config.infer_order="bgr";config.lane_debug=true;
    config.lane_top=.45f;config.lane_bottom=.98f;config.lane_horizon=.36f;
    config.lane_scan_rows=count;config.lane_x1=.15f;config.lane_x2=.95f;config.lane_center=.60f;
    adas::LaneScanner scanner;adas::LaneResult result;
    for(int n=0;n<3;++n) result=adas::analyze_lane_image(frame,config,scanner);
    std::cout<<"scan rows="<<count<<" L="<<adas::line_name(result.left.kind)
      <<" R="<<adas::line_name(result.right.kind)<<" fitted="<<result.right.fitted
      <<" coverage="<<result.right.coverage<<" conf="<<result.right.confidence<<std::endl;
    assert(result.height==count && result.mask.size()==static_cast<size_t>(320*count));
    assert(result.left.fitted);
    // At 60 rows, vertical downsampling can merge paint with adjacent pixels.
    // Rejecting ambiguous samples may yield unknown, never a false dashed line.
    assert(count==60 ? result.left.kind!=adas::LineKind::Dashed : result.left.kind==adas::LineKind::Solid);
    // Coarse sampling can lose short dash gaps; never promote these to solid.
    assert(count==60 ? result.right.kind!=adas::LineKind::Solid : result.right.kind==adas::LineKind::Dashed);
    assert(result.paint_ratio>.005f && result.paint_ratio<.2f && result.occlusion_ratio==0);
    const auto excluded=adas::analyze_lane_image(frame,config,scanner,{{300,210,330,240}});
    assert(excluded.occlusion_ratio>0 && excluded.occlusion_ratio<.03f);
    for(int y=0;y<excluded.height;++y) {
      const auto first=excluded.mask.begin()+y*excluded.width;
      assert(std::count(first,first+excluded.width,uint8_t{2})<excluded.width/8);
    }
  }
  adas::Settings narrow;narrow.infer_order="bgr";narrow.lane_debug=true;
  narrow.lane_top=.75f;narrow.lane_bottom=.95f;
  adas::LaneScanner scanner;
  const auto small=adas::analyze_lane_image(frame,narrow,scanner);
  assert(small.height==adas::lane_roi(narrow).height && small.height<100);
  assert(cv::norm(frame,before,cv::NORM_INF)==0);
}
void lanes_from_images() {
  for(int scenario=0;scenario<9;++scenario) {
    cv::Mat image(190,640,CV_8UC3,cv::Scalar(55,55,55));
    for(int y=0;y<image.rows;++y) {
      const float t=static_cast<float>(y)/(image.rows-1);
      const int shift=scenario==2 ? 46:0;
      const int bend=scenario==3 ? static_cast<int>(28*t*t):0;
      const int left=static_cast<int>(230-130*t)+shift+bend;
      const int right=static_cast<int>(390+135*t)+shift+bend;
      const bool dark=scenario==1 || scenario==7 || (scenario==4 && y>50 && y<140);
      if(dark) image.row(y).setTo(cv::Scalar(25,25,25));
      const int radius=(scenario==5 || scenario==7) ? 1:4;
      const cv::Scalar white=dark ? cv::Scalar(75,75,75):cv::Scalar(210,210,210);
      const cv::Scalar paint=scenario==6 ? cv::Scalar(20,145,175):white;
      cv::line(image,{left-radius,y},{left+radius,y},paint);
      if(y%44<24) cv::line(image,{right-radius,y},{right+radius,y},white);
    }
    if(scenario==8) for(int y=0;y<image.rows;++y) for(int x=0;x<image.cols;++x) {
      const int interference=(x%3==0 ? 12:-6)+(y%4==0 ? 10:-3);
      for(int c=0;c<3;++c) image.at<cv::Vec3b>(y,x)[c]=cv::saturate_cast<uint8_t>(image.at<cv::Vec3b>(y,x)[c]+interference);
    }
    cv::Mat small;cv::resize(image,small,{320,180},0,0,cv::INTER_AREA);
    std::vector<uint8_t> mask;adas::make_lane_mask(small,mask);
    adas::LaneScanner scanner;std::pair<adas::LaneEdge,adas::LaneEdge> result;
    for(int i=0;i<3;++i) result=scanner.detect(mask.data(),320,180,-30);
    std::cout<<"image lane scenario="<<scenario<<" L="<<adas::line_name(result.first.kind)
      <<" R="<<adas::line_name(result.second.kind)<<" coverage="<<result.first.coverage<<","<<result.second.coverage
      <<" right_x="<<result.second.x_at(10)<<","<<result.second.x_at(160)<<std::endl;
    assert(result.first.kind==adas::LineKind::Solid && result.second.kind==adas::LineKind::Dashed);
    assert(result.second.coverage>.4f);
    for(int row:{20,80,155}) {
      const float t=static_cast<float>(row)/179;
      const float shift=scenario==2 ? 23:0,bend=scenario==3 ? 14*t*t:0;
      assert(std::abs(result.first.x_at(row)-(115-65*t+shift+bend))<4);
      assert(std::abs(result.second.x_at(row)-(195+67.5f*t+shift+bend))<4);
    }
  }
  // Blank road and broad horizontal crosswalk stripes must not become lane boundaries.
  for(int scene=0;scene<2;++scene) {
    cv::Mat image(180,320,CV_8UC3,cv::Scalar(70,70,70));
    if(scene) for(int y=10;y<170;y+=25) cv::rectangle(image,{20,y,280,10},cv::Scalar(220,220,220),cv::FILLED);
    std::vector<uint8_t> mask;adas::make_lane_mask(image,mask);
    adas::LaneScanner scanner;
    for(int i=0;i<4;++i) {
      auto result=scanner.detect(mask.data(),320,180);
      assert(result.first.kind==adas::LineKind::Unknown && result.second.kind==adas::LineKind::Unknown);
    }
  }
}
}

int main() {
  lane_texture_regression();
  lane_scan_rows_regression();
  lanes_from_images();
  adas::Settings roi_settings;roi_settings.front_view=3;roi_settings.lane_x1=.1f;roi_settings.lane_x2=.9f;
  roi_settings.lane_top=.45f;roi_settings.lane_bottom=.98f;
  assert(adas::lane_roi(roi_settings)==cv::Rect(704,522,512,191));
  cv::Mat bgr(4,6,CV_8UC3,cv::Scalar(10,40,230));
  cv::Mat display_rgb=adas::make_display_rgb(bgr);
  assert(bgr.at<cv::Vec3b>(0,0)==cv::Vec3b(10,40,230));
  assert(display_rgb.at<cv::Vec3b>(0,0)==cv::Vec3b(230,40,10));
  bgr.setTo(cv::Scalar(0,0,0));
  assert(display_rgb.at<cv::Vec3b>(0,0)==cv::Vec3b(230,40,10));
  for(const std::string order:{"bgr","rgb","brg","rbg","gbr","grb"}) {
    cv::Mat raw(3,7,CV_8UC3);
    cv::Vec3b wire;
    for(int c=0;c<3;++c) wire[c]=order[c]=='b' ? 10:order[c]=='g' ? 40:230;
    raw.setTo(cv::Scalar(wire[0],wire[1],wire[2]));
    const cv::Mat original=raw.clone();
    auto repaired=adas::camera_display_bgr(raw,order);
    assert(repaired.at<cv::Vec3b>(0,0)==cv::Vec3b(10,40,230));
    assert(cv::norm(raw,original,cv::NORM_INF)==0);
    // Overlay is added after channel repair and must retain its semantic color.
    cv::rectangle(repaired,{0,0,1,1},cv::Scalar(0,0,255),cv::FILLED);
    assert(adas::make_display_rgb(repaired).at<cv::Vec3b>(0,0)==cv::Vec3b(255,0,0));
  }
  // A monitor frame and a pole are not road boundaries even if they contain long white runs.
  cv::Mat screen(720,1280,CV_8UC3,cv::Scalar(45,45,45));
  for(int y=170;y<355;++y) {
    cv::line(screen,{12,y},{17,y},cv::Scalar(240,240,240));
    cv::line(screen,{623,y},{629,y},cv::Scalar(240,240,240));
    cv::line(screen,{440,y},{444,y},cv::Scalar(220,220,220));
  }
  adas::Settings screen_settings;adas::LaneScanner screen_scanner;
  for(int n=0;n<3;++n) {
    auto result=adas::analyze_lane_image(screen,screen_settings,screen_scanner);
    assert(!result.left.fitted && !result.right.fitted);
  }
  // Four distinct quadrants expose accidental whole-frame resizing and ROI padding leakage.
  cv::Mat frame(720,1280,CV_8UC3,cv::Scalar(201,202,203));
  frame(cv::Rect(640,0,640,360)).setTo(cv::Scalar(101,102,103));
  frame(cv::Rect(0,360,640,360)).setTo(cv::Scalar(151,152,153));
  frame(cv::Rect(0,0,640,360)).setTo(cv::Scalar(30,10,20)); // R,B,G wire order.
  const cv::Mat before=frame.clone();
  auto detector=adas::make_detector_input(frame);
  assert(detector.rows==384 && detector.cols==640 && detector.isContinuous());
  assert(detector.at<cv::Vec3b>(0,0)==cv::Vec3b(114,114,114));
  assert(detector.at<cv::Vec3b>(12,0)==cv::Vec3b(30,20,10));
  assert(detector.at<cv::Vec3b>(371,639)==cv::Vec3b(30,20,10));
  assert(detector.at<cv::Vec3b>(372,639)==cv::Vec3b(114,114,114));
  assert(cv::norm(frame,before,cv::NORM_INF)==0);
  // A source pixel keeps its position; there is no remaining 2x reduction.
  frame.at<cv::Vec3b>(120,300)=cv::Vec3b(70,80,90);
  detector=adas::make_detector_input(frame);
  assert(detector.at<cv::Vec3b>(132,300)==cv::Vec3b(70,90,80));
  frame.at<cv::Vec3b>(120,300)=cv::Vec3b(30,10,20);
  auto classifier=adas::make_classifier_input(frame,{400,176,440,256});
  assert(classifier.rows==128 && classifier.cols==128 && classifier.isContinuous());
  assert(classifier.at<cv::Vec3b>(64,31)==cv::Vec3b(114,114,114));
  assert(classifier.at<cv::Vec3b>(64,32)==cv::Vec3b(30,20,10));
  assert(classifier.at<cv::Vec3b>(64,95)==cv::Vec3b(30,20,10));
  assert(classifier.at<cv::Vec3b>(64,96)==cv::Vec3b(114,114,114));
  auto clipped=adas::make_classifier_input(frame,{630,350,650,370});
  assert(clipped.at<cv::Vec3b>(64,64)==cv::Vec3b(30,20,10));
  auto edge=adas::make_classifier_input(frame,{-3,-2,7.2f,9.1f});assert(edge.rows==128);
  bool rejected=false;try{adas::make_classifier_input(frame,{4,4,4,8});}catch(...){rejected=true;}
  assert(rejected);
  rejected=false;try{adas::make_classifier_input(frame,{700,20,730,60});}catch(...){rejected=true;}
  assert(rejected);
#ifdef ADAS_HOST_TEST
  adas::TrafficPipeline pipeline("models");adas::Settings config;pipeline.ensure_models(config);
  const auto result=pipeline.infer(frame,config,1,adas::now_ms());
  assert(result.objects.size()==1 && result.objects[0].view==0);
  assert(std::abs(result.objects[0].box.x1-188)<.01 && std::abs(result.objects[0].box.y1-72)<.01);
  size_t input_size=0;const auto *det_bytes=adas_test_input(1,&input_size);
  assert(input_size==640*384*3);
  assert(det_bytes[(12*640)*3]==30 && det_bytes[(12*640)*3+1]==20 && det_bytes[(12*640)*3+2]==10);
  const auto *cls_bytes=adas_test_input(0,&input_size);assert(input_size==128*128*3);
  assert(cls_bytes[(64*128+64)*3]==30 && cls_bytes[(64*128+64)*3+1]==20 && cls_bytes[(64*128+64)*3+2]==10);
  assert(cv::norm(frame,before,cv::NORM_INF)==0);
#endif
  std::cout<<"PASS Q1-only letterbox, RBG-to-RGB, classifier crop, no quadrant leakage, unit-scale coordinates\n";
}
