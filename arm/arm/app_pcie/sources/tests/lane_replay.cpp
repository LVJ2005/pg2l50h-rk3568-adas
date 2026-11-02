#include "lane_vision.h"
#include "frame_color.h"
#include <opencv2/imgcodecs.hpp>
#include <filesystem>
#include <iostream>

int main(int argc,char **argv) {
  if(argc!=3 && argc!=7) {
    std::cerr<<"Usage: lane_replay IMAGE OUTPUT_DIR [crop_x crop_y crop_width crop_height]\n";return 2;
  }
  try {
    cv::setNumThreads(1);
    cv::Mat image=cv::imread(argv[1]);if(image.empty()) throw std::runtime_error("Cannot read image");
    cv::Mat frame;
    if(argc==7) {
      cv::Rect crop(std::stoi(argv[3]),std::stoi(argv[4]),std::stoi(argv[5]),std::stoi(argv[6]));
      if((crop&cv::Rect(0,0,image.cols,image.rows))!=crop) throw std::runtime_error("Crop outside image");
      cv::Mat quadrant;cv::resize(image(crop),quadrant,{640,360});
      frame=cv::Mat(720,1280,CV_8UC3);
      for(int view=0;view<4;++view) quadrant.copyTo(frame(cv::Rect((view%2)*640,(view/2)*360,640,360)));
    } else {
      if(image.size()!=cv::Size(1280,720)) throw std::runtime_error("Provide a 1280x720 board snapshot or an explicit photograph crop");
      frame=image;
    }
    const std::filesystem::path output(argv[2]);std::filesystem::create_directories(output);
    // imread returns a normal BGR image, not the raw RBG wire snapshot.
    adas::Settings config;config.lane_debug=true;config.infer_order="bgr";
    adas::LaneScanner scanner;adas::LaneResult result;
    for(int n=0;n<3;++n) result=adas::analyze_lane_image(frame,config,scanner);
    cv::Mat overlay=frame.clone();const auto rect=adas::lane_roi(config);
    const auto p=[&](float x,float y) {return cv::Point(std::lround(rect.x+x*rect.width),std::lround(rect.y+y*rect.height));};
    cv::polylines(overlay,std::vector<cv::Point>{p(config.lane_trap_tl,0),p(config.lane_trap_tr,0),
        p(config.lane_trap_br,1),p(config.lane_trap_bl,1)},true,cv::Scalar(255,255,0));
    for(const auto *edge:{&result.left,&result.right}) if(edge->fitted) {
      auto point=[&](int y) {return cv::Point(rect.x+edge->x_at(y)*rect.width/result.width,
          rect.y+y*rect.height/result.height);};
      const auto color=edge->kind==adas::LineKind::Solid ? cv::Scalar(0,0,255):
          edge->kind==adas::LineKind::Dashed ? cv::Scalar(0,255,0):cv::Scalar(160,160,160);
      for(int y=edge->first_y;y<edge->last_y;y+=3) cv::line(overlay,point(y),point(std::min(y+3,edge->last_y)),color,2);
    }
    cv::Mat mask(result.height,result.width,CV_8UC1),candidates_rgb(result.height,result.width,CV_8UC3);
    for(int y=0;y<mask.rows;++y) for(int x=0;x<mask.cols;++x) {
      const int v=result.mask[y*mask.cols+x];mask.at<uint8_t>(y,x)=v==1 || v==4 ? 255:v==2 ? 80:v==3 ? 120:0;
      candidates_rgb.at<cv::Vec3b>(y,x)=adas::lane_debug_rgb(v);
    }
    cv::imwrite((output/"lane_overlay.png").string(),overlay);
    cv::imwrite((output/"lane_mask.png").string(),mask);
    cv::Mat candidates_bgr;cv::cvtColor(candidates_rgb,candidates_bgr,cv::COLOR_RGB2BGR);
    cv::imwrite((output/"lane_candidates.png").string(),candidates_bgr);
    cv::imwrite((output/"color_candidates.png").string(),adas::color_comparison(frame(cv::Rect(0,0,640,360))));
    cv::FileStorage report((output/"lane_result.json").string(),cv::FileStorage::WRITE|cv::FileStorage::FORMAT_JSON);
    report<<"roi"<<"["<<rect.x<<rect.y<<rect.width<<rect.height<<"]";
    report<<"width"<<result.width<<"height"<<result.height;
    auto save_edge=[&](const char *name,const adas::LaneEdge &edge) {
      report<<name<<"{"<<"kind"<<adas::line_name(edge.kind)<<"fitted"<<static_cast<int>(edge.fitted)
        <<"confidence"<<edge.confidence<<"coverage"<<edge.coverage<<"intercept"<<edge.intercept
        <<"slope"<<edge.slope<<"curvature"<<edge.curvature<<"first_y"<<edge.first_y<<"last_y"<<edge.last_y;
#ifndef ADAS_BASELINE_EVAL
      report<<"paint"<<adas::paint_name(edge.paint)<<"support_rows"<<edge.support_rows<<"segments"<<edge.segment_count<<"gaps"<<edge.gap_count<<"longest_gap"<<edge.longest_gap;
#endif
      report<<"}";
    };
    save_edge("left",result.left);save_edge("right",result.right);report.release();
    std::cout<<"left="<<adas::line_name(result.left.kind)<<" fit="<<result.left.fitted<<" conf="<<result.left.confidence
      <<" right="<<adas::line_name(result.right.kind)<<" fit="<<result.right.fitted<<" conf="<<result.right.confidence
      <<" coverage="<<result.left.coverage<<","<<result.right.coverage
      <<" scan_rows="<<result.height<<" paint="<<result.paint_ratio<<" occlusion="<<result.occlusion_ratio<<"\n";
    return 0;
  } catch(const std::exception &error) {std::cerr<<error.what()<<"\n";return 1;}
}
