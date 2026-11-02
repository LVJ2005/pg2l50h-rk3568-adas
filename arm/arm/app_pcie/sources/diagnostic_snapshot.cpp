#include "diagnostic_snapshot.h"
#include "frame_color.h"
#include "lane_vision.h"
#include "traffic_detector.h"
#include <opencv2/imgcodecs.hpp>
#include <filesystem>
#include <fstream>
#include <iostream>

namespace adas {
void diagnostic_worker(LatestMailbox<Frame> &hub,Runtime &runtime,std::atomic<bool> &stop,const std::string &directory) {
  uint64_t handled=0;
  while(!stop) {
    const uint64_t request=runtime.snapshot_requested.load();
    if(request!=handled) {
      const auto frame=hub.try_read();
      if(frame) {
        handled=request;
        try {
          const auto s=runtime.settings();
          const std::filesystem::path folder=std::filesystem::absolute(directory)/
              (std::to_string(static_cast<uint64_t>(now_ms()))+"_"+std::to_string(request));
          std::filesystem::create_directories(folder);
          auto save=[&](const char *name,const cv::Mat &image) {
            if(!cv::imwrite((folder/name).string(),image)) throw std::runtime_error("PNG write failed");
          };
          save("raw_bgr.png",frame->bgr);
          save("display_bgr.png",camera_display_bgr(frame->bgr,s.view_order));
          save("inference_q1_bgr.png",camera_display_bgr(frame->bgr(cv::Rect(0,0,640,360)),s.infer_order));
          cv::Mat detector_bgr;cv::cvtColor(make_detector_input(frame->bgr,s.infer_order),detector_bgr,cv::COLOR_RGB2BGR);
          save("detector_input.png",detector_bgr);
          const Box view=view_box(s.front_view);
          save("channel_candidates.png",color_comparison(frame->bgr(cv::Rect(view.x1,view.y1,640,360))));
          std::ofstream binary(folder/"dma_snapshot.bin",std::ios::binary);
          for(int y=0;y<frame->bgr.rows;++y)
            binary.write(reinterpret_cast<const char*>(frame->bgr.ptr(y)),frame->bgr.cols*3);
          binary.close();if(!binary) throw std::runtime_error("raw snapshot write failed");
          LaneScanner scanner;auto config=s;config.lane_debug=true;
          const auto lane=analyze_lane_image(frame->bgr,config,scanner);
          save("lane_roi_bgr.png",camera_display_bgr(frame->bgr(lane_roi(config)),s.infer_order));
          cv::Mat mask(lane.height,lane.width,CV_8UC1),mask_rgb(lane.height,lane.width,CV_8UC3);
          for(int y=0;y<mask.rows;++y) for(int x=0;x<mask.cols;++x) {
            const uint8_t value=lane.mask[y*mask.cols+x];
            mask.at<uint8_t>(y,x)=value==1 || value==4 ? 255:value==2 ? 80:value==3 ? 120:0;
            mask_rgb.at<cv::Vec3b>(y,x)=lane_debug_rgb(value);
          }
          save("lane_mask.png",mask);
          cv::Mat mask_bgr;cv::cvtColor(mask_rgb,mask_bgr,cv::COLOR_RGB2BGR);save("lane_candidates.png",mask_bgr);
          std::ofstream meta(folder/"metadata.txt");
          meta<<"frame_seq="<<frame->sequence<<" captured_ms="<<frame->captured_ms<<"\n"
              <<"snapshot_layout=1280x720_uint8_BGR888_without_overlay\n"
              <<"view_order="<<s.view_order<<" infer_order="<<s.infer_order<<" display_backend="<<s.display_backend<<"\n"
              <<"detector_roi=0,0,640,360 detector_scale=1 pad_top=12 pad_bottom=12\n"
              <<"front_view="<<s.front_view<<" lane_roi="<<s.lane_x1<<','<<s.lane_top<<','<<s.lane_x2<<','<<s.lane_bottom
              <<" lane_center="<<s.lane_center<<" lane_horizon="<<s.lane_horizon<<" lane_vehicle_x="<<s.lane_vehicle_x
              <<" lane_contrast="<<s.lane_contrast<<"\n"
              <<"lane_trapezoid_roi_relative="<<s.lane_trap_tl<<','<<s.lane_trap_tr<<','<<s.lane_trap_bl<<','<<s.lane_trap_br<<"\n"
              <<"lane_debug_colors=white:white_paint,yellow:yellow_paint,grey:weak,purple:rejected,blue:occlusion,dark_blue:outside_road\n"
              <<"lane_scan_rows="<<lane.height<<" lane_paint="<<lane.paint_ratio
              <<" lane_occlusion="<<lane.occlusion_ratio<<" snapshot_mask_without_npu_occluders=1\n";
          meta.close();if(!meta) throw std::runtime_error("metadata write failed");
          runtime.snapshot_completed=request;
          std::cout<<"[SNAPSHOT] "<<folder.string()<<std::endl;
        } catch(const std::exception &e) {std::cerr<<"[SNAPSHOT] "<<e.what()<<std::endl;}
      }
    }
    for(int n=0;n<5 && !stop;++n) std::this_thread::sleep_for(std::chrono::milliseconds(20));
  }
}
}
