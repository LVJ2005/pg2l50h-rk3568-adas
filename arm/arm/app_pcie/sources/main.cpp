#include "frame_capture.h"
#include "frame_color.h"
#include "diagnostic_snapshot.h"
#include "lane_detector.h"
#include "lane_vision.h"
#include "rgb_display.h"
#include "traffic_detector.h"
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <csignal>
#include <cstdlib>
#include <iostream>
#include <sstream>
#include <ctime>

using namespace adas;
namespace {
volatile std::sig_atomic_t interrupted=0;
void on_signal(int) {interrupted=1;}
double thread_cpu_ms() {
  timespec value{};
  if(clock_gettime(CLOCK_THREAD_CPUTIME_ID,&value)!=0) return -1;
  return value.tv_sec*1000.0+value.tv_nsec/1000000.0;
}
std::string env(const char *key,const char *fallback) {
  const char *v=std::getenv(key);return v && *v ? v:fallback;
}
int env_int(const char *key,int fallback,int low,int high) {
  try {size_t used=0;const std::string text=env(key,"");const int n=std::stoi(text,&used);
    if(used==text.size() && n>=low && n<=high) return n;
  } catch(...) {}return fallback;
}
void npu_worker(LatestMailbox<Frame> &hub,Runtime &runtime,std::atomic<bool> &stop,const std::string &models) {
  TrafficPipeline pipeline(models);uint64_t seen=0,last_seq=0,last_generation=0;
  while(!stop) {
    auto frame=hub.wait(seen,stop);
    if(!frame) continue;
    const auto s=runtime.settings();
    if(!s.enabled) continue;
    if(s.generation!=last_generation) {last_generation=s.generation;last_seq=0;}
    if(frame->sequence-last_seq<static_cast<uint64_t>(s.infer_interval)) continue;
    try {
      pipeline.ensure_models(s);
      runtime.loaded(pipeline.det_profile(),pipeline.cls_profile());
      if(runtime.settings().generation!=s.generation) continue;
      // Loading a model may take time. Start with the newest immutable snapshot.
      if(auto latest=hub.try_read()) if(latest->sequence>frame->sequence) frame=std::move(latest);
      if(now_ms()-frame->captured_ms>s.result_ttl_ms) continue;
      if(last_seq && frame->sequence>last_seq+1) runtime.npu_skipped+=frame->sequence-last_seq-1;
      last_seq=frame->sequence;
      auto result=std::make_shared<Perception>(pipeline.infer(frame->bgr,s,frame->sequence,frame->captured_ms));
      runtime.publish_perception(result);
    } catch(const std::exception &e) {
      auto failed=std::make_shared<Perception>();
      failed->generation=s.generation;failed->frame_seq=frame->sequence;
      failed->captured_ms=frame->captured_ms;failed->completed_ms=now_ms();failed->error=e.what();
      runtime.publish_perception(failed);
      runtime.reload_failed(s.generation,pipeline.det_profile(),pipeline.cls_profile(),e.what());
      std::cerr<<"[NPU] "<<e.what()<<std::endl;
      // Backoff is local to this worker; display, capture, lane and UDP stay live.
      for(int n=0;n<20 && !stop;++n) std::this_thread::sleep_for(std::chrono::milliseconds(50));
    }
  }
}
void lane_worker(LatestMailbox<Frame> &hub,Runtime &runtime,std::atomic<bool> &stop) {
  LaneScanner scanner;uint64_t seen=0,generation=0;double due=0,last_sample=0;
  while(!stop) {
    const auto frame=hub.wait(seen,stop);if(!frame) continue;
    const auto s=runtime.settings();
    if(generation!=s.generation) {generation=s.generation;scanner.reset();due=0;}
    if(!s.enabled || !s.lane_enabled || now_ms()<due || now_ms()-frame->captured_ms>s.lane_ttl_ms) continue;
    if(last_sample && frame->captured_ms-last_sample>s.lane_ttl_ms) scanner.reset();
    last_sample=frame->captured_ms;
    due=now_ms()+1000.0/s.lane_hz;
    auto result=std::make_shared<LaneResult>();
    result->generation=s.generation;result->frame_seq=frame->sequence;result->captured_ms=frame->captured_ms;result->view=s.front_view;
    const double start=now_ms();
    const double cpu_start=thread_cpu_ms();
    try {
      std::vector<Box> occluders;
      const auto perception=runtime.perception();
      if(perception && perception->generation==s.generation && frame->captured_ms-perception->captured_ms>=0 &&
          frame->captured_ms-perception->captured_ms<s.result_ttl_ms) {
        for(const auto &d:perception->objects) {
          // Person boxes hide paint; crosswalk boxes often also contain both lanes.
          // Broad horizontal stripes are rejected by the scanner instead of erasing that ROI.
          if(d.class_id!=0 || d.view!=s.front_view) continue;
          occluders.push_back(d.box);
        }
      }
      auto analyzed=analyze_lane_image(frame->bgr,s,scanner,occluders);
      analyzed.frame_seq=frame->sequence;analyzed.captured_ms=frame->captured_ms;
      *result=std::move(analyzed);
    } catch(const std::exception &e) {result->error=e.what();scanner.reset();}
    result->completed_ms=now_ms();result->processing_ms=result->completed_ms-start;
    const double cpu_end=thread_cpu_ms();
    if(cpu_start>=0 && cpu_end>=cpu_start) result->cpu_ms=cpu_end-cpu_start;
    runtime.publish_lane(result);
  }
}
void text(cv::Mat &image,const std::string &value,int y,const cv::Scalar &color) {
  cv::putText(image,value,cv::Point(12,y),cv::FONT_HERSHEY_SIMPLEX,.6,cv::Scalar(0,0,0),3,cv::LINE_AA);
  cv::putText(image,value,cv::Point(12,y),cv::FONT_HERSHEY_SIMPLEX,.6,color,1,cv::LINE_AA);
}
void draw(cv::Mat &image,const Runtime &runtime) {
  const auto s=runtime.settings();const auto p=runtime.perception();const auto lane=runtime.lane();const double now=now_ms();
  if(s.enabled && p && p->generation==s.generation && now-p->captured_ms<=s.result_ttl_ms) {
    for(const auto &d:p->objects) {
      if(!s.draw_all && d.view!=s.front_view) continue;
      const cv::Scalar color=d.class_id==2 ? cv::Scalar(0,200,255):d.class_id==1 ? cv::Scalar(255,180,0):cv::Scalar(50,230,80);
      cv::rectangle(image,cv::Point(static_cast<int>(d.box.x1),static_cast<int>(d.box.y1)),
          cv::Point(static_cast<int>(d.box.x2),static_cast<int>(d.box.y2)),color,2);
      const std::string label=d.class_id==2 ? light_name(d.light):d.class_id==1 ? "crosswalk":"person";
      cv::putText(image,label,cv::Point(static_cast<int>(d.box.x1),std::max(18,static_cast<int>(d.box.y1)-5)),
          cv::FONT_HERSHEY_SIMPLEX,.45,color,1,cv::LINE_AA);
    }
  }
  if(s.enabled && lane && lane->generation==s.generation && now-lane->captured_ms<=s.lane_ttl_ms) {
    if(s.lane_debug) {
      auto p=[&](float x,float y) {return cv::Point(std::lround(lane->roi.x1+x*lane->roi.width()),
          std::lround(lane->roi.y1+y*lane->roi.height()));};
      const std::vector<cv::Point> polygon={p(s.lane_trap_tl,0),p(s.lane_trap_tr,0),p(s.lane_trap_br,1),p(s.lane_trap_bl,1)};
      cv::polylines(image,polygon,true,cv::Scalar(255,255,0),1);
    }
    for(const LaneEdge *e:{&lane->left,&lane->right}) if(e->fitted) {
      const auto point=[&](int y){return cv::Point(static_cast<int>(lane->roi.x1+e->x_at(y)*lane->roi.width()/lane->width),
          static_cast<int>(lane->roi.y1+y*lane->roi.height()/lane->height));};
      const auto color=e->kind==LineKind::Solid ? cv::Scalar(0,0,255):e->kind==LineKind::Dashed ? cv::Scalar(0,220,0):cv::Scalar(160,160,160);
      for(int y=e->first_y;y<e->last_y;y+=4) cv::line(image,point(y),point(std::min(y+4,e->last_y)),color,2);
    }
    std::ostringstream info;info<<"Lane view="<<s.front_view<<" L:"<<line_name(lane->left.kind)
      <<" R:"<<line_name(lane->right.kind)<<" "<<static_cast<int>(lane->processing_ms)<<"ms";
    text(image,info.str(),image.rows-16,cv::Scalar(255,255,255));
  }
  const auto decision=runtime.decision();const auto intent=runtime.intent();
  const auto lane_intent=runtime.lane_intent();
  const auto signal_decision=runtime.signal_decision();const auto lane_decision=runtime.lane_decision();
  const auto pedestrian_decision=runtime.pedestrian_decision();
  const auto color=decision.warning ? cv::Scalar(0,0,255):decision.state=="allow" ? cv::Scalar(0,220,0):cv::Scalar(0,210,255);
  text(image,"ADAS " + std::string(s.enabled ? "ON":"OFF")+" | "+maneuver_name(intent.action)+" | "+decision.state,28,color);
  text(image,decision.reason,53,color);
  text(image,"Signal: "+signal_decision.state+" | Lane "+maneuver_name(lane_intent.action)+
       ": "+lane_decision.state,78,color);
  if(pedestrian_decision.state=="stop")
    text(image,"PEDESTRIAN ON CROSSWALK | BRAKE REQUEST",103,cv::Scalar(0,0,255));
  if(p && !p->error.empty()) text(image,"NPU: "+p->error.substr(0,85),128,cv::Scalar(0,0,255));
}
}
int main(int argc,char **argv) {
  std::string models=env("ADAS_MODEL_DIR","../models"),video;
  bool display=env_int("ADAS_DISPLAY",1,0,1)!=0;
  int seconds=0;
  for(int n=1;n<argc;++n) {
    const std::string arg=argv[n];
    if(arg=="--models" && n+1<argc) models=argv[++n];
    else if(arg=="--video" && n+1<argc) video=argv[++n];
    else if(arg=="--headless") display=false;
    else if(arg=="--seconds" && n+1<argc) {try{seconds=std::stoi(argv[++n]);}catch(...){return 2;}}
    else {std::cout<<"Usage: app [--models DIR] [--video QUAD_VIDEO] [--headless] [--seconds N]\n";return arg=="--help" ? 0:2;}
  }
  if(env_int("FPGA_BGR888",1,0,1)!=1) {std::cerr<<"Requires FPGA BGR888 full-frame DMA\n";return 2;}
  cv::setNumThreads(1);
  Settings settings;settings.det_profile=env("ADAS_DET_PROFILE","fp");settings.cls_profile=env("ADAS_CLS_PROFILE","fp");
  if((settings.det_profile!="fp" && settings.det_profile!="int8") || (settings.cls_profile!="fp" && settings.cls_profile!="int8")) return 2;
  settings.enabled=env("ADAS_MODE","assist")!="off";
  settings.infer_interval=env_int("ADAS_INFER_INTERVAL",1,1,30);settings.lane_hz=env_int("ADAS_LANE_HZ",8,1,20);
  settings.front_view=0;settings.lane_enabled=env_int("ADAS_LANE_ENABLED",1,0,1);
  settings.signal_confirmed=env_int("ADAS_SIGNAL_CONFIRMED",1,0,1);
  settings.lane_debug=env_int("ADAS_LANE_DEBUG",0,0,1);
  settings.lane_contrast=env_int("ADAS_LANE_CONTRAST",12,3,60);
  settings.lane_scan_rows=env_int("ADAS_LANE_SCAN_ROWS",100,60,180);
  settings.display_backend=env("ADAS_DISPLAY_BACKEND","opencv");
  if(settings.display_backend!="opencv" && settings.display_backend!="gtk_rgb") {
    std::cerr<<"ADAS_DISPLAY_BACKEND must be opencv or gtk_rgb\n";return 2;
  }
  const std::string camera_order=env("ADAS_CAMERA_ORDER","rbg");
  settings.view_order=env("ADAS_VIEW_ORDER",camera_order.c_str());
  settings.infer_order=env("ADAS_INFER_ORDER",camera_order.c_str());
  if(!valid_camera_order(settings.view_order)) {std::cerr<<"Invalid ADAS_VIEW_ORDER\n";return 2;}
  if(!valid_camera_order(settings.infer_order)) {std::cerr<<"Invalid ADAS_INFER_ORDER\n";return 2;}
  settings.color_compare=env_int("ADAS_COLOR_COMPARE",0,0,1);
  std::cout<<"[BUILD] ADAS-quad-V2.0.10 | full-frame hardware-MSI capture | lane="<<lane_execution_backend()<<"\n"
    <<"[INPUT] detect=Q1(0,0,640,360) pad=12/12 input=640x384 scale=1 display=1280x720\n"
    <<"[COLOR] DMA unchanged display="<<settings.display_backend<<" view_order="<<settings.view_order
    <<" infer_order="<<settings.infer_order<<" model=RGB\n";
  settings.result_ttl_ms=env_int("ADAS_RESULT_TTL_MS",1000,100,5000);
  Runtime runtime;runtime.configure(settings);LatestMailbox<Frame> hub;std::atomic<bool> stop{false};
  ControlServer control(runtime,env("RECOGNITION_CONTROL_BIND","0.0.0.0"),env_int("RECOGNITION_CONTROL_PORT",9527,1,65535));
  std::string error;if(!control.start(error)) {std::cerr<<error<<std::endl;return 1;}
  std::signal(SIGINT,on_signal);std::signal(SIGTERM,on_signal);
  std::thread capture(capture_frames,std::ref(hub),std::ref(runtime),std::ref(stop),video,env_int("PCIE_USE_MMAP",1,0,1)!=0);
  std::thread npu(npu_worker,std::ref(hub),std::ref(runtime),std::ref(stop),models);
  std::thread lane(lane_worker,std::ref(hub),std::ref(runtime),std::ref(stop));
  std::thread diagnostic(diagnostic_worker,std::ref(hub),std::ref(runtime),std::ref(stop),env("ADAS_DEBUG_DIR","adas_debug"));
  const double started=now_ms();double report=started,last_redraw=0;uint64_t last_seq=0,last_frames=0;
  std::shared_ptr<const Frame> visible;
  std::unique_ptr<RgbDisplay> screen,debug_screen,color_screen;
  double last_colors=0;
  uint64_t debug_seq=0,debug_generation=0;
  try {
    if(display) screen=std::make_unique<RgbDisplay>("ADAS",settings.display_backend);
    while(!interrupted && !stop) {
      const double now=now_ms();if(seconds>0 && now-started>seconds*1000.0) break;
      if(auto latest=hub.try_read()) visible=std::move(latest);
      if(display && visible && (visible->sequence!=last_seq || now-last_redraw>=100)) {
        const auto config=runtime.settings();
        cv::Mat preview=camera_display_bgr(visible->bgr,config.view_order);draw(preview,runtime);
        if(now-visible->captured_ms>settings.result_ttl_ms) text(preview,"VIDEO STALE",128,cv::Scalar(0,0,255));
        if(config.lane_debug) {
          const cv::Scalar colors[]={cv::Scalar(0,0,255),cv::Scalar(0,255,0),cv::Scalar(255,0,0)};
          for(int k=0;k<3;++k) {
            cv::rectangle(preview,cv::Rect(preview.cols-180+k*60,8,56,24),colors[k],cv::FILLED);
            cv::putText(preview,std::string(1,"RGB"[k]),cv::Point(preview.cols-165+k*60,26),
                cv::FONT_HERSHEY_SIMPLEX,.5,cv::Scalar(255,255,255),1);
          }
        }
        screen->show_bgr(std::move(preview));
        last_seq=visible->sequence;last_redraw=now;
      }
      if(display) {
        const int key=screen->poll();
        if(screen->closed() || key=='q' || key=='Q' || key==27 || key==0xff1b) break;
        if(key=='0' || key=='1') {std::string reply;runtime.command(key=='0' ? "MODE off":"MODE assist",reply);}
        if(key=='d' || key=='D') {std::string reply;runtime.command(runtime.settings().lane_debug ? "SET lane_debug 0":"SET lane_debug 1",reply);}
        if(key=='c' || key=='C') {std::string reply;runtime.command(runtime.settings().color_compare ? "SET color_compare 0":"SET color_compare 1",reply);}
        if(key=='s' || key=='S') {std::string reply;runtime.command("SNAPSHOT",reply);}
        const auto lane_result=runtime.lane();
        const auto debug_settings=runtime.settings();
        if(debug_settings.enabled && debug_settings.lane_enabled && debug_settings.lane_debug && lane_result &&
            lane_result->generation==debug_settings.generation && now-lane_result->captured_ms<=debug_settings.lane_ttl_ms && !lane_result->mask.empty()) {
          if(!debug_screen) debug_screen=std::make_unique<RgbDisplay>("Lane - white/yellow paint, grey weak, purple rejected, blue occlusion",settings.display_backend);
          if(!debug_screen->closed() && (lane_result->frame_seq!=debug_seq || lane_result->generation!=debug_generation)) {
            cv::Mat mask_rgb(lane_result->height,lane_result->width,CV_8UC3);
            for(int y=0;y<mask_rgb.rows;++y) for(int x=0;x<mask_rgb.cols;++x) {
              const int v=lane_result->mask[y*mask_rgb.cols+x];
              mask_rgb.at<cv::Vec3b>(y,x)=lane_debug_rgb(v);
            }
            for(const LaneEdge *edge:{&lane_result->left,&lane_result->right}) if(edge->fitted) {
              const cv::Scalar color=edge->kind==LineKind::Solid ? cv::Scalar(255,60,60):
                  edge->kind==LineKind::Dashed ? cv::Scalar(0,255,0):cv::Scalar(160,160,160);
              for(int y=edge->first_y;y<edge->last_y;++y)
                cv::line(mask_rgb,{static_cast<int>(edge->x_at(y)),y},
                    {static_cast<int>(edge->x_at(y+1)),y+1},color,1);
            }
            debug_screen->show_rgb(std::move(mask_rgb));
            debug_seq=lane_result->frame_seq;debug_generation=lane_result->generation;
          }
        } else {debug_screen.reset();debug_seq=0;debug_generation=0;}
        if(debug_settings.color_compare && visible) {
          if(!color_screen) color_screen=std::make_unique<RgbDisplay>("Camera channel candidates",settings.display_backend);
          if(!color_screen->closed() && now-last_colors>=500) {
            const Box view=view_box(debug_settings.front_view);
            color_screen->show_bgr(color_comparison(visible->bgr(cv::Rect(view.x1,view.y1,640,360))));
            last_colors=now;
          }
        } else color_screen.reset();
      }
      if(now-report>=1000) {
        const uint64_t frames=runtime.frames.load();
        std::cout<<"FPS "<<(frames-last_frames)*1000.0/(now-report)<<" | "<<runtime.status()
          <<" mailbox_busy_drop="<<hub.rejected()<<std::endl;
        report=now;last_frames=frames;
      }
      std::this_thread::sleep_for(std::chrono::milliseconds(display ? 3:20));
    }
  } catch(const std::exception &e) {std::cerr<<"[DISPLAY] "<<e.what()<<std::endl;}
  stop=true;hub.wake();
  if(capture.joinable()) capture.join();
  if(npu.joinable()) npu.join();
  if(lane.joinable()) lane.join();
  if(diagnostic.joinable()) diagnostic.join();
  control.stop();return 0;
}
