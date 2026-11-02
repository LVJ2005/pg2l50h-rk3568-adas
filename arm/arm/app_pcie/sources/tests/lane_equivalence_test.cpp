#include "lane_vision.h"
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
#include <cstring>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <numeric>

namespace {
uint64_t digest(const std::vector<uint8_t> &bytes) {
  uint64_t hash=14695981039346656037ull;
  for(uint8_t byte:bytes) {hash^=byte;hash*=1099511628211ull;}
  return hash;
}
void edge(std::ostream &out,const adas::LaneEdge &e) {
  out<<static_cast<int>(e.kind)<<' '<<static_cast<int>(e.paint)<<' '<<e.fitted<<' '
     <<std::hexfloat<<e.confidence<<' '<<e.coverage<<' '<<e.slope<<' '<<e.intercept<<' '<<e.curvature<<' '
     <<e.first_y<<' '<<e.last_y<<' '<<e.support_rows<<' '<<e.segment_count<<' '<<e.gap_count<<' '<<e.longest_gap<<' ';
}
cv::Mat quad(const cv::Mat &q1) {
  cv::Mat frame(720,1280,CV_8UC3);
  for(int k=0;k<4;++k) q1.copyTo(frame(cv::Rect(k%2*640,k/2*360,640,360)));
  return frame;
}
}
int main(int argc,char **argv) {
  if(argc!=4) {std::cerr<<"Usage: lane_equivalence_test DATASET TRACE_FILE TIMING_JSON\n";return 2;}
  try {
    cv::setNumThreads(1);
    cv::FileStorage refs(std::string(argv[1])+"/references.json",cv::FileStorage::READ);
    if(!refs.isOpened()) throw std::runtime_error("Missing references");
    std::ofstream out(argv[2]);
    if(!out) throw std::runtime_error("Cannot create trace");
    std::vector<double> elapsed;
    int count=0;
    for(const auto &sample:refs["samples"]) {
      const std::string name=static_cast<std::string>(sample["file"]);
      cv::Mat full,q1;const auto source=cv::imread(std::string(argv[1])+"/"+name);
      if(source.empty()) throw std::runtime_error("Missing image");
      cv::resize(source,full,{1280,720},0,0,cv::INTER_AREA);
      cv::resize(full,q1,{640,360},0,0,cv::INTER_NEAREST);
      for(int variant=0;variant<12;++variant) {
        adas::Settings s;s.infer_order="bgr";s.lane_debug=true;
        s.lane_scan_rows=std::array<int,3>{60,100,180}[variant%3];
        s.lane_contrast=std::array<int,4>{3,12,24,60}[variant/3];
        if(variant&1) {s.lane_trap_tl=.1f;s.lane_trap_tr=.9f;s.lane_horizon=.53f;}
        adas::LaneScanner scanner;
        for(int frame=0;frame<24;++frame) {
          cv::Mat moved;
          const cv::Mat transform=(cv::Mat_<double>(2,3)<<1,0,frame%5-2,0,1,frame%3-1);
          cv::warpAffine(q1,moved,transform,q1.size(),cv::INTER_LINEAR,cv::BORDER_REPLICATE);
          moved.convertTo(moved,CV_8U,.8+(frame%5)*.1);
          if(frame==12 || frame==13) cv::rectangle(moved,{180,235,170,55},{245,245,245},-1);
          if(frame==18 || frame==19) moved.setTo(cv::Scalar(55,55,55));
          std::vector<adas::Box> occluders;
          if(frame%7==0) occluders.push_back({330,230,400,320});
          const auto image=quad(moved);
          const double start=adas::now_ms();
          const auto result=adas::analyze_lane_image(image,s,scanner,occluders);
          elapsed.push_back(adas::now_ms()-start);
          out<<name<<' '<<variant<<' '<<frame<<' '<<result.width<<' '<<result.height<<' ';
          edge(out,result.left);edge(out,result.right);
          out<<std::hexfloat<<result.paint_ratio<<' '<<result.occlusion_ratio<<' '<<digest(result.mask)<<'\n';
          ++count;
        }
      }
    }
    // Independent masks exercise optional outputs, dense clutter and per-color caps.
    cv::RNG rng(0x225577);
    for(int k=0;k<120;++k) {
      cv::Mat small(60+k%3*40,320,CV_8UC3);
      rng.fill(small,cv::RNG::UNIFORM,0,256);
      if(k%2) cv::GaussianBlur(small,small,{9,9},2);
      std::vector<uint8_t> mask,weak,paint;
      adas::make_lane_mask(small,mask,3+k%58,k%3 ? &weak:nullptr,k%4 ? &paint:nullptr);
      out<<"mask "<<k<<' '<<digest(mask)<<' '<<digest(weak)<<' '<<digest(paint)<<'\n';
      adas::LaneScanner scanner;
      for(int f=0;f<3;++f) {
        const auto pair=scanner.detect(mask.data(),small.cols,small.rows,-20,.5,{},.5,
            weak.empty() ? nullptr:weak.data(),paint.empty() ? nullptr:paint.data());
        out<<"scan "<<k<<' '<<f<<' ';edge(out,pair.first);edge(out,pair.second);out<<'\n';
      }
    }
    std::sort(elapsed.begin(),elapsed.end());
    cv::FileStorage report(argv[3],cv::FileStorage::WRITE|cv::FileStorage::FORMAT_JSON);
    report<<"frames"<<count<<"host_mean_ms"<<std::accumulate(elapsed.begin(),elapsed.end(),0.0)/elapsed.size()
        <<"host_p50_ms"<<elapsed[elapsed.size()/2]<<"host_p95_ms"<<elapsed[elapsed.size()*95/100];
    std::cout<<"Exact-result trace: "<<count<<" image frames, 120 masks, 360 mask-scanner calls\n";
  } catch(const std::exception &e) {std::cerr<<e.what()<<'\n';return 1;}
}
