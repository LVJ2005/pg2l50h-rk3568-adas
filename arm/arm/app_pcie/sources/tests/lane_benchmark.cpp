#include "lane_vision.h"
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
#include <iostream>
#include <numeric>
#include <ctime>

namespace {
double cpu_ms() {
  timespec t{};
  if(clock_gettime(CLOCK_THREAD_CPUTIME_ID,&t)!=0) return -1;
  return t.tv_sec*1000.0+t.tv_nsec/1000000.0;
}
void stats(cv::FileStorage &out,const std::string &name,std::vector<double> values) {
  std::sort(values.begin(),values.end());
  out<<name<<"{"<<"mean"<<std::accumulate(values.begin(),values.end(),0.0)/values.size()
     <<"p50"<<values[values.size()/2]<<"p95"<<values[values.size()*95/100]<<"}";
}
}
int main(int argc,char **argv) {
  if(argc<3 || argc>5) {std::cerr<<"Usage: lane_benchmark DATASET REPORT_JSON [repeats=60] [scan_rows=100]\n";return 2;}
  try {
    const int repeats=argc>3 ? std::stoi(argv[3]):60;
    const int rows=argc>4 ? std::stoi(argv[4]):100;
    if(repeats<1 || repeats>10000 || rows<60 || rows>180) return 2;
    cv::setNumThreads(1);
    cv::FileStorage refs(std::string(argv[1])+"/references.json",cv::FileStorage::READ);
    if(!refs.isOpened() || refs["samples"].empty()) throw std::runtime_error("Missing dataset references");
    std::vector<cv::Mat> frames;
    for(const auto &sample:refs["samples"]) {
      const auto source=cv::imread(std::string(argv[1])+"/"+static_cast<std::string>(sample["file"]));
      if(source.empty()) throw std::runtime_error("Missing source image");
      cv::Mat full,q1;cv::resize(source,full,{1280,720},0,0,cv::INTER_AREA);
      cv::resize(full,q1,{640,360},0,0,cv::INTER_NEAREST);
      cv::Mat frame(720,1280,CV_8UC3);
      for(int k=0;k<4;++k) q1.copyTo(frame(cv::Rect(k%2*640,k/2*360,640,360)));
      frames.push_back(frame);
    }
    adas::Settings s;s.infer_order="bgr";s.lane_scan_rows=rows;
    std::vector<adas::LaneScanner> scanners(frames.size());
    std::vector<double> wall,cpu,pre,mask,seed,fit;
    int fitted=0;
    for(int round=-3;round<repeats;++round) for(size_t k=0;k<frames.size();++k) {
      const double c0=cpu_ms(),t0=adas::now_ms();
      const auto result=adas::analyze_lane_image(frames[k],s,scanners[k]);
      const double t1=adas::now_ms(),c1=cpu_ms();
      if(round<0) continue;
      wall.push_back(t1-t0);cpu.push_back(c0>=0 && c1>=c0 ? c1-c0:-1);
      fitted+=result.left.fitted+result.right.fitted;
#ifdef ADAS_LANE_TIMINGS
      pre.push_back(result.pre_ms);mask.push_back(result.mask_ms);
      seed.push_back(result.seed_ms);fit.push_back(result.fit_ms);
#endif
    }
    cv::FileStorage report(argv[2],cv::FileStorage::WRITE|cv::FileStorage::FORMAT_JSON);
    if(!report.isOpened()) throw std::runtime_error("Cannot write report");
    report<<"scope"<<"offline lane-only CPU; excludes capture, NPU and display contention"
        <<"opencv"<<CV_VERSION<<"scan_rows"<<rows<<"frames"<<static_cast<int>(wall.size())<<"fitted_edges"<<fitted;
    stats(report,"wall_ms",wall);stats(report,"cpu_ms",cpu);
#ifdef ADAS_LANE_TIMINGS
    report<<"backend"<<adas::lane_execution_backend();
    stats(report,"pre_ms",pre);stats(report,"mask_ms",mask);stats(report,"seed_ms",seed);stats(report,"fit_ms",fit);
#endif
    std::cout<<"Lane benchmark: "<<wall.size()<<" frames; report "<<argv[2]<<'\n';
  } catch(const std::exception &e) {std::cerr<<e.what()<<'\n';return 1;}
}
