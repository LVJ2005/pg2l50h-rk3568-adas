#include "lane_vision.h"
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
#include <filesystem>
#include <iostream>
#include <numeric>

namespace {
cv::Mat quad(const cv::Mat &q1) {
  cv::Mat frame(720,1280,CV_8UC3);
  for(int k=0;k<4;++k) q1.copyTo(frame(cv::Rect(k%2*640,k/2*360,640,360)));
  return frame;
}
float x_at(const adas::LaneEdge &edge,const adas::LaneResult &lane,float y) {
  return lane.roi.x1+edge.x_at((y-lane.roi.y1)*lane.height/lane.roi.height())*lane.roi.width()/lane.width;
}
double mean(const std::vector<double> &v) {return v.empty() ? -1:std::accumulate(v.begin(),v.end(),0.0)/v.size();}
double deviation(const std::vector<double> &v) {
  if(v.empty()) return -1;
  const double m=mean(v);double sum=0;for(double value:v) sum+=(value-m)*(value-m);
  return std::sqrt(sum/v.size());
}
}
int main(int argc,char **argv) {
  if(argc<3 || argc>4) {std::cerr<<"Usage: lane_dataset_test DATASET_DIR REPORT_JSON [--assert]\n";return 2;}
  try {
    cv::setNumThreads(1);
    const std::filesystem::path folder(argv[1]);
    cv::FileStorage refs((folder/"references.json").string(),cv::FileStorage::READ);
    if(!refs.isOpened()) throw std::runtime_error("Missing references.json");
    if(refs["samples"].type()!=cv::FileNode::SEQ || refs["samples"].empty()) throw std::runtime_error("Empty sample list");
    cv::FileStorage report(argv[2],cv::FileStorage::WRITE|cv::FileStorage::FORMAT_JSON);
    if(!report.isOpened()) throw std::runtime_error("Cannot write report");
    report<<"measurement_scope"<<"host CPU and synthetic frame perturbations; not board measurements or real video accuracy";
    report<<"samples"<<"[";
    int localized=0,correct=0,wrong=0,unknown=0,total=0;
    int sequence_wrong=0,sequence_good=0,sequence_total=0;double max_jitter=0;
    std::vector<double> timings;
    for(const auto &sample:refs["samples"]) {
      const std::string file=static_cast<std::string>(sample["file"]);
      const cv::Mat source=cv::imread((folder/file).string());
      if(source.empty()) throw std::runtime_error("Cannot read "+file);
      cv::Mat full,q1;cv::resize(source,full,{1280,720},0,0,cv::INTER_AREA);
      cv::resize(full,q1,{640,360},0,0,cv::INTER_NEAREST);
      adas::Settings settings;settings.infer_order="bgr";
      adas::LaneScanner scanner;adas::LaneResult still;
      for(int k=0;k<3;++k) still=adas::analyze_lane_image(quad(q1),settings,scanner);
      report<<"{"<<"file"<<file;
      for(int side=0;side<2;++side) {
        const std::string key=side==0 ? "left":"right";
        const std::string expected=static_cast<std::string>(sample[key]);
        const auto &edge=side==0 ? still.left:still.right;
        std::vector<float> points;sample[key+"_points"]>>points;
        std::vector<double> errors;
        if(edge.fitted) for(size_t k=0;k<points.size();k+=2) errors.push_back(std::abs(x_at(edge,still,points[k+1])-points[k]));
        const double error=mean(errors);const std::string actual=adas::line_name(edge.kind);
        ++total;if(edge.fitted && error<=12) ++localized;
        if(actual==expected) ++correct;else if(actual=="unknown") ++unknown;else ++wrong;
        report<<key<<"{"<<"expected"<<expected<<"actual"<<actual<<"localized"<<static_cast<int>(edge.fitted && error<=12)
            <<"reference_mae_q1_px"<<error<<"}";
      }
      scanner.reset();std::vector<double> residuals[2];int valid[2]{0,0},matches[2]{0,0},wrong_frames[2]{0,0},unknown_frames[2]{0,0};
      for(int k=0;k<36;++k) {
        const int dx=(k%5)-2,dy=(k%3)-1;
        cv::Mat moved,modified;const cv::Mat transform=(cv::Mat_<double>(2,3)<<1,0,dx,0,1,dy);
        cv::warpAffine(q1,moved,transform,q1.size(),cv::INTER_LINEAR,cv::BORDER_REPLICATE);
        moved.convertTo(modified,CV_8U,.9+(k%5)*.05);
        const auto frame=quad(modified);const double start=adas::now_ms();
        const auto result=adas::analyze_lane_image(frame,settings,scanner);timings.push_back(adas::now_ms()-start);
        if(k<5) continue;
        for(int side=0;side<2;++side) {
          const std::string key=side==0 ? "left":"right";
          const auto &edge=side==0 ? result.left:result.right;
          if(!edge.fitted) continue;
          std::vector<float> points;sample[key+"_points"]>>points;
          ++valid[side];
          const std::string actual=adas::line_name(edge.kind);
          if(actual==static_cast<std::string>(sample[key])) ++matches[side];
          else if(actual=="unknown") ++unknown_frames[side];else ++wrong_frames[side];
          residuals[side].push_back(x_at(edge,result,points[1]+dy)-points[0]-dx);
        }
      }
      for(int side=0;side<2;++side) {
        const std::string key=side==0 ? "left_sequence":"right_sequence";
        report<<key<<"{"<<"fitted_frames"<<valid[side]<<"correct_type_frames"<<matches[side]
            <<"wrong_type_frames"<<wrong_frames[side]<<"unknown_type_frames"<<unknown_frames[side]
            <<"residual_std_q1_px"<<deviation(residuals[side])<<"}";
        sequence_wrong+=wrong_frames[side];sequence_good+=matches[side];sequence_total+=31;
        max_jitter=std::max(max_jitter,deviation(residuals[side]));
      }
      const cv::Mat blank(720,1280,CV_8UC3,cv::Scalar(55,55,55));
      const auto empty=adas::analyze_lane_image(blank,settings,scanner);
      if(empty.left.fitted || empty.right.fitted) throw std::runtime_error("Ghost boundary after blank frame");
      report<<"}";
    }
    std::sort(timings.begin(),timings.end());
    report<<"]"<<"static_edges"<<total<<"localized_edges"<<localized<<"correct_types"<<correct
        <<"wrong_types"<<wrong<<"unknown_types"<<unknown<<"host_p50_ms"<<timings[timings.size()/2]
        <<"host_p95_ms"<<timings[timings.size()*95/100]<<"sequence_wrong_types"<<sequence_wrong
        <<"sequence_correct_types"<<sequence_good<<"sequence_total_edges"<<sequence_total
        <<"max_residual_std_q1_px"<<max_jitter;report.release();
    std::cout<<"Real-image sample: localized="<<localized<<"/"<<total<<" correct="<<correct
        <<" wrong="<<wrong<<" unknown="<<unknown<<" (not a dataset accuracy benchmark)\n";
    if(argc==4 && (localized<11 || wrong>0 || correct<11 || sequence_wrong>0 ||
        sequence_good<sequence_total*.9 || max_jitter>4)) return 1;
    return 0;
  } catch(const std::exception &error) {std::cerr<<error.what()<<"\n";return 1;}
}
