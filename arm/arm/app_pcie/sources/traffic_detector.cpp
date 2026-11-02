#include "traffic_detector.h"
#include "detector_decode.h"
#include "frame_color.h"
#include "rknn_api.h"
#include <opencv2/imgproc.hpp>
#include <fstream>
#include <iostream>
#include <stdexcept>

namespace adas {
class RknnModel {
public:
  rknn_context context=0;
  std::vector<rknn_tensor_attr> outputs;
  int width=0,height=0;
  RknnModel(const std::string &path,bool is_detector) {
    try {
      std::ifstream stream(path,std::ios::binary|std::ios::ate);
      if(!stream) throw std::runtime_error("cannot open model: "+path);
      const auto bytes=stream.tellg();
      if(bytes<=0 || bytes>128*1024*1024) throw std::runtime_error("invalid model file size");
      std::vector<uint8_t> model(static_cast<size_t>(bytes));stream.seekg(0);
      if(!stream.read(reinterpret_cast<char*>(model.data()),model.size())) throw std::runtime_error("model read failed");
      if(rknn_init(&context,model.data(),model.size(),0,nullptr)<0) throw std::runtime_error("rknn_init failed: "+path);
      rknn_input_output_num counts{};
      if(rknn_query(context,RKNN_QUERY_IN_OUT_NUM,&counts,sizeof(counts))<0 ||
          counts.n_input!=1 || counts.n_output!=(is_detector ? 9u:1u)) throw std::runtime_error("model input/output count mismatch");
      rknn_tensor_attr input{};
      if(rknn_query(context,RKNN_QUERY_INPUT_ATTR,&input,sizeof(input))<0 || input.n_dims!=4 || input.dims[0]!=1)
        throw std::runtime_error("invalid input tensor");
      int channels;
      if(input.fmt==RKNN_TENSOR_NCHW) {channels=input.dims[1];height=input.dims[2];width=input.dims[3];}
      else if(input.fmt==RKNN_TENSOR_NHWC) {height=input.dims[1];width=input.dims[2];channels=input.dims[3];}
      else throw std::runtime_error("unsupported input layout");
      if(channels!=3 || width!=(is_detector ? 640:128) || height!=(is_detector ? 384:128))
        throw std::runtime_error("model input geometry mismatch");
      outputs.resize(counts.n_output);
      for(unsigned i=0;i<counts.n_output;++i) {
        outputs[i].index=i;
        if(rknn_query(context,RKNN_QUERY_OUTPUT_ATTR,&outputs[i],sizeof(outputs[i]))<0) throw std::runtime_error("output query failed");
        if(is_detector) {
          const int h=48>>(i/3),w=80>>(i/3),c=i%3==0 ? 64:i%3==1 ? 3:1;
          layout(outputs[i],c,h,w);
        } else if(outputs[i].n_dims!=2 || outputs[i].dims[0]!=1 || outputs[i].dims[1]!=8)
          throw std::runtime_error("classifier expected [1,8] output");
      }
      std::cout<<"[MODEL] "<<path<<" input="<<width<<'x'<<height<<" outputs="<<counts.n_output<<std::endl;
    } catch(...) {if(context) {rknn_destroy(context);context=0;}throw;}
  }
  ~RknnModel() {if(context) rknn_destroy(context);}
  static bool layout(const rknn_tensor_attr &a,int c,int h,int w) {
    if(a.n_dims!=4 || a.dims[0]!=1 || a.n_elems!=static_cast<unsigned>(c*h*w))
      throw std::runtime_error("detector output shape mismatch");
    if(a.dims[1]==static_cast<unsigned>(c) && a.dims[2]==static_cast<unsigned>(h) && a.dims[3]==static_cast<unsigned>(w)) return false;
    if(a.dims[1]==static_cast<unsigned>(h) && a.dims[2]==static_cast<unsigned>(w) && a.dims[3]==static_cast<unsigned>(c)) return true;
    throw std::runtime_error("detector output grid/layout mismatch");
  }
};
namespace {
class ModelOutput {
public:
  RknnModel &model;
  std::vector<rknn_output> values;
  bool acquired=false;
  ModelOutput(RknnModel &m,const cv::Mat &rgb):model(m),values(m.outputs.size()) {
    if(!rgb.isContinuous() || rgb.type()!=CV_8UC3 || rgb.cols!=m.width || rgb.rows!=m.height)
      throw std::runtime_error("invalid RGB input buffer");
    rknn_input input{};input.index=0;input.type=RKNN_TENSOR_UINT8;input.fmt=RKNN_TENSOR_NHWC;
    input.pass_through=0;input.buf=const_cast<uint8_t*>(rgb.data);input.size=rgb.total()*rgb.elemSize();
    if(rknn_inputs_set(m.context,1,&input)<0 || rknn_run(m.context,nullptr)<0) throw std::runtime_error("RKNN inference failed");
    for(unsigned i=0;i<values.size();++i) {values[i].index=i;values[i].want_float=1;}
    if(rknn_outputs_get(m.context,values.size(),values.data(),nullptr)<0) throw std::runtime_error("RKNN output acquisition failed");
    acquired=true;
    for(unsigned i=0;i<values.size();++i) {
      if(!values[i].buf || values[i].size<m.outputs[i].n_elems*sizeof(float)) {
        rknn_outputs_release(m.context,values.size(),values.data());acquired=false;
        throw std::runtime_error("RKNN output buffer too small");
      }
    }
  }
  ~ModelOutput() {if(acquired) rknn_outputs_release(model.context,values.size(),values.data());}
};
}
cv::Mat make_detector_input(const cv::Mat &frame,const std::string &camera_order) {
  if(frame.type()!=CV_8UC3 || frame.cols!=1280 || frame.rows!=720) throw std::runtime_error("expected 1280x720 quad frame");
  const cv::Mat first=frame(cv::Rect(0,0,640,360));
  cv::Mat boxed;
  // Isolate the ROI: bottom padding must not borrow pixels from the third quadrant.
  cv::copyMakeBorder(first,boxed,12,12,0,0,cv::BORDER_CONSTANT|cv::BORDER_ISOLATED,cv::Scalar(114,114,114));
  return camera_model_rgb(boxed,camera_order);
}
cv::Mat make_classifier_input(const cv::Mat &frame,const Box &box,const std::string &camera_order) {
  if(frame.type()!=CV_8UC3 || frame.cols!=1280 || frame.rows!=720) throw std::runtime_error("expected 1280x720 quad frame");
  const int left=std::max(0,static_cast<int>(std::floor(box.x1)));
  const int top=std::max(0,static_cast<int>(std::floor(box.y1)));
  const int right=std::min(640,static_cast<int>(std::ceil(box.x2)));
  const int bottom=std::min(360,static_cast<int>(std::ceil(box.y2)));
  if(right<=left || bottom<=top) throw std::runtime_error("empty light crop");
  const auto roi=frame(cv::Rect(left,top,right-left,bottom-top));
  const double scale=std::min(128.0/roi.cols,128.0/roi.rows);
  const int w=std::clamp(static_cast<int>(std::nearbyint(roi.cols*scale)),1,128);
  const int h=std::clamp(static_cast<int>(std::nearbyint(roi.rows*scale)),1,128);
  cv::Mat resized,boxed(128,128,CV_8UC3,cv::Scalar(114,114,114));
  cv::resize(roi,resized,cv::Size(w,h),0,0,cv::INTER_LINEAR);
  resized.copyTo(boxed(cv::Rect((128-w)/2,(128-h)/2,w,h)));
  return camera_model_rgb(boxed,camera_order);
}
TrafficPipeline::TrafficPipeline(std::string directory):directory_(std::move(directory)) {}
TrafficPipeline::~TrafficPipeline()=default;
void TrafficPipeline::ensure_models(const Settings &s) {
  if(s.det_profile==det_profile_ && s.cls_profile==cls_profile_) return;
  const std::string det=directory_+"/det-v6-640x384-"+(s.det_profile=="fp" ? "fp":"i8")+".rknn";
  const std::string cls=directory_+"/cls-v1.0-128-"+(s.cls_profile=="fp" ? "fp":"i8")+".rknn";
  std::unique_ptr<RknnModel> new_det,new_cls;
  if(s.det_profile!=det_profile_) new_det=std::make_unique<RknnModel>(det,true);
  if(s.cls_profile!=cls_profile_) new_cls=std::make_unique<RknnModel>(cls,false);
  // Commit the pair only after both requested contexts pass validation.
  if(new_det) detector_=std::move(new_det);
  if(new_cls) classifier_=std::move(new_cls);
  det_profile_=s.det_profile;cls_profile_=s.cls_profile;
}
Perception TrafficPipeline::infer(const cv::Mat &frame,const Settings &s,uint64_t seq,double captured) {
  if(!detector_ || !classifier_) throw std::runtime_error("traffic models not loaded");
  Perception p;p.frame_seq=seq;p.generation=s.generation;p.captured_ms=captured;
  const double start=now_ms();p.queue_ms=start-captured;
  const cv::Mat rgb=make_detector_input(frame,s.infer_order);p.pre_ms=now_ms()-start;
  {
    const double t=now_ms();ModelOutput outputs(*detector_,rgb);p.det_ms=now_ms()-t;
    std::array<DetectorHead,3> heads;
    for(int n=0;n<3;++n) {
      auto &h=heads[n];h.height=48>>n;h.width=80>>n;
      h.boxes=static_cast<float*>(outputs.values[n*3].buf);
      h.scores=static_cast<float*>(outputs.values[n*3+1].buf);
      h.boxes_nhwc=RknnModel::layout(detector_->outputs[n*3],64,h.height,h.width);
      h.scores_nhwc=RknnModel::layout(detector_->outputs[n*3+1],3,h.height,h.width);
    }
    const double post=now_ms();p.objects=decode_heads(heads,s);p.post_ms=now_ms()-post;
  }
  std::vector<size_t> lights;
  for(size_t n=0;n<p.objects.size();++n) if(p.objects[n].class_id==2) lights.push_back(n);
  std::stable_sort(lights.begin(),lights.end(),[&](size_t a,size_t b){
    return (p.objects[a].view==s.front_view)>(p.objects[b].view==s.front_view);
  });
  const double classify=now_ms();
  for(const size_t index:lights) {
    if(p.cls_runs>=s.max_lights) break;
    auto &d=p.objects[index];const Box view=view_box(0);
    // A crop spanning a quad seam has no single-camera interpretation.
    if(d.view!=0 || d.box.x1<view.x1 || d.box.x2>view.x2 || d.box.y1<view.y1 || d.box.y2>view.y2) continue;
    const cv::Mat crop=make_classifier_input(frame,d.box,s.infer_order);
    ModelOutput output(*classifier_,crop);
    const auto scores=static_cast<const float*>(output.values[0].buf);
    int best=0;
    for(int c=0;c<8;++c) {
      if(!std::isfinite(scores[c])) throw std::runtime_error("non-finite classifier output");
      if(scores[c]>scores[best]) best=c;
    }
    d.light=static_cast<Light>(best);d.light_score=scores[best];++p.cls_runs;
  }
  p.cls_ms=now_ms()-classify;p.completed_ms=now_ms();return p;
}
} // namespace adas
