#include "detector_decode.h"
#include <stdexcept>

namespace adas {
namespace {
float at(const float *p,int channels,int cells,int cell,int channel,bool nhwc) {
  return p[nhwc ? cell*channels+channel:channel*cells+cell];
}
float dfl(const DetectorHead &head,int cell,int side) {
  const int count=head.height*head.width;
  float peak=-1e30f;
  for(int k=0;k<16;++k) {
    const float v=at(head.boxes,64,count,cell,side*16+k,head.boxes_nhwc);
    if(!std::isfinite(v)) throw std::runtime_error("non-finite DFL output");
    peak=std::max(peak,v);
  }
  float sum=0,weighted=0;
  for(int k=0;k<16;++k) {
    const float p=std::exp(at(head.boxes,64,count,cell,side*16+k,head.boxes_nhwc)-peak);
    sum+=p;weighted+=k*p;
  }
  return weighted/sum;
}
}
std::vector<Detection> decode_heads(const std::array<DetectorHead,3> &heads,const Settings &settings) {
  const std::array<float,3> thresholds{{settings.person_conf,settings.crosswalk_conf,settings.light_conf}};
  std::array<std::vector<Detection>,3> candidates;
  for(const auto &head:heads) {
    if(!head.boxes || !head.scores || head.height<=0 || head.width<=0)
      throw std::runtime_error("invalid detector head");
    const int cells=head.height*head.width;
    const float sx=640.f/head.width,sy=384.f/head.height;
    for(int cell=0;cell<cells;++cell) {
      int label=0;float score=-1;
      for(int c=0;c<3;++c) {
        const float v=at(head.scores,3,cells,cell,c,head.scores_nhwc);
        if(!std::isfinite(v)) throw std::runtime_error("non-finite detector score");
        if(v>score) {score=v;label=c;}
      }
      // Scores are already sigmoid probabilities. Decode DFL only after filtering.
      if(score<thresholds[label]) continue;
      const float x=cell%head.width+.5f,y=cell/head.width+.5f;
      Detection d;
      d.class_id=label;d.score=score;
      d.box={(x-dfl(head,cell,0))*sx,(y-dfl(head,cell,1))*sy,
             (x+dfl(head,cell,2))*sx,(y+dfl(head,cell,3))*sy};
      candidates[label].push_back(d);
    }
  }
  std::vector<Detection> result;
  for(auto &group:candidates) {
    std::sort(group.begin(),group.end(),[](const Detection &a,const Detection &b){return a.score>b.score;});
    // Bound worst-case CPU work for noisy frames; normal scenes never reach this cap.
    if(group.size()>300) group.resize(300);
    std::vector<Detection> kept;
    for(auto d:group) {
      bool suppressed=false;
      for(const auto &other:kept) if(overlap(d.box,other.box)>settings.nms_iou) {suppressed=true;break;}
      if(suppressed) continue;
      kept.push_back(d);
      // Q1 is already 640x360. Remove padding without the old full-frame 2x scale.
      d.box.x1=std::clamp(d.box.x1,0.f,640.f);
      d.box.x2=std::clamp(d.box.x2,0.f,640.f);
      d.box.y1=std::clamp(d.box.y1,12.f,372.f)-12.f;
      d.box.y2=std::clamp(d.box.y2,12.f,372.f)-12.f;
      if(d.box.width()<1 || d.box.height()<1) continue;
      d.view=0;
      result.push_back(d);
      if(kept.size()>=100) break;
    }
  }
  std::sort(result.begin(),result.end(),[](const Detection &a,const Detection &b){return a.score>b.score;});
  return result;
}
} // namespace adas
