#include "lane_vision.h"
#include "frame_color.h"
#include <opencv2/imgproc.hpp>
#include <opencv2/core/hal/intrin.hpp>
#include <algorithm>

namespace adas {
namespace {
#if CV_SIMD128 && !defined(ADAS_LANE_DISABLE_SIMD)
// All intermediates fit signed/unsigned 16-bit lanes. No approximate reciprocal
// or floating-point threshold is used; the scalar path remains the reference.
void classify_eight(const cv::v_uint16x8 &b,const cv::v_uint16x8 &g,const cv::v_uint16x8 &r,
    const uint8_t *lum,const uint8_t *local,const int16_t *yellow_narrow,
    const int16_t *yellow_signal,const int16_t *yellow_hat,int level,int contrast,
    cv::v_uint16x8 &strong,cv::v_uint16x8 &color,cv::v_uint16x8 &weak) {
  using namespace cv;
  const auto hi=v_load_expand(lum),lo=v_min(b,v_min(g,r)),rg=v_min(r,g);
  const auto one=v_setall_u16(1),zero=v_setzero_u16();
  const auto neutral=((hi-lo)<=v_setall_u16(28)) |
      ((hi-lo)*v_setall_u16(100)<=hi*v_setall_u16(35));
  const auto ratio=(r*v_setall_u16(100)>=g*v_setall_u16(75)) &
      (g*v_setall_u16(100)>=r*v_setall_u16(60));
  const auto yellow=(r>v_setall_u16(55)) & (g>v_setall_u16(45)) &
      (rg>=b+v_setall_u16(8)) & (b*v_setall_u16(100)<rg*v_setall_u16(98)) &
      ratio & v_reinterpret_as_u16(v_load(yellow_narrow)>=v_setall_s16(4));
  const auto white=neutral & (hi>=v_setall_u16(level)) &
      (v_load_expand(local)>=v_setall_u16(contrast));
  strong=(white|yellow)&one;
  color=v_select(yellow,v_setall_u16(2),white&one);
  weak=zero;
  if(!yellow_hat) return;
  const auto ridge=v_reinterpret_as_s16(hi)*v_setall_s16(2)-
      v_reinterpret_as_s16(v_load_expand(lum-6))-v_reinterpret_as_s16(v_load_expand(lum+6));
  const auto chroma_ridge=v_load(yellow_signal)*v_setall_s16(2)-
      v_load(yellow_signal-6)-v_load(yellow_signal+6);
  const auto faint_white=neutral & (v_load_expand(local)>=v_setall_u16(std::max(7,contrast*2/3))) &
      v_reinterpret_as_u16(ridge>=v_setall_s16(12));
  const auto faint_yellow=ratio & v_reinterpret_as_u16(v_load(yellow_hat)>=v_setall_s16(8)) &
      v_reinterpret_as_u16(chroma_ridge>=v_setall_s16(12));
  weak=(hi>=v_setall_u16(30)) & (faint_white|faint_yellow) & one;
  color=v_select((color==zero)&(weak!=zero),v_select(faint_yellow,v_setall_u16(2),one),color);
}
#endif
}
const char *lane_execution_backend() {
#if CV_SIMD128 && !defined(ADAS_LANE_DISABLE_SIMD)
#if CV_NEON
  return "neon128";
#else
  return "simd128";
#endif
#else
  return "scalar";
#endif
}
cv::Rect lane_roi(const Settings &s) {
  const Box view=view_box(s.front_view);
  const int x1=std::lround(view.x1+640*s.lane_x1),x2=std::lround(view.x1+640*s.lane_x2);
  const int y1=std::lround(view.y1+360*s.lane_top),y2=std::lround(view.y1+360*s.lane_bottom);
  return {x1,y1,x2-x1,y2-y1};
}
std::pair<int,int> lane_road_bounds(const Settings &s,int y,int height,int width) {
  const float t=std::clamp(y/static_cast<float>(std::max(1,height-1)),0.f,1.f);
  return {std::clamp(static_cast<int>(std::lround((s.lane_trap_tl*(1-t)+s.lane_trap_bl*t)*width)),0,width),
      std::clamp(static_cast<int>(std::lround((s.lane_trap_tr*(1-t)+s.lane_trap_br*t)*width)),0,width)};
}
cv::Vec3b lane_debug_rgb(uint8_t value) {
  switch(value) {
    case 1:return {255,255,255};
    case 2:return {0,80,255};
    case 3:return {90,90,90};
    case 4:return {255,220,0};
    case 5:return {130,40,100};
    case 6:return {20,35,45};
    default:return {0,0,0};
  }
}
void make_lane_mask(const cv::Mat &bgr,std::vector<uint8_t> &mask,int contrast,std::vector<uint8_t> *weak,
    std::vector<uint8_t> *paint) {
  CV_Assert(bgr.type()==CV_8UC3 && !bgr.empty());
  cv::Mat smooth,gray(bgr.size(),CV_8UC1),hat;
  cv::GaussianBlur(bgr,smooth,cv::Size(3,3),.5);
  cv::Mat chroma(bgr.size(),CV_16SC1),chroma_hat,chroma_narrow;
  // One pass produces the same maximum-channel brightness and yellow chroma.
  for(int y=0;y<bgr.rows;++y) {
    const auto *pixels=smooth.ptr<cv::Vec3b>(y);
    auto *lum=gray.ptr<uint8_t>(y);auto *yellow=chroma.ptr<int16_t>(y);
    int x=0;
#if CV_SIMD128 && !defined(ADAS_LANE_DISABLE_SIMD)
    for(;x+16<=bgr.cols;x+=16) {
      cv::v_uint8x16 b,g,r;cv::v_load_deinterleave(reinterpret_cast<const uint8_t*>(pixels+x),b,g,r);
      cv::v_store(lum+x,cv::v_max(b,cv::v_max(g,r)));
      cv::v_uint16x8 b0,b1,rg0,rg1;
      cv::v_expand(b,b0,b1);cv::v_expand(cv::v_min(r,g),rg0,rg1);
      cv::v_store(yellow+x,cv::v_reinterpret_as_s16(rg0)-cv::v_reinterpret_as_s16(b0));
      cv::v_store(yellow+x+8,cv::v_reinterpret_as_s16(rg1)-cv::v_reinterpret_as_s16(b1));
    }
#endif
    for(;x<bgr.cols;++x) {
      const auto p=pixels[x];lum[x]=std::max({p[0],p[1],p[2]});
      yellow[x]=std::min(p[1],p[2])-p[0];
    }
  }
  static const cv::Mat wide_kernel=cv::getStructuringElement(cv::MORPH_RECT,cv::Size(31,1));
  static const cv::Mat narrow_kernel=cv::getStructuringElement(cv::MORPH_RECT,cv::Size(9,1));
  // Horizontal top-hat preserves narrow paint without joining vertical dash gaps.
  cv::morphologyEx(gray,hat,cv::MORPH_TOPHAT,wide_kernel);
  if(weak) cv::morphologyEx(chroma,chroma_hat,cv::MORPH_TOPHAT,wide_kernel);
  cv::morphologyEx(chroma,chroma_narrow,cv::MORPH_TOPHAT,narrow_kernel);
  if(weak) weak->assign(bgr.total(),0);
  if(paint) paint->assign(bgr.total(),0);
  std::vector<uint8_t> colors(bgr.total(),0);
  mask.assign(bgr.total(),0);
  const int faint_contrast=std::max(7,contrast*2/3);
  for(int y=0;y<bgr.rows;++y) {
    const auto *pixels=smooth.ptr<cv::Vec3b>(y);
    const auto *lum=gray.ptr<uint8_t>(y),*local=hat.ptr<uint8_t>(y);
    const auto *yellow_signal=chroma.ptr<int16_t>(y),*yellow_narrow=chroma_narrow.ptr<int16_t>(y);
    const auto *yellow_hat=weak ? chroma_hat.ptr<int16_t>(y):nullptr;
    auto *strong_row=mask.data()+y*bgr.cols,*color_row=colors.data()+y*bgr.cols;
    auto *weak_row=weak ? weak->data()+y*bgr.cols:nullptr;
    std::array<int,256> histogram{};
    const int x_begin=bgr.cols/6,x_end=bgr.cols*5/6;
    for(int x=x_begin;x<x_end;++x) ++histogram[lum[x]];
    int median=0,count=0;
    for(;median<255;++median) if((count+=histogram[median])>=(x_end-x_begin)/2) break;
    const int paint_level=std::max(45,median+18);
    int vector_end=6;
#if CV_SIMD128 && !defined(ADAS_LANE_DISABLE_SIMD)
    for(;vector_end+16<=bgr.cols-6;vector_end+=16) {
      const int x=vector_end;
      cv::v_uint8x16 b,g,r;cv::v_load_deinterleave(reinterpret_cast<const uint8_t*>(pixels+x),b,g,r);
      cv::v_uint16x8 b0,b1,g0,g1,r0,r1,strong0,strong1,color0,color1,weak0,weak1;
      cv::v_expand(b,b0,b1);cv::v_expand(g,g0,g1);cv::v_expand(r,r0,r1);
      classify_eight(b0,g0,r0,lum+x,local+x,yellow_narrow+x,yellow_signal+x,
          weak ? yellow_hat+x:nullptr,paint_level,contrast,strong0,color0,weak0);
      classify_eight(b1,g1,r1,lum+x+8,local+x+8,yellow_narrow+x+8,yellow_signal+x+8,
          weak ? yellow_hat+x+8:nullptr,paint_level,contrast,strong1,color1,weak1);
      cv::v_store(strong_row+x,cv::v_pack(strong0,strong1));
      cv::v_store(color_row+x,cv::v_pack(color0,color1));
      if(weak) cv::v_store(weak_row+x,cv::v_pack(weak0,weak1));
    }
#endif
    for(int x=0;x<bgr.cols;) {
      if(x==6 && vector_end>6) {x=vector_end;continue;}
      const int b=pixels[x][0],g=pixels[x][1],r=pixels[x][2];
      const int hi=lum[x],lo=std::min({r,g,b});
      const bool neutral=hi-lo<=std::max(28,hi*35/100);
      const bool yellow=(r>55) & (g>45) & (std::min(r,g)-b>=8) & (b*100<std::min(r,g)*98) &
          (r*100>=g*75) & (g*100>=r*60) & (yellow_narrow[x]>=4);
      const bool white=neutral & (lum[x]>=paint_level) & (local[x]>=contrast);
      // Faded yellow paint can be darker than blue-grey asphalt. Its chroma
      // must not depend on the bright-white top-hat threshold.
      strong_row[x]=white | yellow;
      color_row[x]=yellow ? 2:white ? 1:0;
      if(weak) {
        const int left=std::max(0,x-6),right=std::min(bgr.cols-1,x+6);
        const bool faint_white=neutral & (local[x]>=faint_contrast) &
            (2*lum[x]-lum[left]-lum[right]>=12);
        const bool faint_yellow=(r*100>=g*75) & (g*100>=r*60) & (yellow_hat[x]>=8) &
            (2*yellow_signal[x]-yellow_signal[left]-yellow_signal[right]>=12);
        weak_row[x]=(lum[x]>=30) & (faint_white | faint_yellow);
        color_row[x]=(!color_row[x] & (weak_row[x]!=0)) ? (faint_yellow ? 2:1):color_row[x];
      }
      ++x;
    }
    // A stripe needs darker / less yellow road on BOTH flanks. Do not turn a
    // rejected reflection into an occluder, and never close longitudinal gaps.
    const float depth=y/static_cast<float>(std::max(1,bgr.rows-1));
    const int max_width=std::max(5,static_cast<int>(std::lround(bgr.cols*(.014f+.024f*depth))));
    for(int x=0;x<bgr.cols;) {
      const int index=y*bgr.cols+x;
      if(mask[index]!=1) {++x;continue;}
      const int first=x,color=colors[index];
      while(x<bgr.cols && mask[y*bgr.cols+x]==1 && colors[y*bgr.cols+x]==color) ++x;
      const int end=x,width=end-first;
      auto signal=[&](int pos) {return color==2 ? static_cast<int>(yellow_signal[pos]):static_cast<int>(lum[pos]);};
      float inside=0,left=0,right=0;
      for(int pos=first;pos<end;++pos) inside+=signal(pos);
      inside/=width;
      int count_left=0,count_right=0;
      for(int d=1;d<=3;++d) {
        if(first-d>=0) {left+=signal(first-d);++count_left;}
        if(end-1+d<bgr.cols) {right+=signal(end-1+d);++count_right;}
      }
      const float required=color==2 ? 3.f:std::max(3.f,contrast/3.f);
      const bool stripe=width<=max_width && count_left && count_right &&
          inside-left/count_left>=required && inside-right/count_right>=required;
      if(!stripe) for(int pos=first;pos<end;++pos) {
        const int k=y*bgr.cols+pos;mask[k]=5;
        if(weak) (*weak)[k]=0;
      }
    }
  }
  if(paint) *paint=std::move(colors);
}
LaneResult analyze_lane_image(const cv::Mat &bgr,const Settings &s,LaneScanner &scanner,const std::vector<Box> &occluders) {
  CV_Assert(bgr.type()==CV_8UC3 && bgr.cols==1280 && bgr.rows==720);
  const double started=now_ms();
  LaneResult result;result.generation=s.generation;result.view=s.front_view;
  const cv::Rect roi=lane_roi(s);
  result.height=std::min(roi.height,std::clamp(s.lane_scan_rows,60,180));
  result.roi={static_cast<float>(roi.x),static_cast<float>(roi.y),
      static_cast<float>(roi.x+roi.width),static_cast<float>(roi.y+roi.height)};
  cv::Mat raw;cv::resize(bgr(roi),raw,cv::Size(result.width,result.height),0,0,cv::INTER_AREA);
  cv::Mat small=camera_display_bgr(raw,s.infer_order);
  const double pre_done=now_ms();result.pre_ms=pre_done-started;
  std::vector<uint8_t> mask,weak,paint;
  make_lane_mask(small,mask,s.lane_contrast,&weak,&paint);
  for(int y=0;y<result.height;++y) {
    const auto bounds=lane_road_bounds(s,y,result.height,result.width);
    for(int x=0;x<result.width;++x) if(x<bounds.first || x>=bounds.second) {
      const int index=y*result.width+x;mask[index]=6;weak[index]=0;paint[index]=0;
    }
  }
  for(const auto &box:occluders) {
    const int x1=std::clamp(static_cast<int>((box.x1-roi.x)*result.width/roi.width),0,result.width);
    const int x2=std::clamp(static_cast<int>(std::ceil((box.x2-roi.x)*result.width/roi.width)),0,result.width);
    const int y1=std::clamp(static_cast<int>((box.y1-roi.y)*result.height/roi.height),0,result.height);
    const int y2=std::clamp(static_cast<int>(std::ceil((box.y2-roi.y)*result.height/roi.height)),0,result.height);
    for(int y=y1;y<y2;++y) for(int x=x1;x<x2;++x) if(mask[y*result.width+x]!=6)
      mask[y*result.width+x]=weak[y*result.width+x]=2;
  }
  const float horizon=(s.lane_horizon-s.lane_top)/(s.lane_bottom-s.lane_top)*result.height;
  const float center=(s.lane_center-s.lane_x1)/(s.lane_x2-s.lane_x1);
  const float vehicle_x=(s.lane_vehicle_x-s.lane_x1)/(s.lane_x2-s.lane_x1);
  const double mask_done=now_ms();result.mask_ms=mask_done-pre_done;
  const int search_top=std::clamp(static_cast<int>(horizon+result.height*.12f),0,result.height/2);
  std::vector<LaneEdge> seeds;
  seeds.reserve(64);
  for(int color:{1,2}) {
    cv::Mat branch(result.height,result.width,CV_8UC1,cv::Scalar(0));
    for(int y=search_top;y<branch.rows;++y) {
      auto *row=branch.ptr<uint8_t>(y);
      const auto *strong=mask.data()+y*branch.cols,*colors=paint.data()+y*branch.cols;
      for(int x=0;x<branch.cols;++x) row[x]=strong[x]==1 && colors[x]==color ? 1:0;
    }
    std::vector<cv::Vec4i> lines;cv::HoughLinesP(branch,lines,1,CV_PI/180,10,8,8);
    int added=0;
    for(const auto &line:lines) {
      const float dy=line[3]-line[1];if(std::abs(dy)<6) continue;
      LaneEdge seed;seed.paint=static_cast<PaintColor>(color);
      seed.slope=(line[2]-line[0])/dy;seed.intercept=line[0]-seed.slope*line[1];
      seeds.push_back(seed);if(++added==32) break;
    }
  }
  const double seed_done=now_ms();result.seed_ms=seed_done-mask_done;
  const auto edges=scanner.detect(mask.data(),result.width,result.height,horizon,center,seeds,vehicle_x,weak.data(),paint.data());
  result.fit_ms=now_ms()-seed_done;
  result.left=edges.first;result.right=edges.second;
  result.paint_ratio=static_cast<float>(std::count(mask.begin(),mask.end(),uint8_t{1}))/mask.size();
  result.occlusion_ratio=static_cast<float>(std::count(mask.begin(),mask.end(),uint8_t{2}))/mask.size();
  if(s.lane_debug) {
    for(size_t i=0;i<mask.size();++i) {
      if(mask[i]==1 && paint[i]==2) mask[i]=4;
      else if(mask[i]==0 && weak[i]==1) mask[i]=3;
    }
    result.mask=std::move(mask);
  }
  return result;
}
}
