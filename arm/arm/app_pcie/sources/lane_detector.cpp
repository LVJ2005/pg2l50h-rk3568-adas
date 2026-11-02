#include "lane_detector.h"
#include <iterator>
#include <limits>

namespace adas {
namespace {
struct Point {float x,y;};
struct Run {float first,last;PaintColor paint;int order=0;float center() const {return (first+last)*.5f;}};
struct Row {
  std::vector<Run> runs;
  std::array<std::vector<Run>,3> ordered;
};
using Rows=std::vector<Row>;
struct Candidate {LaneEdge edge;float merit=0;};
float nearest(const Row &row,float x,float tolerance,PaintColor paint) {
  const auto &runs=row.ordered[static_cast<int>(paint)];
  float best=-1,distance=tolerance;
  int order=std::numeric_limits<int>::max();
  auto test=[&](const Run &run) {
    const float candidate=std::clamp(x,run.first,run.last);
    const float gap=std::abs(candidate-x);
    if(gap<tolerance && (gap<distance || (gap==distance && run.order<order))) {
      best=run.center();distance=gap;order=run.order;
    }
  };
  if(runs.size()<=4) for(const auto &run:runs) test(run);
  else {
    // Disjoint sorted intervals need only the two neighbors of x. Original
    // row order breaks exact-distance ties, including density-capped rows.
    const auto next=std::lower_bound(runs.begin(),runs.end(),x,
        [](const Run &run,float value){return run.last<value;});
    if(next!=runs.end()) test(*next);
    if(next!=runs.begin()) test(*std::prev(next));
  }
  return best;
}
bool fit(const std::vector<Point> &points,int h,LaneEdge &edge,bool quadratic) {
  if(points.size()<6) return false;
  const int n=quadratic ? 3:2;double m[3][4]{};
  for(const auto &p:points) {
    const double t=p.y/h,terms[3]={1,t,t*t};
    for(int i=0;i<n;++i) {
      for(int j=0;j<n;++j) m[i][j]+=terms[i]*terms[j];
      m[i][n]+=terms[i]*p.x;
    }
  }
  for(int i=0;i<n;++i) {
    int pivot=i;for(int j=i+1;j<n;++j) if(std::abs(m[j][i])>std::abs(m[pivot][i])) pivot=j;
    for(int k=0;k<=n;++k) std::swap(m[i][k],m[pivot][k]);
    if(std::abs(m[i][i])<1e-8) return false;
    const double divisor=m[i][i];for(int k=i;k<=n;++k) m[i][k]/=divisor;
    for(int j=0;j<n;++j) if(j!=i) {
      const double factor=m[j][i];for(int k=i;k<=n;++k) m[j][k]-=factor*m[i][k];
    }
  }
  edge.intercept=m[0][n];edge.slope=m[1][n]/h;edge.curvature=quadratic ? m[2][n]/(h*h):0;
  return true;
}
bool geometry(const LaneEdge &edge,int w,int h,int side,float center) {
  const float bottom=edge.x_at(h-1),top=edge.x_at(0),axis=center*w;
  if(!std::isfinite(bottom) || !std::isfinite(top) || bottom<.025f*w || bottom>.975f*w ||
      top<-.15f*w || top>1.15f*w) return false;
  // The vanishing point is not the centre of the vehicle's lane at the bottom.
  if(side==0 ? bottom>axis+w*.035f : bottom<axis-w*.035f) return false;
  if(side==0 ? bottom-top>-w*.035f : bottom-top<w*.035f) return false;
  return std::abs(edge.slope)<4.f && std::abs(edge.curvature*h)<1.f;
}
std::vector<Candidate> candidates(const Rows &rows,int w,int h,int side,
    const LaneEdge &previous,float center,float horizon,const std::vector<LaneEdge> &seeds,PaintColor paint) {
  std::vector<Point> upper,lower;
  const int search_top=std::clamp(static_cast<int>(horizon+h*.12f),2,static_cast<int>(h*.48f));
  for(int y=search_top;y<h-2;y+=2) for(const auto &run:rows[y].runs) {
    if(paint!=PaintColor::Unknown && run.paint!=paint) continue;
    const Point point{run.center(),static_cast<float>(y)};
    if(y<h*.70f) upper.push_back(point);
    if(y>=h*.40f) lower.push_back(point);
  }
  std::vector<Candidate> pool;const float tolerance=std::max(2.f,w*.008f);
  auto score=[&](LaneEdge edge) {
    edge.paint=paint;
    if(!geometry(edge,w,h,side,center)) return;
    int support=0,first=h,last=0;float residual=0,weighted=0;
    for(int y=search_top;y<h-2;y+=2) {
      const float x=nearest(rows[y],edge.x_at(y),tolerance,paint);
      if(x>=0) {++support;first=std::min(first,y);last=y;residual+=std::abs(x-edge.x_at(y));weighted+=.2f+.8f*y*y/(h*h);}
    }
    if(support<5 || last-first<h*.25f || last<h*.55f) return;
    const float quality=weighted/(h*.5f)-residual/(std::max(1,support)*tolerance)*.15f+
        (last-first)/static_cast<float>(h)*.2f;
    edge.fitted=true;
    for(auto &old:pool) if(std::abs(old.edge.x_at(h-1)-edge.x_at(h-1))<w*.012f &&
        std::abs(old.edge.x_at(h*.4f)-edge.x_at(h*.4f))<w*.012f) {
      if(quality>old.merit) old={edge,quality};
      return;
    }
    pool.push_back({edge,quality});
    std::sort(pool.begin(),pool.end(),[](const Candidate &a,const Candidate &b){return a.merit>b.merit;});
    if(pool.size()>12) pool.resize(12);
  };
  if(previous.fitted && previous.paint==paint) score(previous);
  for(const auto &seed:seeds) if(seed.paint==paint) score(seed);
  uint32_t rng=0x9e3779b9;
  auto random=[&] {rng^=rng<<13;rng^=rng>>17;rng^=rng<<5;return rng;};
  const int trials=seeds.empty() ? 384:(previous.fitted ? 64:160);
  for(int trial=0;trial<trials && !upper.empty() && !lower.empty();++trial) {
    const auto p=upper[random()%upper.size()],q=lower[random()%lower.size()];
    if(q.y-p.y<h*.20f) continue;
    LaneEdge edge;edge.slope=(q.x-p.x)/(q.y-p.y);edge.intercept=p.x-edge.slope*p.y;score(edge);
  }
  for(auto &candidate:pool) {
    auto &edge=candidate.edge;std::vector<Point> points;points.reserve(h);
    for(int pass=0;pass<2;++pass) {
      points.clear();
      for(int y=search_top;y<h-2;++y) {
        const float x=nearest(rows[y],edge.x_at(y),tolerance,paint);
        if(x>=0) points.push_back({x,static_cast<float>(y)});
      }
      LaneEdge linear=edge,curve=edge;
      if(!fit(points,h,linear,false)) {edge.fitted=false;break;}
      edge=linear;
      if(points.size()>=12 && fit(points,h,curve,true) && geometry(curve,w,h,side,center)) {
        float le=0,ce=0;for(const auto &p:points) {le+=std::abs(p.x-linear.x_at(p.y));ce+=std::abs(p.x-curve.x_at(p.y));}
        if(ce<le*.65f) edge=curve;
      }
    }
    if(!geometry(edge,w,h,side,center)) edge.fitted=false;
  }
  return pool;
}
float intersection(const LaneEdge &left,const LaneEdge &right,int h) {
  const float ls=(left.x_at(h-1)-left.x_at(0))/(h-1),rs=(right.x_at(h-1)-right.x_at(0))/(h-1);
  return std::abs(ls-rs)<1e-5f ? -10000.f:(right.x_at(0)-left.x_at(0))/(ls-rs);
}
LaneEdge measure(const uint8_t *mask,const Rows &rows,LaneEdge edge,int w,int h,float road_top,
    const uint8_t *weak,const uint8_t *paint,std::vector<int> &occupancy,std::vector<int> &strong_occupancy) {
  if(!edge.fitted) return {};
  const float tolerance=std::max(2.f,w*.008f);
  const int first_row=std::clamp(static_cast<int>(road_top+h*.06f),2,h-15);
  std::fill(occupancy.begin(),occupancy.end(),-1);
  std::fill(strong_occupancy.begin(),strong_occupancy.end(),-1);
  int observed=0,hits=0;float residual=0;
  edge.first_y=h;edge.last_y=0;
  for(int y=first_row;y<h-2;++y) {
    const int x=std::lround(edge.x_at(y));if(x<2 || x>=w-2) continue;
    bool excluded=false;for(int dx=-2;dx<=2;++dx)
      excluded|=mask[y*w+x+dx]==2 || mask[y*w+x+dx]==6;
    if(excluded) continue;
    float match=nearest(rows[y],edge.x_at(y),tolerance,edge.paint);
    if(match<0 && mask[y*w+x]==5) {occupancy[y]=strong_occupancy[y]=-2;continue;}
    const bool strong=match>=0;strong_occupancy[y]=strong;
    // Weak photometric evidence can confirm a geometry seeded by strong paint;
    // it never creates candidates or bridges a known occlusion.
    if(match<0 && weak) for(int dx=-2;dx<=2;++dx) if(weak[y*w+x+dx]==1 &&
        (mask[y*w+x+dx]!=1 || !paint || paint[y*w+x+dx]==static_cast<uint8_t>(edge.paint))) {
      match=edge.x_at(y);break;
    }
    occupancy[y]=match>=0;++observed;
    if(match>=0) {++hits;residual+=std::abs(match-edge.x_at(y));}
    if(strong) {edge.first_y=std::min(edge.first_y,y);edge.last_y=y;}
  }
  const int road_rows=h-2-first_row;
  if(hits<8 || edge.last_y-edge.first_y<std::max(12.f,road_rows*.40f) || observed<road_rows*.5f) return {};
  for(int y=first_row+1;y<h-3;++y) if(occupancy[y]==0 && occupancy[y-1]==1 && occupancy[y+1]==1) occupancy[y]=1;
  int runs=0,gaps=0,longest=0,run=0,gap=0,inside_observed=0,inside_hits=0;bool had_run=false;
  const int min_run=std::max(3,h/60),min_gap=std::max(3,static_cast<int>(std::lround(h*.045f)));
  int strong_runs=0,strong_gaps=0,strong_run=0,strong_gap=0,strong_hits=0;bool strong_had_run=false;
  int ambiguous=0,strong_ambiguous=0;const int max_ambiguous=std::max(2,h/40);
  for(int y=edge.first_y;y<=edge.last_y;++y) {
    // Short uncertain dash endpoints do not prove a gap, but must not erase
    // adjacent, actually observed road gaps. Broad uncertainty breaks the chain.
    if(strong_occupancy[y]==-2) {
      strong_run=0;
      if(++strong_ambiguous>max_ambiguous) {strong_gap=0;strong_had_run=false;}
    }
    else if(strong_occupancy[y]<0) {strong_run=strong_gap=strong_ambiguous=0;strong_had_run=false;}
    else if(strong_occupancy[y]) {
      strong_ambiguous=0;
      ++strong_hits;
      if(strong_gap>=std::max(3,h/40) && strong_had_run) ++strong_gaps;
      strong_gap=0;if(++strong_run==min_run) {++strong_runs;strong_had_run=true;}
    } else {strong_ambiguous=0;strong_run=0;++strong_gap;}
    if(occupancy[y]==-2) {
      run=0;
      if(++ambiguous>max_ambiguous) {gap=0;had_run=false;}
      continue;
    }
    if(occupancy[y]<0) {run=gap=0;had_run=false;continue;}
    ambiguous=0;
    ++inside_observed;
    if(occupancy[y]) {
      ++inside_hits;
      if(gap && had_run) {longest=std::max(longest,gap);if(gap>=min_gap) ++gaps;}
      gap=0;if(++run==min_run) {++runs;had_run=true;}
    } else {run=0;++gap;}
  }
  edge.coverage=static_cast<float>(inside_hits)/std::max(1,inside_observed);
  edge.confidence=std::clamp(1.f-residual/std::max(1,hits)/(tolerance*1.4f),0.f,1.f)*
      std::min(1.f,(edge.last_y-edge.first_y)/std::max(1.f,road_rows*.65f));
  edge.support_rows=hits;edge.segment_count=runs;edge.gap_count=gaps;edge.longest_gap=longest;edge.kind=LineKind::Unknown;
  if(edge.confidence>=.55f) {
    if(edge.coverage>=.88f && longest<min_gap && edge.last_y-edge.first_y>=road_rows*.60f)
      edge.kind=LineKind::Solid;
    else if(edge.coverage>=.22f && edge.coverage<=.88f &&
        ((runs>=3 && gaps>=2) || (strong_runs>=3 && strong_gaps>=2 &&
        static_cast<float>(strong_hits)/std::max(1,inside_observed)<=.75f))) edge.kind=LineKind::Dashed;
  }
  return edge;
}
}
void LaneScanner::reset() {
  for(int k=0;k<2;++k) {previous_[k]={};pending_geometry_[k]={};reacquire_[k]=0;pending_[k]=LineKind::Unknown;streak_[k]=0;}
}
std::pair<LaneEdge,LaneEdge> LaneScanner::detect(const uint8_t *mask,int w,int h,float horizon,float center,
    const std::vector<LaneEdge> &seeds,float vehicle_x,const uint8_t *weak,const uint8_t *paint) {
  if(!mask || w<32 || h<32) {reset();return {};}
  if(!std::isfinite(horizon) || horizon<-h*2.f || horizon>h*.8f) horizon=-h*.2f;
  center=std::clamp(center,.05f,.95f);vehicle_x=std::clamp(vehicle_x,.05f,.95f);Rows rows(h);
  for(int y=0;y<h;++y) {
    auto &runs=rows[y].runs;
    for(int x=1;x<w-1;) {
      if(mask[y*w+x]!=1) {++x;continue;}
      const int first=x;const auto color=paint ? static_cast<PaintColor>(paint[y*w+x]):PaintColor::Unknown;
      while(x<w-1 && mask[y*w+x]==1 && (!paint || paint[y*w+x]==static_cast<uint8_t>(color))) ++x;
      if(x-first<=std::max(8,w/14)) runs.push_back({static_cast<float>(first),static_cast<float>(x-1),color});
    }
    if(runs.size()>32) {
      std::stable_sort(runs.begin(),runs.end(),[&](const Run &a,const Run &b) {
        const float qa=std::abs(a.center()-vehicle_x*w)+(a.last-a.first<1 ? w:0);
        const float qb=std::abs(b.center()-vehicle_x*w)+(b.last-b.first<1 ? w:0);
        return qa<qb;
      });
      if(paint) {
        std::vector<Run> kept;int count[3]{};
        for(const auto &run:runs) {
          const auto color=static_cast<int>(run.paint);
          if(color>=1 && color<=2 && count[color]<16) {kept.push_back(run);++count[color];}
        }
        runs=std::move(kept);
      } else runs.resize(32);
    }
    for(size_t i=0;i<runs.size();++i) {
      auto run=runs[i];run.order=static_cast<int>(i);
      rows[y].ordered[0].push_back(run);
      const auto color=static_cast<int>(run.paint);
      if(color==1 || color==2) rows[y].ordered[color].push_back(run);
    }
    for(auto &list:rows[y].ordered) std::sort(list.begin(),list.end(),
        [](const Run &a,const Run &b){return a.last<b.last;});
  }
  std::vector<int> occupancy(h),strong_occupancy(h);
  auto measure_edge=[&](const LaneEdge &edge,float top) {
    return measure(mask,rows,edge,w,h,top,weak,paint,occupancy,strong_occupancy);
  };
  auto branch_candidates=[&](int side) {
    if(!paint) return candidates(rows,w,h,side,previous_[side],vehicle_x,horizon,seeds,PaintColor::Unknown);
    std::vector<Candidate> pool;
    for(auto color:{PaintColor::White,PaintColor::Yellow}) {
      auto branch=candidates(rows,w,h,side,previous_[side],vehicle_x,horizon,seeds,color);
      // Reserve slots for both colors: dense white clutter cannot evict yellow.
      if(branch.size()>6) branch.resize(6);
      pool.insert(pool.end(),branch.begin(),branch.end());
    }
    std::sort(pool.begin(),pool.end(),[](const Candidate &a,const Candidate &b){return a.merit>b.merit;});
    return pool;
  };
  const auto left=branch_candidates(0),right=branch_candidates(1);
  // A pair only changes measure() through its integer first_row. Cache identical
  // candidate/first_row evaluations within this frame, never across frames.
  using Measurement=std::pair<int,LaneEdge>;
  std::vector<std::vector<Measurement>> left_cache(left.size()),right_cache(right.size());
  auto measured=[&](const Candidate &candidate,float top,std::vector<Measurement> &cache) {
    const int first_row=std::clamp(static_cast<int>(top+h*.06f),2,h-15);
    for(const auto &entry:cache) if(entry.first==first_row) return entry.second;
    const auto value=measure_edge(candidate.edge,top);
    cache.emplace_back(first_row,value);return value;
  };
  LaneEdge edges[2];float best=-1000,road_top=horizon+h*.06f;
  for(size_t li=0;li<left.size();++li) for(size_t ri=0;ri<right.size();++ri) {
    const auto &l=left[li],&r=right[ri];
    if(!l.edge.fitted || !r.edge.fitted) continue;
    const float width=r.edge.x_at(h-1)-l.edge.x_at(h-1),vp_y=intersection(l.edge,r.edge,h);
    const float vp_x=(l.edge.x_at(vp_y)+r.edge.x_at(vp_y))*.5f;
    if(width<w*.22f || width>w*.90f || vp_y<-h*2.f || vp_y>h*.72f ||
        std::abs(vp_x-center*w)>w*.35f || l.edge.x_at(h-1)>vehicle_x*w+w*.035f ||
        r.edge.x_at(h-1)<vehicle_x*w-w*.035f) continue;
    const float active_top=std::max(vp_y,horizon+h*.06f);
    const auto measured_left=measured(l,active_top,left_cache[li]);
    if(!measured_left.fitted) continue;
    const auto measured_right=measured(r,active_top,right_cache[ri]);
    if(!measured_right.fitted) continue;
    float merit=l.merit+r.merit-std::abs(width/w-.40f)*1.5f-std::abs(vp_x-center*w)/w*.25f;
    merit+=(measured_left.support_rows+measured_right.support_rows)/static_cast<float>(h)*.3f;
    for(int k=0;k<2;++k) if(previous_[k].fitted) {
      const auto &e=k==0 ? l.edge:r.edge;
      if(e.paint==previous_[k].paint) merit+=.35f*std::max(0.f,1.f-
          (std::abs(e.x_at(h-1)-previous_[k].x_at(h-1))+
           std::abs(e.x_at(h*.4f)-previous_[k].x_at(h*.4f)))/(w*.12f));
    }
    if(merit>best) {best=merit;edges[0]=l.edge;edges[1]=r.edge;road_top=active_top;}
  }
  if(best<=-999) for(int k=0;k<2;++k) for(const auto &candidate:k==0 ? left:right) {
    if(candidate.edge.fitted && std::abs(candidate.edge.x_at(horizon)-center*w)<w*.25f &&
        measure_edge(candidate.edge,road_top).fitted) {
      edges[k]=candidate.edge;break;
    }
  }
  for(int k=0;k<2;++k) {
    if(previous_[k].fitted && edges[k].fitted && previous_[k].paint==edges[k].paint &&
        std::abs(previous_[k].x_at(h-1)-edges[k].x_at(h-1))<w*.08f &&
        std::abs(previous_[k].x_at(h*.4f)-edges[k].x_at(h*.4f))<w*.08f) {
      edges[k].intercept=edges[k].intercept*.55f+previous_[k].intercept*.45f;
      edges[k].slope=edges[k].slope*.55f+previous_[k].slope*.45f;
      edges[k].curvature=edges[k].curvature*.55f+previous_[k].curvature*.45f;
    }
    edges[k]=measure_edge(edges[k],road_top);
    const bool jump=previous_[k].fitted && edges[k].fitted &&
        (previous_[k].paint!=edges[k].paint || std::abs(previous_[k].x_at(h-1)-edges[k].x_at(h-1))>w*.08f);
    if(jump) {
      const auto &pending=pending_geometry_[k];
      const bool repeated=pending.fitted && pending.paint==edges[k].paint && std::abs(pending.x_at(h-1)-edges[k].x_at(h-1))<w*.04f &&
          std::abs(pending.x_at(h*.4f)-edges[k].x_at(h*.4f))<w*.04f;
      reacquire_[k]=repeated ? reacquire_[k]+1:1;pending_geometry_[k]=edges[k];
      if(reacquire_[k]<2) {
        const auto supported=measure_edge(previous_[k],road_top);
        if(supported.fitted && supported.confidence>=.65f) edges[k]=supported;
        else {edges[k]={};pending_[k]=LineKind::Unknown;streak_[k]=0;continue;}
      }
    } else {reacquire_[k]=0;pending_geometry_[k]={};}
    if(edges[k].kind==LineKind::Unknown) {pending_[k]=LineKind::Unknown;streak_[k]=0;}
    else if(edges[k].kind==pending_[k] && !jump) streak_[k]=std::min(10,streak_[k]+1);
    else {pending_[k]=edges[k].kind;streak_[k]=1;}
    previous_[k]=edges[k];if(streak_[k]<2) edges[k].kind=LineKind::Unknown;
  }
  return {edges[0],edges[1]};
}
} // namespace adas
