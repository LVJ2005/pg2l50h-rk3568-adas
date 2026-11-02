#include "adas_types.h"

namespace adas {
namespace {
Light merge(Light previous, Light current, Light red, Light green) {
  if(previous == red || current == red) return red;
  if(current == green) return green;
  return previous;
}
bool fresh(double captured, int ttl, double now) {
  return captured > 0 && now >= captured && now-captured <= ttl;
}
}
SignalSummary summarize_signals(const Perception &p, const Settings &s, const SignalSummary &old) {
  SignalSummary r;
  r.generation=p.generation; r.frame_seq=p.frame_seq; r.captured_ms=p.captured_ms;
  if(!p.error.empty()) return r;
  const Box view=view_box(s.front_view);
  bool off_seen=false;
  for(const auto &d:p.objects) {
    if(d.class_id!=2 || d.view!=s.front_view || d.light_score<s.cls_conf) continue;
    const float x=((d.box.x1+d.box.x2)*.5f-view.x1)/640.f;
    const float y=((d.box.y1+d.box.y2)*.5f-view.y1)/360.f;
    if(x<s.signal_roi[0] || y<s.signal_roi[1] || x>s.signal_roi[2] || y>s.signal_roi[3]) continue;
    r.present=true;
    switch(d.light) {
      case Light::Yellow: r.yellow=true; break;
      case Light::StraightRed: case Light::StraightGreen:
        r.straight=merge(r.straight,d.light,Light::StraightRed,Light::StraightGreen);
        r.straight_seen_ms=p.captured_ms; break;
      case Light::LeftRed: case Light::LeftGreen:
        r.left=merge(r.left,d.light,Light::LeftRed,Light::LeftGreen);
        r.left_seen_ms=p.captured_ms; break;
      case Light::RightRed: case Light::RightGreen:
        r.right=merge(r.right,d.light,Light::RightRed,Light::RightGreen);
        r.right_seen_ms=p.captured_ms; break;
      case Light::Off: off_seen=true; break;
      default: break;
    }
  }
  const bool same_generation=old.generation==p.generation && old.frame_seq!=p.frame_seq;
  auto retain=[&](Light &current,double &seen,int &streak,Light previous,double previous_seen,
                  int previous_streak,Light green) {
    const bool recent=same_generation && fresh(previous_seen,s.result_ttl_ms,p.captured_ms);
    if(current==Light::Unknown && recent && !off_seen) {
      current=previous;seen=previous_seen;streak=previous_streak;
    } else if(current==green && !r.yellow) {
      streak=recent && previous==green ? std::min(1000,previous_streak+1):1;
    }
    if(current!=green || r.yellow) streak=0;
  };
  retain(r.straight,r.straight_seen_ms,r.straight_green_streak,old.straight,
         old.straight_seen_ms,old.straight_green_streak,Light::StraightGreen);
  retain(r.left,r.left_seen_ms,r.left_green_streak,old.left,
         old.left_seen_ms,old.left_green_streak,Light::LeftGreen);
  retain(r.right,r.right_seen_ms,r.right_green_streak,old.right,
         old.right_seen_ms,old.right_green_streak,Light::RightGreen);
  return r;
}
PedestrianSummary summarize_pedestrians(const Perception &p, const Settings &s,
                                        const PedestrianSummary &old) {
  PedestrianSummary r;
  r.generation=p.generation;r.frame_seq=p.frame_seq;r.captured_ms=p.captured_ms;
  const Box view=view_box(s.front_view);
  if(p.error.empty()) {
    for(const auto &d:p.objects) if(d.view==s.front_view) {
      if(d.class_id==0 && d.score>=s.person_conf) r.person_seen=true;
      if(d.class_id==1 && d.score>=s.crosswalk_conf) r.crosswalk_seen=true;
    }
    for(const auto &person:p.objects) {
      if(person.class_id!=0 || person.view!=s.front_view || person.score<s.person_conf) continue;
      const float foot_x=(person.box.x1+person.box.x2)*.5f;
      const float foot_y=person.box.y2;
      const float normalized_x=(foot_x-view.x1)/view.width();
      if(normalized_x<s.pedestrian_path_left || normalized_x>s.pedestrian_path_right) continue;
      for(const auto &crosswalk:p.objects) {
        if(crosswalk.class_id!=1 || crosswalk.view!=s.front_view ||
           crosswalk.score<s.crosswalk_conf) continue;
        const float margin_x=std::max(6.f,crosswalk.box.width()*.06f);
        const float margin_y=std::max(6.f,crosswalk.box.height()*.15f);
        if(foot_x>=crosswalk.box.x1-margin_x && foot_x<=crosswalk.box.x2+margin_x &&
           foot_y>=crosswalk.box.y1-margin_y && foot_y<=crosswalk.box.y2+margin_y) {
          r.on_crosswalk=true;break;
        }
      }
      if(r.on_crosswalk) break;
    }
  }
  const bool previous_frame=old.generation==p.generation && old.frame_seq!=p.frame_seq &&
      p.captured_ms>=old.captured_ms && p.captured_ms-old.captured_ms<=s.result_ttl_ms;
  const bool recent_evidence=previous_frame && fresh(old.observed_ms,s.pedestrian_hold_ms,p.captured_ms);
  if(r.on_crosswalk) {
    r.observed_ms=p.captured_ms;
    r.evidence_hits=recent_evidence ? std::min(1000,old.evidence_hits+1):1;
    r.brake_requested=r.evidence_hits>=2 || (recent_evidence && old.brake_requested);
  } else if(recent_evidence) {
    r.observed_ms=old.observed_ms;
    r.evidence_hits=old.evidence_hits;
    r.brake_requested=old.brake_requested;
  }
  return r;
}
Decision evaluate_pedestrian(const Settings &s,const PedestrianSummary &p,double now) {
  Decision d;
  if(!s.enabled || !s.pedestrian_enabled) { d.reason="pedestrian_off";return d; }
  if(p.generation!=s.generation) {
    d.state="unknown";d.reason="pedestrian_stale";return d;
  }
  if(p.brake_requested && fresh(p.observed_ms,s.pedestrian_hold_ms,now)) {
    d.state="stop";d.reason="pedestrian_on_crosswalk";d.warning=true;return d;
  }
  if(!fresh(p.captured_ms,s.result_ttl_ms,now)) {
    d.state="unknown";d.reason="pedestrian_stale";return d;
  }
  const bool confirming=p.evidence_hits>0 && fresh(p.observed_ms,s.pedestrian_hold_ms,now);
  d.state=confirming ? "confirming":"clear";
  d.reason=confirming ? "pedestrian_confirming":"no_pedestrian_risk";
  return d;
}
Decision evaluate(const Settings &s,const Intent &i,const SignalSummary &sig,const LaneResult &lane,double now) {
  Decision d;
  d.signal_age_ms=sig.captured_ms>0 ? now-sig.captured_ms:-1;
  d.lane_age_ms=lane.captured_ms>0 ? now-lane.captured_ms:-1;
  if(!s.enabled) { d.reason="assist_off"; return d; }
  if(i.action==Maneuver::Stop) return d;
  if(!fresh(i.received_ms,s.command_ttl_ms,now)) { d.reason="command_expired"; return d; }
  const bool left=i.action==Maneuver::Left || i.action==Maneuver::LaneLeft;
  const bool right=i.action==Maneuver::Right || i.action==Maneuver::LaneRight;
  const bool lane_only=i.action==Maneuver::LaneLeft || i.action==Maneuver::LaneRight;
  const bool lane_valid=s.lane_enabled && lane.generation==s.generation && lane.view==s.front_view &&
      lane.error.empty() && fresh(lane.captured_ms,s.lane_ttl_ms,now);
  if(lane_only && lane_valid && ((left && lane.left.kind==LineKind::Solid) || (right && lane.right.kind==LineKind::Solid))) {
    d.state="warning"; d.reason=left ? "left_solid_line":"right_solid_line"; d.warning=true; return d;
  }
  if(lane_only) {
    const auto kind=left ? lane.left.kind:lane.right.kind;
    const double seen=left ? lane.left.observed_ms:lane.right.observed_ms;
    const bool side_fresh=lane_valid && fresh(seen>0 ? seen:lane.captured_ms,s.lane_ttl_ms,now);
    d.state=side_fresh && kind==LineKind::Dashed ? "allow":"unknown";
    d.reason=d.state=="allow" ? "dashed_line":"lane_unknown";
    return d;
  }
  if(sig.generation!=s.generation || !fresh(sig.captured_ms,s.result_ttl_ms,now)) {
    d.state="unknown"; d.reason="signal_stale"; return d;
  }
  Light state=sig.straight;
  int streak=sig.straight_green_streak;
  if(left) { state=sig.left; streak=sig.left_green_streak; }
  if(right) { state=sig.right; streak=sig.right_green_streak; }
  const double seen=left ? sig.left_seen_ms:right ? sig.right_seen_ms:sig.straight_seen_ms;
  d.signal_age_ms=seen>0 ? now-seen:-1;
  if(!fresh(seen>0 ? seen:sig.captured_ms,s.result_ttl_ms,now)) {
    d.state="unknown"; d.reason="signal_stale"; return d;
  }
  if(state==Light::StraightRed || state==Light::LeftRed || state==Light::RightRed) {
    d.state="stop"; d.reason="red_signal"; d.warning=true; return d;
  }
  if(sig.yellow) { d.state="warning"; d.reason="yellow_direction_unresolved"; d.warning=true; return d; }
  // A recognized intersection signal allows right-turn advice when no dedicated arrow is seen.
  if(right && state==Light::Unknown && s.signal_confirmed &&
     ((fresh(sig.straight_seen_ms,s.result_ttl_ms,now) &&
       (sig.straight==Light::StraightRed ||
        (sig.straight==Light::StraightGreen && sig.straight_green_streak>=s.green_confirm))) ||
      (fresh(sig.left_seen_ms,s.result_ttl_ms,now) &&
       (sig.left==Light::LeftRed ||
        (sig.left==Light::LeftGreen && sig.left_green_streak>=s.green_confirm))))) {
    d.state="allow";d.reason="right_turn_yield";
    d.signal_age_ms=now-(fresh(sig.straight_seen_ms,s.result_ttl_ms,now) ?
                          sig.straight_seen_ms:sig.left_seen_ms);return d;
  }
  d.state="unknown"; d.reason="signal_unknown";
  if(state==Light::StraightGreen || state==Light::LeftGreen || state==Light::RightGreen) {
    d.reason="green_confirming";
    if(!s.signal_confirmed) d.reason="signal_assignment_required";
    else if(streak>=s.green_confirm) { d.state="allow"; d.reason="green_signal"; }
  }
  return d;
}
} // namespace adas
