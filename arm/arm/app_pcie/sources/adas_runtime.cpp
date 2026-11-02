#include "recognition_control.h"
#include <cctype>
#include <iomanip>
#include <sstream>

namespace adas {
namespace {
bool number(const std::string &text,float low,float high,float &out) {
  try { size_t used=0; out=std::stof(text,&used);
    return used==text.size() && std::isfinite(out) && out>=low && out<=high;
  } catch(...) { return false; }
}
std::string token(std::string value) {
  if(value.empty()) return "none";
  for(auto &c:value) if(std::isspace(static_cast<unsigned char>(c)) || c=='=') c='_';
  return value.substr(0,200);
}
bool active(const Decision &d) { return d.state!="idle"; }
Decision combine(const Decision &signal,const Decision &lane,const Decision &pedestrian) {
  if(pedestrian.state=="stop") return pedestrian;
  if(!active(signal)) return lane;
  if(!active(lane)) return signal;
  if(signal.state=="stop") return signal;
  if(lane.state=="warning") return lane;
  if(signal.state=="warning") return signal;
  Decision d=signal;
  if(signal.state=="allow" && lane.state=="allow") {
    d.reason="both_allowed";
  } else {
    d.state="unknown";
    d.reason=signal.state=="unknown" ? signal.reason:lane.reason;
  }
  return d;
}
}
Settings Runtime::settings() const { std::lock_guard<std::mutex> lock(mutex_); return settings_; }
Intent Runtime::intent() const { std::lock_guard<std::mutex> lock(mutex_); return intent_; }
Intent Runtime::lane_intent() const { std::lock_guard<std::mutex> lock(mutex_); return lane_intent_; }
void Runtime::configure(const Settings &s) { std::lock_guard<std::mutex> lock(mutex_); settings_=s; }
std::shared_ptr<const Perception> Runtime::perception() const { return std::atomic_load(&perception_); }
std::shared_ptr<const LaneResult> Runtime::lane() const { return std::atomic_load(&lane_); }
void Runtime::publish_perception(std::shared_ptr<const Perception> p) {
  std::lock_guard<std::mutex> lock(mutex_);
  if(p->generation!=settings_.generation) return;
  signals_=summarize_signals(*p,settings_,signals_);
  pedestrians_=summarize_pedestrians(*p,settings_,pedestrians_);
  std::atomic_store(&perception_,std::move(p));
}
void Runtime::publish_lane(std::shared_ptr<const LaneResult> p) {
  std::lock_guard<std::mutex> lock(mutex_);
  if(p->generation!=settings_.generation) return;
  auto next=std::make_shared<LaneResult>(*p);
  const auto old=std::atomic_load(&lane_);
  auto retain=[&](LaneEdge &edge,const LaneEdge *previous) {
    if(edge.kind!=LineKind::Unknown) edge.observed_ms=p->captured_ms;
    else if(previous && old->generation==p->generation && previous->observed_ms>0 &&
            p->captured_ms>=previous->observed_ms &&
            p->captured_ms-previous->observed_ms<=settings_.lane_ttl_ms) edge=*previous;
  };
  retain(next->left,old ? &old->left:nullptr);
  retain(next->right,old ? &old->right:nullptr);
  std::atomic_store(&lane_,std::shared_ptr<const LaneResult>(next));
}
void Runtime::loaded(const std::string &det,const std::string &cls,const std::string &error) {
  std::lock_guard<std::mutex> lock(mutex_);
  loaded_det_=det; loaded_cls_=cls; model_error_=error;
}
void Runtime::reload_failed(uint64_t generation,const std::string &det,const std::string &cls,const std::string &error) {
  std::lock_guard<std::mutex> lock(mutex_);
  model_error_=error;
  if(settings_.generation==generation && det!="none" && cls!="none") {
    settings_.det_profile=det; settings_.cls_profile=cls; ++settings_.generation;
  }
}
Decision Runtime::decision() const {
  const auto l=lane();
  std::lock_guard<std::mutex> lock(mutex_);
  const auto now=now_ms();
  return combine(evaluate(settings_,intent_,signals_,l ? *l:LaneResult{},now),
                 evaluate(settings_,lane_intent_,signals_,l ? *l:LaneResult{},now),
                 evaluate_pedestrian(settings_,pedestrians_,now));
}
Decision Runtime::signal_decision() const {
  const auto l=lane();
  std::lock_guard<std::mutex> lock(mutex_);
  return evaluate(settings_,intent_,signals_,l ? *l:LaneResult{},now_ms());
}
Decision Runtime::lane_decision() const {
  const auto l=lane();
  std::lock_guard<std::mutex> lock(mutex_);
  return evaluate(settings_,lane_intent_,signals_,l ? *l:LaneResult{},now_ms());
}
Decision Runtime::pedestrian_decision() const {
  std::lock_guard<std::mutex> lock(mutex_);
  return evaluate_pedestrian(settings_,pedestrians_,now_ms());
}
std::string Runtime::status(const std::string &section) const {
  const auto p=perception(); const auto l=lane();
  std::lock_guard<std::mutex> lock(mutex_);
  const auto now=now_ms();
  const auto signal_d=evaluate(settings_,intent_,signals_,l ? *l:LaneResult{},now);
  const auto lane_d=evaluate(settings_,lane_intent_,signals_,l ? *l:LaneResult{},now);
  const auto pedestrian_d=evaluate_pedestrian(settings_,pedestrians_,now);
  const auto d=combine(signal_d,lane_d,pedestrian_d);
  const auto &s=settings_;
  const bool lane_current=s.enabled && s.lane_enabled && l && l->generation==s.generation &&
      now_ms()-l->captured_ms<=s.lane_ttl_ms;
  std::ostringstream o; o<<std::fixed<<std::setprecision(3);
  if(section=="config") {
    o<<"OK generation="<<s.generation<<" result_ttl_ms="<<s.result_ttl_ms<<" lane_ttl_ms="<<s.lane_ttl_ms
      <<" pedestrian_enabled="<<s.pedestrian_enabled
      <<" pedestrian_hold_ms="<<s.pedestrian_hold_ms
      <<" pedestrian_path_left="<<s.pedestrian_path_left<<" pedestrian_path_right="<<s.pedestrian_path_right
      <<" command_ttl_ms="<<s.command_ttl_ms<<" green_confirm="<<s.green_confirm
      <<" person_conf="<<s.person_conf<<" crosswalk_conf="<<s.crosswalk_conf<<" light_conf="<<s.light_conf
      <<" cls_conf="<<s.cls_conf<<" nms_iou="<<s.nms_iou<<" draw_all="<<s.draw_all
      <<" lane_top="<<s.lane_top<<" lane_bottom="<<s.lane_bottom<<" lane_horizon="<<s.lane_horizon
      <<" lane_x1="<<s.lane_x1<<" lane_x2="<<s.lane_x2<<" lane_center="<<s.lane_center
      <<" lane_vehicle_x="<<s.lane_vehicle_x<<" lane_scan_rows="<<s.lane_scan_rows
      <<" lane_trap_tl="<<s.lane_trap_tl<<" lane_trap_tr="<<s.lane_trap_tr
      <<" lane_trap_bl="<<s.lane_trap_bl<<" lane_trap_br="<<s.lane_trap_br
      <<" lane_contrast="<<s.lane_contrast<<" lane_hz="<<s.lane_hz<<" infer_interval="<<s.infer_interval
      <<" max_lights="<<s.max_lights<<" signal_roi="<<s.signal_roi[0]<<','<<s.signal_roi[1]<<','<<s.signal_roi[2]<<','<<s.signal_roi[3];
    return o.str();
  }
  if(section=="lane") {
    o<<"OK generation="<<s.generation<<" lane_age="<<d.lane_age_ms<<" lane_current="<<lane_current;
    if(l) o<<" lane_ms="<<l->processing_ms<<" lane_actual_rows="<<l->height
        <<" lane_pre_ms="<<l->pre_ms<<" lane_mask_ms="<<l->mask_ms
        <<" lane_seed_ms="<<l->seed_ms<<" lane_fit_ms="<<l->fit_ms<<" lane_cpu_ms="<<l->cpu_ms
        <<" lane_left="<<(lane_current ? line_name(l->left.kind):"unknown")
        <<" lane_right="<<(lane_current ? line_name(l->right.kind):"unknown")
        <<" lane_left_support="<<l->left.support_rows<<" lane_right_support="<<l->right.support_rows
        <<" lane_left_segments="<<l->left.segment_count<<" lane_right_segments="<<l->right.segment_count
        <<" lane_left_gaps="<<l->left.gap_count<<" lane_right_gaps="<<l->right.gap_count
        <<" lane_left_coverage="<<l->left.coverage<<" lane_right_coverage="<<l->right.coverage
        <<" lane_left_paint="<<paint_name(l->left.paint)<<" lane_right_paint="<<paint_name(l->right.paint)
        <<" lane_paint="<<l->paint_ratio<<" lane_occlusion="<<l->occlusion_ratio
        <<" lane_error="<<token(l->error);
    return o.str();
  }
  const bool compact=section=="summary";
  auto error_token=[&](const std::string &value) {return token(value).substr(0,compact ? 32:200);};
  o<<"OK version=ADAS-quad-V2.0.10 mode="<<(s.enabled ? "assist":"off")<<" det_model="<<s.det_profile<<" cls_model="<<s.cls_profile
   <<" loaded_det="<<loaded_det_<<" loaded_cls="<<loaded_cls_<<" generation="<<s.generation
    <<" action="<<maneuver_name(intent_.action)<<" command_seq="<<intent_.seq
    <<" lane_action="<<maneuver_name(lane_intent_.action)
    <<" signal_state="<<signal_d.state<<" signal_reason="<<signal_d.reason
    <<" lane_state="<<lane_d.state<<" lane_reason="<<lane_d.reason
    <<" pedestrian_state="<<pedestrian_d.state<<" pedestrian_reason="<<pedestrian_d.reason
    <<" pedestrian_enabled="<<s.pedestrian_enabled
    <<" person_seen="<<pedestrians_.person_seen<<" crosswalk_seen="<<pedestrians_.crosswalk_seen
    <<" person_on_crosswalk="<<pedestrians_.on_crosswalk
    <<" brake_request="<<(pedestrian_d.state=="stop")
    <<" state="<<d.state<<" reason="<<d.reason<<" warning="<<d.warning
   <<" front_view="<<s.front_view<<" signal_confirmed="<<s.signal_confirmed
   <<" input_order=bgr display_order=rgb lane_debug="<<s.lane_debug<<" lane_contrast="<<s.lane_contrast
   <<" view_order="<<s.view_order<<" color_compare="<<s.color_compare
   <<" infer_order="<<s.infer_order<<" detection_view=0 detection_scale=1"
   <<" display_backend="<<s.display_backend
   <<" straight="<<light_name(signals_.straight)<<" left="<<light_name(signals_.left)
   <<" right="<<light_name(signals_.right)<<" yellow="<<signals_.yellow
   <<" signal_age="<<d.signal_age_ms<<" lane_age="<<d.lane_age_ms
   <<" lane_left="<<(lane_current ? line_name(l->left.kind):"unknown")
   <<" lane_right="<<(lane_current ? line_name(l->right.kind):"unknown")
   <<" infer_interval="<<s.infer_interval<<" lane_hz="<<s.lane_hz<<" lane_enabled="<<s.lane_enabled;
  if(!compact) o<<" detection_roi=0,0,640,360"
   <<" result_ttl_ms="<<s.result_ttl_ms<<" lane_ttl_ms="<<s.lane_ttl_ms
   <<" command_ttl_ms="<<s.command_ttl_ms<<" green_confirm="<<s.green_confirm
   <<" person_conf="<<s.person_conf<<" crosswalk_conf="<<s.crosswalk_conf<<" light_conf="<<s.light_conf
   <<" cls_conf="<<s.cls_conf<<" nms_iou="<<s.nms_iou<<" draw_all="<<s.draw_all
   <<" lane_horizon="<<s.lane_horizon<<" max_lights="<<s.max_lights
   <<" lane_x1="<<s.lane_x1<<" lane_x2="<<s.lane_x2
   <<" lane_center="<<s.lane_center<<" lane_vehicle_x="<<s.lane_vehicle_x;
  o<<" lane_top="<<s.lane_top<<" lane_bottom="<<s.lane_bottom<<" lane_scan_rows="<<s.lane_scan_rows
   <<" snapshot_requested="<<snapshot_requested.load()
   <<" snapshot_completed="<<snapshot_completed.load()
   <<" cap="<<cap_ms.load()<<" copy="<<copy_ms.load()<<" frames="<<frames.load()
   <<" capture_errors="<<capture_errors.load()<<" skipped="<<npu_skipped.load();
  if(p) {
    o<<" pre="<<p->pre_ms<<" det_npu="<<p->det_ms<<" cls_npu="<<p->cls_ms
      <<" detections="<<p->objects.size()<<" infer_error="<<error_token(p->error);
    if(!compact) o<<" frame_seq="<<p->frame_seq<<" post="<<p->post_ms<<" cls_runs="<<p->cls_runs
      <<" queue="<<p->queue_ms<<" age="<<now_ms()-p->captured_ms;
  }
  if(l) {
    o<<" lane_ms="<<l->processing_ms<<" lane_left_conf="<<(lane_current ? l->left.confidence:0)
      <<" lane_right_conf="<<(lane_current ? l->right.confidence:0);
    if(!compact) o<<" lane_pre_ms="<<l->pre_ms<<" lane_mask_ms="<<l->mask_ms
      <<" lane_seed_ms="<<l->seed_ms<<" lane_fit_ms="<<l->fit_ms<<" lane_cpu_ms="<<l->cpu_ms
      <<" lane_left_coverage="<<l->left.coverage
      <<" lane_right_coverage="<<l->right.coverage
      <<" lane_left_support="<<(lane_current ? l->left.support_rows:0)
      <<" lane_right_support="<<(lane_current ? l->right.support_rows:0)
      <<" lane_left_segments="<<(lane_current ? l->left.segment_count:0)
      <<" lane_right_segments="<<(lane_current ? l->right.segment_count:0)
      <<" lane_left_gaps="<<(lane_current ? l->left.gap_count:0)
      <<" lane_right_gaps="<<(lane_current ? l->right.gap_count:0);
    o<<" lane_left_fitted="<<(lane_current && l->left.fitted)
      <<" lane_right_fitted="<<(lane_current && l->right.fitted)
      <<" lane_actual_rows="<<(lane_current ? l->height:0)
      <<" lane_paint="<<(lane_current ? l->paint_ratio:0)
      <<" lane_occlusion="<<(lane_current ? l->occlusion_ratio:0)
      <<" lane_error="<<error_token(l->error);
  }
  o<<" model_error="<<error_token(model_error_);
  if(!compact) o<<" signal_roi="<<s.signal_roi[0]<<','<<s.signal_roi[1]<<','<<s.signal_roi[2]<<','<<s.signal_roi[3];
  return o.str();
}
bool Runtime::command(const std::string &command,std::string &response) {
  if(command.size()>512) { response="ERR command_too_long"; return false; }
  std::istringstream in(command); std::vector<std::string> args; std::string part;
  while(in>>part) args.push_back(part);
  auto fail=[&](const std::string &why){response="ERR "+why; return false;};
  if(args.empty()) return fail("empty_command");
  for(auto &c:args[0]) c=static_cast<char>(std::toupper(static_cast<unsigned char>(c)));
  if(args[0]=="PING" && args.size()==1) {response="OK PONG"; return true;}
  if(args[0]=="STATUS") {
    if(args.size()==1) {response=status("summary");return true;}
    if(args.size()==2 && (args[1]=="CONFIG" || args[1]=="LANE")) {
      response=status(args[1]=="CONFIG" ? "config":"lane");return true;
    }
    return fail("STATUS_requires_CONFIG_or_LANE");
  }
  if(args[0]=="SNAPSHOT" && args.size()==1) {
    const auto revision=++snapshot_requested;response="OK snapshot_requested="+std::to_string(revision);return true;
  }
  if(args[0]=="HELP" && args.size()==1) {
    response="OK MODE_assist/off MODEL_det/cls_fp/int8 DRIVE_stop/straight/left/right_seq_client LANE_stop/left/right_seq_client SET_key_value ROI_x1_y1_x2_y2 LANE_ROI_x1_y1_x2_y2 LANE_TRAP_tl_tr_bl_br PEDESTRIAN_PATH_left_right SNAPSHOT STATUS PING";return true;
  }
  std::lock_guard<std::mutex> lock(mutex_);
  auto &s=settings_;
  if(args[0]=="DRIVE" || args[0]=="LANE") {
    const bool lane_command=args[0]=="LANE";
    if(args.size()!=4) return fail("action_requires_action_seq_client");
    Maneuver action;
    if(args[1]=="stop") action=Maneuver::Stop;
    else if(lane_command && args[1]=="left") action=Maneuver::LaneLeft;
    else if(lane_command && args[1]=="right") action=Maneuver::LaneRight;
    else if(!lane_command && args[1]=="straight") action=Maneuver::Straight;
    else if(!lane_command && args[1]=="left") action=Maneuver::Left;
    else if(!lane_command && args[1]=="right") action=Maneuver::Right;
    else if(!lane_command && args[1]=="lane_left") action=Maneuver::LaneLeft;
    else if(!lane_command && args[1]=="lane_right") action=Maneuver::LaneRight;
    else return fail("invalid_action");
    const bool target_lane=lane_command || action==Maneuver::LaneLeft || action==Maneuver::LaneRight;
    Intent &target=target_lane ? lane_intent_:intent_;
    if(args[2].empty() || args[2].find_first_not_of("0123456789")!=std::string::npos) return fail("invalid_sequence");
    uint64_t seq; try{seq=std::stoull(args[2]);}catch(...){return fail("invalid_sequence");}
    if(!seq || args[3].size()>32 || args[3].find_first_not_of("abcdefghijklmnopqrstuvwxyzABCDEFGHIJKLMNOPQRSTUVWXYZ0123456789_-")!=std::string::npos)
      return fail("invalid_client_or_sequence");
    const double now=now_ms();
    if(target.client==args[3] && seq<=target.seq) return fail("stale_sequence");
    if(target.client!=args[3] && target.action!=Maneuver::Stop &&
       now-target.received_ms<s.command_ttl_ms && action!=Maneuver::Stop) return fail("client_busy");
    target={action,seq,now,args[3]};
    response="OK "+args[0]+" action="+std::string(maneuver_name(action))+" seq="+std::to_string(seq); return true;
  }
  if(args[0]=="MODE") {
    if(args.size()!=2 || (args[1]!="assist" && args[1]!="off")) return fail("MODE_requires_assist_or_off");
    s.enabled=args[1]=="assist";
  } else if(args[0]=="MODEL") {
    if(args.size()!=3 || (args[1]!="det" && args[1]!="cls") || (args[2]!="fp" && args[2]!="int8"))
      return fail("MODEL_requires_det/cls_fp/int8");
    (args[1]=="det" ? s.det_profile:s.cls_profile)=args[2];
  } else if(args[0]=="LANE_TRAP") {
    if(args.size()!=5) return fail("LANE_TRAP_requires_tl_tr_bl_br");
    std::array<float,4> trap;
    for(int k=0;k<4;++k) if(!number(args[k+1],0,1,trap[k])) return fail("invalid_lane_trapezoid");
    if(trap[1]-trap[0]<.1f || trap[3]-trap[2]<.2f) return fail("lane_trapezoid_too_narrow");
    s.lane_trap_tl=trap[0];s.lane_trap_tr=trap[1];s.lane_trap_bl=trap[2];s.lane_trap_br=trap[3];
  } else if(args[0]=="LANE_ROI") {
    if(args.size()!=5) return fail("LANE_ROI_requires_four_coordinates");
    std::array<float,4> roi;
    for(int k=0;k<4;++k) if(!number(args[k+1],0,1,roi[k])) return fail("invalid_lane_roi");
    if(roi[2]-roi[0]<.2f || roi[3]-roi[1]<.15f) return fail("lane_roi_too_small");
    s.lane_x1=roi[0];s.lane_top=roi[1];s.lane_x2=roi[2];s.lane_bottom=roi[3];
  } else if(args[0]=="PEDESTRIAN_PATH") {
    if(args.size()!=3) return fail("PEDESTRIAN_PATH_requires_left_right");
    float left,right;
    if(!number(args[1],0,1,left) || !number(args[2],0,1,right) || right-left<.1f)
      return fail("invalid_pedestrian_path");
    s.pedestrian_path_left=left;s.pedestrian_path_right=right;
  } else if(args[0]=="ROI") {
    if(args.size()!=5) return fail("ROI_requires_four_normalized_coordinates");
    std::array<float,4> roi;
    for(int k=0;k<4;++k) if(!number(args[k+1],0,1,roi[k])) return fail("invalid_ROI");
    if(roi[2]<=roi[0] || roi[3]<=roi[1]) return fail("invalid_ROI");
    s.signal_roi=roi; s.signal_confirmed=false;
  } else if(args[0]=="SET") {
    if(args.size()!=3) return fail("SET_requires_key_value");
    const auto &key=args[1]; float v;
    if(key=="camera_order" || key=="infer_order") {
      if(!valid_camera_order(args[2])) return fail("invalid_camera_order");
      s.infer_order=args[2];
      if(key=="camera_order") s.view_order=args[2];
    } else if(key=="view_order") {
      if(!valid_camera_order(args[2])) return fail("view_order_requires_bgr_rgb_brg_rbg_gbr_grb");
      s.view_order=args[2];response="OK view_order="+s.view_order;return true;
    } else if(key=="color_compare" || key=="lane_debug") {
      if(args[2]!="0" && args[2]!="1") return fail("invalid_boolean");
      (key=="color_compare" ? s.color_compare:s.lane_debug)=args[2]=="1";
      response="OK "+key+"="+args[2];return true;
    } else if(key=="lane_trap_tl" || key=="lane_trap_tr" || key=="lane_trap_bl" || key=="lane_trap_br") {
      if(!number(args[2],0,1,v)) return fail("invalid_lane_trapezoid");
      Settings next=s;
      if(key=="lane_trap_tl") next.lane_trap_tl=v;
      else if(key=="lane_trap_tr") next.lane_trap_tr=v;
      else if(key=="lane_trap_bl") next.lane_trap_bl=v;
      else next.lane_trap_br=v;
      if(next.lane_trap_tr-next.lane_trap_tl<.1f || next.lane_trap_br-next.lane_trap_bl<.2f)
        return fail("lane_trapezoid_too_narrow");
      s=next;
    } else if(key=="lane_vehicle_x") {
      if(!number(args[2],.1f,.9f,v)) return fail("invalid_lane_vehicle_x");
      s.lane_vehicle_x=v;
    } else if(key=="lane_center") {
      if(!number(args[2],.25f,.75f,v)) return fail("invalid_lane_center");
      s.lane_center=v;
    } else if(key=="pedestrian_path_left" || key=="pedestrian_path_right") {
      if(!number(args[2],0,1,v)) return fail("invalid_pedestrian_path");
      if((key=="pedestrian_path_left" && v>s.pedestrian_path_right-.1f) ||
         (key=="pedestrian_path_right" && v<s.pedestrian_path_left+.1f))
        return fail("pedestrian_path_too_narrow");
      (key=="pedestrian_path_left" ? s.pedestrian_path_left:s.pedestrian_path_right)=v;
    } else if(key=="person_conf" || key=="crosswalk_conf" || key=="light_conf" || key=="cls_conf" || key=="nms_iou") {
      if(!number(args[2],.01f,.99f,v)) return fail("invalid_threshold");
      if(key=="person_conf") s.person_conf=v; else if(key=="crosswalk_conf") s.crosswalk_conf=v;
      else if(key=="light_conf") s.light_conf=v; else if(key=="cls_conf") s.cls_conf=v; else s.nms_iou=v;
    } else if(key=="lane_x1" || key=="lane_x2") {
      if(!number(args[2],0,1,v)) return fail("invalid_lane_roi");
      if((key=="lane_x1" && v>s.lane_x2-.2f) || (key=="lane_x2" && v<s.lane_x1+.2f)) return fail("lane_roi_too_narrow");
      (key=="lane_x1" ? s.lane_x1:s.lane_x2)=v;
    } else if(key=="lane_top" || key=="lane_bottom" || key=="lane_horizon") {
      if(!number(args[2],0,1,v)) return fail("invalid_lane_roi");
      if((key=="lane_top" && v>s.lane_bottom-.15f) || (key=="lane_bottom" && v<s.lane_top+.15f)) return fail("lane_roi_too_small");
      if(key=="lane_horizon") s.lane_horizon=v;
      else (key=="lane_top" ? s.lane_top:s.lane_bottom)=v;
    } else {
      float low=0,high=0;
      if(key=="infer_interval") {low=1;high=30;} else if(key=="lane_hz") {low=1;high=20;}
      else if(key=="lane_contrast") {low=3;high=60;}
      else if(key=="front_view") high=0;
      else if(key=="max_lights") {low=1;high=32;}
      else if(key=="result_ttl_ms" || key=="lane_ttl_ms" || key=="command_ttl_ms") {low=100;high=5000;}
      else if(key=="pedestrian_hold_ms") {low=100;high=3000;}
      else if(key=="green_confirm") {low=1;high=10;}
      else if(key=="lane_scan_rows") {low=60;high=180;}
      else if(key=="lane_enabled" || key=="pedestrian_enabled" ||
              key=="draw_all" ||
              key=="signal_confirmed" || key=="lane_debug") high=1;
      else return fail("unknown_key");
      if(key=="front_view" && args[2]!="0") return fail("Q1_only_front_view_must_be_0");
      if(!number(args[2],low,high,v) || std::floor(v)!=v) return fail("invalid_integer");
      const int n=static_cast<int>(v);
      if(key=="infer_interval") s.infer_interval=n; else if(key=="lane_hz") s.lane_hz=n;
      else if(key=="lane_contrast") s.lane_contrast=n; else if(key=="lane_debug") s.lane_debug=n;
      else if(key=="front_view") {s.front_view=n;s.signal_confirmed=false;}
      else if(key=="max_lights") s.max_lights=n; else if(key=="result_ttl_ms") s.result_ttl_ms=n;
      else if(key=="lane_ttl_ms") s.lane_ttl_ms=n; else if(key=="command_ttl_ms") s.command_ttl_ms=n;
      else if(key=="pedestrian_hold_ms") s.pedestrian_hold_ms=n;
      else if(key=="green_confirm") s.green_confirm=n;
      else if(key=="lane_scan_rows") s.lane_scan_rows=n;
      else if(key=="lane_enabled") s.lane_enabled=n;
      else if(key=="pedestrian_enabled") s.pedestrian_enabled=n;
      else if(key=="draw_all") s.draw_all=n; else s.signal_confirmed=n;
    }
  } else return fail("unknown_command");
  ++s.generation; signals_=SignalSummary{};pedestrians_=PedestrianSummary{};
  response="OK generation="+std::to_string(s.generation); return true;
}
} // namespace adas
