#include "recognition_control.h"
#include "detector_decode.h"
#include "lane_detector.h"
#include "latest_mailbox.h"
#include <cassert>
#include <iostream>

using namespace adas;
namespace {
void rule_tests() {
  Settings s;s.signal_confirmed=true;
  const double now=10000;
  Intent i{Maneuver::Straight,1,now-10,"test"};
  SignalSummary signal;signal.generation=s.generation;signal.captured_ms=now-50;
  LaneResult lane;lane.generation=s.generation;lane.captured_ms=now-50;lane.view=0;
  signal.straight=Light::StraightRed;
  assert(evaluate(s,i,signal,lane,now).state=="stop");
  assert(evaluate(s,i,signal,lane,now).warning);
  signal.yellow=true;assert(evaluate(s,i,signal,lane,now).state=="stop");signal.yellow=false;
  signal.straight=Light::StraightGreen;signal.straight_green_streak=2;
  assert(evaluate(s,i,signal,lane,now).state=="allow");
  s.signal_confirmed=false;
  assert(evaluate(s,i,signal,lane,now).state=="unknown");s.signal_confirmed=true;
  signal.yellow=true;assert(evaluate(s,i,signal,lane,now).warning);signal.yellow=false;
  signal.captured_ms=now-s.result_ttl_ms-1;assert(evaluate(s,i,signal,lane,now).state=="unknown");signal.captured_ms=now-50;
  i.action=Maneuver::Left;signal.left=Light::LeftGreen;signal.left_green_streak=2;
  assert(evaluate(s,i,signal,lane,now).state=="allow");
  lane.left.kind=LineKind::Solid;assert(evaluate(s,i,signal,lane,now).state=="allow");
  signal.left=Light::LeftRed;assert(evaluate(s,i,signal,lane,now).state=="stop");
  signal.left=Light::LeftGreen;
  lane.left.kind=LineKind::Dashed;assert(evaluate(s,i,signal,lane,now).state=="allow");
  lane.right.kind=LineKind::Solid;assert(!evaluate(s,i,signal,lane,now).warning);
  i.action=Maneuver::Right;signal.right=Light::Unknown;signal.right_seen_ms=0;
  signal.straight=Light::StraightRed;
  assert(evaluate(s,i,signal,lane,now).state=="unknown");
  signal.straight_seen_ms=now-50;
  assert(evaluate(s,i,signal,lane,now).reason=="right_turn_yield");
  signal.straight=Light::StraightGreen;signal.straight_green_streak=2;
  assert(evaluate(s,i,signal,lane,now).reason=="right_turn_yield");
  signal.yellow=true;assert(evaluate(s,i,signal,lane,now).state=="warning");signal.yellow=false;
  signal.straight_seen_ms=now-s.result_ttl_ms-1;
  signal.left=Light::LeftRed;signal.left_seen_ms=now-50;
  assert(evaluate(s,i,signal,lane,now).reason=="right_turn_yield");
  signal.left_seen_ms=0;
  assert(evaluate(s,i,signal,lane,now).state=="unknown");
  signal.straight_seen_ms=now-50;
  i.action=Maneuver::Right;signal.right=Light::RightRed;
  assert(evaluate(s,i,signal,lane,now).state=="stop");
  signal.right=Light::RightGreen;signal.right_green_streak=2;
  assert(evaluate(s,i,signal,lane,now).state=="allow");
  i.action=Maneuver::LaneRight;assert(evaluate(s,i,signal,lane,now).warning);
  lane.right.kind=LineKind::Dashed;assert(evaluate(s,i,signal,lane,now).state=="allow");
  lane.generation++;assert(evaluate(s,i,signal,lane,now).state=="unknown");
  i.received_ms=now-5000;assert(evaluate(s,i,signal,lane,now).state=="idle");
  Perception p;p.generation=s.generation;p.frame_seq=1;p.captured_ms=now;
  Detection d;d.class_id=2;d.view=0;d.box={200,10,230,40};d.light=Light::StraightGreen;d.light_score=.9f;
  p.objects={d};auto summary=summarize_signals(p,s,{});
  assert(summary.straight_green_streak==1);
  p.frame_seq=2;p.captured_ms+=50;summary=summarize_signals(p,s,summary);
  assert(summary.straight_green_streak==2);
  p.frame_seq=3;p.captured_ms+=50;p.objects.clear();summary=summarize_signals(p,s,summary);
  assert(summary.straight==Light::StraightGreen && summary.straight_green_streak==2);
  p.frame_seq=4;p.captured_ms+=50;p.objects={d};p.objects[0].light=Light::Off;
  assert(summarize_signals(p,s,summary).straight==Light::Unknown);
  p.objects={d};
  d.light=Light::StraightRed;p.objects.push_back(d);
  assert(summarize_signals(p,s,summary).straight==Light::StraightRed);
  p.objects[1].view=1;
  assert(summarize_signals(p,s,summary).straight==Light::StraightGreen);
}
void decoder_tests() {
  std::array<std::vector<float>,3> boxes,scores;
  std::array<DetectorHead,3> heads;
  for(int n=0;n<3;++n) {
    const int h=48>>n,w=80>>n,cells=h*w;
    boxes[n].assign(64*cells,-80);scores[n].assign(3*cells,0);
    heads[n]={boxes[n].data(),scores[n].data(),h,w,false,false};
  }
  const int cell=12*80+25,cells=48*80;
  for(int side=0;side<4;++side) boxes[0][(side*16+2)*cells+cell]=80;
  scores[0][2*cells+cell]=.3f;
  Settings s;
  auto result=decode_heads(heads,s);assert(result.size()==1);
  assert(result[0].class_id==2 && result[0].view==0);
  assert(std::abs(result[0].box.x1-188)<.01 && std::abs(result[0].box.y1-72)<.01);
  assert(result[0].box.x2<=640 && result[0].box.y2<=360 && result[0].view==0);
  const auto expected=result[0].box;
  std::vector<float> b(boxes[0].size()),c(scores[0].size());
  for(int index=0;index<cells;++index) {
    for(int channel=0;channel<64;++channel) b[index*64+channel]=boxes[0][channel*cells+index];
    for(int channel=0;channel<3;++channel) c[index*3+channel]=scores[0][channel*cells+index];
  }
  heads[0].boxes=b.data();heads[0].scores=c.data();heads[0].boxes_nhwc=heads[0].scores_nhwc=true;
  result=decode_heads(heads,s);assert(result.size()==1 && result[0].box.x1==expected.x1);
  s.light_conf=.31f;assert(decode_heads(heads,s).empty());
}
void lane_tests() {
  constexpr int w=192,h=96;std::vector<uint8_t> mask(w*h,0);
  for(int y=0;y<h;++y) {
    const int left=static_cast<int>(w*(.38f-.22f*y/(h-1)));
    const int right=static_cast<int>(w*(.62f+.22f*y/(h-1)));
    for(int dx=-2;dx<=2;++dx) {
      mask[y*w+left+dx]=1;
      if((y/12)%2==0) mask[y*w+right+dx]=1;
    }
  }
  LaneScanner scanner;std::pair<LaneEdge,LaneEdge> lanes;
  for(int n=0;n<3;++n) lanes=scanner.detect(mask.data(),w,h);
  assert(lanes.first.kind==LineKind::Solid && lanes.second.kind==LineKind::Dashed);
  // Strong screen edges beside actual lanes must not displace the valid lane fits.
  for(int y=0;y<h;++y) for(int dx=-2;dx<=2;++dx) {
    mask[y*w+8+dx]=1;mask[y*w+w-9+dx]=1;
  }
  scanner.reset();for(int n=0;n<3;++n) lanes=scanner.detect(mask.data(),w,h);
  assert(lanes.first.kind==LineKind::Solid && lanes.second.kind==LineKind::Dashed);
  std::fill(mask.begin(),mask.end(),0);
  lanes=scanner.detect(mask.data(),w,h);
  assert(lanes.first.kind==LineKind::Unknown && lanes.second.kind==LineKind::Unknown);
  // Regression: a bright monitor bezel / vertical pole must never become a solid boundary.
  std::fill(mask.begin(),mask.end(),0);scanner.reset();
  for(int y=0;y<h;++y) for(int dx=-2;dx<=2;++dx) {
    mask[y*w+8+dx]=1;mask[y*w+w-9+dx]=1;
  }
  for(int n=0;n<3;++n) lanes=scanner.detect(mask.data(),w,h);
  assert(!lanes.first.fitted && !lanes.second.fitted);
  // A known occluder is not evidence of a dashed boundary.
  std::fill(mask.begin(),mask.end(),0);scanner.reset();
  for(int y=0;y<h;++y) {
    const int x=static_cast<int>(w*(.38f-.22f*y/(h-1)));
    for(int dx=-3;dx<=3;++dx) mask[y*w+x+dx]=(y>30 && y<45) ? 2:1;
  }
  for(int n=0;n<3;++n) lanes=scanner.detect(mask.data(),w,h);
  assert(lanes.first.kind!=LineKind::Dashed);
  // Structurally rejected reflection samples are not measured road gaps.
  scanner.reset();
  for(int y=0;y<h;++y) {
    const int x=static_cast<int>(w*(.38f-.22f*y/(h-1)));
    for(int dx=-3;dx<=3;++dx) mask[y*w+x+dx]=(y>30 && y<45) ? 5:1;
  }
  for(int n=0;n<3;++n) lanes=scanner.detect(mask.data(),w,h);
  assert(lanes.first.kind==LineKind::Solid && lanes.first.gap_count==0);
  // A few uncertain dash-end pixels must not erase genuine measured gaps.
  std::fill(mask.begin(),mask.end(),0);scanner.reset();
  for(int y=0;y<h;++y) {
    const int x=static_cast<int>(w*(.38f-.22f*y/(h-1)));
    const int phase=y%24;
    for(int dx=-3;dx<=3;++dx) if(phase<14) mask[y*w+x+dx]=phase>=12 ? 5:1;
  }
  for(int n=0;n<3;++n) lanes=scanner.detect(mask.data(),w,h);
  assert(lanes.first.kind==LineKind::Dashed && lanes.first.gap_count>=2);
  // One unexplained occlusion alone is not enough to declare a dashed line.
  std::fill(mask.begin(),mask.end(),0);scanner.reset();
  for(int y=0;y<h;++y) if(y<30 || y>50) {
    const int x=static_cast<int>(w*(.38f-.22f*y/(h-1)));
    for(int dx=-2;dx<=2;++dx) mask[y*w+x+dx]=1;
  }
  for(int n=0;n<3;++n) lanes=scanner.detect(mask.data(),w,h);
  assert(lanes.first.kind!=LineKind::Dashed);
  // Removal of all current paint must immediately discard the prior fit.
  std::fill(mask.begin(),mask.end(),0);
  lanes=scanner.detect(mask.data(),w,h);
  assert(!lanes.first.fitted && lanes.first.kind==LineKind::Unknown);
  for(int y=10;y<90;y+=12) for(int dy=0;dy<5;++dy)
    for(int x=15;x<w-15;++x) mask[(y+dy)*w+x]=1;
  scanner.reset();lanes=scanner.detect(mask.data(),w,h);
  assert(lanes.first.kind==LineKind::Unknown && lanes.second.kind==LineKind::Unknown);
}
void runtime_tests() {
  Runtime r;std::string reply;
  const auto generation=r.settings().generation;
  assert(!r.command("SET cls_conf nan",reply));
  assert(!r.command("SET lane_hz 0",reply));
  assert(!r.command("SET lane_scan_rows 59",reply));
  assert(!r.command("SET lane_scan_rows 181",reply));
  assert(!r.command("SET lane_vehicle_x nan",reply));
  assert(!r.command("SET lane_vehicle_x .95",reply));
  assert(!r.command("SET lane_contrast nan",reply));
  assert(!r.command("SET lane_contrast 0",reply));
  assert(!r.command("SET lane_x1 .95",reply));
  assert(!r.command("SET lane_x2 .05",reply));
  assert(!r.command("LANE_TRAP .8 .2 .1 .9",reply));
  assert(!r.command("LANE_TRAP 0 nan 0 1",reply));
  assert(!r.command("SET lane_trap_tl .9",reply));
  assert(!r.command("ROI 1 0 0 1",reply));
  assert(r.settings().generation==generation);
  assert(r.command("SET view_order rbg",reply));
  assert(r.command("SET color_compare 1",reply));
  assert(r.command("SNAPSHOT",reply));
  assert(r.snapshot_requested.load()==1 && r.settings().generation==generation);
  assert(!r.command("SET view_order rrr",reply));
  assert(!r.command("LANE_ROI .9 .8 .1 .2",reply));
  assert(r.command("DRIVE straight 1 client",reply));
  assert(r.settings().generation==generation);
  assert(!r.command("DRIVE right 1 client",reply));
  assert(!r.command("DRIVE left 2 another",reply));
  assert(r.command("LANE left 1 client",reply));
  assert(r.intent().action==Maneuver::Straight && r.lane_intent().action==Maneuver::LaneLeft);
  assert(r.command("LANE stop 2 client",reply));
  assert(r.command("DRIVE stop 2 client",reply));
  assert(r.command("MODEL det int8",reply));
  const auto s=r.settings();assert(s.det_profile=="int8");
  r.reload_failed(s.generation,"fp","fp","test_failure");
  assert(r.settings().det_profile=="fp");
  assert(!r.command("SET front_view 2",reply));
  assert(r.command("SET front_view 0",reply));
  auto p=std::make_shared<Perception>();p->generation=generation;p->captured_ms=now_ms();
  r.publish_perception(p);assert(!r.perception());
  assert(r.status().find("front_view=0")!=std::string::npos);
  const auto color_generation=r.settings().generation;
  assert(r.command("SET camera_order rbg",reply));
  assert(r.settings().generation==color_generation+1 && r.settings().infer_order=="rbg" && r.settings().view_order=="rbg");
  assert(r.command("SET lane_scan_rows 100",reply));
  assert(r.settings().lane_scan_rows==100);
  assert(r.command("SET lane_vehicle_x .5",reply));
  const auto trap_generation=r.settings().generation;
  assert(r.command("LANE_TRAP .25 .75 .04 .96",reply));
  assert(r.settings().generation==trap_generation+1 && r.settings().lane_trap_tl==.25f);
  assert(r.command("SET lane_trap_tl .30",reply));
  const auto valid=r.settings();
  assert(!r.command("SET lane_trap_tr .31",reply));
  assert(r.settings().generation==valid.generation && r.settings().lane_trap_tr==valid.lane_trap_tr);
  assert(r.command("STATUS CONFIG",reply));assert(reply.find("lane_trap_tl=0.300")!=std::string::npos);
}
void udp_status_tests() {
  Runtime r;std::string reply;
  auto p=std::make_shared<Perception>();p->generation=r.settings().generation;p->captured_ms=now_ms();
  p->error=std::string(500,'x');r.publish_perception(p);
  auto lane=std::make_shared<LaneResult>();lane->generation=r.settings().generation;lane->captured_ms=now_ms();
  lane->error=std::string(500,'y');r.publish_lane(lane);
  r.loaded("int8","int8",std::string(500,'z'));
  r.frames=UINT64_MAX;r.capture_errors=UINT64_MAX;r.npu_skipped=UINT64_MAX;
  for(const char *command:{"STATUS","STATUS CONFIG","STATUS LANE"}) {
    assert(r.command(command,reply));assert(reply.size()<1400);
  }
  assert(reply.find("lane_pre_ms=")!=std::string::npos);
  assert(reply.find("lane_mask_ms=")!=std::string::npos);
  assert(reply.find("lane_seed_ms=")!=std::string::npos);
  assert(reply.find("lane_fit_ms=")!=std::string::npos);
  assert(reply.find("lane_cpu_ms=")!=std::string::npos);
  assert(!r.command("STATUS INVALID",reply));
}
void weak_evidence_tests() {
  constexpr int w=192,h=96;
  std::vector<uint8_t> strong(w*h,0),weak(w*h,1);
  LaneScanner scanner;
  for(int n=0;n<3;++n) {
    const auto result=scanner.detect(strong.data(),w,h,-20,.5f,{},.5f,weak.data());
    assert(!result.first.fitted && !result.second.fitted);
  }
}
void lane_reacquisition_tests() {
  constexpr int w=192,h=96;
  auto road=[](int shift) {
    std::vector<uint8_t> mask(w*h,0);
    for(int y=0;y<h;++y) {
      const float t=static_cast<float>(y)/(h-1);
      for(int x:{static_cast<int>(w*(.38f-.22f*t))+shift,static_cast<int>(w*(.62f+.22f*t))+shift})
        for(int dx=-2;dx<=2;++dx) if(x+dx>=0 && x+dx<w) mask[y*w+x+dx]=1;
    }
    return mask;
  };
  LaneScanner scanner;auto first=road(0);std::pair<LaneEdge,LaneEdge> result;
  for(int n=0;n<3;++n) result=scanner.detect(first.data(),w,h);
  assert(result.first.kind==LineKind::Solid && result.second.kind==LineKind::Solid);
  const auto shifted=road(20);const auto jump=scanner.detect(shifted.data(),w,h);
  assert(!jump.first.fitted && !jump.second.fitted);
  for(int n=0;n<3;++n) result=scanner.detect(shifted.data(),w,h);
  assert(result.first.kind==LineKind::Solid && result.second.kind==LineKind::Solid);
  assert(std::abs(result.first.x_at(h-1)-(w*.16f+20))<4);
  std::vector<uint8_t> blank(w*h,0);result=scanner.detect(blank.data(),w,h);
  assert(!result.first.fitted && !result.second.fitted);
}
void confirmation_tests() {
  Runtime r;std::string reply;uint64_t sequence=0;
  assert(r.settings().signal_confirmed);
  auto green=[&] {
    auto p=std::make_shared<Perception>();
    p->generation=r.settings().generation;p->frame_seq=++sequence;p->captured_ms=now_ms();
    Detection light;light.class_id=2;light.view=0;light.box={200,20,240,60};
    light.light=Light::StraightGreen;light.light_score=.95f;p->objects={light};
    r.publish_perception(p);
  };
  assert(r.command("DRIVE straight 1 confirmation",reply));
  green();assert(r.decision().reason=="green_confirming");
  green();assert(r.decision().state=="allow");
  assert(r.command("ROI 0 0 1 1",reply));
  assert(!r.settings().signal_confirmed);
  green();green();assert(r.decision().reason=="signal_assignment_required");
  assert(r.command("SET signal_confirmed 1",reply));
  green();green();assert(r.decision().state=="allow");
  assert(r.command("SET signal_confirmed 0",reply));
  green();green();assert(r.decision().state=="unknown");
}
void pedestrian_brake_tests() {
  Settings s;const double started=10000;
  Perception p;p.generation=s.generation;p.frame_seq=1;p.captured_ms=started;
  Detection zebra;zebra.class_id=1;zebra.view=0;zebra.score=.9f;zebra.box={120,190,520,260};
  Detection person;person.class_id=0;person.view=0;person.score=.9f;person.box={290,110,330,215};
  p.objects={zebra,person};
  auto first=summarize_pedestrians(p,s,{});
  assert(first.person_seen && first.crosswalk_seen && first.on_crosswalk);
  assert(evaluate_pedestrian(s,first,started).state=="confirming");
  p.frame_seq=2;p.captured_ms+=50;p.objects.clear();
  auto gap=summarize_pedestrians(p,s,first);
  assert(gap.evidence_hits==1 && evaluate_pedestrian(s,gap,p.captured_ms).state=="confirming");
  p.frame_seq=3;p.captured_ms+=50;p.objects={zebra,person};
  auto second=summarize_pedestrians(p,s,gap);
  assert(second.evidence_hits==2 && second.brake_requested);
  assert(evaluate_pedestrian(s,second,p.captured_ms).state=="stop");
  p.frame_seq=4;p.captured_ms+=50;p.objects.clear();
  auto missed=summarize_pedestrians(p,s,second);
  assert(!missed.on_crosswalk && missed.brake_requested);
  assert(evaluate_pedestrian(s,missed,p.captured_ms).state=="stop");
  assert(evaluate_pedestrian(s,missed,second.observed_ms+s.pedestrian_hold_ms+1).state=="clear");
  p.frame_seq=5;p.captured_ms+=50;person.box={25,110,65,215};p.objects={zebra,person};
  auto outside=summarize_pedestrians(p,s,{});
  assert(outside.person_seen && outside.crosswalk_seen && !outside.on_crosswalk);
  person.box={290,30,330,100};p.objects={zebra,person};
  assert(!summarize_pedestrians(p,s,{}).on_crosswalk);
  person.box={290,110,330,215};person.score=.1f;p.objects={zebra,person};
  assert(!summarize_pedestrians(p,s,{}).on_crosswalk);
  person.score=.9f;person.view=1;p.objects={zebra,person};
  assert(!summarize_pedestrians(p,s,{}).on_crosswalk);
  s.pedestrian_enabled=false;
  assert(evaluate_pedestrian(s,second,started+50).state=="idle");

  Runtime runtime;std::string reply;uint64_t sequence=0;
  assert(runtime.command("DRIVE straight 1 pedestrian",reply));
  auto publish=[&](bool occupant) {
    auto frame=std::make_shared<Perception>();frame->generation=runtime.settings().generation;
    frame->frame_seq=++sequence;frame->captured_ms=now_ms();
    Detection light;light.class_id=2;light.view=0;light.box={200,10,230,40};
    light.light=Light::StraightGreen;light.light_score=.9f;
    frame->objects={light};
    if(occupant) frame->objects.insert(frame->objects.end(),{zebra,person});
    runtime.publish_perception(frame);
  };
  person.view=0;
  publish(false);publish(false);
  assert(runtime.signal_decision().state=="allow");
  publish(true);assert(runtime.pedestrian_decision().state=="confirming");
  publish(true);
  assert(runtime.signal_decision().state=="allow");
  assert(runtime.pedestrian_decision().state=="stop");
  assert(runtime.decision().state=="stop" && runtime.decision().reason=="pedestrian_on_crosswalk");
  assert(runtime.status().find("brake_request=1")!=std::string::npos);
  assert(runtime.command("DRIVE stop 2 pedestrian",reply));
  assert(runtime.decision().reason=="pedestrian_on_crosswalk");
  publish(false);assert(runtime.decision().state=="stop");
  assert(runtime.command("SET pedestrian_enabled 0",reply));
  assert(runtime.pedestrian_decision().state=="idle");
  assert(!runtime.command("SET pedestrian_hold_ms 50",reply));
  assert(!runtime.command("SET pedestrian_path_left .80",reply));
  assert(runtime.command("SET pedestrian_path_left .20",reply));
  assert(!runtime.command("PEDESTRIAN_PATH .80 .81",reply));
  assert(runtime.command("PEDESTRIAN_PATH .25 .75",reply));
  assert(runtime.settings().pedestrian_path_left==.25f);
}
void missed_lane_frame_tests() {
  Runtime r;std::string reply;
  assert(r.command("LANE right 1 lanecheck",reply));
  auto visible=std::make_shared<LaneResult>();
  visible->generation=r.settings().generation;visible->captured_ms=now_ms()-100;
  visible->right.kind=LineKind::Dashed;
  r.publish_lane(visible);
  assert(r.lane_decision().state=="allow");
  auto missed=std::make_shared<LaneResult>();
  missed->generation=visible->generation;missed->captured_ms=visible->captured_ms+50;
  r.publish_lane(missed);
  assert(r.lane()->right.kind==LineKind::Dashed);
  assert(r.lane_decision().state=="allow");
  missed->captured_ms=visible->captured_ms+r.settings().lane_ttl_ms+1;
  r.publish_lane(missed);
  assert(r.lane()->right.kind==LineKind::Unknown);
}
void combined_decision_tests() {
  Runtime r;std::string reply;uint64_t sequence=0;
  assert(r.command("DRIVE straight 1 combined",reply));
  assert(r.command("LANE right 1 combined",reply));
  auto lane=std::make_shared<LaneResult>();
  lane->generation=r.settings().generation;lane->captured_ms=now_ms();
  lane->right.kind=LineKind::Dashed;r.publish_lane(lane);
  auto light=[&](Light state) {
    auto p=std::make_shared<Perception>();p->generation=r.settings().generation;
    p->frame_seq=++sequence;p->captured_ms=now_ms();
    Detection d;d.class_id=2;d.view=0;d.box={200,10,230,40};
    d.light=state;d.light_score=.9f;p->objects={d};r.publish_perception(p);
  };
  light(Light::StraightGreen);light(Light::StraightGreen);
  assert(r.decision().state=="allow" && r.decision().reason=="both_allowed");
  light(Light::StraightRed);
  assert(r.decision().state=="stop" && r.lane_decision().state=="allow");
  lane=std::make_shared<LaneResult>();lane->generation=r.settings().generation;
  lane->captured_ms=now_ms();lane->right.kind=LineKind::Solid;r.publish_lane(lane);
  assert(r.decision().state=="stop" && r.lane_decision().warning);
  light(Light::StraightGreen);light(Light::StraightGreen);
  assert(r.decision().state=="warning");
}
void mailbox_tests() {
  LatestMailbox<int> box;std::atomic<bool> stop{false};uint64_t seen=0;
  assert(box.publish(std::make_shared<const int>(1)));
  assert(box.publish(std::make_shared<const int>(2)));
  auto held=box.wait(seen,stop,1);assert(held && *held==2);
  box.publish(std::make_shared<const int>(3));assert(*held==2);
  assert(*box.try_read()==3);
  std::atomic<bool> done{false};
  std::thread slow([&]{uint64_t rev=0;auto v=box.wait(rev,stop);assert(v);std::this_thread::sleep_for(std::chrono::milliseconds(200));done=true;});
  const double start=now_ms();
  for(int n=0;n<1000;++n) box.publish(std::make_shared<const int>(n));
  assert(now_ms()-start<150 && !done);
  slow.join();
}
}
int main() {
  rule_tests();decoder_tests();lane_tests();runtime_tests();udp_status_tests();weak_evidence_tests();lane_reacquisition_tests();confirmation_tests();pedestrian_brake_tests();missed_lane_frame_tests();combined_decision_tests();mailbox_tests();
  std::cout<<"PASS signals, lane, pedestrian brake priority/hold, runtime generations, nonblocking mailbox\n";
}
