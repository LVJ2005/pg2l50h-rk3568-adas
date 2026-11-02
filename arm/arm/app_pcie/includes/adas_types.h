#pragma once

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <string>
#include <vector>

namespace adas {
inline bool valid_camera_order(const std::string &order) {
  return order=="bgr" || order=="rgb" || order=="brg" || order=="rbg" || order=="gbr" || order=="grb";
}
inline double now_ms() {
  return std::chrono::duration<double, std::milli>(
      std::chrono::steady_clock::now().time_since_epoch()).count();
}
struct Box {
  float x1 = 0, y1 = 0, x2 = 0, y2 = 0;
  float width() const { return std::max(0.f, x2 - x1); }
  float height() const { return std::max(0.f, y2 - y1); }
};
inline float overlap(const Box &a, const Box &b) {
  const float w = std::max(0.f, std::min(a.x2, b.x2) - std::max(a.x1, b.x1));
  const float h = std::max(0.f, std::min(a.y2, b.y2) - std::max(a.y1, b.y1));
  return w * h / std::max(1e-6f, a.width()*a.height()+b.width()*b.height()-w*h);
}
enum class Light : int {
  Unknown = -1, Yellow = 0, StraightRed, LeftRed, RightRed,
  StraightGreen, RightGreen, LeftGreen, Off
};
inline const char *light_name(Light state) {
  static const char *names[] = {"yellow", "straight_red", "left_red", "right_red",
      "straight_green", "right_green", "left_green", "off"};
  const int n = static_cast<int>(state);
  return n >= 0 && n < 8 ? names[n] : "unknown";
}
enum class Maneuver { Stop, Straight, Left, Right, LaneLeft, LaneRight };
inline const char *maneuver_name(Maneuver m) {
  switch(m) {
    case Maneuver::Straight: return "straight";
    case Maneuver::Left: return "left";
    case Maneuver::Right: return "right";
    case Maneuver::LaneLeft: return "lane_left";
    case Maneuver::LaneRight: return "lane_right";
    default: return "stop";
  }
}
enum class LineKind { Unknown, Solid, Dashed };
enum class PaintColor : uint8_t { Unknown=0, White=1, Yellow=2 };
inline const char *paint_name(PaintColor color) {
  return color==PaintColor::White ? "white":color==PaintColor::Yellow ? "yellow":"unknown";
}
inline const char *line_name(LineKind k) {
  return k == LineKind::Solid ? "solid" : k == LineKind::Dashed ? "dashed" : "unknown";
}
struct Detection {
  Box box;
  int class_id = -1, view = -1;
  float score = 0, light_score = 0;
  Light light = Light::Unknown;
};
struct Perception {
  uint64_t generation = 0, frame_seq = 0;
  double captured_ms = 0, completed_ms = 0;
  double pre_ms = 0, det_ms = 0, post_ms = 0, cls_ms = 0, queue_ms = 0;
  int cls_runs = 0;
  std::vector<Detection> objects;
  std::string error;
};
struct LaneEdge {
  LineKind kind = LineKind::Unknown;
  double observed_ms = 0;
  PaintColor paint = PaintColor::Unknown;
  float confidence = 0, coverage = 0;
  float slope = 0, intercept = 0, curvature = 0;
  int first_y = 0, last_y = 0;
  int support_rows = 0, segment_count = 0, gap_count = 0, longest_gap = 0;
  bool fitted = false;
  float x_at(float y) const { return (curvature*y+slope)*y+intercept; }
};
struct LaneResult {
  uint64_t generation = 0, frame_seq = 0;
  int view = 0;
  double captured_ms = 0, completed_ms = 0, processing_ms = 0;
  double pre_ms = 0, mask_ms = 0, seed_ms = 0, fit_ms = 0, cpu_ms = -1;
  Box roi;
  int width = 320, height = 100;
  LaneEdge left, right;
  float paint_ratio = 0, occlusion_ratio = 0;
  std::vector<uint8_t> mask;
  std::string error;
};
struct Settings {
  uint64_t generation = 1;
  bool enabled = true, lane_enabled = true, draw_all = true;
  bool signal_confirmed = true;
  bool lane_debug = false, pedestrian_enabled = true;
  bool color_compare = false;
  std::string view_order = "rbg", infer_order = "rbg";
  std::string display_backend = "opencv";
  std::string det_profile = "fp", cls_profile = "fp";
  int infer_interval = 1, lane_hz = 8, front_view = 0, max_lights = 8;
  int result_ttl_ms = 1000, lane_ttl_ms = 900, command_ttl_ms = 1500;
  int pedestrian_hold_ms = 700;
  float pedestrian_path_left = .15f, pedestrian_path_right = .85f;
  int green_confirm = 2;
  float person_conf = .40f, crosswalk_conf = .50f, light_conf = .20f;
  float nms_iou = .70f, cls_conf = .65f;
  float lane_top = .58f, lane_bottom = .94f, lane_horizon = .62f;
  float lane_x1 = 0.f, lane_x2 = 1.f;
  // Trapezoid x coordinates relative to the rectangular lane ROI.
  float lane_trap_tl = .28f, lane_trap_tr = .72f;
  float lane_trap_bl = .02f, lane_trap_br = .98f;
  int lane_contrast = 12;
  int lane_scan_rows = 100;
  float lane_center = .5f;
  float lane_vehicle_x = .5f;
  // Coordinates within the selected 640x360 quadrant, normalized to [0,1].
  std::array<float,4> signal_roi{{0.f,0.f,1.f,1.f}};
};
struct Intent {
  Maneuver action = Maneuver::Stop;
  uint64_t seq = 0;
  double received_ms = 0;
  std::string client;
};
struct SignalSummary {
  Light straight = Light::Unknown, left = Light::Unknown, right = Light::Unknown;
  double straight_seen_ms = 0, left_seen_ms = 0, right_seen_ms = 0;
  bool yellow = false, present = false;
  int straight_green_streak = 0, left_green_streak = 0, right_green_streak = 0;
  uint64_t frame_seq = 0, generation = 0;
  double captured_ms = 0;
};
struct PedestrianSummary {
  uint64_t generation = 0, frame_seq = 0;
  double captured_ms = 0, observed_ms = 0;
  bool person_seen = false, crosswalk_seen = false, on_crosswalk = false;
  bool brake_requested = false;
  int evidence_hits = 0;
};
struct Decision {
  std::string state = "idle", reason = "stop";
  bool warning = false;
  double signal_age_ms = -1, lane_age_ms = -1;
};
inline int quadrant(const Box &b) {
  return (((b.y1+b.y2)*.5f >= 360.f) ? 2 : 0) +
         (((b.x1+b.x2)*.5f >= 640.f) ? 1 : 0);
}
inline Box view_box(int view) {
  const float x = (view%2)*640.f, y = (view/2)*360.f;
  return {x,y,x+640.f,y+360.f};
}
SignalSummary summarize_signals(const Perception &, const Settings &, const SignalSummary &);
PedestrianSummary summarize_pedestrians(const Perception &, const Settings &, const PedestrianSummary &);
Decision evaluate(const Settings &, const Intent &, const SignalSummary &, const LaneResult &, double now);
Decision evaluate_pedestrian(const Settings &, const PedestrianSummary &, double now);
} // namespace adas
