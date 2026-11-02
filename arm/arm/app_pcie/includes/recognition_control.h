#pragma once
#include "adas_types.h"
#include <atomic>
#include <memory>
#include <mutex>
#include <thread>

namespace adas {
class Runtime {
public:
  Settings settings() const;
  Intent intent() const;
  Intent lane_intent() const;
  bool command(const std::string &,std::string &);
  std::string status(const std::string &section="full") const;
  void publish_perception(std::shared_ptr<const Perception>);
  void publish_lane(std::shared_ptr<const LaneResult>);
  std::shared_ptr<const Perception> perception() const;
  std::shared_ptr<const LaneResult> lane() const;
  Decision decision() const;
  Decision signal_decision() const;
  Decision lane_decision() const;
  Decision pedestrian_decision() const;
  void configure(const Settings &);
  void loaded(const std::string &,const std::string &,const std::string &error="");
  void reload_failed(uint64_t,const std::string &,const std::string &,const std::string &);
  std::atomic<uint64_t> frames{0},capture_errors{0},npu_skipped{0};
  std::atomic<uint64_t> snapshot_requested{0},snapshot_completed{0};
  std::atomic<double> cap_ms{0},copy_ms{0};
private:
  mutable std::mutex mutex_;
  Settings settings_;
  Intent intent_;
  Intent lane_intent_;
  SignalSummary signals_;
  PedestrianSummary pedestrians_;
  std::string loaded_det_="none",loaded_cls_="none",model_error_;
  std::shared_ptr<const Perception> perception_;
  std::shared_ptr<const LaneResult> lane_;
};
class ControlServer {
public:
  ControlServer(Runtime &,std::string address,int port);
  ~ControlServer();
  bool start(std::string &error);
  void stop();
private:
  void run();
  Runtime &runtime_;
  std::string address_;
  int port_,socket_=-1;
  std::atomic<bool> stopping_{false};
  std::thread worker_;
};
} // namespace adas
