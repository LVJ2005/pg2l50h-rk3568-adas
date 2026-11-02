#pragma once
#include <atomic>
#include <chrono>
#include <condition_variable>
#include <memory>
#include <mutex>
#include <utility>

// Never holds a lock while copying pixels, processing images or running RKNN.
template<class T> class LatestMailbox {
public:
  bool publish(std::shared_ptr<const T> value) {
    {
      std::unique_lock<std::mutex> lock(mutex_,std::try_to_lock);
      if(!lock.owns_lock()) { rejected_++; return false; }
      latest_.swap(value);
      ++revision_;
    }
    changed_.notify_all();
    return true;
  }
  std::shared_ptr<const T> try_read() const {
    std::unique_lock<std::mutex> lock(mutex_,std::try_to_lock);
    return lock.owns_lock() ? latest_:nullptr;
  }
  std::shared_ptr<const T> wait(uint64_t &seen,const std::atomic<bool> &stop,int ms=50) const {
    std::unique_lock<std::mutex> lock(mutex_);
    changed_.wait_for(lock,std::chrono::milliseconds(ms),[&]{return stop.load() || revision_!=seen;});
    if(stop || revision_==seen) return nullptr;
    seen=revision_;
    return latest_;
  }
  void wake() const { changed_.notify_all(); }
  uint64_t rejected() const { return rejected_.load(); }
private:
  mutable std::mutex mutex_;
  mutable std::condition_variable changed_;
  std::shared_ptr<const T> latest_;
  uint64_t revision_=0;
  std::atomic<uint64_t> rejected_{0};
};
