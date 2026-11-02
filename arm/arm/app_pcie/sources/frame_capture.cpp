#include "frame_capture.h"
#include <opencv2/imgproc.hpp>
#include <opencv2/videoio.hpp>
#include <cerrno>
#include <cstring>
#include <fcntl.h>
#include <iostream>
#include <stdexcept>
#include <sys/ioctl.h>
#include <sys/mman.h>
#include <unistd.h>

namespace adas {
namespace {
constexpr size_t kFrameBytes=1280*720*3;
constexpr size_t kMapBytes=2*kFrameBytes;
constexpr unsigned long kMap=_IOWR('S',2,int),kStart=_IOWR('S',5,int);
constexpr unsigned long kWait=_IOWR('S',6,int),kUnmap=_IOWR('S',7,int);
constexpr unsigned long kMeta=_IOWR('S',10,int),kRelease=_IOWR('S',11,int);
struct DmaOperation {unsigned current_len,offset_addr,cmd;uint8_t read_buf[4096],write_buf[4096];};
struct MsiOperation {unsigned flag,index,size,seq;};
void checked(int fd,unsigned long command,void *arg,const char *name) {
  if(ioctl(fd,command,arg)<0) throw std::runtime_error(std::string(name)+": "+std::strerror(errno));
}
void capture_video(LatestMailbox<Frame> &hub,Runtime &runtime,std::atomic<bool> &stop,const std::string &path) {
  cv::VideoCapture video(path);
  if(!video.isOpened()) throw std::runtime_error("cannot open test video");
  const double fps=video.get(cv::CAP_PROP_FPS),period=1000.0/(fps>0 && fps<=120 ? fps:30);
  uint64_t seq=0;
  while(!stop) {
    const double start=now_ms();cv::Mat source;
    if(!video.read(source)) {video.set(cv::CAP_PROP_POS_FRAMES,0);if(!video.read(source)) break;}
    auto frame=std::make_shared<Frame>();
    if(source.cols!=1280 || source.rows!=720) cv::resize(source,frame->bgr,cv::Size(1280,720));
    else frame->bgr=source;
    frame->sequence=++seq;frame->captured_ms=now_ms();
    hub.publish(frame);runtime.frames++;runtime.cap_ms=now_ms()-start;
    const int delay=static_cast<int>(period-(now_ms()-start));
    for(int ms=0;ms<delay && !stop;ms+=5) std::this_thread::sleep_for(std::chrono::milliseconds(5));
  }
}
}
void capture_frames(LatestMailbox<Frame> &hub,Runtime &runtime,std::atomic<bool> &stop,const std::string &video,bool use_mmap) {
  int fd=-1;void *mapping=MAP_FAILED;bool mapped=false;
  DmaOperation operation{};operation.current_len=1280*3/4;
  try {
    if(!video.empty()) {capture_video(hub,runtime,stop,video);return;}
    fd=open("/dev/pango_pci_driver",O_RDWR);
    if(fd<0) throw std::runtime_error("open PCIe driver: "+std::string(std::strerror(errno)));
    checked(fd,kMap,&operation,"DMA map");mapped=true;
    if(use_mmap) {
      mapping=mmap(nullptr,kMapBytes,PROT_READ|PROT_WRITE,MAP_SHARED,fd,0);
      if(mapping==MAP_FAILED) std::cerr<<"[CAPTURE] mmap failed; using copy ioctl"<<std::endl;
    }
    std::vector<uint8_t> copied(mapping==MAP_FAILED ? kFrameBytes:0);
    checked(fd,kStart,&operation,"DMA start");
    uint64_t seq=0;
    while(!stop) {
      const double start=now_ms();
      if(ioctl(fd,kWait,mapping==MAP_FAILED ? copied.data():nullptr)<0) {
        runtime.capture_errors++;
        if(!stop) std::cerr<<"[CAPTURE] full-frame wait: "<<std::strerror(errno)<<std::endl;
        std::this_thread::sleep_for(std::chrono::milliseconds(20));continue;
      }
      const double received=now_ms();bool owned=true;
      try {
        MsiOperation meta{};checked(fd,kMeta,&meta,"DMA metadata");
        if(meta.flag!=1 || meta.size!=kFrameBytes || meta.index>=2) throw std::runtime_error("invalid full-frame metadata");
        const uint8_t *data=mapping==MAP_FAILED ? copied.data():static_cast<uint8_t*>(mapping)+meta.index*kFrameBytes;
        auto frame=std::make_shared<Frame>();
        // FPGA cannot reuse this DMA slot until the private pixel copy is complete.
        frame->bgr=cv::Mat(720,1280,CV_8UC3,const_cast<uint8_t*>(data)).clone();
        frame->captured_ms=received;frame->sequence=++seq;
        checked(fd,kRelease,&meta,"DMA release");owned=false;
        runtime.cap_ms=received-start;runtime.copy_ms=now_ms()-received;
        hub.publish(frame);runtime.frames++;
      } catch(...) {
        if(owned) ioctl(fd,kRelease,nullptr);
        throw;
      }
    }
  } catch(const std::exception &e) {
    runtime.capture_errors++;std::cerr<<"[CAPTURE] "<<e.what()<<std::endl;
  }
  if(mapping!=MAP_FAILED) munmap(mapping,kMapBytes);
  if(mapped) ioctl(fd,kUnmap,&operation);
  if(fd>=0) close(fd);
  hub.wake();
}
} // namespace adas
