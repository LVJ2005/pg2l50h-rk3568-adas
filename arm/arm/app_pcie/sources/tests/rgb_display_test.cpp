#include "rgb_display.h"
#include "frame_color.h"
#include <opencv2/imgproc.hpp>
#include <cassert>
#include <chrono>
#include <thread>
#include <iostream>

int main(int argc,char **argv) {
  if(argc>1 && std::string(argv[1])=="opencv") {
    adas::RgbDisplay legacy("OpenCV display regression","opencv");
    cv::Mat bgr(120,360,CV_8UC3);
    bgr(cv::Rect(0,0,120,120)).setTo(cv::Scalar(0,0,255));
    bgr(cv::Rect(120,0,120,120)).setTo(cv::Scalar(0,255,0));
    bgr(cv::Rect(240,0,120,120)).setTo(cv::Scalar(255,0,0));
    legacy.show_bgr(std::move(bgr));
    for(int n=0;n<20;++n) {legacy.poll();std::this_thread::sleep_for(std::chrono::milliseconds(10));}
    assert(!legacy.closed());
    std::cout<<"PASS legacy OpenCV BGR display window and events\n";return 0;
  }
  adas::RgbDisplay screen("RGB display regression");
  cv::Mat bgr(120,360,CV_8UC3);
  bgr(cv::Rect(0,0,120,120)).setTo(cv::Scalar(0,0,255));
  bgr(cv::Rect(120,0,120,120)).setTo(cv::Scalar(0,255,0));
  bgr(cv::Rect(240,0,120,120)).setTo(cv::Scalar(255,0,0));
  screen.show_bgr(std::move(bgr));
  for(int n=0;n<20;++n) {screen.poll();std::this_thread::sleep_for(std::chrono::milliseconds(10));}
  GList *windows=gtk_window_list_toplevels();assert(windows);
  GtkWidget *widget=GTK_WIDGET(windows->data);
  GdkWindow *window=gtk_widget_get_window(widget);
#if GTK_MAJOR_VERSION >= 3
  GdkPixbuf *shot=gdk_pixbuf_get_from_window(window,0,0,360,120);
#else
  GdkPixbuf *shot=gdk_pixbuf_get_from_drawable(nullptr,window,nullptr,0,0,0,0,360,120);
#endif
  assert(shot);
  assert(gdk_pixbuf_save(shot,"build/rgb_display_test.png","png",nullptr,nullptr));
  const auto *bytes=gdk_pixbuf_get_pixels(shot);
  const int channels=gdk_pixbuf_get_n_channels(shot),stride=gdk_pixbuf_get_rowstride(shot);
  for(int k=0;k<3;++k) {
    const auto *pixel=bytes+60*stride+(60+k*120)*channels;
    std::cout<<"sample="<<k<<" rgb="<<int(pixel[0])<<","<<int(pixel[1])<<","<<int(pixel[2])<<std::endl;
    for(int c=0;c<3;++c) assert(pixel[c]==(k==c ? 255:0));
  }
  g_object_unref(shot);g_list_free(windows);
  std::cout<<"PASS GTK window pixels are red / green / blue; RGB buffer ownership retained\n";
}
