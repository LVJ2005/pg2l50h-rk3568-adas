#include "rgb_display.h"
#include "frame_color.h"
#include <opencv2/highgui.hpp>
#include <stdexcept>

namespace adas {
RgbDisplay::RgbDisplay(const char *title,const std::string &backend):opencv_(backend=="opencv"),title_(title) {
  if(backend!="opencv" && backend!="gtk_rgb") throw std::invalid_argument("display backend must be opencv or gtk_rgb");
  if(opencv_) {cv::namedWindow(title_,cv::WINDOW_NORMAL);return;}
  if(!gtk_init_check(nullptr,nullptr)) throw std::runtime_error("GTK display unavailable; check DISPLAY or set ADAS_DISPLAY=0");
  window_=gtk_window_new(GTK_WINDOW_TOPLEVEL);image_=gtk_image_new();
  gtk_window_set_title(GTK_WINDOW(window_),title);
  gtk_container_add(GTK_CONTAINER(window_),image_);
  g_signal_connect(window_,"destroy",G_CALLBACK(+[](GtkWidget *,gpointer p) {
    static_cast<RgbDisplay*>(p)->window_=nullptr;
  }),this);
  g_signal_connect(window_,"key-press-event",G_CALLBACK(+[](GtkWidget *,GdkEventKey *event,gpointer p)->gboolean {
    auto *display=static_cast<RgbDisplay*>(p);display->key_=event->keyval;return FALSE;
  }),this);
  gtk_widget_show_all(window_);
}
RgbDisplay::~RgbDisplay() {
  if(opencv_) {try {cv::destroyWindow(title_);} catch(const cv::Exception &) {} }
  else if(window_) gtk_widget_destroy(window_);
}
bool RgbDisplay::closed() const {
  if(!opencv_) return window_==nullptr;
  // Some OpenCV GTK builds return -1 for this unsupported property even while visible.
  try {return cv::getWindowProperty(title_,cv::WND_PROP_VISIBLE)==0;} catch(const cv::Exception &) {return true;}
}
void RgbDisplay::show_bgr(cv::Mat &&bgr) {
  CV_Assert(bgr.type()==CV_8UC3);
  if(opencv_) {retained_=std::move(bgr);cv::imshow(title_,retained_);}
  else show_rgb(make_display_rgb(bgr));
}
void RgbDisplay::show_rgb(cv::Mat &&rgb) {
  if(opencv_) {
    cv::Mat bgr;cv::cvtColor(rgb,bgr,cv::COLOR_RGB2BGR);show_bgr(std::move(bgr));return;
  }
  if(!window_) return;
  CV_Assert(rgb.type()==CV_8UC3);
  auto *owned=new cv::Mat(std::move(rgb));
  GdkPixbuf *pixels=gdk_pixbuf_new_from_data(owned->data,GDK_COLORSPACE_RGB,FALSE,8,
      owned->cols,owned->rows,static_cast<int>(owned->step),
      +[](guchar *,gpointer p) {delete static_cast<cv::Mat*>(p);},owned);
  if(!pixels) {delete owned;throw std::runtime_error("Cannot create RGB pixbuf");}
  gtk_image_set_from_pixbuf(GTK_IMAGE(image_),pixels);g_object_unref(pixels);
}
int RgbDisplay::poll() {
  if(opencv_) return cv::waitKey(1);
  // Bound event pumping so a busy GUI cannot monopolize the main loop.
  for(int n=0;n<16 && gtk_events_pending();++n) gtk_main_iteration_do(FALSE);
  const int key=key_;key_=-1;return key;
}
}
