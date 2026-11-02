#pragma once
#include <gtk/gtk.h>
#include <opencv2/core.hpp>

namespace adas {
class RgbDisplay {
public:
  explicit RgbDisplay(const char *title,const std::string &backend="gtk_rgb");
  ~RgbDisplay();
  RgbDisplay(const RgbDisplay &)=delete;
  RgbDisplay &operator=(const RgbDisplay &)=delete;
  // RGB only. Takes ownership until GTK releases the pixbuf; caller must not mutate aliases.
  void show_rgb(cv::Mat &&rgb);
  void show_bgr(cv::Mat &&bgr);
  int poll();
  bool closed() const;
private:
  bool opencv_=false;
  std::string title_;
  cv::Mat retained_;
  GtkWidget *window_=nullptr,*image_=nullptr;
  int key_=-1;
};
}
