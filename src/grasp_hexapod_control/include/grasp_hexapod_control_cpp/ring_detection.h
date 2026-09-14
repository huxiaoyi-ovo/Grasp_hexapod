#pragma once
#include <opencv2/core.hpp>
#include <string>
namespace grasp_hexapod_ring {
struct RingDetection {
  bool valid = false;
  cv::Point2d center{0, 0};
  double diameter = 0, ratio_error = 0;
  int matched = 0;
  std::string reason = "no ring detection";
};
RingDetection detectRings(const cv::Mat& gray);
}
