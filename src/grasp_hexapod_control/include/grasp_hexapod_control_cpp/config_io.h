// 配置文件加载：工作空间边界 CSV（control.py 的 np.loadtxt + np.interp）
// 与 dock_system.yaml（dock_mode.py 的 load_dock_system）。
#pragma once

#include "grasp_hexapod_control_cpp/types.h"

#include <map>
#include <string>
#include <vector>

namespace grasp_hexapod_control_cpp {

// 离线生成的足端安全工作空间表：列 0=z（递增），列 1=rho_min，列 2=rho_max。
struct WorkspaceBoundary {
  std::vector<double> z;
  std::vector<double> rho_min;
  std::vector<double> rho_max;

  std::size_t size() const { return z.size(); }
};

// 解析 workspace_bounds.csv；'#' 开头的行为注释（与 np.loadtxt 默认一致）。
WorkspaceBoundary loadWorkspaceBoundary(const std::string& path);

// np.interp 等价实现：xs 必须递增；越界取端点值，内部线性插值。
double interpMonotonic(double x, const std::vector<double>& xs,
                       const std::vector<double>& ys);

// ---------------------------------------------------------------------------
// dock_system.yaml（对应 dock_mode.py::load_dock_system 的全部校验）。
// ---------------------------------------------------------------------------

struct DockSystem {
  std::string path;
  std::vector<int> tag_ids;
  double tag_size_m = 0.0;
  std::map<int, std::string> tag_frames;
  Matrix4d lock_from_camera = Matrix4d::Identity();
  std::map<int, Matrix4d> pin_from_tag;
  bool real_calibrated = false;
};

DockSystem loadDockSystem(const std::string& path);

}  // namespace grasp_hexapod_control_cpp
