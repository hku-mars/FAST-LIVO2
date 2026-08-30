#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <fstream>
#include <random>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <fast_livo/Diagnostics.h>
#include <image_transport/image_transport.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <sensor_msgs/PointCloud2.h>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

#include <Eigen/Geometry>
#include <opencv2/core.hpp>

#include <glk/pointcloud_buffer.hpp>
#include <glk/primitives/primitives.hpp>
#include <glk/texture_opencv.hpp>
#include <glk/thin_lines.hpp>
#include <guik/viewer/light_viewer.hpp>
#include <imgui.h>
#include <implot.h>

using CloudRGB = pcl::PointCloud<pcl::PointXYZRGB>;

constexpr std::size_t kChunkPoints = 50000;
constexpr std::size_t kPlotCap = 600;
constexpr std::size_t kRangeSamples = 16384;

enum ColorMode : int { kRgb = 0, kHeatmap = 1 };

const auto kPlotT0 = std::chrono::steady_clock::now();

double plotTime() {
  return std::chrono::duration<double>(std::chrono::steady_clock::now() - kPlotT0).count();
}

struct Series {
  std::vector<double> x;
  std::vector<double> y;
};

void pushXY(Series& s, double t, double value) {
  s.x.push_back(t);
  s.y.push_back(value);
  if (s.y.size() > kPlotCap) {
    const auto drop = static_cast<std::ptrdiff_t>(s.y.size() - kPlotCap);
    s.x.erase(s.x.begin(), s.x.begin() + drop);
    s.y.erase(s.y.begin(), s.y.begin() + drop);
  }
}

struct DrawState {
  std::mutex mutex;
  CloudRGB::Ptr pending_scan;
  bool pending_rgb = false;
  nav_msgs::Path::ConstPtr pending_path;
  geometry_msgs::Pose pending_pose;
  bool have_pose = false;
  std::shared_ptr<cv::Mat> pending_image;
  fast_livo::Diagnostics pending_diag;
  bool have_diag = false;

  CloudRGB chunk;
  std::size_t map_points = 0;
  int map_chunks = 0;
  bool have_rgb = false;
  int color_mode = kRgb;
  int heatmap_axis = 2;
  bool track = true;
  bool show_path = true;
  float image_scale = 0.55f;
  std::vector<Eigen::Vector3f> range_samples;
  std::size_t range_seen = 0;
  std::shared_ptr<glk::Texture> image_tex;

  Series lio_down, lio_icp, lio_voxel, lio_cur, lio_avg, lio_resid;
  Series vio_retr, vio_jac, vio_gen, vio_upd, vio_patch, vio_cur, vio_avg;
  Series feat_lio_eff, feat_vio_retr, feat_vio_map, feat_pre_in, feat_pre_out;
  Series noise_pos, noise_rot, noise_vel;
  double cpu_pct = 0;
  double gpu_pct = 0;
  double gpu_mem_pct = 0;
  double mem_pct = 0;
  double fps = 0;
};

struct HostCounters {
  unsigned long long idle = 0;
  unsigned long long total = 0;
  double gpu = 0;
  double gpu_mem = 0;
  std::chrono::steady_clock::time_point last_gpu = std::chrono::steady_clock::now();
};

bool cloudHasXyz(const sensor_msgs::PointCloud2& msg) {
  if (msg.data.empty() || msg.width == 0) {
    return false;
  }
  bool x = false;
  bool y = false;
  bool z = false;
  for (const auto& field : msg.fields) {
    if (field.name == "x") {
      x = true;
    }
    if (field.name == "y") {
      y = true;
    }
    if (field.name == "z") {
      z = true;
    }
  }
  return x && y && z;
}

bool cloudHasRgb(const sensor_msgs::PointCloud2& msg) {
  for (const auto& field : msg.fields) {
    if (field.name == "rgb" || field.name == "rgba" || field.name == "r") {
      return true;
    }
  }
  return false;
}

Eigen::Isometry3f poseTf(const geometry_msgs::Pose& pose) {
  Eigen::Isometry3f tf = Eigen::Isometry3f::Identity();
  tf.translation() = Eigen::Vector3f(
    static_cast<float>(pose.position.x),
    static_cast<float>(pose.position.y),
    static_cast<float>(pose.position.z));
  Eigen::Quaternionf q(
    static_cast<float>(pose.orientation.w),
    static_cast<float>(pose.orientation.x),
    static_cast<float>(pose.orientation.y),
    static_cast<float>(pose.orientation.z));
  if (q.norm() < 1e-6f) {
    q = Eigen::Quaternionf::Identity();
  } else {
    q.normalize();
  }
  tf.linear() = q.toRotationMatrix();
  return tf;
}

std::shared_ptr<glk::PointCloudBuffer> makeCloudBuffer(const CloudRGB& cloud, bool colored) {
  std::vector<Eigen::Vector3f> points;
  std::vector<Eigen::Vector4f> colors;
  points.reserve(cloud.size());
  if (colored) {
    colors.reserve(cloud.size());
  }

  for (const auto& point : cloud.points) {
    if (!std::isfinite(point.x) || !std::isfinite(point.y) || !std::isfinite(point.z)) {
      continue;
    }
    points.emplace_back(point.x, point.y, point.z);
    if (colored) {
      colors.emplace_back(point.r / 255.0f, point.g / 255.0f, point.b / 255.0f, 1.0f);
    }
  }

  auto buffer = std::make_shared<glk::PointCloudBuffer>(points);
  if (colored && !colors.empty()) {
    buffer->add_color(colors);
  }
  return buffer;
}

void appendFinite(CloudRGB& dst, const CloudRGB& src) {
  dst.reserve(dst.size() + src.size());
  for (const auto& point : src.points) {
    if (!std::isfinite(point.x) || !std::isfinite(point.y) || !std::isfinite(point.z)) {
      continue;
    }
    dst.push_back(point);
  }
}

void sampleRange(DrawState& state, const CloudRGB& cloud) {
  static std::mt19937 rng{std::random_device{}()};
  for (const auto& point : cloud.points) {
    if (!std::isfinite(point.x) || !std::isfinite(point.y) || !std::isfinite(point.z)) {
      continue;
    }
    const Eigen::Vector3f p(point.x, point.y, point.z);
    ++state.range_seen;
    if (state.range_samples.size() < kRangeSamples) {
      state.range_samples.push_back(p);
      continue;
    }
    std::uniform_int_distribution<std::size_t> pick(0, state.range_seen - 1);
    const std::size_t j = pick(rng);
    if (j < kRangeSamples) {
      state.range_samples[j] = p;
    }
  }
}

float quantile(const std::vector<float>& sorted, double p) {
  if (sorted.empty()) {
    return 0.0f;
  }
  const double idx = p * static_cast<double>(sorted.size() - 1);
  const std::size_t i = static_cast<std::size_t>(idx);
  const std::size_t j = std::min(sorted.size() - 1, i + 1);
  const float t = static_cast<float>(idx - static_cast<double>(i));
  return sorted[i] * (1.0f - t) + sorted[j] * t;
}

Eigen::Vector2f inlierAxisRange(const std::vector<Eigen::Vector3f>& samples, int axis) {
  if (samples.empty()) {
    return Eigen::Vector2f(0.0f, 1.0f);
  }
  std::vector<float> values;
  values.reserve(samples.size());
  for (const auto& p : samples) {
    values.push_back(p[axis]);
  }
  std::sort(values.begin(), values.end());
  const float q1 = quantile(values, 0.25);
  const float q3 = quantile(values, 0.75);
  const float iqr = q3 - q1;
  // Tukey: drop samples outside [Q1 - 1.5 IQR, Q3 + 1.5 IQR], then take min/max.
  const float lo_fence = q1 - 1.5f * iqr;
  const float hi_fence = q3 + 1.5f * iqr;
  float lo = values.back();
  float hi = values.front();
  bool any = false;
  for (float v : values) {
    if (v < lo_fence || v > hi_fence) {
      continue;
    }
    lo = std::min(lo, v);
    hi = std::max(hi, v);
    any = true;
  }
  if (!any) {
    lo = values.front();
    hi = values.back();
  }
  if (hi - lo < 1.0e-3f) {
    hi = lo + 1.0f;
  }
  return Eigen::Vector2f(lo, hi);
}

guik::ShaderSetting pointSetting(const DrawState& state) {
  if (state.color_mode == kRgb && state.have_rgb) {
    return guik::VertexColor();
  }
  const int a = std::clamp(state.heatmap_axis, 0, 2);
  Eigen::Vector3f axis = Eigen::Vector3f::Zero();
  axis[a] = 1.0f;
  return guik::Rainbow().set_rainbow_axis(axis).set_rainbow_range(inlierAxisRange(state.range_samples, a));
}

void applyPointStyle(guik::LightViewer* viewer, DrawState& state) {
  const guik::ShaderSetting setting = pointSetting(state);
  auto current = viewer->find_drawable("current");
  if (current.second) {
    viewer->update_drawable("current", current.second, setting);
  }
  for (int i = 0; i < state.map_chunks; ++i) {
    const std::string name = "map_" + std::to_string(i);
    auto drawable = viewer->find_drawable(name);
    if (drawable.second) {
      viewer->update_drawable(name, drawable.second, setting);
    }
  }
}

void showCurrent(guik::LightViewer* viewer, const CloudRGB& cloud, const DrawState& state) {
  if (cloud.empty()) {
    viewer->remove_drawable("current");
    return;
  }
  viewer->update_drawable("current", makeCloudBuffer(cloud, state.have_rgb), pointSetting(state));
}

void sealChunk(guik::LightViewer* viewer, DrawState& state) {
  if (state.chunk.empty()) {
    return;
  }

  const std::string name = "map_" + std::to_string(state.map_chunks);
  viewer->update_drawable(name, makeCloudBuffer(state.chunk, state.have_rgb), pointSetting(state));
  state.map_points += state.chunk.size();
  state.map_chunks += 1;
  state.chunk.clear();
}

void clearMap(guik::LightViewer* viewer, DrawState& state) {
  for (int i = 0; i < state.map_chunks; ++i) {
    viewer->remove_drawable("map_" + std::to_string(i));
  }
  viewer->remove_drawable("current");
  viewer->remove_drawable("path");
  state.chunk.clear();
  state.map_points = 0;
  state.map_chunks = 0;
  state.have_rgb = false;
  state.range_samples.clear();
  state.range_seen = 0;
}

void showPath(guik::LightViewer* viewer, const nav_msgs::Path& path, bool show) {
  if (!show) {
    viewer->remove_drawable("path");
    return;
  }

  std::vector<Eigen::Vector3f> line;
  line.reserve(path.poses.size());
  for (const auto& stamped : path.poses) {
    const auto& p = stamped.pose.position;
    if (!std::isfinite(p.x) || !std::isfinite(p.y) || !std::isfinite(p.z)) {
      continue;
    }
    line.emplace_back(static_cast<float>(p.x), static_cast<float>(p.y), static_cast<float>(p.z));
  }

  if (line.size() < 2) {
    return;
  }

  auto traj = std::make_shared<glk::ThinLines>(line, true);
  traj->set_line_width(2.0f);
  viewer->update_drawable("path", traj, guik::FlatGreen());
}

void showPose(guik::LightViewer* viewer, const geometry_msgs::Pose& pose, bool track) {
  const Eigen::Isometry3f tf = poseTf(pose);
  viewer->update_drawable(
    "pose",
    glk::Primitives::coordinate_system(),
    guik::VertexColor(tf * Eigen::UniformScaling<float>(1.5f)));
  // lookat writes the orbit pivot. Skip while the mouse is in the 3D view so
  // rotate/pan is not overwritten every odometry tick.
  if (
    track && !ImGui::GetIO().WantCaptureMouse && !ImGui::IsMouseDown(ImGuiMouseButton_Left) &&
    !ImGui::IsMouseDown(ImGuiMouseButton_Right) && !ImGui::IsMouseDown(ImGuiMouseButton_Middle)) {
    viewer->lookat(tf.translation());
  }
}

void setImageTexture(DrawState& state, const cv::Mat& image) {
  if (image.empty() || image.cols < 1 || image.rows < 1) {
    return;
  }
  state.image_tex = glk::create_texture(image);
}

void drawHudImage(const DrawState& state) {
  if (!state.image_tex) {
    return;
  }
  const Eigen::Vector2i size = state.image_tex->size();
  const ImVec2 draw(
    static_cast<float>(size[0]) * state.image_scale,
    static_cast<float>(size[1]) * state.image_scale);
  ImGui::Image(reinterpret_cast<void*>(static_cast<std::intptr_t>(state.image_tex->id())), draw);
}

void plotSeries(const char* label, const Series& s) {
  if (!s.y.empty()) {
    ImPlot::PlotLine(label, s.x.data(), s.y.data(), static_cast<int>(s.y.size()));
  }
}

bool beginMetricPlot(const char* title, float height) {
  if (!ImPlot::BeginPlot(title, ImVec2(-1.0f, height))) {
    return false;
  }
  ImPlot::SetupAxes(nullptr, nullptr, ImPlotAxisFlags_AutoFit, ImPlotAxisFlags_AutoFit);
  return true;
}

void applyDiag(DrawState& state, const fast_livo::Diagnostics& d) {
  const double t = plotTime();
  if (d.source == fast_livo::Diagnostics::SOURCE_LIO) {
    pushXY(state.lio_down, t, d.lio_downsample);
    pushXY(state.lio_icp, t, d.lio_icp);
    pushXY(state.lio_voxel, t, d.lio_update_voxel);
    pushXY(state.lio_cur, t, d.lio_current_total);
    pushXY(state.lio_avg, t, d.lio_average_total);
    pushXY(state.lio_resid, t, d.lio_avg_residual);
    pushXY(state.feat_lio_eff, t, d.lio_effective_feat);
  }
  if (d.source == fast_livo::Diagnostics::SOURCE_VIO) {
    pushXY(state.vio_retr, t, d.vio_retrieve);
    pushXY(state.vio_jac, t, d.vio_jacobian_ekf);
    pushXY(state.vio_gen, t, d.vio_generate);
    pushXY(state.vio_upd, t, d.vio_update_points);
    pushXY(state.vio_patch, t, d.vio_update_patch);
    pushXY(state.vio_cur, t, d.vio_current_total);
    pushXY(state.vio_avg, t, d.vio_average_total);
    pushXY(state.feat_vio_retr, t, d.vio_retrieve_n);
    pushXY(state.feat_vio_map, t, d.vio_sparse_map);
  }
  pushXY(state.feat_pre_in, t, d.preprocess_in);
  pushXY(state.feat_pre_out, t, d.preprocess_out);
  pushXY(state.noise_pos, t, d.pos_std);
  pushXY(state.noise_rot, t, d.rot_std);
  pushXY(state.noise_vel, t, d.vel_std);
}

void drawPlots(const DrawState& state) {
  ImGui::SetNextWindowPos(ImVec2(1240.0f, 20.0f), ImGuiCond_FirstUseEver);
  ImGui::SetNextWindowSize(ImVec2(660.0f, 500.0f), ImGuiCond_FirstUseEver);
  if (!ImGui::Begin("plots")) {
    ImGui::End();
    return;
  }

  if (ImGui::BeginTabBar("plot_tabs")) {
    if (ImGui::BeginTabItem("timing")) {
      const float h = std::max(80.0f, (ImGui::GetContentRegionAvail().y - 8.0f) * 0.5f);
      if (beginMetricPlot("LIO (s)", h)) {
        plotSeries("DownSample", state.lio_down);
        plotSeries("ICP", state.lio_icp);
        plotSeries("updateVoxelMap", state.lio_voxel);
        plotSeries("current", state.lio_cur);
        plotSeries("average", state.lio_avg);
        ImPlot::EndPlot();
      }
      if (beginMetricPlot("VIO (s)", h)) {
        plotSeries("retrieve", state.vio_retr);
        plotSeries("jacobian+EKF", state.vio_jac);
        plotSeries("generate", state.vio_gen);
        plotSeries("update points", state.vio_upd);
        plotSeries("update patch", state.vio_patch);
        plotSeries("current", state.vio_cur);
        plotSeries("average", state.vio_avg);
        ImPlot::EndPlot();
      }
      ImGui::EndTabItem();
    }
    if (ImGui::BeginTabItem("quality")) {
      const float h = std::max(80.0f, (ImGui::GetContentRegionAvail().y - 12.0f) / 3.0f);
      if (beginMetricPlot("residual (m)", h)) {
        plotSeries("average residual", state.lio_resid);
        ImPlot::EndPlot();
      }
      if (beginMetricPlot("features", h)) {
        plotSeries("LIO effective", state.feat_lio_eff);
        plotSeries("VIO retrieve", state.feat_vio_retr);
        plotSeries("VIO sparse map", state.feat_vio_map);
        plotSeries("preprocess in", state.feat_pre_in);
        plotSeries("preprocess out", state.feat_pre_out);
        ImPlot::EndPlot();
      }
      if (beginMetricPlot("path noise 1-sigma", h)) {
        plotSeries("pos (m)", state.noise_pos);
        plotSeries("rot (rad)", state.noise_rot);
        plotSeries("vel (m/s)", state.noise_vel);
        ImPlot::EndPlot();
      }
      ImGui::EndTabItem();
    }
    ImGui::EndTabBar();
  }
  ImGui::End();
}

double readCpuPercent(HostCounters& host) {
  std::ifstream in("/proc/stat");
  std::string cpu;
  unsigned long long user = 0, nice = 0, system = 0, idle = 0, iowait = 0, irq = 0, softirq = 0, steal = 0;
  in >> cpu >> user >> nice >> system >> idle >> iowait >> irq >> softirq >> steal;
  if (!in) {
    return 0;
  }
  const unsigned long long idle_all = idle + iowait;
  const unsigned long long total = user + nice + system + idle_all + irq + softirq + steal;
  const unsigned long long d_idle = idle_all - host.idle;
  const unsigned long long d_total = total - host.total;
  host.idle = idle_all;
  host.total = total;
  if (d_total == 0) {
    return 0;
  }
  return 100.0 * static_cast<double>(d_total - d_idle) / static_cast<double>(d_total);
}

double readMemPercent() {
  std::ifstream in("/proc/meminfo");
  double total = 0;
  double avail = 0;
  std::string key;
  double value = 0;
  std::string unit;
  while (in >> key >> value >> unit) {
    if (key == "MemTotal:") {
      total = value;
    } else if (key == "MemAvailable:") {
      avail = value;
    }
    if (total > 0 && avail > 0) {
      break;
    }
  }
  if (total <= 0) {
    return 0;
  }
  return 100.0 * (1.0 - avail / total);
}

void readGpu(HostCounters& host, const std::chrono::steady_clock::time_point now) {
  if (std::chrono::duration<double>(now - host.last_gpu).count() < 0.5) {
    return;
  }
  host.last_gpu = now;
  FILE* pipe = popen("nvidia-smi --query-gpu=utilization.gpu,memory.used,memory.total --format=csv,noheader,nounits 2>/dev/null", "r");
  if (pipe == nullptr) {
    return;
  }
  char line[256];
  if (fgets(line, sizeof(line), pipe) != nullptr) {
    double util = 0;
    double used = 0;
    double total = 0;
    if (std::sscanf(line, "%lf , %lf , %lf", &util, &used, &total) >= 1) {
      host.gpu = util;
      if (total > 0) {
        host.gpu_mem = 100.0 * used / total;
      }
    }
  }
  pclose(pipe);
}

void sampleHost(DrawState& state, HostCounters& host, int& frames, std::chrono::steady_clock::time_point& last_t) {
  const auto now = std::chrono::steady_clock::now();
  ++frames;
  const double dt = std::chrono::duration<double>(now - last_t).count();
  if (dt < 0.25) {
    return;
  }
  state.cpu_pct = readCpuPercent(host);
  state.mem_pct = readMemPercent();
  readGpu(host, now);
  state.gpu_pct = host.gpu;
  state.gpu_mem_pct = host.gpu_mem;
  state.fps = static_cast<double>(frames) / dt;
  frames = 0;
  last_t = now;
}

void applyPending(guik::LightViewer* viewer, DrawState& state) {
  CloudRGB::Ptr scan;
  bool rgb = false;
  nav_msgs::Path::ConstPtr path;
  geometry_msgs::Pose pose;
  bool have_pose = false;
  std::shared_ptr<cv::Mat> image;
  fast_livo::Diagnostics diag;
  bool have_diag = false;
  {
    std::lock_guard<std::mutex> lock(state.mutex);
    scan.swap(state.pending_scan);
    rgb = state.pending_rgb;
    path.swap(state.pending_path);
    pose = state.pending_pose;
    have_pose = state.have_pose;
    state.have_pose = false;
    image.swap(state.pending_image);
    if (state.have_diag) {
      diag = state.pending_diag;
      have_diag = true;
      state.have_diag = false;
    }
  }

  if (scan && !scan->empty()) {
    if (rgb) {
      state.have_rgb = true;
    }
    sampleRange(state, *scan);
    showCurrent(viewer, *scan, state);
    appendFinite(state.chunk, *scan);
    if (state.chunk.size() >= kChunkPoints) {
      sealChunk(viewer, state);
    }
    if (state.color_mode == kHeatmap || !state.have_rgb) {
      applyPointStyle(viewer, state);
    }
  }
  if (have_diag) {
    applyDiag(state, diag);
  }
  if (path) {
    showPath(viewer, *path, state.show_path);
  }
  if (have_pose) {
    showPose(viewer, pose, state.track);
  }
  if (image) {
    setImageTexture(state, *image);
  }
}

void onCloud(const sensor_msgs::PointCloud2ConstPtr& msg, const std::shared_ptr<DrawState>& state) {
  if (!cloudHasXyz(*msg)) {
    return;
  }

  CloudRGB::Ptr scan(new CloudRGB);
  pcl::fromROSMsg(*msg, *scan);
  if (scan->empty()) {
    return;
  }

  std::lock_guard<std::mutex> lock(state->mutex);
  state->pending_scan = scan;
  state->pending_rgb = cloudHasRgb(*msg);
}

void onPath(const nav_msgs::PathConstPtr& msg, const std::shared_ptr<DrawState>& state) {
  std::lock_guard<std::mutex> lock(state->mutex);
  state->pending_path = msg;
}

void onOdom(const nav_msgs::OdometryConstPtr& msg, const std::shared_ptr<DrawState>& state) {
  std::lock_guard<std::mutex> lock(state->mutex);
  state->pending_pose = msg->pose.pose;
  state->have_pose = true;
}

void onImage(const sensor_msgs::ImageConstPtr& msg, const std::shared_ptr<DrawState>& state) {
  cv_bridge::CvImageConstPtr cv_ptr;
  try {
    cv_ptr = cv_bridge::toCvShare(msg, "bgr8");
  } catch (const cv_bridge::Exception&) {
    return;
  }
  if (cv_ptr->image.empty()) {
    return;
  }

  auto image = std::make_shared<cv::Mat>(cv_ptr->image.clone());
  std::lock_guard<std::mutex> lock(state->mutex);
  state->pending_image = image;
}

void onDiag(const fast_livo::DiagnosticsConstPtr& msg, const std::shared_ptr<DrawState>& state) {
  std::lock_guard<std::mutex> lock(state->mutex);
  state->pending_diag = *msg;
  state->have_diag = true;
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "fastlivo_viewer");
  ros::NodeHandle nh;

  auto viewer = guik::LightViewer::instance(Eigen::Vector2i(1920, 1080), false, "FAST-LIVO2");
  viewer->enable_vsync();
  viewer->use_orbit_camera_control();
  viewer->shader_setting().set_point_size(0.05f);
  viewer->shader_setting().set_point_scale_metric();
  viewer->shader_setting().set_point_shape_circle();

  auto state = std::make_shared<DrawState>();
  viewer->register_ui_callback("ui", [viewer, state] {
    ImGui::Begin("FAST-LIVO2", nullptr, ImGuiWindowFlags_AlwaysAutoResize);
    ImGui::Checkbox("track", &state->track);
    ImGui::SameLine();
    ImGui::Checkbox("path", &state->show_path);
    ImGui::SliderFloat("image scale", &state->image_scale, 0.2f, 1.5f, "%.2f");
    ImGui::Text(
      "CPU  %.0f%%   GPU  %.0f%%   GPU mem  %.0f%%   MEM  %.0f%%   FPS  %.0f",
      state->cpu_pct,
      state->gpu_pct,
      state->gpu_mem_pct,
      state->mem_pct,
      state->fps);
    ImGui::Text("points: %zu  chunks: %d", state->map_points + state->chunk.size(), state->map_chunks);
    if (ImGui::Button("Clear map")) {
      clearMap(viewer, *state);
    }
    const int prev_mode = state->color_mode;
    const int prev_axis = state->heatmap_axis;
    ImGui::RadioButton("RGB", &state->color_mode, kRgb);
    ImGui::SameLine();
    ImGui::RadioButton("heatmap", &state->color_mode, kHeatmap);
    if (state->color_mode == kHeatmap) {
      ImGui::SameLine();
      ImGui::RadioButton("X", &state->heatmap_axis, 0);
      ImGui::SameLine();
      ImGui::RadioButton("Y", &state->heatmap_axis, 1);
      ImGui::SameLine();
      ImGui::RadioButton("Z", &state->heatmap_axis, 2);
    }
    if (prev_mode != state->color_mode || prev_axis != state->heatmap_axis) {
      applyPointStyle(viewer, *state);
    }
    drawHudImage(*state);
    ImGui::End();
    drawPlots(*state);
  });

  ros::Subscriber cloud_sub = nh.subscribe<sensor_msgs::PointCloud2>(
    "/cloud_registered",
    1,
    [state](const sensor_msgs::PointCloud2ConstPtr& msg) { onCloud(msg, state); });
  ros::Subscriber path_sub = nh.subscribe<nav_msgs::Path>(
    "/path",
    1,
    [state](const nav_msgs::PathConstPtr& msg) { onPath(msg, state); });
  ros::Subscriber odom_sub = nh.subscribe<nav_msgs::Odometry>(
    "/aft_mapped_to_init",
    1,
    [state](const nav_msgs::OdometryConstPtr& msg) { onOdom(msg, state); });
  ros::Subscriber diag_sub = nh.subscribe<fast_livo::Diagnostics>(
    "/fast_livo/diagnostics",
    20,
    [state](const fast_livo::DiagnosticsConstPtr& msg) { onDiag(msg, state); });

  image_transport::ImageTransport it(nh);
  image_transport::Subscriber image_sub = it.subscribe(
    "/rgb_img",
    1,
    [state](const sensor_msgs::ImageConstPtr& msg) { onImage(msg, state); });

  ros::AsyncSpinner spinner(2);
  spinner.start();

  HostCounters host;
  readCpuPercent(host);
  int frames = 0;
  auto last_t = std::chrono::steady_clock::now();

  while (ros::ok() && viewer->spin_once()) {
    sampleHost(*state, host, frames, last_t);
    applyPending(viewer, *state);
  }

  ros::shutdown();
  return 0;
}
