// SPDX-License-Identifier: Apache-2.0
// File-based configuration: nested struct schema + YAML loader + dotted
// overrides. Mirrors amr/core/config.py. Struct defaults are canonical; a YAML
// file (or override) may set any subset of keys. Unknown keys and type
// mismatches raise ConfigError with a dotted path.
#pragma once

#include <cmath>
#include <map>
#include <stdexcept>
#include <string>
#include <vector>

namespace amr_core {

struct ConfigError : std::runtime_error {
  explicit ConfigError(const std::string& msg) : std::runtime_error(msg) {}
};

struct RobotConfig {
  double radius = 0.18;
  double max_lin_vel = 0.6;
  double max_ang_vel = 1.8;
  double max_lin_acc = 0.8;
  double max_ang_acc = 2.5;
};

struct LidarConfig {
  int num_beams = 240;
  double angle_min = -M_PI;
  double angle_max = M_PI;  // increment = (max - min) / num_beams
  double range_min = 0.12;
  double range_max = 8.0;
  double noise_std = 0.01;
  int scan_every = 2;  // emit a scan every N sim steps
};

struct OdomNoiseConfig {
  double alpha_v = 0.03;  // std(v_meas) = alpha_v*|v| + floor
  double alpha_w = 0.03;
  double floor = 1e-4;
};

struct SimConfig {
  double dt = 0.05;
  std::string world_file = "configs/worlds/office.yaml";
  OdomNoiseConfig odom_noise{};
};

struct MappingConfig {
  double resolution = 0.05;
  double l_occ = 0.85;
  double l_free = -0.4;
  double l_clamp = 10.0;
  double occupied_thresh = 0.65;
  double free_thresh = 0.25;
  int beam_subsample = 2;
};

struct SlamConfig {
  double keyframe_trans = 0.2;
  double keyframe_rot = 0.35;
  double min_motion = 0.02;
  int match_beams = 80;
  double coarse_window_xy = 0.15;
  double coarse_step_xy = 0.05;
  double coarse_window_theta = 0.12;
  double coarse_step_theta = 0.03;
  double fine_step_xy = 0.025;
  double fine_step_theta = 0.01;
  double blur_sigma_cells = 1.5;
  double min_match_score = 0.1;
};

struct LikelihoodConfig {
  double sigma_hit = 0.2;
  double z_hit = 0.9;
  double z_rand = 0.1;
  double max_dist = 2.0;
  int beam_subsample = 5;
};

struct LocalizationConfig {
  int num_particles = 500;
  std::vector<double> alphas{0.05, 0.05, 0.05, 0.05};
  std::vector<double> init_std{0.25, 0.25, 0.15};
  double resample_neff_frac = 0.5;
  LikelihoodConfig likelihood{};
};

struct CostmapConfig {
  int occupied_thresh = 65;
  bool unknown_is_lethal = true;
  double inflation_radius = 0.45;
  double cost_decay = 6.0;
};

struct AstarConfig {
  double w_cost = 4.0;
  bool simplify = true;
};

struct DwaConfig {
  double sim_time = 1.5;
  double sim_dt = 0.1;
  int v_samples = 8;
  int w_samples = 15;
  double lookahead = 0.8;
  double w_progress = 1.0;
  double w_heading = 0.6;
  double w_clearance = 0.4;
  double w_velocity = 0.3;
};

struct PlanningConfig {
  CostmapConfig costmap{};
  AstarConfig astar{};
  DwaConfig dwa{};
};

struct NavConfig {
  double goal_tol_xy = 0.25;
  double replan_period = 4.0;
  double path_block_check_dist = 1.0;
  int max_recoveries = 3;
  double recovery_rotate_speed = 0.8;
  double recovery_backup_dist = 0.3;
  double recovery_backup_speed = 0.1;
};

struct LoggingConfig {
  std::string level = "INFO";
  std::string file = "logs/amr.log";
  int max_bytes = 1000000;
  int backup_count = 3;
  bool console = true;
  std::map<std::string, std::string> module_levels{};
};

struct GuiConfig {
  int refresh_ms = 66;
  int px_per_cell = 3;
  double speed_factor = 1.0;  // worker pacing; 0 = run flat out
};

struct AmrConfig {
  int seed = 42;
  RobotConfig robot{};
  LidarConfig lidar{};
  SimConfig sim{};
  MappingConfig mapping{};
  SlamConfig slam{};
  LocalizationConfig localization{};
  PlanningConfig planning{};
  NavConfig nav{};
  LoggingConfig logging{};
  GuiConfig gui{};
};

/// Load config from a YAML file (empty path -> all defaults), then apply dotted
/// overrides of the form "a.b.c=value" (value parsed as YAML). Throws
/// ConfigError on unknown keys or type mismatches.
AmrConfig load_config(const std::string& path,
                      const std::vector<std::string>& overrides = {});

}  // namespace amr_core
