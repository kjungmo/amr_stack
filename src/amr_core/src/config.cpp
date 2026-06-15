// SPDX-License-Identifier: Apache-2.0
#include "amr_core/config.hpp"

#include <yaml-cpp/yaml.h>

#include <exception>
#include <sstream>
#include <string>
#include <vector>

namespace amr_core {
namespace {

template <typename T>
T conv(const YAML::Node& n, const std::string& path) {
  try {
    return n.as<T>();
  } catch (const std::exception&) {
    throw ConfigError(path + ": bad value for expected type");
  }
}

void require_map(const YAML::Node& n, const std::string& path) {
  if (!n.IsMap()) {
    throw ConfigError(path + ": expected a mapping");
  }
}

std::vector<std::string> split(const std::string& s, char sep) {
  std::vector<std::string> out;
  std::string cur;
  std::istringstream ss(s);
  while (std::getline(ss, cur, sep)) {
    out.push_back(cur);
  }
  return out;
}

std::string trim(const std::string& s) {
  const auto a = s.find_first_not_of(" \t");
  if (a == std::string::npos) return "";
  const auto b = s.find_last_not_of(" \t");
  return s.substr(a, b - a + 1);
}

// ---- per-struct loaders -----------------------------------------------------

void load_robot(RobotConfig& c, const YAML::Node& n) {
  require_map(n, "robot");
  for (const auto& kv : n) {
    const std::string k = kv.first.as<std::string>();
    const std::string p = "robot." + k;
    if (k == "radius") c.radius = conv<double>(kv.second, p);
    else if (k == "max_lin_vel") c.max_lin_vel = conv<double>(kv.second, p);
    else if (k == "max_ang_vel") c.max_ang_vel = conv<double>(kv.second, p);
    else if (k == "max_lin_acc") c.max_lin_acc = conv<double>(kv.second, p);
    else if (k == "max_ang_acc") c.max_ang_acc = conv<double>(kv.second, p);
    else throw ConfigError("unknown config key '" + p + "'");
  }
}

void load_lidar(LidarConfig& c, const YAML::Node& n) {
  require_map(n, "lidar");
  for (const auto& kv : n) {
    const std::string k = kv.first.as<std::string>();
    const std::string p = "lidar." + k;
    if (k == "num_beams") c.num_beams = conv<int>(kv.second, p);
    else if (k == "angle_min") c.angle_min = conv<double>(kv.second, p);
    else if (k == "angle_max") c.angle_max = conv<double>(kv.second, p);
    else if (k == "range_min") c.range_min = conv<double>(kv.second, p);
    else if (k == "range_max") c.range_max = conv<double>(kv.second, p);
    else if (k == "noise_std") c.noise_std = conv<double>(kv.second, p);
    else if (k == "scan_every") c.scan_every = conv<int>(kv.second, p);
    else throw ConfigError("unknown config key '" + p + "'");
  }
}

void load_odom_noise(OdomNoiseConfig& c, const YAML::Node& n) {
  require_map(n, "sim.odom_noise");
  for (const auto& kv : n) {
    const std::string k = kv.first.as<std::string>();
    const std::string p = "sim.odom_noise." + k;
    if (k == "alpha_v") c.alpha_v = conv<double>(kv.second, p);
    else if (k == "alpha_w") c.alpha_w = conv<double>(kv.second, p);
    else if (k == "floor") c.floor = conv<double>(kv.second, p);
    else throw ConfigError("unknown config key '" + p + "'");
  }
}

void load_sim(SimConfig& c, const YAML::Node& n) {
  require_map(n, "sim");
  for (const auto& kv : n) {
    const std::string k = kv.first.as<std::string>();
    const std::string p = "sim." + k;
    if (k == "dt") c.dt = conv<double>(kv.second, p);
    else if (k == "world_file") c.world_file = conv<std::string>(kv.second, p);
    else if (k == "odom_noise") load_odom_noise(c.odom_noise, kv.second);
    else throw ConfigError("unknown config key '" + p + "'");
  }
}

void load_mapping(MappingConfig& c, const YAML::Node& n) {
  require_map(n, "mapping");
  for (const auto& kv : n) {
    const std::string k = kv.first.as<std::string>();
    const std::string p = "mapping." + k;
    if (k == "resolution") c.resolution = conv<double>(kv.second, p);
    else if (k == "l_occ") c.l_occ = conv<double>(kv.second, p);
    else if (k == "l_free") c.l_free = conv<double>(kv.second, p);
    else if (k == "l_clamp") c.l_clamp = conv<double>(kv.second, p);
    else if (k == "occupied_thresh") c.occupied_thresh = conv<double>(kv.second, p);
    else if (k == "free_thresh") c.free_thresh = conv<double>(kv.second, p);
    else if (k == "beam_subsample") c.beam_subsample = conv<int>(kv.second, p);
    else throw ConfigError("unknown config key '" + p + "'");
  }
}

void load_slam(SlamConfig& c, const YAML::Node& n) {
  require_map(n, "slam");
  for (const auto& kv : n) {
    const std::string k = kv.first.as<std::string>();
    const std::string p = "slam." + k;
    if (k == "keyframe_trans") c.keyframe_trans = conv<double>(kv.second, p);
    else if (k == "keyframe_rot") c.keyframe_rot = conv<double>(kv.second, p);
    else if (k == "min_motion") c.min_motion = conv<double>(kv.second, p);
    else if (k == "match_beams") c.match_beams = conv<int>(kv.second, p);
    else if (k == "coarse_window_xy") c.coarse_window_xy = conv<double>(kv.second, p);
    else if (k == "coarse_step_xy") c.coarse_step_xy = conv<double>(kv.second, p);
    else if (k == "coarse_window_theta") c.coarse_window_theta = conv<double>(kv.second, p);
    else if (k == "coarse_step_theta") c.coarse_step_theta = conv<double>(kv.second, p);
    else if (k == "fine_step_xy") c.fine_step_xy = conv<double>(kv.second, p);
    else if (k == "fine_step_theta") c.fine_step_theta = conv<double>(kv.second, p);
    else if (k == "blur_sigma_cells") c.blur_sigma_cells = conv<double>(kv.second, p);
    else if (k == "min_match_score") c.min_match_score = conv<double>(kv.second, p);
    else throw ConfigError("unknown config key '" + p + "'");
  }
}

void load_likelihood(LikelihoodConfig& c, const YAML::Node& n) {
  require_map(n, "localization.likelihood");
  for (const auto& kv : n) {
    const std::string k = kv.first.as<std::string>();
    const std::string p = "localization.likelihood." + k;
    if (k == "sigma_hit") c.sigma_hit = conv<double>(kv.second, p);
    else if (k == "z_hit") c.z_hit = conv<double>(kv.second, p);
    else if (k == "z_rand") c.z_rand = conv<double>(kv.second, p);
    else if (k == "max_dist") c.max_dist = conv<double>(kv.second, p);
    else if (k == "beam_subsample") c.beam_subsample = conv<int>(kv.second, p);
    else throw ConfigError("unknown config key '" + p + "'");
  }
}

void load_localization(LocalizationConfig& c, const YAML::Node& n) {
  require_map(n, "localization");
  for (const auto& kv : n) {
    const std::string k = kv.first.as<std::string>();
    const std::string p = "localization." + k;
    if (k == "num_particles") c.num_particles = conv<int>(kv.second, p);
    else if (k == "alphas") c.alphas = conv<std::vector<double>>(kv.second, p);
    else if (k == "init_std") c.init_std = conv<std::vector<double>>(kv.second, p);
    else if (k == "resample_neff_frac") c.resample_neff_frac = conv<double>(kv.second, p);
    else if (k == "likelihood") load_likelihood(c.likelihood, kv.second);
    else throw ConfigError("unknown config key '" + p + "'");
  }
}

void load_costmap(CostmapConfig& c, const YAML::Node& n) {
  require_map(n, "planning.costmap");
  for (const auto& kv : n) {
    const std::string k = kv.first.as<std::string>();
    const std::string p = "planning.costmap." + k;
    if (k == "occupied_thresh") c.occupied_thresh = conv<int>(kv.second, p);
    else if (k == "unknown_is_lethal") c.unknown_is_lethal = conv<bool>(kv.second, p);
    else if (k == "inflation_radius") c.inflation_radius = conv<double>(kv.second, p);
    else if (k == "cost_decay") c.cost_decay = conv<double>(kv.second, p);
    else throw ConfigError("unknown config key '" + p + "'");
  }
}

void load_astar(AstarConfig& c, const YAML::Node& n) {
  require_map(n, "planning.astar");
  for (const auto& kv : n) {
    const std::string k = kv.first.as<std::string>();
    const std::string p = "planning.astar." + k;
    if (k == "w_cost") c.w_cost = conv<double>(kv.second, p);
    else if (k == "simplify") c.simplify = conv<bool>(kv.second, p);
    else throw ConfigError("unknown config key '" + p + "'");
  }
}

void load_dwa(DwaConfig& c, const YAML::Node& n) {
  require_map(n, "planning.dwa");
  for (const auto& kv : n) {
    const std::string k = kv.first.as<std::string>();
    const std::string p = "planning.dwa." + k;
    if (k == "sim_time") c.sim_time = conv<double>(kv.second, p);
    else if (k == "sim_dt") c.sim_dt = conv<double>(kv.second, p);
    else if (k == "v_samples") c.v_samples = conv<int>(kv.second, p);
    else if (k == "w_samples") c.w_samples = conv<int>(kv.second, p);
    else if (k == "lookahead") c.lookahead = conv<double>(kv.second, p);
    else if (k == "w_progress") c.w_progress = conv<double>(kv.second, p);
    else if (k == "w_heading") c.w_heading = conv<double>(kv.second, p);
    else if (k == "w_clearance") c.w_clearance = conv<double>(kv.second, p);
    else if (k == "w_velocity") c.w_velocity = conv<double>(kv.second, p);
    else throw ConfigError("unknown config key '" + p + "'");
  }
}

void load_planning(PlanningConfig& c, const YAML::Node& n) {
  require_map(n, "planning");
  for (const auto& kv : n) {
    const std::string k = kv.first.as<std::string>();
    const std::string p = "planning." + k;
    if (k == "costmap") load_costmap(c.costmap, kv.second);
    else if (k == "astar") load_astar(c.astar, kv.second);
    else if (k == "dwa") load_dwa(c.dwa, kv.second);
    else throw ConfigError("unknown config key '" + p + "'");
  }
}

void load_nav(NavConfig& c, const YAML::Node& n) {
  require_map(n, "nav");
  for (const auto& kv : n) {
    const std::string k = kv.first.as<std::string>();
    const std::string p = "nav." + k;
    if (k == "goal_tol_xy") c.goal_tol_xy = conv<double>(kv.second, p);
    else if (k == "replan_period") c.replan_period = conv<double>(kv.second, p);
    else if (k == "path_block_check_dist") c.path_block_check_dist = conv<double>(kv.second, p);
    else if (k == "max_recoveries") c.max_recoveries = conv<int>(kv.second, p);
    else if (k == "recovery_rotate_speed") c.recovery_rotate_speed = conv<double>(kv.second, p);
    else if (k == "recovery_backup_dist") c.recovery_backup_dist = conv<double>(kv.second, p);
    else if (k == "recovery_backup_speed") c.recovery_backup_speed = conv<double>(kv.second, p);
    else throw ConfigError("unknown config key '" + p + "'");
  }
}

void load_logging(LoggingConfig& c, const YAML::Node& n) {
  require_map(n, "logging");
  for (const auto& kv : n) {
    const std::string k = kv.first.as<std::string>();
    const std::string p = "logging." + k;
    if (k == "level") c.level = conv<std::string>(kv.second, p);
    else if (k == "file") c.file = conv<std::string>(kv.second, p);
    else if (k == "max_bytes") c.max_bytes = conv<int>(kv.second, p);
    else if (k == "backup_count") c.backup_count = conv<int>(kv.second, p);
    else if (k == "console") c.console = conv<bool>(kv.second, p);
    else if (k == "module_levels")
      c.module_levels = conv<std::map<std::string, std::string>>(kv.second, p);
    else throw ConfigError("unknown config key '" + p + "'");
  }
}

void load_gui(GuiConfig& c, const YAML::Node& n) {
  require_map(n, "gui");
  for (const auto& kv : n) {
    const std::string k = kv.first.as<std::string>();
    const std::string p = "gui." + k;
    if (k == "refresh_ms") c.refresh_ms = conv<int>(kv.second, p);
    else if (k == "px_per_cell") c.px_per_cell = conv<int>(kv.second, p);
    else if (k == "speed_factor") c.speed_factor = conv<double>(kv.second, p);
    else throw ConfigError("unknown config key '" + p + "'");
  }
}

void load_amr(AmrConfig& c, const YAML::Node& n) {
  require_map(n, "root");
  for (const auto& kv : n) {
    const std::string k = kv.first.as<std::string>();
    if (k == "seed") c.seed = conv<int>(kv.second, "seed");
    else if (k == "robot") load_robot(c.robot, kv.second);
    else if (k == "lidar") load_lidar(c.lidar, kv.second);
    else if (k == "sim") load_sim(c.sim, kv.second);
    else if (k == "mapping") load_mapping(c.mapping, kv.second);
    else if (k == "slam") load_slam(c.slam, kv.second);
    else if (k == "localization") load_localization(c.localization, kv.second);
    else if (k == "planning") load_planning(c.planning, kv.second);
    else if (k == "nav") load_nav(c.nav, kv.second);
    else if (k == "logging") load_logging(c.logging, kv.second);
    else if (k == "gui") load_gui(c.gui, kv.second);
    else throw ConfigError("unknown config key(s) [" + k + "] under 'root'");
  }
}

// ---- override injection -----------------------------------------------------

// Build a nested map node {keys[0]: {keys[1]: ... : leaf}} bottom-up.
YAML::Node build_nested(const std::vector<std::string>& keys,
                        const YAML::Node& leaf) {
  YAML::Node node = YAML::Clone(leaf);
  for (auto it = keys.rbegin(); it != keys.rend(); ++it) {
    YAML::Node parent(YAML::NodeType::Map);
    parent[*it] = node;
    node.reset(parent);
  }
  return node;
}

// Deep-merge src map into target map (scalars in src overwrite).
void merge_into(YAML::Node target, const YAML::Node& src) {
  for (const auto& kv : src) {
    const std::string key = kv.first.as<std::string>();
    const YAML::Node val = kv.second;
    if (val.IsMap() && target[key] && target[key].IsMap()) {
      merge_into(target[key], val);
    } else {
      target[key] = YAML::Clone(val);
    }
  }
}

}  // namespace

AmrConfig load_config(const std::string& path,
                      const std::vector<std::string>& overrides) {
  YAML::Node root(YAML::NodeType::Map);
  if (!path.empty()) {
    YAML::Node loaded;
    try {
      loaded = YAML::LoadFile(path);
    } catch (const std::exception& e) {
      throw ConfigError("cannot read config '" + path + "': " + e.what());
    }
    if (loaded && !loaded.IsNull()) {
      if (!loaded.IsMap()) {
        throw ConfigError("config root must be a mapping");
      }
      root = loaded;
    }
  }

  for (const auto& ov : overrides) {
    const auto eq = ov.find('=');
    if (eq == std::string::npos) {
      throw ConfigError("override '" + ov + "' must look like a.b.c=value");
    }
    const std::string dotted = trim(ov.substr(0, eq));
    const std::string raw = ov.substr(eq + 1);
    const std::vector<std::string> keys = split(dotted, '.');
    if (keys.empty()) {
      throw ConfigError("override '" + ov + "' has an empty key path");
    }
    YAML::Node leaf;
    try {
      leaf = YAML::Load(raw);
    } catch (const std::exception&) {
      leaf = YAML::Node(raw);
    }
    YAML::Node nested = build_nested(keys, leaf);
    merge_into(root, nested);
  }

  AmrConfig cfg;
  load_amr(cfg, root);
  return cfg;
}

}  // namespace amr_core
