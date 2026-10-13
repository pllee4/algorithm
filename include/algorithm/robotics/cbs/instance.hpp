/*
 * instance.hpp
 *
 * Created on: Oct 04, 2026 12:00
 * Description: MAPF instance loader. Reads the movingai.com benchmark
 * format (.map + .scen) and converts into the types used by our CBS
 * solver (Coordinate-based grid, Agent start/goal pairs).
 *
 * Copyright (c) 2026 Pin Loon Lee (pllee4)
 */

#pragma once

#include <string>
#include <vector>

#include "algorithm/robotics/cbs/cbs.hpp"
#include "algorithm/robotics/shared_type/data_type.hpp"

namespace pllee4::graph {

class Instance {
 public:
  Instance() = default;

  bool LoadMap(const std::string &map_file);

  bool LoadAgents(const std::string &scen_file, int num_agents);

  [[nodiscard]] int width() const noexcept { return width_; }
  [[nodiscard]] int height() const noexcept { return height_; }
  [[nodiscard]] const std::vector<Coordinate> &obstacles() const noexcept {
    return obstacles_;
  }
  [[nodiscard]] const std::vector<Agent> &agents() const noexcept {
    return agents_;
  }

  void PrintMap() const;
  void PrintAgents() const;

  static void PrintPaths(const Solution &solution, int width);
  static void PrintResults(const CBS::Result &result, double runtime_s);
  static void SaveResults(const std::string &file, const CBS::Result &result,
                          double runtime_s, const std::string &instance_name);
  static void SavePaths(const std::string &file, const Solution &solution);

 private:
  int width_{};
  int height_{};
  std::vector<Coordinate> obstacles_;
  std::vector<Agent> agents_;
};

}  // namespace pllee4::graph
