/*
 * instance.cpp
 *
 * Created on: Oct 04, 2026 12:00
 * Description: Implementation of Instance, see instance.hpp.
 *
 * Copyright (c) 2026 Pin Loon Lee (pllee4)
 */

#include "algorithm/robotics/cbs/instance.hpp"

#include <fstream>
#include <iostream>
#include <sstream>

namespace pllee4::graph {

bool Instance::LoadMap(const std::string &map_file) {
  auto in = std::ifstream{map_file};
  if (!in.is_open()) return false;

  auto line = std::string{};
  // First line: "type octile" (Nathan's benchmark) or "rows,cols" (custom).
  std::getline(in, line);
  if (line.find("type") != std::string::npos) {
    // Nathan's movingai format:  type octile / height N / width N / map
    auto read_value = [&](const std::string &key) {
      std::getline(in, line);
      return std::stoi(line.substr(key.size()));
    };
    height_ = read_value("height ");
    width_ = read_value("width ");
    std::getline(in, line);  // skip "map"
  } else {
    // rows,cols (custom format)
    auto ss = std::istringstream{line};
    auto comma = char{};
    ss >> height_ >> comma >> width_;
  }

  obstacles_.clear();
  for (auto row = 0; row < height_; ++row) {
    std::getline(in, line);
    for (auto col = 0; col < width_ &&
                       col < static_cast<int>(line.size());
         ++col) {
      if (line[col] != '.') obstacles_.push_back(Coordinate{col, row});
    }
  }
  return true;
}

bool Instance::LoadAgents(const std::string &scen_file, int num_agents) {
  auto in = std::ifstream{scen_file};
  if (!in.is_open()) return false;

  auto line = std::string{};
  std::getline(in, line);  // "version 1"

  agents_.clear();
  agents_.reserve(num_agents);
  auto count = 0;
  while (count < num_agents && std::getline(in, line)) {
    auto ss = std::istringstream{line};
    auto bucket = 0, cols = 0, rows = 0;
    auto start_col = 0, start_row = 0, goal_col = 0, goal_row = 0;
    auto map_name = std::string{};
    auto opt = 0.0;
    ss >> bucket >> map_name >> cols >> rows >> start_col >> start_row >>
        goal_col >> goal_row >> opt;
    if (ss.fail()) continue;
    agents_.push_back(
        {Coordinate{start_col, start_row}, Coordinate{goal_col, goal_row}});
    ++count;
  }
  return count == num_agents;
}

void Instance::PrintMap() const {
  for (auto row = 0; row < height_; ++row) {
    auto line = std::vector<char>(width_, '.');
    for (const auto &o : obstacles_) {
      if (o.y == row) line[o.x] = '@';
    }
    std::cout.write(line.data(), width_);
    std::cout << '\n';
  }
}

void Instance::PrintAgents() const {
  for (auto i = 0; i < static_cast<int>(agents_.size()); ++i) {
    const auto &a = agents_[i];
    std::cout << "Agent " << i << " : S=(" << a.start.y << "," << a.start.x
              << ") ; G=(" << a.goal.y << "," << a.goal.x << ")\n";
  }
}

void Instance::PrintPaths(const Solution &solution, int /*width*/) {
  for (auto i = 0; i < static_cast<int>(solution.size()); ++i) {
    std::cout << "Agent " << i << ": ";
    for (const auto &c : solution[i])
      std::cout << "(" << c.y << "," << c.x << ")->";
    std::cout << '\n';
  }
}

void Instance::PrintResults(const CBS::Result &result, double runtime_s) {
  if (result.solved)
    std::cout << "Optimal,";
  else
    std::cout << "Timeout,";
  std::cout << result.cost << "," << runtime_s << ","
            << result.expanded << "," << result.generated << ","
            << result.makespan << '\n';
}

void Instance::SaveResults(const std::string &file,
                           const CBS::Result &result, double runtime_s,
                           const std::string &instance_name) {
  auto check = std::ifstream{file};
  const auto exists = check.good();
  check.close();

  auto out = std::ofstream{file, std::ios::app};
  if (!exists) {
    out << "runtime,#high-level expanded,#high-level generated,"
           "solution cost,makespan,instance name\n";
  }
  out << runtime_s << "," << result.expanded << "," << result.generated
      << "," << result.cost << "," << result.makespan << ","
      << instance_name << '\n';
}

void Instance::SavePaths(const std::string &file,
                         const Solution &solution) {
  auto out = std::ofstream{file};
  for (auto i = 0; i < static_cast<int>(solution.size()); ++i) {
    out << "Agent " << i << ": ";
    for (const auto &c : solution[i])
      out << "(" << c.y << "," << c.x << ")->";
    out << '\n';
  }
}

}  // namespace pllee4::graph
