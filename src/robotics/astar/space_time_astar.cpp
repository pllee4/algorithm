/*
 * space_time_astar.cpp
 *
 * Created on: Sep 21, 2026 22:05
 * Description: Implementation of Space-Time A*, see space_time_astar.hpp.
 *
 * Copyright (c) 2026 Pin Loon Lee (pllee4)
 */

#include "algorithm/robotics/astar/space_time_astar.hpp"

#include <algorithm>
#include <queue>

namespace pllee4::graph {

void SpaceTimeAstar::SetMapStorageSize(const size_t x_size,
                                       const size_t y_size) {
  map_storage_ = std::make_unique<MapStorage>(x_size, y_size);
  start_and_end_set_ = false;
  heuristic_.clear();
  ClearSearch();
}

bool SpaceTimeAstar::SetOccupiedGrid(
    const std::vector<Coordinate> &occupied_grid) {
  if (!map_storage_) return false;
  heuristic_.clear();
  ClearSearch();
  auto &map = map_storage_->GetMap();
  for (const auto &coordinate : occupied_grid) {
    if (!map_storage_->Contains(coordinate)) return false;
    map[coordinate.x][coordinate.y].occupied = true;
  }
  return true;
}

bool SpaceTimeAstar::SetStartAndDestination(const Coordinate &start,
                                            const Coordinate &dest) {
  if (!map_storage_ || !map_storage_->Contains(start) ||
      !map_storage_->Contains(dest))
    return false;
  // the heuristic is per destination; same start and destination keeps the
  // search, like the other A* variants
  if (!(start_and_end_set_ && dest_ == dest)) heuristic_.clear();
  if (!(start_and_end_set_ && start_ == start && dest_ == dest)) ClearSearch();
  start_ = start;
  dest_ = dest;
  start_and_end_set_ = true;
  return true;
}

std::optional<std::vector<Coordinate>> SpaceTimeAstar::StepOverPathFinding() {
  const auto new_states = StepOverSpaceTimePathFinding();
  if (!new_states) return std::nullopt;
  auto new_coordinates = std::vector<Coordinate>{};
  new_coordinates.reserve(new_states->size());
  for (const auto &state : *new_states) new_coordinates.push_back(state.at);
  return new_coordinates;
}

bool SpaceTimeAstar::FindPath() {
  if (!start_and_end_set_) return false;
  while (StepOverSpaceTimePathFinding().has_value()) {
  }
  return (goal_node_ != -1);
}

std::optional<std::vector<Coordinate>> SpaceTimeAstar::GetPath() {
  if (goal_node_ == -1) return std::nullopt;
  auto path = std::vector<Coordinate>{};
  for (auto i = goal_node_; i != -1; i = nodes_.at(i).parent) {
    path.push_back(nodes_.at(i).loc);
  }
  std::reverse(path.begin(), path.end());
  return path;
}

void SpaceTimeAstar::Reset() {
  if (map_storage_) map_storage_->Reset();
  constraints_.clear();
  max_timestep_ = kNoTimestepLimit;
  start_and_end_set_ = false;
  heuristic_.clear();
  ClearSearch();
}

void SpaceTimeAstar::SetConstraints(const std::vector<Constraint> &constraints) {
  constraints_ = constraints;
  ClearSearch();
}

void SpaceTimeAstar::SetMaxTimestep(const int max_timestep) {
  max_timestep_ = max_timestep;
  ClearSearch();
}

std::optional<std::vector<State>>
SpaceTimeAstar::StepOverSpaceTimePathFinding() {
  if (!start_and_end_set_) return std::nullopt;
  if (!constraint_table_) BeginSearch();
  if (goal_node_ != -1 || open_.empty()) return std::nullopt;

  const auto cur = open_.top().node;
  open_.pop();
  const auto node = nodes_.at(cur);  // copy: nodes_ may reallocate below

  if (node.loc == dest_ && (node.t >= constraint_table_->GetHoldingTime())) {
    goal_node_ = cur;
    return std::nullopt;
  }

  auto new_states = std::vector<State>{};
  if (node.t + 1 > max_timestep_) return new_states;

  // Successors = 4 moves + wait.
  auto succ = GetNeighbours(node.loc);
  succ.push_back(node.loc);  // wait action, also costs one timestep

  for (const auto &next : succ) {
    const auto next_t = node.t + 1;
    const auto heuristic = Heuristic(next);
    if (heuristic == kInf) continue;  // goal unreachable from here
    if (constraint_table_->IsBlocked(next, next_t))
      continue;  // vertex constraint
    if (constraint_table_->IsBlocked(node.loc, next, next_t))
      continue;  // edge constraint
    if (next_t + heuristic > max_timestep_) continue;  // cannot finish in time

    // skip visited
    if (!generated_.insert(State{next, next_t}).second)
      continue;  // g == t, so never re-open

    nodes_.push_back(Node{next, next_t, cur});
    open_.push(OpenEntry{next_t + heuristic, next_t,
                         static_cast<int>(nodes_.size()) - 1});
    new_states.push_back(State{next, next_t});
  }
  return new_states;
}

void SpaceTimeAstar::ClearSearch() {
  constraint_table_.reset();
  nodes_.clear();
  generated_.clear();
  open_ = std::priority_queue<OpenEntry>{};
  goal_node_ = -1;
}

void SpaceTimeAstar::BeginSearch() {
  if (heuristic_.empty()) heuristic_ = BackwardBfs();
  constraint_table_ = ConstraintTable{constraints_, dest_};

  const auto start_heuristic = Heuristic(start_);
  if (start_heuristic == kInf) return;  // goal unreachable: nothing to do
  nodes_.push_back(Node{start_, 0, -1});
  generated_.insert(State{start_, 0});
  open_.push(OpenEntry{start_heuristic, 0, 0});
}

std::vector<std::vector<int>> SpaceTimeAstar::BackwardBfs() const {
  auto dist = std::vector<std::vector<int>>{};
  for (const auto &column : map_storage_->GetMap())
    dist.emplace_back(column.size(), kInf);
  if (!IsFree(dest_)) return dist;
  auto q = std::queue<Coordinate>{};
  dist[dest_.x][dest_.y] = 0;
  q.push(dest_);
  while (!q.empty()) {
    const auto current = q.front();  // copy: pop() below destroys the front
    q.pop();
    for (const auto &n : GetNeighbours(current)) {
      if (dist[n.x][n.y] != kInf) continue;
      dist[n.x][n.y] = dist[current.x][current.y] + 1;
      q.push(n);
    }
  }
  return dist;
}

bool SpaceTimeAstar::IsFree(const Coordinate &coordinate) const {
  return !map_storage_->GetMap()[coordinate.x][coordinate.y].occupied;
}

std::vector<Coordinate> SpaceTimeAstar::GetNeighbours(
    const Coordinate &coordinate) const {
  auto neighbours = std::vector<Coordinate>{};
  neighbours.reserve(motion_constraint_.size());
  for (size_t k = 0; k < motion_constraint_.size(); ++k) {
    const auto new_coordinate =
        Coordinate{coordinate.x + motion_constraint_.dx[k],
                   coordinate.y + motion_constraint_.dy[k]};
    if (!map_storage_->Contains(new_coordinate)) continue;
    if (IsFree(new_coordinate)) neighbours.push_back(new_coordinate);
  }
  return neighbours;
}

int SpaceTimeAstar::Heuristic(const Coordinate &c) const {
  return heuristic_[c.x][c.y];
}

}  // namespace pllee4::graph
