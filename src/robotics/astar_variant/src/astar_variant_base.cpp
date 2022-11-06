/*
 * astar_variant_base.cpp
 *
 * Created on: Oct 26, 2022 22:31
 * Description:
 *
 * Copyright (c) 2022 Pin Loon Lee (pllee4)
 */

#include "algorithm/robotics/astar_variant/astar_variant_base.hpp"

namespace pllee4::graph {
void AstarVariantBase::SetMapStorageSize(const size_t x_size,
                                         const size_t y_size) {
  map_storage_ = std::make_unique<MapStorage>(x_size, y_size);
  std::vector<std::vector<bool>> visited_map(x_size,
                                             std::vector<bool>(y_size, false));
  visited_map_ = visited_map;
}

bool AstarVariantBase::SetOccupiedGrid(
    const std::vector<Coordinate> &occupied_grid) {
  if (map_storage_) {
    auto &map = map_storage_->GetMap();
    for (const auto &coordinate : occupied_grid) {
      if (map_storage_->Contains(coordinate)) {
        map[coordinate.x][coordinate.y].occupied = true;
      } else {
        return false;
      }
    }
    return true;
  }
  return false;
}

bool AstarVariantBase::SetStartAndDestination(const Coordinate &start,
                                              const Coordinate &dest) {
  if (map_storage_) {
    if (map_storage_->Contains(start) && map_storage_->Contains(dest)) {
      // new start / destination
      if (reset_called_ || !(start_ == start && dest_ == dest)) {
        start_ = start;
        dest_ = dest;

        if (reset_called_) {
          reset_called_ = false;
        } else {
          // clear possible historical search
          ResetFunc(true);
        }

        auto &map = map_storage_->GetMap();
        traverse_path_.insert(
            std::make_pair(0, std::make_pair(start.x, start.y)));
        map[start.x][start.y].parent_coordinate = start;
        map[start.x][start.y].f = 0;
        map[start.x][start.y].g = 0;
        map[start.x][start.y].h = 0;
      }
      start_and_end_set_ = true;
      return true;
    }
  }
  return false;
}

std::optional<std::vector<Coordinate>> AstarVariantBase::StepOverPathFinding() {
  if (found_path_) return std::nullopt;
  if (start_and_end_set_) {
    if (!traverse_path_.empty()) {
      const auto travelled_path = traverse_path_.begin();
      traverse_path_.erase(traverse_path_.begin());

      const auto [i, j] = travelled_path->second;

      // mark node as visited
      visited_map_[i][j] = true;

      auto &map = map_storage_->GetMap();

      std::vector<Coordinate> expanded_nodes;

      for (size_t k = 0; k < motion_constraint_.size(); ++k) {
        Coordinate coordinate;
        coordinate.x = i + motion_constraint_.dx[k];
        coordinate.y = j + motion_constraint_.dy[k];
        if (!map_storage_->Contains(coordinate)) continue;

        if (coordinate == dest_) {
          map[coordinate.x][coordinate.y].parent_coordinate = {i, j};
          found_path_ = true;
          return std::nullopt;
        }

        // not occupied
        if (!map[coordinate.x][coordinate.y].occupied &&
            !visited_map_[coordinate.x][coordinate.y]) {
          const auto dist_travelled = sqrt(abs(motion_constraint_.dx[k]) +
                                           abs(motion_constraint_.dy[k]));

          const auto astar_variant_spec = GetAstarVariantSpec();

          const auto new_g = map[i][j].g + dist_travelled;
          const auto new_h = astar_variant_spec.heuristic_func(
              coordinate.x - dest_.x, coordinate.y - dest_.y);
          const auto new_f =
              astar_variant_spec.w1 * new_g + astar_variant_spec.w2 * new_h;

          if (map[coordinate.x][coordinate.y].f ==
                  std::numeric_limits<MapStorage::CostDataType>::max() ||
              map[coordinate.x][coordinate.y].f >= new_f) {
            map[coordinate.x][coordinate.y].f = new_f;
            map[coordinate.x][coordinate.y].g = new_g;
            map[coordinate.x][coordinate.y].h = new_h;
            map[coordinate.x][coordinate.y].parent_coordinate = {i, j};

            traverse_path_.insert(std::make_pair(
                new_f, std::make_pair(coordinate.x, coordinate.y)));

            expanded_nodes.push_back(coordinate);
          }
        }
      }
      return expanded_nodes;
    }
  }
  return std::nullopt;
}
bool AstarVariantBase::FindPath() {
  if (!start_and_end_set_) return false;
  while (StepOverPathFinding().has_value()) {
    StepOverPathFinding();
  }
  return found_path_;
}

std::optional<std::vector<Coordinate>> AstarVariantBase::GetPath() {
  if (found_path_) {
    auto &map = map_storage_->GetMap();
    std::vector<Coordinate> path;
    auto [x, y] = dest_;
    while (!(map[x][y].parent_coordinate.x == x &&
             map[x][y].parent_coordinate.y == y)) {
      path.emplace_back(Coordinate{x, y});
      const auto [temp_x, temp_y] = map[x][y].parent_coordinate;
      std::tie(x, y) = std::pair(temp_x, temp_y);
    }
    path.emplace_back(Coordinate{x, y});
    std::reverse(path.begin(), path.end());
    return path;
  } else {
    return std::nullopt;
  }
}

void AstarVariantBase::Reset() { ResetFunc(); }

void AstarVariantBase::SetMotionConstraint(
    const MotionConstraint &motion_constraint) {
  motion_constraint_ = motion_constraint;
}

void AstarVariantBase::ResetFunc(bool reset_cost_only) {
  traverse_path_.clear();
  for (auto &row : visited_map_) {
    std::fill(row.begin(), row.end(), false);
  }
  if (reset_cost_only) {
    map_storage_->ResetCost();
  } else {
    map_storage_->Reset();
    reset_called_ = true;
  }

  found_path_ = false;
}

}  // namespace pllee4::graph