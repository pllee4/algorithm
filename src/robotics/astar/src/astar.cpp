/*
 * astar.cpp
 *
 * Created on: Mar 19, 2022 14:43
 * Description:
 *
 * Copyright (c) 2022 Pin Loon Lee (pllee4)
 */

#include "algorithm/robotics/astar/astar.hpp"

#include <algorithm>
#include <cmath>
#include <iostream>
#include <set>
#include <tuple>

namespace pllee4::graph {

AStar::AStar(const MotionConstraintType &motion_constraint_type)
    : motion_constraint_type_(motion_constraint_type) {
  if (motion_constraint_type == MotionConstraintType::CARDINAL_MOTION) {
    motion_constraint_.dx = {1, 0, -1, 0};
    motion_constraint_.dy = {0, 1, 0, -1};
  } else {
    motion_constraint_.dx = {1, 0, -1, 0, 1, -1, 1, -1};
    motion_constraint_.dy = {0, 1, 0, -1, 1, -1, -1, 1};
  }
}

bool AStar::SetOccupancyGrid(const std::vector<Coordinate> &occupancy_grid,
                             int x_size, int y_size) {
  map_x_size_ = x_size;
  map_y_size_ = y_size;
  // assign occupancy
  std::vector<std::vector<Cell>> map(x_size, std::vector<Cell>(y_size));
  for (const auto &coordinate : occupancy_grid) {
    if (IsValidCoordinate(coordinate)) {
      map[coordinate.x][coordinate.y].occupied = true;
    } else {
      return false;
    }
  }
  map_ = map;
  return true;
}
std::vector<Coordinate> AStar::GetPath(const Coordinate &dest) {
  std::vector<Coordinate> path;
  auto [x, y] = dest;
  while (!(map_[x][y].parent_coordinate.x == x &&
           map_[x][y].parent_coordinate.y == y)) {
    path.push_back({x, y});
    auto [temp_x, temp_y] = map_[x][y].parent_coordinate;
    std::tie(x, y) = std::pair(temp_x, temp_y);
  }
  path.push_back({x, y});
  std::reverse(path.begin(), path.end());
  return path;
}

bool AStar::FindPath(const Coordinate &src, const Coordinate &dest) {
  if (!IsValidCoordinate(src)) return false;
  if (!IsValidCoordinate(dest)) return false;

  std::vector<std::vector<bool>> visited(map_x_size_,
                                         std::vector<bool>(map_y_size_, false));

  std::set<std::pair<int, std::pair<int, int>>> traverse_path;

  traverse_path.insert(std::make_pair(0, std::make_pair(src.x, src.y)));

  map_[src.x][src.y].parent_coordinate = src;

  map_[src.x][src.y].f = 0;
  map_[src.x][src.y].g = 0;
  map_[src.x][src.y].h = 0;

  bool found_path{false};

  while (!traverse_path.empty()) {
    auto travelled_path = traverse_path.begin();
    traverse_path.erase(traverse_path.begin());

    auto [i, j] = travelled_path->second;

    // mark node as visited
    visited[i][j] = true;

    for (size_t k = 0; k < motion_constraint_.size(); ++k) {
      Coordinate coordinate;
      coordinate.x = i + motion_constraint_.dx[k];
      coordinate.y = j + motion_constraint_.dy[k];
      // ensure the coordinate is within the range
      if (!IsValidCoordinate(coordinate)) continue;
      // if reach destination
      if (coordinate == dest) {
        map_[coordinate.x][coordinate.y].parent_coordinate = {i, j};
        found_path = true;
        break;
      }
      // not occupied
      if (!map_[coordinate.x][coordinate.y].occupied &&
          !visited[coordinate.x][coordinate.y]) {
        auto dist_travelled =
            sqrt(abs(motion_constraint_.dx[k]) + abs(motion_constraint_.dy[k]));
        auto new_g = map_[i][j].g + dist_travelled;
        auto new_h = ComputeH(coordinate, dest);
        auto new_f = new_g + new_h;

        if (map_[coordinate.x][coordinate.y].f ==
                std::numeric_limits<decltype(Cell::f)>::max() ||
            map_[coordinate.x][coordinate.y].f >= new_f) {
          map_[coordinate.x][coordinate.y].f = new_f;
          map_[coordinate.x][coordinate.y].g = new_g;
          map_[coordinate.x][coordinate.y].h = new_h;
          map_[coordinate.x][coordinate.y].parent_coordinate = {i, j};

          std::cout << "traverse to (" << coordinate.x << "," << coordinate.y
                    << ")" << std::endl;
          traverse_path.insert(std::make_pair(
              new_f, std::make_pair(coordinate.x, coordinate.y)));
        }
      }
    }

    if (found_path) {
      auto path = GetPath(dest);
      for (const auto &[x, y] : path) {
        std::cout << "(" << x << ", " << y << ")->" << std::endl;
      }
      std::cout << "The end" << std::endl;
      break;
    }
  }
  return found_path;
}

double AStar::ComputeH(const Coordinate &curr, const Coordinate &dest) const {
  auto dx = curr.x - dest.x;
  auto dy = curr.y - dest.y;
  double h;
  switch (motion_constraint_type_) {
    case MotionConstraintType::CARDINAL_ORDINAL_MOTION:
      // Diagonal
      h = std::max(abs(dx), abs(dy));
      break;
    case MotionConstraintType::ANY_MOTION:
      // Euclidean
      h = sqrt(dx * dx + dy * dy);
      break;
    case MotionConstraintType::CARDINAL_MOTION:
    default:
      // Manhattan
      h = static_cast<double>(abs(dx) + abs(dy));
      break;
  }
  return h;
}

}  // namespace pllee4::graph