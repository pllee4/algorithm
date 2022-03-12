/*
 * dijikstra.cpp
 *
 * Created on: Mar 11, 2022 21:16
 * Description:
 *
 * Copyright (c) 2022 Pin Loon Lee (pllee4)
 */

#include "dijikstra/dijikstra.hpp"

#include <algorithm>
#include <iostream>
#include <set>
#include <stdexcept>

namespace pllee4 {
namespace graph {
Dijikstra::Dijikstra(struct MotionConstraint& motion_constraint) {
  if (motion_constraint.dx.size() != motion_constraint.dy.size()) {
    throw std::invalid_argument{
        "Incompatible size of dx and dy for motion constraint"};
  } else {
    motion_constraint_ = motion_constraint;
  }
}

bool Dijikstra::SetOccupancyGrid(const std::vector<Coordinate>& occupancy_grid,
                                 int x_size, int y_size) {
  map_x_size_ = x_size;
  map_y_size_ = y_size;
  // assign occupancy
  std::vector<std::vector<Cell>> map(x_size, std::vector<Cell>(y_size));
  for (const auto& coordinate : occupancy_grid) {
    if (IsValidCoordinate(coordinate)) {
      map[coordinate.x][coordinate.y].occupied = true;
    } else {
      return false;
    }
  }
  map_ = map;
  return true;
}

std::vector<Coordinate> Dijikstra::GetPath(const Coordinate& dest) {
  std::vector<Coordinate> path;
  auto x = dest.x;
  auto y = dest.y;
  while (!(map_[x][y].parent_coordinate.x == x &&
           map_[x][y].parent_coordinate.y == y)) {
    path.push_back({x, y});
    auto temp_x = map_[x][y].parent_coordinate.x;
    auto temp_y = map_[x][y].parent_coordinate.y;
    x = temp_x;
    y = temp_y;
  }
  path.push_back({x, y});
  std::reverse(path.begin(), path.end());
  return path;
}

bool Dijikstra::FindPath(const Coordinate& src, const Coordinate& dest) {
  if (!IsValidCoordinate(src)) {
    std::cout << "Not a valid source" << std::endl;
    return false;
  }
  if (!IsValidCoordinate(dest)) {
    std::cout << "Not a valid destination" << std::endl;
    return false;
  }

  std::vector<std::vector<bool>> visited(map_x_size_,
                                         std::vector<bool>(map_y_size_, false));

  std::set<std::pair<int, std::pair<int, int>>> traverse_path;

  traverse_path.insert(std::make_pair(0, std::make_pair(src.x, src.y)));

  map_[src.x][src.y].parent_coordinate = src;

  map_[src.x][src.y].cost = 0;

  bool found_path{false};

  while (!traverse_path.empty()) {
    auto travelled_path = traverse_path.begin();
    traverse_path.erase(traverse_path.begin());

    auto i = travelled_path->second.first;
    auto j = travelled_path->second.second;

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
        std::cout << "Found path!" << std::endl;
        map_[coordinate.x][coordinate.y].parent_coordinate.x = i;
        map_[coordinate.x][coordinate.y].parent_coordinate.y = j;
        found_path = true;
        break;
      }
      // not occupied
      if (!map_[coordinate.x][coordinate.y].occupied &&
          !visited[coordinate.x][coordinate.y]) {
        auto new_cost = map_[i][j].cost + 1;

        if (map_[coordinate.x][coordinate.y].cost == INT_MAX ||
            map_[coordinate.x][coordinate.y].cost >= new_cost) {
          map_[coordinate.x][coordinate.y].cost = new_cost;

          map_[coordinate.x][coordinate.y].parent_coordinate.x = i;
          map_[coordinate.x][coordinate.y].parent_coordinate.y = j;

          std::cout << "traverse to (" << coordinate.x << "," << coordinate.y
                    << ")" << std::endl;
          traverse_path.insert(std::make_pair(
              new_cost, std::make_pair(coordinate.x, coordinate.y)));
        }
      }
    }
    if (found_path) {
      auto path = GetPath(dest);
      for (const auto& coordinate : path) {
        std::cout << "(" << coordinate.x << ", " << coordinate.y << ")->"
                  << std::endl;
      }
      std::cout << "The end" << std::endl;
      break;
    }
  }
  return true;
}

}  // namespace graph
}  // namespace pllee4