// #include "dijikstra/dijikstra.hpp"

#include <unistd.h>

#include <algorithm>
#include <climits>
#include <cstdlib>
#include <iostream>
#include <list>
#include <queue>
#include <set>
#include <unordered_map>
#include <utility>
#include <vector>

#include "gmock/gmock.h"
#include "gtest/gtest.h"

struct Coordinate {
  int x;
  int y;
  bool operator==(const Coordinate& other) const {
    return (x == other.x && y == other.y);
  }
};

struct Cell {
  Coordinate parent_coordinate;
  int cost{INT_MAX};
  bool occupied{false};
  bool operator==(const Cell& other) const {
    return (parent_coordinate == other.parent_coordinate);
  }
};

/*
y
^
|
|
|
|_________>x
*/
class PathPlanning {
 public:
  PathPlanning() = default;
  ~PathPlanning() = default;

  bool SetOccupancyGrid(const std::vector<Coordinate>& occupancy_grid,
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

  std::vector<Coordinate> GetPath(const Coordinate& dest) {
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

  void Dijikstra(const Coordinate& src, const Coordinate& dest) {
    if (!IsValidCoordinate(src)) {
      std::cout << "Not a valid source" << std::endl;
    }
    if (!IsValidCoordinate(dest)) {
      std::cout << "Not a valid destination" << std::endl;
    }

    std::vector<std::vector<bool>> visited(
        map_x_size_, std::vector<bool>(map_y_size_, false));

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

      for (int k = 0; k < 4; ++k) {
        Coordinate coordinate;
        coordinate.x = i + dx_[k];
        coordinate.y = j + dy_[k];
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
  }

  void PrintMap() {
    for (size_t i = 0; i < map_.size(); ++i) {
      for (size_t j = 0; j < map_[0].size(); ++j) {
        if (map_[i][j].occupied) {
          std::cout << "Occupied cell: (" << i << "," << j << ")" << std::endl;
        }
      }
    }
  }

 private:
  std::vector<int> dx_ = {1, 0, -1, 0};
  std::vector<int> dy_ = {0, 1, 0, -1};

  std::vector<std::vector<Cell>> map_;

  int map_x_size_;
  int map_y_size_;

  bool IsValidCoordinate(const Coordinate& coordinate) const {
    return (coordinate.x >= 0 && coordinate.x < map_x_size_ &&
            coordinate.y >= 0 && coordinate.y < map_y_size_);
  }
};

int main(int argc, char** argv) {
  PathPlanning path_planning;

  std::vector<Coordinate> occupied_map = {{2, 2}, {2, 3}, {2, 4},
                                          {3, 2}, {3, 3}, {3, 4}};
  if (!path_planning.SetOccupancyGrid(occupied_map, 6, 6)) {
    std::cout << "Invalid occupancy grid!" << std::endl;
  } else {
    path_planning.PrintMap();
  }
  path_planning.Dijikstra({0, 3}, {5, 1});

  return 0;
}