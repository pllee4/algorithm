#include <algorithm>
#include <iostream>
#include <optional>

template <typename T>
struct Point {
  T x;
  T y;
};

/**
 * @brief Get the Cross Product of segment (a, b) and (a, c)
 *
 * Example:
 *           W(3, 4)
 *                      V(5, 2)
 * (0,0)
 *
 * V x W = VxWy - WxVy
 *     = | 5 3 |
 *       | 2 4 |
 *     = (5 * 4) - (2 * 3) = 14 (counter-clockwise)
 *
 * @tparam T
 * @param a
 * @param b
 * @param c
 * @return T positive for counter-clockwise, negative for clockwise, zero for
 * collinear
 */
template <typename T>
T GetCrossProduct(const Point<T> &a, const Point<T> &b, const Point<T> &c) {
  return ((b.x - a.x) * (c.y - a.y) - (b.y - a.y) * (c.x - a.x));
}

template <typename T>
Point<T> SwapAndGetFirstPoint(std::vector<Point<T>> &points) {
  auto iterator = std::min_element(
      points.begin(), points.end(), [](const Point<T> a, const Point<T> b) {
        return ((a.y < b.y) || ((a.y == b.y) && (a.x < b.x)));
      });
  auto old_first_point = points[0];
  points[0] = *iterator;
  *iterator = old_first_point;
  return points[0];
}

template <typename T>
std::vector<Point<T>> GetConvexHull(std::vector<Point<T>> &points) {
  if (points.size() < 3) {
    return std::vector<Point<T>>{};
  }
  auto first_point = SwapAndGetFirstPoint(points);
  std::sort(points.begin() + 1, points.end(),
            [&](const Point<T> &b, const Point<T> &c) {
              return (GetCrossProduct(first_point, b, c) < 0);
            });

  std::vector<Point<T>> result;
  auto it = points.begin();

  auto value = *it++;
  std::cout << "Adding... " << value.x << " " << value.y << std::endl;

  result.push_back(value);

  value = *it++;
  std::cout << "Adding... " << value.x << " " << value.y << std::endl;

  result.push_back(value);

  value = *it++;
  std::cout << "Adding... " << value.x << " " << value.y << std::endl;

  result.push_back(value);

  while (it != points.end()) {
    // StepConverge
    // Pop off any points that make a convex angle with *it
    while (GetCrossProduct(*(result.rbegin() + 1), *(result.rbegin()), *it) >=
           0) {
      std::cout << "Popping out: " << result.back().x << " " << result.back().y
                << std::endl;
      result.pop_back();
    }

    auto value = *it++;
    std::cout << "Adding... " << value.x << " " << value.y << std::endl;
    result.push_back(value);
  }

  return result;
}

template <typename T>
class ConvexHull {
 public:
  using Callback = std::function<void(const Point<T> &)>;
  explicit ConvexHull(const std::vector<Point<T>> &points)
      : points_(points), current_index_(0) {
    auto first_point = SwapAndGetFirstPoint(points_);
    std::sort(points_.begin() + 1, points_.end(),
              [&](const Point<T> &b, const Point<T> &c) {
                return (GetCrossProduct(first_point, b, c) < 0);
              });
    initialize_first_three_points_ = true;
  }

  std::optional<std::vector<Point<T>>> StepForward(Callback insert_callback,
                                                   Callback pop_callback) {
    if (points_.size() < 3) {
      return std::nullopt;
    } else {
      if (initialize_first_three_points_) {
        for (auto i = 0; i < 3; ++i) {
          InsertCurrentValueWithCallback(insert_callback);
        }
        initialize_first_three_points_ = false;
        return std::nullopt;
      }
    }

    if (current_index_ < points_.size()) {
      if (GetCrossProduct(*(results_.rbegin() + 1), *(results_.rbegin()),
                          points_.at(current_index_)) >= 0) {
        PopCurrentValueWithCallback(pop_callback);
      } else {
        InsertCurrentValueWithCallback(insert_callback);
      }
    } else {
      return results_;
    }

    return std::nullopt;
  }

 private:
  void InsertCurrentValueWithCallback(Callback insert_callback) {
    const auto value = points_.at(current_index_);
    insert_callback(value);
    results_.push_back(value);
    current_index_++;
  }

  void PopCurrentValueWithCallback(Callback pop_callback) {
    pop_callback(results_.back());
    results_.pop_back();
  }

  std::vector<Point<T>> points_;
  std::vector<Point<T>> results_;
  size_t current_index_;
  bool initialize_first_three_points_;
};

int main() {
  auto points = std::vector<Point<int>>{{0, 3}, {1, 1}, {2, 2}, {4, 4},
                                        {0, 0}, {1, 2}, {3, 1}, {3, 3}};
  auto result = GetConvexHull(points);
  std::cout << "Results" << std::endl;
  for (const auto value : result) {
    std::cout << value.x << " " << value.y << std::endl;
  }

  std::cout << "-------------------" << std::endl;
  auto convex_hull = ConvexHull(points);
  while (convex_hull.StepForward(
             [](const Point<int> &point) {
               std::cout << "Adding... " << point.x << " " << point.y
                         << std::endl;
             },
             [](const Point<int> &point) {
               std::cout << "Popping out: " << point.x << " " << point.y
                         << std::endl;
             }) == std::nullopt) {
  }
  auto new_result = convex_hull.StepForward([](const Point<int> &) {},
                                            [](const Point<int> &) {});
  std::cout << "Results" << std::endl;
  for (const auto value : *new_result) {
    std::cout << value.x << " " << value.y << std::endl;
  }
}