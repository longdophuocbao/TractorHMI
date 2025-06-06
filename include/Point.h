#ifndef POINT_H
#define POINT_H
#include <cmath>
#include <limits>
struct Point
{
  double x = 0.0;
  double y = 0.0;

  Point() = default;
  Point(double x_val, double y_val) : x(x_val), y(y_val) {}

  Point operator-(const Point &other) const { return Point(x - other.x, y - other.y); }
  Point operator+(const Point &other) const { return Point(x + other.x, y + other.y); }
  Point operator*(double scalar) const { return Point(x * scalar, y * scalar); }
  double dot(const Point &other) const { return x * other.x + y * other.y; }
  double length() const { return std::sqrt(x * x + y * y); }

  double lengthSq() const
  {
    return x * x + y * y;
  }
  Point normalized() const
  {
    double l = length();
    if (l > 1e-9)
    { // Tránh chia cho 0
      return Point(x / l, y / l);
    }
    return Point(0.0, 0.0); // Trả về vector không nếu độ dài quá nhỏ
  }
  Point perpendicular() const { return Point(-y, x); } // Vector vuông góc 90 độ
};
#endif // POINT_H