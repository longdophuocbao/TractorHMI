#ifndef FUNCTION_H
#define FUNCTION_H

#include <vector>
#include "Point.h" // Nếu cấu trúc Point được tách riêng, hãy include file này
// Forward declaration của HMI_Display
class HMI_Display;

// Cấu trúc đại diện cho một đoạn thẳng
struct Segment
{
  Point p1;
  Point p2;
};

// Enum trạng thái chương trình
enum ProgramState
{
  WAITING_FOR_POINTS,
  POINTS_ENTERED_READY_TO_DRAW,
  PATH_DISPLAYED
};

struct GpsPoint
{
  double longitude; // X
  double latitude;  // Y
};

// Khai báo các hàm
double degreesToRadians(double degrees);
void resetAndClear();
void handlePointInput();
void calculateScaleAndTransformPoints();
Point transformGpsToScreen(GpsPoint gp);
void drawFieldAndPath();
bool segmentIntersection(const Point &p1, const Point &p2, const Point &p3, const Point &p4, Point &intersection_point);
std::vector<Segment> clipLineWithPolygon(const Point &line_origin, const Point &line_dir_normalized, const std::vector<Point> &polygon_vertices, double current_working_width);
bool isInside(const Point &p, const std::vector<Point> &polygon);
int findClosestEdgeStartIndex(const std::vector<Point> &polygon, const Point &p, Point &closest_point_on_polygon);
void generatePath(const std::vector<Point> &polygon_vertices,
                  double workingWidth,
                  const Point &current_tractor_screen_pos, // Vị trí máy cày hiện tại
                  int initial_path_direction_preference,   // Tương tự StartDir cũ
                  HMI_Display &hmi_display);
// void generatePath(const std::vector<Point> &polygon_vertices, double workingWidth, int startVertexIndex, int startDirGlobal, HMI_Display &hmi_display);
void updateAndDrawTractorPositionHMI();
void readAndProcessGpsData();
void getTractorPicDimensions(int pic_id, int &width, int &height);
#endif // FUNCTION_H