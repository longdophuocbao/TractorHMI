#include <Arduino.h>
#include <vector>
#include <HardwareSerial.h>
#include <algorithm> // Cho std::min, std::max, std::sort, std::reverse, std::round
#include <cmath>     // Cho std::sqrt, std::fabs, std::cos, std::sin, M_PI
#include <limits>    // Cho std::numeric_limits
#include <iostream>  // Cho std::cerr (chỉ dùng để cảnh báo, nên xóa khi chạy trên device)

// --- Cấu hình ---
const int NEXTION_RX_PIN = 18;
const int NEXTION_TX_PIN = 19;
const long NEXTION_BAUD_RATE = 9600;
HardwareSerial NextionSerial = Serial2; // Sử dụng Serial2 cho Nextion

// KÍCH THƯỚC MÀN HÌNH NEXTION VÀ PADDING
const int SCREEN_WIDTH_PX = 800;
const int SCREEN_HEIGHT_PX = 480;
const int SCREEN_PADDING_PX = 10;

// Chiều rộng làm việc thực tế (mét) và quy đổi sang pixel
float working_width_real_meters = 4.0f; // Ví dụ: nông cụ rộng 4 mét
int working_width_px = 0;               // Chiều rộng làm việc đã scale sang pixel (sẽ được tính toán)

// Hằng số cho chuyển đổi GPS
const float METERS_PER_DEGREE_LATITUDE = 111132.954f; // Hằng số gần đúng

// Cấu trúc để lưu trữ điểm tọa độ GPS (kinh độ, vĩ độ)
struct GpsPoint
{
  double longitude; // X
  double latitude;  // Y
};

// Cấu trúc để lưu trữ một điểm tọa độ màn hình (pixel) - Sử dụng double
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

// Cấu trúc đại diện cho một đoạn thẳng
struct Segment
{
  Point p1;
  Point p2;
};

// Các biến điều khiển đường đi (có thể thay đổi qua Serial)
int StartPoint = 0;        // Chỉ số đỉnh bắt đầu gần nhất (mặc định là 0)
int parallelEdgeIndex = 1; // Chỉ số đỉnh bắt đầu của cạnh định hướng (mặc định là cạnh 0->1)
// int StartDir = 1; // Hướng bắt đầu (0: trên->dưới, 1: dưới->lên) - Hiện không dùng trực tiếp trong generatePath

// Lớp HMI_Display (như bạn cung cấp)
class HMI_Display
{
public:
  HMI_Display(HardwareSerial &serial = NextionSerial) : _serial(&serial) {}

  void begin()
  {
    _serial->begin(NEXTION_BAUD_RATE, SERIAL_8N1, NEXTION_RX_PIN, NEXTION_TX_PIN);
    delay(100); // Chờ serial ổn định
  }

  void drawPic(int x, int y, int pic_id)
  {
    String command = "pic " + String(x) + "," + String(y) + "," + String(pic_id);
    sendCommand(command);
  }

  void drawLine(int x1, int y1, int x2, int y2, int color)
  {
    String command = "line " + String(x1) + "," + String(y1) + "," + String(x2) + "," + String(y2) + "," + String(color);
    sendCommand(command);
  }

  // Phiên bản nhận Point (double) - tiện lợi hơn
  void drawLine(const Point &p1, const Point &p2, int color)
  {
    // Ép kiểu sang int khi vẽ lên màn hình Nextion
    drawLine(static_cast<int>(std::round(p1.x)), static_cast<int>(std::round(p1.y)),
             static_cast<int>(std::round(p2.x)), static_cast<int>(std::round(p2.y)),
             color);
  }

  void clearScreen(int color)
  {
    String command = "cls " + String(color);
    sendCommand(command);
    delay(50); // cls có thể cần delay lâu hơn
  }

  void drawPointMarker(int x, int y, int color)
  {
    const int radius = 4; // Bán kính điểm đánh dấu
    String command = "cirs " + String(x) + "," + String(y) + "," + String(radius) + "," + String(color);
    sendCommand(command);
  }

  // Phiên bản nhận Point (double)
  void drawPointMarker(const Point &p, int color)
  {
    drawPointMarker(static_cast<int>(std::round(p.x)), static_cast<int>(std::round(p.y)), color);
  }

private:
  HardwareSerial *_serial;
  void sendCommand(String cmd)
  {
    if (_serial)
    {
      _serial->print(cmd);
      _serial->write(0xff);
      _serial->write(0xff);
      _serial->write(0xff);
      _serial->flush();
    }
    delay(5); // Delay nhỏ giữa các lệnh
  }
};

// --- Khởi tạo đối tượng HMI ---
HMI_Display hmi(NextionSerial);

// --- Định nghĩa màu Nextion ---
const int WHITE = 65535;
const int BLACK = 0;
const int GREEN = 2016;
const int BLUE = 31;
const int RED = 63488;
const int YELLOW = 65504;

// --- Biến toàn cục ---
std::vector<GpsPoint> field_vertices_gps; // Lưu tọa độ GPS gốc
std::vector<Point> field_vertices_screen; // Lưu tọa độ màn hình đã scale (Point double)

double min_lon_gps, max_lon_gps, min_lat_gps, max_lat_gps; // Biên GPS
float scale_factor_combined;                               // Hệ số scale chung từ độ sang pixel
float offset_x_for_screen, offset_y_for_screen;            // Offset để căn giữa trên màn hình
double average_latitude_rad = 0.0;                         // Vĩ độ trung bình (radian) để tính toán chuyển đổi

// Trạng thái chương trình
enum ProgramState
{
  WAITING_FOR_POINTS,
  POINTS_ENTERED_READY_TO_DRAW,
  PATH_DISPLAYED
};
ProgramState currentState = WAITING_FOR_POINTS;

// --- Khai báo hàm ---
void generatePath(const std::vector<Point> &polygon_vertices, double workingWidth, int startVertexIndex, int edgeChoiceForOrientation, HMI_Display &hmi_display);
void calculateScaleAndTransformPoints();
Point transformGpsToScreen(GpsPoint gp);
void handlePointInput();
void drawFieldAndPath();
void resetAndClear();
double degreesToRadians(double degrees);
bool segmentIntersection(const Point &p1, const Point &p2, const Point &p3, const Point &p4, Point &intersection_point);
std::vector<Segment> clipLineWithPolygon(const Point &line_origin, const Point &line_dir_normalized, const std::vector<Point> &polygon_vertices);
bool isInside(const Point &p, const std::vector<Point> &polygon);

// --- Hàm setup ---
void setup()
{
  Serial.begin(115200);
  while (!Serial)
    ; // Chờ Serial Monitor mở (cho một số board)

  hmi.begin(); // Khởi tạo giao tiếp Nextion
  delay(1000); // Chờ HMI khởi động

  resetAndClear(); // Xóa màn hình và reset trạng thái ban đầu
  Serial.println(F("--- May Nong Nghiep - Dieu Huong Zigzag (GPS) ---"));
  Serial.println(F("Man hinh: 800x480"));
}

// --- Hàm loop ---
void loop()
{
  switch (currentState)
  {
  case WAITING_FOR_POINTS:
    handlePointInput();
    break;
  case POINTS_ENTERED_READY_TO_DRAW:
    drawFieldAndPath();
    currentState = PATH_DISPLAYED;
    Serial.println(F("Da ve xong. Go 'clear'/'change' de thay doi hoac nhap lai."));
    break;
  case PATH_DISPLAYED:
    // Cho phép xóa hoặc thay đổi thông số sau khi đã vẽ
    handlePointInput(); // Tiếp tục lắng nghe lệnh 'clear', 'change...'
    break;
  }
}

// --- Triển khai các hàm ---

double degreesToRadians(double degrees)
{
  return degrees * M_PI / 180.0;
}

void resetAndClear()
{
  field_vertices_gps.clear();
  field_vertices_screen.clear();
  hmi.clearScreen(WHITE); // Xóa màn hình HMI với màu trắng
  currentState = WAITING_FOR_POINTS;
  StartPoint = 0; // Reset các giá trị về mặc định
  parallelEdgeIndex = 1;
  // StartDir = 1;
  Serial.println(F("---------------------------------------------"));
  Serial.println(F("Da xoa. Nhap lai toa do GPS (KinhDo,ViDo). Vi du: 106.6602,10.7769"));
  Serial.println(F("Nhap it nhat 3 diem."));
  Serial.println(F("Go 'done' khi hoan tat."));
  Serial.println(F("Go 'change start {index}' de doi diem bat dau (vi du: change start 2)."));
  Serial.println(F("Go 'change edge {index}' de doi canh dinh huong (vi du: change edge 1)."));
  // Serial.println(F("Go 'change dir {0|1}' de doi huong ban dau.")); // Nếu cần
}

void handlePointInput()
{
  if (Serial.available() > 0)
  {
    String input = Serial.readStringUntil('\n');
    input.trim();

    if (input.equalsIgnoreCase("done"))
    {
      // Tùy chọn: Thêm điểm cố định để test nhanh
      field_vertices_gps.push_back({120.26566389135687, 22.626949475307885});
      field_vertices_gps.push_back({120.26609465404412, 22.627049646169556});
      field_vertices_gps.push_back({120.26614325919708, 22.626493423503955});
      field_vertices_gps.push_back({120.26563962832518, 22.62629604619295});

      if (field_vertices_gps.size() >= 3)
      {
        Serial.println(F("Hoan tat nhap diem. Dang tinh toan scale va xu ly..."));
        calculateScaleAndTransformPoints();
        currentState = POINTS_ENTERED_READY_TO_DRAW;
      }
      else
      {
        Serial.println(F("Loi: Can it nhat 3 diem GPS. Vui long nhap them."));
      }
    }
    else if (input.equalsIgnoreCase("clear"))
    {
      resetAndClear();
    }
    else if (input.equalsIgnoreCase("change start "))
    {
      // int newStart = input.substring(13).toInt(); // Lấy số sau "change start "
      // if (newStart >= 0 && newStart < field_vertices_gps.size())
      // {
      //   StartPoint = newStart;
      //   Serial.printf("Doi diem bat dau gan nhat thanh dinh %d\n", StartPoint);
      //   if (currentState == PATH_DISPLAYED)
      //   {
      //     currentState = POINTS_ENTERED_READY_TO_DRAW;
      //   } // Vẽ lại nếu đang hiển thị
      // }
      // else
      // {
      //   Serial.println(F("Loi: Chi so diem bat dau khong hop le."));
      // }
    }
    else if (input.equalsIgnoreCase("change edge "))
    {
      // int newEdge = input.substring(12).toInt(); // Lấy số sau "change edge "
      // if (newEdge >= 0 && newEdge < field_vertices_gps.size())
      // {
      //   parallelEdgeIndex = newEdge;
      //   Serial.printf("Doi canh dinh huong thanh canh bat dau tu dinh %d\n", parallelEdgeIndex);
      //   if (currentState == PATH_DISPLAYED)
      //   {
      //     currentState = POINTS_ENTERED_READY_TO_DRAW;
      //   } // Vẽ lại nếu đang hiển thị
      // }
      // else
      // {
      //   Serial.println(F("Loi: Chi so canh dinh huong khong hop le."));
      // }
    }
    // else if (input.equalsIgnoreCase("ChangeDir")) { // Bỏ comment nếu muốn dùng StartDir
    //     StartDir = (StartDir + 1) % 2;
    //     Serial.printf("StartDir: %d\n", StartDir);
    //     if (currentState == PATH_DISPLAYED) { currentState = POINTS_ENTERED_READY_TO_DRAW; }
    // }
    else
    { // Xử lý nhập tọa độ GPS
      int commaIndex = input.indexOf(',');
      if (commaIndex != -1)
      {
        String lonStr = input.substring(0, commaIndex);
        String latStr = input.substring(commaIndex + 1);
        lonStr.trim();
        latStr.trim();

        double lon = lonStr.toDouble();
        double lat = latStr.toDouble();

        // Kiểm tra cơ bản (toDouble trả về 0.0 nếu lỗi, trừ khi chuỗi là "0" hoặc "0.xx")
        bool lonValid = !(lon == 0.0 && !lonStr.equals("0") && !lonStr.startsWith("0."));
        bool latValid = !(lat == 0.0 && !latStr.equals("0") && !latStr.startsWith("0."));

        if (lonValid && latValid)
        {
          field_vertices_gps.push_back({lon, lat});
          Serial.print(F("Da them diem GPS: (Lon: "));
          Serial.print(lon, 6);
          Serial.print(F(", Lat: "));
          Serial.print(lat, 6);
          Serial.println(F(")"));
          Serial.print(F("Tong so diem: "));
          Serial.println(field_vertices_gps.size());
        }
        else
        {
          Serial.println(F("Loi: Dinh dang GPS khong hop le. Su dung KinhDo,ViDo"));
        }
      }
      else
      {
        Serial.println(F("Loi: Lenh khong hop le."));
      }
    }
  }
}

void calculateScaleAndTransformPoints()
{
  if (field_vertices_gps.empty())
    return;

  // 1. Tìm biên GPS và tính vĩ độ trung bình
  min_lon_gps = max_lon_gps = field_vertices_gps[0].longitude;
  min_lat_gps = max_lat_gps = field_vertices_gps[0].latitude;
  double sum_lat_for_avg = 0;

  for (const auto &gp : field_vertices_gps)
  {
    min_lon_gps = std::min(min_lon_gps, gp.longitude);
    max_lon_gps = std::max(max_lon_gps, gp.longitude);
    min_lat_gps = std::min(min_lat_gps, gp.latitude);
    max_lat_gps = std::max(max_lat_gps, gp.latitude);
    sum_lat_for_avg += gp.latitude;
  }
  average_latitude_rad = degreesToRadians(sum_lat_for_avg / field_vertices_gps.size());

  Serial.print("GPS Bounds: Lon[");
  Serial.print(min_lon_gps, 6);
  Serial.print(" - ");
  Serial.print(max_lon_gps, 6);
  Serial.print("], Lat[");
  Serial.print(min_lat_gps, 6);
  Serial.print(" - ");
  Serial.print(max_lat_gps, 6);
  Serial.println("]");
  Serial.print("Average Latitude (rad): ");
  Serial.println(average_latitude_rad, 6);

  // 2. Tính toán hệ số scale
  double world_width_degrees = max_lon_gps - min_lon_gps;
  double world_height_degrees = max_lat_gps - min_lat_gps;

  // Tránh chia cho 0 nếu tất cả các điểm thẳng hàng
  if (std::fabs(world_width_degrees) < 1e-9)
    world_width_degrees = 1e-9;
  if (std::fabs(world_height_degrees) < 1e-9)
    world_height_degrees = 1e-9;

  // Tính scale riêng cho X và Y để vừa với màn hình (trừ padding)
  float available_screen_width = SCREEN_WIDTH_PX - 2 * SCREEN_PADDING_PX;
  float available_screen_height = SCREEN_HEIGHT_PX - 2 * SCREEN_PADDING_PX;

  float scale_x_deg_to_px = available_screen_width / world_width_degrees;
  float scale_y_deg_to_px = available_screen_height / world_height_degrees;

  // Chọn scale nhỏ hơn để đảm bảo vừa cả hai chiều và giữ đúng tỷ lệ
  scale_factor_combined = std::min(scale_x_deg_to_px, scale_y_deg_to_px);

  Serial.print("Combined scale factor (px/degree): ");
  Serial.println(scale_factor_combined, 4);

  // 3. Tính toán offset để căn giữa trên màn hình
  float scaled_world_width_px = world_width_degrees * scale_factor_combined;
  float scaled_world_height_px = world_height_degrees * scale_factor_combined;

  offset_x_for_screen = SCREEN_PADDING_PX + (available_screen_width - scaled_world_width_px) / 2.0f;
  offset_y_for_screen = SCREEN_PADDING_PX + (available_screen_height - scaled_world_height_px) / 2.0f; // Offset cho Y

  Serial.print("Screen offset X (px): ");
  Serial.print(offset_x_for_screen, 2);
  Serial.print(", Screen offset Y (px): ");
  Serial.println(offset_y_for_screen, 2);

  // 4. Chuyển đổi tất cả các điểm GPS sang tọa độ màn hình
  field_vertices_screen.clear();
  for (const auto &gps_pt : field_vertices_gps)
  {
    field_vertices_screen.push_back(transformGpsToScreen(gps_pt));
  }

  // 5. Tính working_width_px dựa trên working_width_real_meters và scale
  // Chuyển mét sang độ kinh độ tại vĩ độ trung bình
  double meters_per_degree_lon_at_avg_lat = METERS_PER_DEGREE_LATITUDE * std::cos(average_latitude_rad);
  if (std::fabs(meters_per_degree_lon_at_avg_lat) < 1.0)
    meters_per_degree_lon_at_avg_lat = 1.0; // Tránh chia cho 0

  double working_width_as_degrees_lon = working_width_real_meters / meters_per_degree_lon_at_avg_lat;
  // Chuyển độ kinh độ sang pixel màn hình
  working_width_px = static_cast<int>(std::round(working_width_as_degrees_lon * scale_factor_combined));

  if (working_width_px < 1)
    working_width_px = 1; // Đảm bảo chiều rộng tối thiểu là 1 pixel
  Serial.print("Working width real (m): ");
  Serial.print(working_width_real_meters);
  Serial.print(" -> Approx as degrees Lon: ");
  Serial.print(working_width_as_degrees_lon, 8);
  Serial.print(" -> Working width (px): ");
  Serial.println(working_width_px);
}

Point transformGpsToScreen(GpsPoint gp)
{
  // Chuyển đổi kinh độ (X) sang pixel X
  double sx = offset_x_for_screen + (gp.longitude - min_lon_gps) * scale_factor_combined;
  // Chuyển đổi vĩ độ (Y) sang pixel Y, lật ngược trục Y
  // Vĩ độ lớn hơn (Bắc) -> Y pixel nhỏ hơn (gần đỉnh màn hình)
  double sy = offset_y_for_screen + (max_lat_gps - gp.latitude) * scale_factor_combined;
  // Trả về Point với tọa độ double
  return {sx, sy};
}

void drawFieldAndPath()
{
  if (field_vertices_screen.empty())
  {
    Serial.println(F("Loi: Khong co diem da chuyen doi de ve."));
    return;
  }

  hmi.clearScreen(WHITE); // Xóa màn hình trước khi vẽ mới
  delay(100);             // Chờ màn hình xóa

  // Vẽ các điểm đỉnh đã scale
  Serial.println(F("Ve cac diem dinh (man hinh):"));
  for (const auto &screen_pt : field_vertices_screen)
  {
    hmi.drawPointMarker(screen_pt, RED); // Dùng hàm nhận Point
    Serial.print(F("  ("));
    Serial.print(screen_pt.x, 1);
    Serial.print(F(", "));
    Serial.print(screen_pt.y, 1);
    Serial.println(F(")"));
  }

  // Vẽ các cạnh của thửa ruộng đã scale
  if (field_vertices_screen.size() >= 2)
  {
    Serial.println(F("Ve cac canh thua ruong..."));
    for (size_t i = 0; i < field_vertices_screen.size(); ++i)
    {
      hmi.drawLine(field_vertices_screen[i], field_vertices_screen[(i + 1) % field_vertices_screen.size()], BLUE);
    }
  }

  // Tạo và vẽ đường đi zigzag
  if (field_vertices_screen.size() >= 3 && working_width_px > 0)
  {
    StartPoint = 0;
    parallelEdgeIndex = 0;
    Serial.print(F("Dang tao duong di voi working_width_px = "));
    Serial.print(working_width_px);
    Serial.print(F(", startVertexIndex = "));
    Serial.print(StartPoint);
    Serial.print(F(", edgeChoiceForOrientation = "));
    Serial.println(parallelEdgeIndex);
    
    // Gọi hàm generatePath với các tham số đã tính toán
    // Chuyển working_width_px sang double khi gọi
    generatePath(field_vertices_screen, static_cast<double>(working_width_px), StartPoint, parallelEdgeIndex, hmi);

    Serial.println(F("Da hoan thanh tao va ve duong di."));
  }
  else
  {
    if (field_vertices_screen.size() < 3)
    {
      Serial.println(F("Loi: Can it nhat 3 dinh da chuyen doi de tao duong di."));
    }
    if (working_width_px <= 0)
    {
      Serial.println(F("Loi: Chieu rong lam viec (pixel) khong hop le."));
    }
  }
}

// --- Triển khai các hàm hình học ---

// Kiểm tra giao điểm 2 đoạn thẳng (p1-p2 và p3-p4)
bool segmentIntersection(const Point &p1, const Point &p2, const Point &p3, const Point &p4, Point &intersection_point)
{
  double dx1 = p2.x - p1.x;
  double dy1 = p2.y - p1.y;
  double dx2 = p4.x - p3.x;
  double dy2 = p4.y - p3.y;
  double dx3 = p1.x - p3.x;
  double dy3 = p1.y - p3.y;

  double det = dx1 * dy2 - dy1 * dx2;
  double t_num = dx3 * dy2 - dy3 * dx2;
  double u_num = dx1 * dy3 - dy1 * dx3; // Lưu ý dấu của u_num so với một số công thức khác

  // Sử dụng dung sai nhỏ để xử lý lỗi số thực
  const double INTERSECTION_EPSILON  = 1e-9;

  if (std::fabs(det) < INTERSECTION_EPSILON )
  {
    // Các đường thẳng song song hoặc trùng nhau
    // Kiểm tra xem chúng có trùng nhau một phần không (phức tạp hơn, tạm bỏ qua)
    return false;
  }

  double t = t_num / det;
  double u = u_num / det; // u = -(dx1 * dy3 - dy1 * dx3) / det

  // Kiểm tra xem điểm giao cắt có nằm trên cả hai đoạn thẳng không (0 <= t <= 1 và 0 <= u <= 1)
  if (t >= -INTERSECTION_EPSILON && t <= 1.0 + INTERSECTION_EPSILON && u >= -INTERSECTION_EPSILON && u <= 1.0 + INTERSECTION_EPSILON)
  {
    intersection_point.x = p1.x + t * dx1;
    intersection_point.y = p1.y + t * dy1;
    return true;
  }

  return false; // Giao điểm nằm ngoài ít nhất một đoạn thẳng
}

// // Kiểm tra điểm có nằm trong đa giác không (Ray Casting)
// bool isInside(const Point &p, const std::vector<Point> &polygon)
// {
//   int n = polygon.size();
//   if (n < 3)
//     return false;
//   bool inside = false;
//   double px = p.x;
//   double py = p.y;

//   for (int i = 0, j = n - 1; i < n; j = i++)
//   {
//     double vxi = polygon[i].x;
//     double vyi = polygon[i].y;
//     double vxj = polygon[j].x;
//     double vyj = polygon[j].y;

//     // Kiểm tra xem ray cắt cạnh không
//     bool intersect = ((vyi > py) != (vyj > py)) &&
//                      (px < (vxj - vxi) * (py - vyi) / (vyj - vyi) + vxi);

//     if (intersect)
//     {
//       inside = !inside;
//     }
//   }
//   return inside;
// }

const double GEOMETRY_EPSILON = 1e-9;

// --- Triển khai các hàm hình học ---

// Hàm trợ giúp: Tìm giao điểm của đường thẳng vô hạn và một đoạn thẳng
// Line: O + t*D (D là vector chuẩn hóa)
// Segment: P1 + u*S (S = P2-P1), 0 <= u <= 1
// Trả về true nếu giao điểm nằm trên đoạn thẳng (0<=u<=1)
bool intersectLineSegment(const Point &line_origin, const Point &line_dir_normalized, // Đường thẳng vô hạn
                          const Point &seg_p1, const Point &seg_p2,                   // Đoạn thẳng
                          Point &intersection_point)                                  // Output: điểm giao cắt
{
  Point seg_dir = seg_p2 - seg_p1;           // Vector hướng của đoạn thẳng
  Point diff_origins = line_origin - seg_p1; // Vector nối gốc đoạn thẳng tới gốc đường thẳng

  double Dx = line_dir_normalized.x;
  double Dy = line_dir_normalized.y;
  double Sx = seg_dir.x;
  double Sy = seg_dir.y;
  double ODiffx = diff_origins.x;
  double ODiffy = diff_origins.y;

  // Tính định thức (perp_dot(D, S))
  double det = Dx * Sy - Dy * Sx;

  // Kiểm tra nếu đường thẳng và đoạn thẳng gần như song song
  if (std::fabs(det) < GEOMETRY_EPSILON)
  {
    // Song song hoặc trùng nhau.
    // Để xử lý đầy đủ trường hợp trùng nhau cần kiểm tra phức tạp hơn.
    // Tạm thời coi là không cắt để đơn giản hóa.
    return false;
  }

  // Tính tham số u cho đoạn thẳng (kiểm tra xem giao điểm có nằm trên đoạn 0 <= u <= 1)
  // u = perp_dot(P1 - O, D) / perp_dot(D, S) = perp_dot(-diff_origins, D) / det
  double u_num = -ODiffx * Dy + ODiffy * Dx; // perp_dot(-diff_origins, D)
  double u = u_num / det;

  // Chỉ chấp nhận giao điểm nếu nó nằm trên đoạn thẳng (trong khoảng [0, 1] với dung sai)
  if (u >= -GEOMETRY_EPSILON && u <= 1.0 + GEOMETRY_EPSILON)
  {
    // Tính điểm giao cắt thực tế bằng cách dùng tham số u
    intersection_point = seg_p1 + seg_dir * u;
    return true;
  }

  // Giao điểm nằm ngoài đoạn thẳng
  return false;
}

// Hàm kiểm tra điểm có nằm trong đa giác không (Ray Casting)
// Cần cẩn thận với các điểm nằm trên biên
bool isInside(const Point &p, const std::vector<Point> &polygon)
{
  int n = polygon.size();
  if (n < 3)
    return false;
  bool inside = false;
  double px = p.x;
  double py = p.y;

  for (int i = 0, j = n - 1; i < n; j = i++)
  {
    double vxi = polygon[i].x;
    double vyi = polygon[i].y;
    double vxj = polygon[j].x;
    double vyj = polygon[j].y;

    // Xử lý trường hợp điểm nằm trên cạnh ngang (có thể cần thiết)
    // if (((vyi == py && vxi <= px && px <= vxj) || (vyj == py && vxj <= px && px <= vxi)) && (vyi == vyj)) {
    //     return true; // Điểm nằm trên cạnh ngang -> coi là trong (hoặc ngoài tùy yêu cầu)
    // }
    // Xử lý trường hợp điểm nằm trên cạnh dọc (có thể cần thiết)
    // if (((vxi == px && vyi <= py && py <= vyj) || (vxj == px && vyj <= py && py <= vyi)) && (vxi == vxj)) {
    //     return true; // Điểm nằm trên cạnh dọc -> coi là trong (hoặc ngoài tùy yêu cầu)
    // }

    // Kiểm tra nếu tia ngang từ p cắt cạnh (vi, vj)
    bool intersect = ((vyi > py) != (vyj > py)) &&                        // Cạnh phải cắt qua đường ngang của tia
                     (px < (vxj - vxi) * (py - vyi) / (vyj - vyi) + vxi); // Điểm cắt phải nằm bên phải p

    // Xử lý trường hợp cạnh ngang bị bỏ qua bởi điều kiện trên
    if ((vyi == py) && (vyj == py))
    {
      // Nếu p nằm giữa cạnh ngang thì không nên đổi trạng thái `inside`
      // Nhưng nếu p nằm trên cạnh, hàm có thể trả về true/false không nhất quán
      // Đây là một điểm yếu của Ray Casting cơ bản khi điểm nằm trên biên ngang.
    }
    else if (intersect)
    {
      inside = !inside; // Đảo trạng thái trong/ngoài
    }
  }
  return inside;
}

// --- Hàm clipLineWithPolygon (Triển khai đầy đủ hơn) ---
/**
 * @brief Cắt một đường thẳng vô hạn bởi một đa giác đơn giản.
 *
 * @param line_origin Một điểm bất kỳ trên đường thẳng.
 * @param line_dir_normalized Vector hướng đã chuẩn hóa của đường thẳng.
 * @param polygon_vertices Vector các đỉnh của đa giác theo thứ tự.
 * @return std::vector<Segment> Danh sách các đoạn thẳng là phần giao của đường thẳng và đa giác.
 */
std::vector<Segment> clipLineWithPolygon(const Point &line_origin, const Point &line_dir_normalized, const std::vector<Point> &polygon_vertices)
{
  std::vector<Segment> clipped_segments;
  std::vector<Point> intersections;
  size_t n_poly = polygon_vertices.size();

  if (n_poly < 3)
  {
    Serial.println("ClipLP: Loi - Da giac khong du dinh.");
    return clipped_segments; // Không thể cắt với đa giác không hợp lệ
  }
  if (line_dir_normalized.length() < GEOMETRY_EPSILON)
  {
    Serial.println("ClipLP: Loi - Vector huong duong thang khong hop le.");
    return clipped_segments;
  }

  // 1. Tìm tất cả các giao điểm của đường thẳng vô hạn với các cạnh của đa giác
  for (size_t i = 0; i < n_poly; ++i)
  {
    const Point &poly_p1 = polygon_vertices[i];
    const Point &poly_p2 = polygon_vertices[(i + 1) % n_poly]; // Cạnh cuối nối về đỉnh đầu
    Point current_intersection;

    // Sử dụng hàm trợ giúp để tìm giao điểm đường thẳng - đoạn thẳng
    if (intersectLineSegment(line_origin, line_dir_normalized, poly_p1, poly_p2, current_intersection))
    {
      intersections.push_back(current_intersection);
    }
  }
  // Serial.printf("ClipLP: Found %d raw intersections.\n", intersections.size());

  // Nếu không có giao điểm hoặc chỉ có 1 (tiếp tuyến tại điểm không phải đỉnh?), không có đoạn nào bên trong
  if (intersections.size() < 2)
  {
    // Serial.println("ClipLP: Less than 2 intersections found.");
    return clipped_segments;
  }

  // 2. Sắp xếp các điểm giao cắt dọc theo hướng của đường thẳng
  std::sort(intersections.begin(), intersections.end(), [&](const Point &a, const Point &b)
            {
        // Chiếu vector từ gốc đường thẳng tới điểm giao cắt lên hướng đường thẳng
        double proj_a = (a - line_origin).dot(line_dir_normalized);
        double proj_b = (b - line_origin).dot(line_dir_normalized);
        return proj_a < proj_b; });
  // Serial.println("ClipLP: Intersections sorted.");

  // 3. Loại bỏ các điểm giao cắt trùng lặp (do đi qua đỉnh hoặc lỗi số thực)
  // Cần một hàm so sánh Point với dung sai
  auto points_are_close = [](const Point &a, const Point &b)
  {
    return (a - b).length() < GEOMETRY_EPSILON * 10; // Tăng nhẹ dung sai cho unique
  };
  intersections.erase(std::unique(intersections.begin(), intersections.end(), points_are_close), intersections.end());
  // Serial.printf("ClipLP: Found %d unique intersections.\n", intersections.size());

  // Số lượng giao điểm phải là chẵn (trừ trường hợp đặc biệt)
  if (intersections.size() % 2 != 0)
  {
    Serial.printf("ClipLP: Canh bao - So luong giao diem duy nhat (%d) la so le! Co the do loi so thuc hoac tiep tuyen.\n", intersections.size());
    // Có thể thử bỏ điểm cuối nếu nghi ngờ lỗi, hoặc chỉ xử lý các cặp chẵn đầu tiên
    if (!intersections.empty())
    { // Bỏ điểm cuối nếu số lẻ
      intersections.pop_back();
    }
  }

  // 4. Lặp qua các cặp giao điểm đã sắp xếp và kiểm tra trung điểm
  for (size_t i = 0; i + 1 < intersections.size(); i += 2)
  {
    const Point &p_start = intersections[i];
    const Point &p_end = intersections[i + 1];

    // Tránh các đoạn có độ dài gần bằng 0
    if ((p_end - p_start).length() < GEOMETRY_EPSILON)
    {
      continue;
    }

    // Tính trung điểm của đoạn tạo bởi cặp giao điểm
    Point mid_point = (p_start + p_end) * 0.5;

    // 5. Kiểm tra xem trung điểm có nằm bên trong đa giác không
    if (isInside(mid_point, polygon_vertices))
    {
      // Nếu trung điểm nằm trong, đoạn thẳng này là phần giao hợp lệ
      clipped_segments.push_back({p_start, p_end});
    }
    // else {
    //     Serial.printf("ClipLP: Midpoint (%.1f, %.1f) between intersection %d and %d is OUTSIDE.\n", mid_point.x, mid_point.y, i, i+1);
    // }
  }

  // Serial.printf("ClipLP: Returning %d clipped segments.\n", clipped_segments.size());
  return clipped_segments;
}

// --- Hàm generatePath (Tạo đường đi Ziczac) ---
void generatePath(const std::vector<Point> &polygon_vertices,
                  double workingWidth, // Sử dụng double
                  int startVertexIndex,
                  int edgeChoiceForOrientation,
                  HMI_Display &hmi_display)
{
  // Không cần xóa màn hình ở đây, vì đã xóa trong drawFieldAndPath
  // Không cần vẽ lại đa giác ở đây

  // 1. Kiểm tra đầu vào cơ bản
  if (polygon_vertices.size() < 3)
  {
    Serial.println(F("GP: Loi - Da giac khong du dinh."));
    return;
  }
  if (workingWidth <= 0)
  {
    Serial.println(F("GP: Loi - Chieu rong lam viec khong hop le."));
    return;
  }
  if (edgeChoiceForOrientation < 0 || edgeChoiceForOrientation >= polygon_vertices.size())
  {
    Serial.printf("GP: Loi - Chi so canh dinh huong %d khong hop le.\n", edgeChoiceForOrientation);
    return;
  }
  if (startVertexIndex < 0 || startVertexIndex >= polygon_vertices.size())
  {
    Serial.printf("GP: Canh bao - Chi so dinh bat dau %d khong hop le, dung dinh 0.\n", startVertexIndex);
    startVertexIndex = 0;
  }

  // 2. Tính toán hướng quét và hướng offset (chuẩn hóa)
  size_t num_vertices = polygon_vertices.size();
  const Point &edge_p1 = polygon_vertices[edgeChoiceForOrientation];
  const Point &edge_p2 = polygon_vertices[(edgeChoiceForOrientation + 1) % num_vertices];
  Point primary_dir = (edge_p2 - edge_p1); // Vector hướng chính chưa chuẩn hóa
  if (primary_dir.length() < 1e-9)
  {
    Serial.println(F("GP: Loi - Canh dinh huong co do dai bang 0."));
    return;
  }

  Point primary_dir_normalized = primary_dir.normalized();              // Hướng chính chuẩn hóa
  Point offset_dir_normalized = primary_dir_normalized.perpendicular(); // Hướng offset (vuông góc) chuẩn hóa

  Serial.printf("GP: Primary dir: (%.2f, %.2f)\n", primary_dir_normalized.x, primary_dir_normalized.y);
  Serial.printf("GP: Offset dir: (%.2f, %.2f)\n", offset_dir_normalized.x, offset_dir_normalized.y);

  // 3. Xác định phạm vi quét (chiếu các đỉnh lên hướng offset)
  double min_proj = std::numeric_limits<double>::max();
  double max_proj = std::numeric_limits<double>::lowest();
  Point origin_proj = polygon_vertices[0]; // Chọn một đỉnh làm gốc chiếu

  for (const auto &vertex : polygon_vertices)
  {
    double proj = (vertex - origin_proj).dot(offset_dir_normalized); // Chiếu lên hướng offset chuẩn hóa
    min_proj = std::min(min_proj, proj);
    max_proj = std::max(max_proj, proj);
  }
  Serial.printf("GP: Projection range on offset dir: [%.2f, %.2f]\n", min_proj, max_proj);

  // 4. Tạo các đường quét song song và cắt chúng với đa giác
  std::vector<Segment> all_path_segments;
  double current_offset_dist = min_proj + workingWidth / 2.0; // Khoảng cách offset đầu tiên

  while (current_offset_dist <= max_proj)
  {
    // Tính điểm gốc trên đường quét hiện tại
    Point line_origin = origin_proj + offset_dir_normalized * current_offset_dist;

    // *** Gọi hàm cắt đường thẳng bởi đa giác ***
    // *** LƯU Ý: HÀM NÀY CẦN ĐƯỢC TRIỂN KHAI ĐẦY ĐỦ ***
    std::vector<Segment> segments_on_line = clipLineWithPolygon(line_origin, primary_dir_normalized, polygon_vertices);

    // Thêm các đoạn tìm được (nếu có) vào danh sách tổng
    all_path_segments.insert(all_path_segments.end(), segments_on_line.begin(), segments_on_line.end());

    // Tăng khoảng cách offset cho lần lặp tiếp theo
    current_offset_dist += workingWidth;
  }
  Serial.printf("GP: Found %d raw path segments after clipping.\n", all_path_segments.size());

  if (all_path_segments.empty())
  {
    Serial.println(F("GP: Khong tao duoc doan duong di nao sau khi clip. Kiem tra ham clipLineWithPolygon."));
    return; // Không có đường đi nào được tạo
  }

  // 5. Sắp xếp các đoạn đường đi theo thứ tự quét (dọc theo hướng offset)
  std::sort(all_path_segments.begin(), all_path_segments.end(), [&](const Segment &a, const Segment &b)
            {
        // Sử dụng trung điểm của mỗi đoạn để xác định thứ tự
        Point center_a = (a.p1 + a.p2) * 0.5;
        Point center_b = (b.p1 + b.p2) * 0.5;
        // Chiếu trung điểm lên hướng offset để sắp xếp
        return (center_a - origin_proj).dot(offset_dir_normalized) < (center_b - origin_proj).dot(offset_dir_normalized); });
  Serial.println(F("GP: Sorted path segments."));

  // 6. Nối các đoạn thành đường đi ziczac (Boustrophedon)
  std::vector<Point> final_path_points;
  Point last_point; // Điểm cuối cùng của đoạn trước đó

  if (!all_path_segments.empty())
  {
    // Xử lý đoạn đầu tiên: quyết định bắt đầu từ p1 hay p2
    // (Có thể thêm logic phức tạp hơn dựa trên startVertexIndex ở đây)
    Segment first_seg = all_path_segments[0];
    // Tạm thời luôn bắt đầu theo thứ tự p1->p2 cho đoạn đầu
    final_path_points.push_back(first_seg.p1);
    final_path_points.push_back(first_seg.p2);
    last_point = first_seg.p2; // Lưu điểm cuối
  }

  // Nối các đoạn còn lại
  for (size_t i = 1; i < all_path_segments.size(); ++i)
  {
    Segment current_seg = all_path_segments[i];
    Point start_pt, end_pt;

    // Chọn điểm bắt đầu của đoạn hiện tại (start_pt) sao cho gần điểm kết thúc của đoạn trước (last_point) nhất
    if ((current_seg.p1 - last_point).length() < (current_seg.p2 - last_point).length())
    {
      start_pt = current_seg.p1;
      end_pt = current_seg.p2;
    }
    else
    {
      start_pt = current_seg.p2;
      end_pt = current_seg.p1;
    }

    // Thêm điểm bắt đầu (nó cũng là điểm kết nối/rẽ) và điểm kết thúc của đoạn hiện tại vào đường đi cuối cùng
    final_path_points.push_back(start_pt);
    final_path_points.push_back(end_pt);
    last_point = end_pt; // Cập nhật điểm cuối cùng cho lần lặp tiếp theo
  }
  Serial.printf("GP: Generated final path with %d points.\n", final_path_points.size());

  // 7. Điều chỉnh điểm bắt đầu dựa trên startVertexIndex
  if (!final_path_points.empty() && startVertexIndex >= 0 && startVertexIndex < polygon_vertices.size())
  {
    Point target_start_vertex = polygon_vertices[startVertexIndex];
    double dist_to_path_start = (final_path_points.front() - target_start_vertex).length();
    double dist_to_path_end = (final_path_points.back() - target_start_vertex).length();

    // Nếu điểm cuối của đường đi gần đỉnh yêu cầu hơn điểm đầu, đảo ngược toàn bộ đường đi
    if (dist_to_path_end < dist_to_path_start)
    {
      std::reverse(final_path_points.begin(), final_path_points.end());
      Serial.printf("GP: Path reversed to start closer to target vertex %d.\n", startVertexIndex);
    }
  }
  // Thêm logic cho StartDir nếu cần:
  // if (StartDir == 0 && !final_path_points.empty()) { // Ví dụ: StartDir 0 muốn đi từ trên xuống
  //     Point first_p = final_path_points[0];
  //     Point second_p = final_path_points.size() > 1 ? final_path_points[1] : first_p;
  //     if (second_p.y > first_p.y) { // Nếu đang đi lên, đảo ngược
  //          std::reverse(final_path_points.begin(), final_path_points.end());
  //          Serial.println(F("GP: Path reversed based on StartDir=0."));
  //     }
  // } // Tương tự cho StartDir = 1

  // 8. Vẽ đường đi cuối cùng lên HMI
  if (final_path_points.size() >= 2)
  {
    Serial.println(F("GP: Drawing final path..."));
    for (size_t i = 0; i < final_path_points.size() - 1; ++i)
    {
      hmi_display.drawLine(final_path_points[i], final_path_points[i + 1], YELLOW);
    }
    // Tùy chọn: Đánh dấu điểm bắt đầu và kết thúc
    hmi_display.drawPointMarker(final_path_points.front(), GREEN);
    hmi_display.drawPointMarker(final_path_points.back(), RED);
  }
  else
  {
    Serial.println(F("GP: Loi - Duong di cuoi cung khong du diem de ve."));
  }
  Serial.println(F("GP: Path generation and drawing function finished."));
}