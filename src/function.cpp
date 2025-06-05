#include "function.h"
#include <cmath>
#include <algorithm>
#include <iostream>
#include "Point.h"
#include "globals.h"

double degreesToRadians(double degrees)
{
  return degrees * M_PI / 180.0;
}

void resetAndClear()
{
  field_vertices_gps.clear();
  field_vertices_screen.clear();
  //hmi.clearScreen(SCREEN_BACKGROUND_COLOR); // Xóa màn hình HMI với màu trắng
  currentState = WAITING_FOR_POINTS;
  StartPoint = 0; // Reset các giá trị về mặc định
  StartDir = 0;
  has_valid_previous_tractor_pos = false;
  Serial.println(F("---------------------------------------------"));
  Serial.println(F("Da xoa. Nhap lai toa do GPS (KinhDo,ViDo). Vi du: 106.6602,10.7769"));
  Serial.println(F("Nhap it nhat 3 diem."));
  Serial.println(F("Go 'done' khi hoan tat."));
  Serial.println(F("Go 'change start {index}' de doi diem bat dau (vi du: change start 2)."));
  Serial.println(F("Go 'change edge {index}' de doi canh dinh huong (vi du: change edge 1)."));
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
      // field_vertices_gps.push_back({120.26567222968173, 22.62686995853145});
      // field_vertices_gps.push_back({120.26572065449193, 22.626871898407174});
      // field_vertices_gps.push_back({120.26572388062435, 22.626916585260386});
      // field_vertices_gps.push_back({120.2659776718851, 22.62691692916119});
      // field_vertices_gps.push_back({120.26597749948263, 22.626875018365492});
      // field_vertices_gps.push_back({120.26602567463398, 22.626870349228255});

      // field_vertices_gps.push_back({120.26602428567377, 22.626499333091026});
      // field_vertices_gps.push_back({120.26566899973842, 22.62647562599012});

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
      hmi.visPic(9, 0); // Ẩn ảnh máy cày
      // resetAndClear();
    }
    else if (input.equalsIgnoreCase("changestart"))
    {
      int newStart = StartPoint++; // Lấy số sau "change start "
      if (newStart >= 0 && newStart < field_vertices_gps.size())
      {
        StartPoint = newStart;
        Serial.printf("Doi diem bat dau gan nhat thanh dinh %d\n", StartPoint);
        if (currentState == PATH_DISPLAYED)
        {
          currentState = POINTS_ENTERED_READY_TO_DRAW;
        } // Vẽ lại nếu đang hiển thị
      }
      else
      {
        Serial.println(F("Loi: Chi so diem bat dau khong hop le."));
      }
    }
    else if (input.equalsIgnoreCase("ChangeDir"))
    { // Bỏ comment nếu muốn dùng StartDir
      StartDir = (StartDir + 1) % 2;
      Serial.printf("StartDir: %d\n", StartDir);
      if (currentState == PATH_DISPLAYED)
      {
        currentState = POINTS_ENTERED_READY_TO_DRAW;
      }
    }
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
  char recei = hmi.receiveCommand(); // Gọi hàm nhận lệnh
  if (recei == 0x12) // Nếu có lệnh mới nhận được
  {
    Serial.printf("snum %d, longitude: %f, Latitude: %f\n",g_svnum, g_longitude, g_latitude);
    field_vertices_gps.push_back({g_longitude, g_latitude});
    digitalWrite(33, HIGH); // Bật LED tích hợp
    delay(200);             // Giữ LED sáng trong 1 giây
    digitalWrite(33, LOW);  // Tắt LED tích hợp
  }
  if (recei == 0x14)
  {
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
    digitalWrite(33, HIGH); // Bật LED tích hợp
    delay(100);             // Giữ LED sáng trong 1 giây
    digitalWrite(33, LOW);
    delay(100); // Giữ LED sáng trong 1 giây
    digitalWrite(33, HIGH); // Bật LED tích hợp
    delay(100);
    digitalWrite(33, LOW);
    delay(100);             // Giữ LED sáng trong 1 giây
    digitalWrite(33, HIGH); // Bật LED tích hợp
    delay(100);
    digitalWrite(33, LOW);  // Tắt LED tích hợp
    StartPoint = field_vertices_gps.size()-1;
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

  // hmi.clearScreen(SCREEN_BACKGROUND_COLOR); // Xóa màn hình trước khi vẽ mới

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
    
    Serial.print(F("Dang tao duong di voi working_width_px = "));
    Serial.print(working_width_px);
    Serial.print(F(", startVertexIndex = "));
    Serial.print(StartPoint);
    Serial.print(F(", StartDir = "));
    Serial.println(StartDir);

    // Gọi hàm generate Path với các tham số đã tính toán
    // Chuyển working_width_px sang double khi gọi
    generatePath(field_vertices_screen, static_cast<double>(working_width_px), StartPoint, StartDir, hmi);

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
  const double INTERSECTION_EPSILON = 1e-9;

  if (std::fabs(det) < INTERSECTION_EPSILON)
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

bool isInside(const Point &p, const std::vector<Point> &polygon)
{
  int n = polygon.size();
  if (n < 3)
    return false;
  bool inside = false;
  double p_x = p.x;
  double p_y = p.y;

  for (int i = 0, j = n - 1; i < n; j = i++)
  {
    double v_i_x = polygon[i].x;
    double v_i_y = polygon[i].y;
    double v_j_x = polygon[j].x;
    double v_j_y = polygon[j].y;

    // Kiểm tra nếu điểm p trùng với một đỉnh của đa giác
    if ((std::fabs(p_x - v_i_x) < GEOMETRY_EPSILON && std::fabs(p_y - v_i_y) < GEOMETRY_EPSILON))
    {
      return true; // Coi như nằm trong nếu trùng đỉnh
    }

    // Kiểm tra nếu điểm p nằm trên một cạnh ngang của đa giác
    if (std::fabs(v_i_y - v_j_y) < GEOMETRY_EPSILON && // Cạnh là ngang
        std::fabs(p_y - v_i_y) < GEOMETRY_EPSILON &&   // Điểm p cùng độ cao với cạnh ngang
        (p_x > std::min(v_i_x, v_j_x) - GEOMETRY_EPSILON && p_x < std::max(v_i_x, v_j_x) + GEOMETRY_EPSILON))
    {              // p.x nằm giữa hai đầu mút cạnh
      return true; // Coi như nằm trong nếu trên cạnh ngang
    }

    // Kiểm tra nếu điểm p nằm trên một cạnh dọc của đa giác
    if (std::fabs(v_i_x - v_j_x) < GEOMETRY_EPSILON && // Cạnh là dọc
        std::fabs(p_x - v_i_x) < GEOMETRY_EPSILON &&   // Điểm p cùng hoành độ với cạnh dọc
        (p_y > std::min(v_i_y, v_j_y) - GEOMETRY_EPSILON && p_y < std::max(v_i_y, v_j_y) + GEOMETRY_EPSILON))
    {              // p.y nằm giữa hai đầu mút cạnh
      return true; // Coi như nằm trong nếu trên cạnh dọc
    }

    // Áp dụng thuật toán Ray Casting chuẩn
    if (((v_i_y <= p_y && p_y < v_j_y) || (v_j_y <= p_y && p_y < v_i_y)) && // p_y nằm giữa [v_i_y, v_j_y) hoặc [v_j_y, v_i_y)
        (p_x < (v_j_x - v_i_x) * (p_y - v_i_y) / (v_j_y - v_i_y + ((v_j_y == v_i_y) ? GEOMETRY_EPSILON : 0.0)) + v_i_x))
    { // Thêm epsilon nhỏ để tránh chia cho 0 nếu cạnh ngang (mặc dù đã check ở trên)
      inside = !inside;
    }
  }
  return inside;
}

std::vector<Segment> clipLineWithPolygon(const Point &line_origin,
                                         const Point &line_dir_normalized,
                                         const std::vector<Point> &polygon_vertices,
                                         double current_working_width)
{
  std::vector<Segment> clipped_and_shrunk_segments;
  std::vector<Point> intersections;
  size_t n_poly = polygon_vertices.size();

  // 1. Kiểm tra đầu vào cơ bản
  if (n_poly < 3)
  {
    // Serial.println("ClipLP: Loi - Da giac khong du dinh."); // Gỡ comment nếu cần debug
    return clipped_and_shrunk_segments;
  }
  if (line_dir_normalized.length() < GEOMETRY_EPSILON)
  {
    // Serial.println("ClipLP: Loi - Vector huong duong thang khong hop le."); // Gỡ comment nếu cần debug
    return clipped_and_shrunk_segments;
  }

  // 2. Tìm tất cả các giao điểm của đường thẳng vô hạn với các cạnh của đa giác
  for (size_t i = 0; i < n_poly; ++i)
  {
    const Point &poly_p1 = polygon_vertices[i];
    const Point &poly_p2 = polygon_vertices[(i + 1) % n_poly]; // Cạnh cuối nối về đỉnh đầu
    Point current_intersection;

    if (intersectLineSegment(line_origin, line_dir_normalized, poly_p1, poly_p2, current_intersection))
    {
      intersections.push_back(current_intersection);
    }
  }

  // Nếu không có đủ 2 giao điểm, không thể tạo đoạn thẳng
  if (intersections.size() < 2)
  {
    return clipped_and_shrunk_segments;
  }

  // 3. Sắp xếp các điểm giao cắt dọc theo hướng của đường thẳng
  std::sort(intersections.begin(), intersections.end(), [&](const Point &a, const Point &b)
            {
double proj_a = (a - line_origin).dot(line_dir_normalized);
double proj_b = (b - line_origin).dot(line_dir_normalized);
return proj_a < proj_b; });

  // 4. Loại bỏ các điểm giao cắt trùng lặp (do đi qua đỉnh hoặc lỗi số thực)
  auto points_are_close = [](const Point &a, const Point &b)
  {
    return (a - b).length() < GEOMETRY_EPSILON * 10; // Dung sai lớn hơn một chút cho unique
  };
  intersections.erase(std::unique(intersections.begin(), intersections.end(), points_are_close), intersections.end());

  // Cảnh báo nếu số lượng giao điểm duy nhất là lẻ (trừ trường hợp chỉ có 1 điểm - tiếp tuyến)
  if (intersections.size() % 2 != 0 && intersections.size() > 1)
  {
    // Serial.printf("ClipLP: Canh bao - So luong giao diem duy nhat (%d) la so le! Giao diem cuoi se duoc bo qua khi tao cap.\n", intersections.size());
  }

  // 5. Lặp qua các cặp giao điểm đã sắp xếp, kiểm tra trung điểm và co ngắn đoạn
  double half_tool_width = current_working_width / 2.0;

  for (size_t i = 0; i + 1 < intersections.size(); i += 2)
  {
    const Point &p_boundary_start = intersections[i];   // Điểm đầu của đoạn cắt thô trên biên
    const Point &p_boundary_end = intersections[i + 1]; // Điểm cuối của đoạn cắt thô trên biên

    double raw_segment_length = (p_boundary_end - p_boundary_start).length();

    // Bỏ qua nếu đoạn thô ban đầu quá ngắn (gần như là một điểm)
    if (raw_segment_length < GEOMETRY_EPSILON)
    {
      continue;
    }

    // Kiểm tra trung điểm của đoạn thô có nằm trong đa giác không
    Point mid_point_raw = (p_boundary_start + p_boundary_end) * 0.5;
    if (!isInside(mid_point_raw, polygon_vertices))
    {
      // Serial.printf("ClipLP: Trung diem (%.1f,%.1f) cua doan tho nam NGOAI da giac, bo qua.\n", mid_point_raw.x, mid_point_raw.y);
      continue; // Trung điểm không nằm trong, đoạn này không hợp lệ
    }

    // Tiến hành co ngắn đoạn thẳng
    // Chỉ co ngắn và thêm vào nếu đoạn thô đủ dài ít nhất bằng chiều rộng nông cụ
    // để sau khi co lại từ cả hai phía, nó vẫn còn là một đoạn có chiều dài dương.
    if (raw_segment_length >= current_working_width - GEOMETRY_EPSILON)
    {
      Point dir_raw_segment = (p_boundary_end - p_boundary_start).normalized();

      Point shrunk_p1 = p_boundary_start + dir_raw_segment * half_tool_width;
      Point shrunk_p2 = p_boundary_end - dir_raw_segment * half_tool_width;

      // Kiểm tra lại chiều dài của đoạn đã co ngắn
      // Dùng dot product để đảm bảo shrunk_p2 không "vượt qua" shrunk_p1
      if ((shrunk_p2 - shrunk_p1).dot(dir_raw_segment) >= -GEOMETRY_EPSILON)
      {
        // Yêu cầu chiều dài tối thiểu cho một đường path thực tế (ví dụ > 0.5 pixel)
        // Điều này quan trọng hơn việc dot product chỉ >= 0, vì một điểm cũng có dot product = 0.
        if ((shrunk_p2 - shrunk_p1).length() > 0.5)
        { // Có thể điều chỉnh ngưỡng này
          clipped_and_shrunk_segments.push_back({shrunk_p1, shrunk_p2});
        }
        else
        {
          // Serial.printf("ClipLP: Doan (%.1f,%.1f)-(%.1f,%.1f) sau khi co ngan thanh diem hoac qua ngan (%.2fpx), bo qua.\n",
          //               p_boundary_start.x, p_boundary_start.y, p_boundary_end.x, p_boundary_end.y, (shrunk_p2 - shrunk_p1).length());
        }
      }
      else
      {
        // Serial.printf("ClipLP: Doan (%.1f,%.1f)-(%.1f,%.1f) sau khi co ngan bi am chieu dai, bo qua.\n",
        //               p_boundary_start.x, p_boundary_start.y, p_boundary_end.x, p_boundary_end.y);
      }
    }
    else
    {
      // Đoạn thô có chiều dài nhỏ hơn workingWidth.
      // Nông cụ không thể đi hết chiều dài này mà vẫn giữ khoảng cách half_tool_width với cả hai đầu biên.
      // Serial.printf("ClipLP: Doan tho (%.1f,%.1f)-(%.1f,%.1f) dai %.2fpx < workingWidth %.2fpx, khong du de co ngan, bo qua.\n",
      //               p_boundary_start.x, p_boundary_start.y, p_boundary_end.x, p_boundary_end.y, raw_segment_length, current_working_width);
    }
  }

  return clipped_and_shrunk_segments;
}
std::vector<Point> g_final_path_points;
std::vector<Point> g_polygon_vertices;

void generatePath(const std::vector<Point> &polygon_vertices,
                                        double workingWidth,
                                        int startVertexIndex, // StartPoint từ biến toàn cục
                                        int startDirGlobal,   // StartDir từ biến toàn cục
                                        HMI_Display &hmi_display)
{
  g_polygon_vertices = polygon_vertices; // Lưu đa giác vào biến toàn cục để vẽ sau
  // 1. Kiểm tra đầu vào cơ bản và xác định actualEdgeChoiceForOrientation
  // ... (Logic này giữ nguyên như phiên bản trước, xác định actualEdgeChoiceForOrientation
  //      dựa trên startVertexIndex và startDirGlobal) ...
  size_t num_vertices = polygon_vertices.size();
  if (num_vertices < 3)
  { /* ... */
    return;
  }
  if (workingWidth <= 0)
  { /* ... */
    return;
  }
  if (startVertexIndex < 0 || startVertexIndex >= num_vertices)
  {
    startVertexIndex = 0; /* ... */
  }

  int actualEdgeChoiceForOrientation = 0;
  if (startDirGlobal == 0)
  {
    actualEdgeChoiceForOrientation = (startVertexIndex - 1 + num_vertices) % num_vertices;
    Serial.printf("GP: StartDir=0, chon canh TRUOC dinh %d (canh noi dinh %d -> %d) lam chuan.\n",
                  startVertexIndex, actualEdgeChoiceForOrientation, startVertexIndex);
  }
  else
  {
    actualEdgeChoiceForOrientation = startVertexIndex;
    Serial.printf("GP: StartDir=1, chon canh SAU dinh %d (canh noi dinh %d -> %d) lam chuan.\n",
                  startVertexIndex, actualEdgeChoiceForOrientation, (actualEdgeChoiceForOrientation + 1) % num_vertices);
  }

  // Vẽ đa giác gốc
  if (polygon_vertices.size() >= 3)
  {
    for (size_t i = 0; i < polygon_vertices.size(); ++i)
    {
      hmi_display.drawLine(polygon_vertices[i], polygon_vertices[(i + 1) % polygon_vertices.size()], BLUE);
    }
  }

  // 2. Tính toán hướng quét (primary_dir) và hướng offset ban đầu (offset_dir)
  const Point &edge_ref_p1 = polygon_vertices[actualEdgeChoiceForOrientation];                      // Điểm đầu của cạnh chuẩn
  const Point &edge_ref_p2 = polygon_vertices[(actualEdgeChoiceForOrientation + 1) % num_vertices]; // Điểm cuối
  Point primary_dir = (edge_ref_p2 - edge_ref_p1);
  if (primary_dir.length() < GEOMETRY_EPSILON)
  { /* ... */
    return;
  }

  Point primary_dir_normalized = primary_dir.normalized();
  Point initial_offset_dir_normalized = primary_dir_normalized.perpendicular(); // Hướng vuông góc ban đầu

  // --- THAY ĐỔI BƯỚC 3 & 4: Xác định hướng offset "vào trong" và bắt đầu quét từ cạnh chuẩn ---

  Point edge_midpoint = (edge_ref_p1 + edge_ref_p2) * 0.5; // Trung điểm cạnh chuẩn
  Point inward_offset_dir = initial_offset_dir_normalized; // Giả sử hướng này là vào trong

  // Kiểm tra xem initial_offset_dir_normalized có thực sự hướng vào trong đa giác từ cạnh chuẩn không
  // Tạo một điểm thử nghiệm nhỏ bên trong từ trung điểm cạnh theo hướng offset
  Point test_point_inward = edge_midpoint + inward_offset_dir * (workingWidth * 0.1); // Bước nhỏ vào trong
  if (!isInside(test_point_inward, polygon_vertices))
  {
    // Nếu không phải, đảo ngược hướng offset
    inward_offset_dir = inward_offset_dir * -1.0;
    test_point_inward = edge_midpoint + inward_offset_dir * (workingWidth * 0.1); // Kiểm tra lại
    if (!isInside(test_point_inward, polygon_vertices))
    {
      // Vẫn không vào trong, có thể đa giác rất hẹp hoặc isInside có vấn đề ở biên
      // Hoặc cạnh được chọn là một phần của vùng lõm "hướng ra ngoài"
      Serial.println(F("GP: Canh bao - Khong xac dinh duoc huong offset vao trong tu canh chuan. Su dung mac dinh."));
      // Trong trường hợp này, `initial_offset_dir_normalized` có thể không đúng,
      // nhưng chúng ta vẫn tiếp tục với inward_offset_dir (có thể đã đảo hoặc chưa)
    }
    else
    {
      Serial.println(F("GP: Huong offset da dao nguoc de di vao trong."));
    }
  }
  else
  {
    Serial.println(F("GP: Huong offset ban dau duoc xac nhan la di vao trong."));
  }

  // 4. Tạo các đường quét song song bắt đầu từ cạnh chuẩn và tiến vào trong
  std::vector<Segment> all_path_segments;
  double accumulated_offset_distance = workingWidth / 2.0; // Khoảng cách offset đầu tiên từ cạnh chuẩn
  bool segments_found_on_last_line = true;
  int max_sweeps = (int)((std::max(SCREEN_WIDTH_PX, SCREEN_HEIGHT_PX) * 1.5) / workingWidth) + 10; // Giới hạn số lượt quét an toàn
  int sweep_count = 0;

  Serial.println(F("GP: Bat dau tao duong quet tu canh chuan."));
  while (segments_found_on_last_line && sweep_count < max_sweeps)
  {
    // Điểm gốc của đường quét hiện tại được dịch chuyển từ trung điểm cạnh chuẩn
    // dọc theo hướng inward_offset_dir
    Point current_line_origin = edge_midpoint + inward_offset_dir * accumulated_offset_distance;

    std::vector<Segment> segments_on_line = clipLineWithPolygon(current_line_origin, primary_dir_normalized, polygon_vertices,
                                                                workingWidth); // Cắt đường thẳng với đa giác

    if (!segments_on_line.empty())
    {
      all_path_segments.insert(all_path_segments.end(), segments_on_line.begin(), segments_on_line.end());
      segments_found_on_last_line = true; // Tiếp tục nếu vẫn tìm thấy
    }
    else
    {
      // Nếu không tìm thấy đoạn nào trên đường quét này:
      // - Nếu trước đó đã có đoạn thì có thể đã quét hết đa giác.
      // - Nếu chưa có đoạn nào mà đã quét vài lần thì có thể có vấn đề.
      if (!all_path_segments.empty() && sweep_count > 0)
      {                                      // Đã có đoạn và đã quét ít nhất 1 lần hợp lệ
        segments_found_on_last_line = false; // Dừng lại
        Serial.println(F("GP: Khong tim thay doan nao tren duong quet hien tai, ket thuc quet."));
      }
      else if (all_path_segments.empty() && sweep_count > 5)
      { // Quét nhiều lần mà vẫn rỗng
        segments_found_on_last_line = false;
        Serial.println(F("GP: Khong tim thay doan nao sau nhieu luot quet, ket thuc quet."));
      }
      // Nếu all_path_segments vẫn rỗng và sweep_count còn nhỏ, vẫn cho thử tiếp
    }

    accumulated_offset_distance += workingWidth;
    sweep_count++;
  }

  Serial.printf("GP: Hoan thanh tao duong quet. Luot quet: %d. Tong so doan tho TRUOC UNIQUE: %d\n", sweep_count, all_path_segments.size());

  // --- BƯỚC MỚI: Loại bỏ các đoạn (Segment) giống hệt nhau ---
  if (!all_path_segments.empty())
  {
    // Sắp xếp tạm thời để std::unique hoạt động hiệu quả với comparator
    // Sắp xếp theo tọa độ điểm đầu, rồi điểm cuối để các đoạn giống nhau nằm cạnh nhau
    std::sort(all_path_segments.begin(), all_path_segments.end(), [](const Segment &a, const Segment &b)
              {
if (std::fabs(a.p1.x - b.p1.x) > GEOMETRY_EPSILON) return a.p1.x < b.p1.x;
if (std::fabs(a.p1.y - b.p1.y) > GEOMETRY_EPSILON) return a.p1.y < b.p1.y;
if (std::fabs(a.p2.x - b.p2.x) > GEOMETRY_EPSILON) return a.p2.x < b.p2.x;
return a.p2.y < b.p2.y; });

    // Định nghĩa hàm so sánh hai Segment (coi là bằng nhau nếu các điểm đầu/cuối tương ứng gần nhau)
    // Hoặc nếu chúng là cùng một đoạn nhưng hướng ngược lại (p1 của a = p2 của b VÀ p2 của a = p1 của b)
    auto segments_are_equivalent = [](const Segment &a, const Segment &b)
    {
      bool same_direction = (a.p1 - b.p1).length() < GEOMETRY_EPSILON * 5 &&
                            (a.p2 - b.p2).length() < GEOMETRY_EPSILON * 5;
      bool reverse_direction = (a.p1 - b.p2).length() < GEOMETRY_EPSILON * 5 &&
                               (a.p2 - b.p1).length() < GEOMETRY_EPSILON * 5;
      return same_direction || reverse_direction;
    };
    all_path_segments.erase(std::unique(all_path_segments.begin(), all_path_segments.end(), segments_are_equivalent), all_path_segments.end());
    Serial.printf("GP: Tong so doan tho SAU UNIQUE: %d\n", all_path_segments.size());
  }

  // DEBUG: In ra tất cả các đoạn trong all_path_segments TRƯỚC KHI SẮP XẾP
  if (all_path_segments.empty())
  {
    Serial.println(F("GP_DEBUG: all_path_segments Rỗng TRUOC KHI SAP XEP."));
  }
  else
  {
    Serial.println(F("GP_DEBUG: Cac doan trong all_path_segments TRUOC KHI SAP XEP:"));
    for (size_t k = 0; k < all_path_segments.size(); ++k)
    {
      const auto &seg = all_path_segments[k];
      Serial.printf("  Seg %d: (%.1f, %.1f) -> (%.1f, %.1f)\n",
                    k, seg.p1.x, seg.p1.y, seg.p2.x, seg.p2.y);
    }
  }

  // 5. Sắp xếp các đoạn đường đi theo thứ tự quét
  // Gốc chiếu để sắp xếp nên là một điểm trên cạnh chuẩn, ví dụ edge_ref_p1
  const Point &edge_ref_p1_for_sort = polygon_vertices[actualEdgeChoiceForOrientation]; // Đổi tên biến để rõ ràng
  std::sort(all_path_segments.begin(), all_path_segments.end(), [&](const Segment &a, const Segment &b)
            {
Point center_a = (a.p1 + a.p2) * 0.5;
Point center_b = (b.p1 + b.p2) * 0.5;
// Chiếu lên inward_offset_dir so với điểm đầu của cạnh chuẩn
return (center_a - edge_ref_p1_for_sort).dot(inward_offset_dir) < (center_b - edge_ref_p1_for_sort).dot(inward_offset_dir); });

  // 6. Nối các đoạn thành đường đi ziczac (Boustrophedon)
  std::vector<Point> final_path_points;
  Point last_point;

  if (!all_path_segments.empty())
  {
    Segment first_seg = all_path_segments[0]; // Bây giờ first_seg là duy nhất
    Point p_start_init = first_seg.p1;
    Point p_end_init = first_seg.p2;

    // Quyết định hướng đi ban đầu của first_seg dựa trên startVertexIndex
    if (startVertexIndex >= 0 && startVertexIndex < num_vertices)
    {
      Point target_v = polygon_vertices[startVertexIndex];
      if ((first_seg.p2 - target_v).length() < (first_seg.p1 - target_v).length())
      {
        p_start_init = first_seg.p2; // p2 gần target_v hơn, bắt đầu từ p2
        p_end_init = first_seg.p1;
        Serial.println(F("GP: Dao huong doan quet dau tien de gan hon dinh bat dau."));
      }
    }
    final_path_points.push_back(p_start_init);
    final_path_points.push_back(p_end_init);
    last_point = p_end_init;
  }
  // Phần còn lại của vòng lặp nối ziczac giữ nguyên
  for (size_t i = 1; i < all_path_segments.size(); ++i)
  {
    // ... (logic nối các đoạn còn lại như cũ)
    Segment current_seg = all_path_segments[i];
    Point start_pt, end_pt;
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
    final_path_points.push_back(start_pt);
    final_path_points.push_back(end_pt);
    last_point = end_pt;
  }
  Serial.printf("GP: Generated final path with %d points.\n", final_path_points.size());

  // 7. Điều chỉnh điểm bắt đầu dựa trên startVertexIndex (đảo ngược toàn bộ đường nếu cần)
  // ... (Logic này giữ nguyên, startVertexIndex ở đây là StartPoint của bạn) ...
  if (!final_path_points.empty() && startVertexIndex >= 0 && startVertexIndex < num_vertices)
  {
    Point target_start_vertex = polygon_vertices[startVertexIndex];
    double dist_to_path_start = (final_path_points.front() - target_start_vertex).length();
    double dist_to_path_end = (final_path_points.back() - target_start_vertex).length();
    if (dist_to_path_end < dist_to_path_start)
    {
      std::reverse(final_path_points.begin(), final_path_points.end());
      Serial.printf("GP: Path reversed to start closer to target vertex %d.\n", startVertexIndex);
    }
  }

  // 8. Vẽ đường đi cuối cùng lên HMI
  // ... (Logic này giữ nguyên) ...
  if (final_path_points.size() >= 2)
  {
    Serial.println(F("GP: Drawing final path..."));
    hmi_display.drawPointMarker(final_path_points.front(), GREEN); // Vẽ điểm đầu tiên
    //hmi_display.drawPic(final_path_points.front().x - TRACTOR_PIC_WIDTH / 2, final_path_points.front().y - TRACTOR_PIC_HEIGHT / 2, TRACTOR_PIC_ID_DEFAULT);
    hmi.visPic(25,1);
    String command = "p1.x=" + String((int)(final_path_points.front().x - TRACTOR_PIC_FIXED_WIDTH / 2));
    hmi.sendCommand(command);
    command = "p1.y=" + String((int)(final_path_points.front().y - TRACTOR_PIC_FIXED_HEIGHT / 2));
    hmi.sendCommand(command);
    g_final_path_points = final_path_points;
    for (size_t i = 0; i < final_path_points.size() - 1; ++i)
    {
      hmi_display.drawLine(final_path_points[i], final_path_points[i + 1], YELLOW);
    }
    hmi_display.drawPointMarker(final_path_points.back(), RED); // Vẽ điểm cuối cùng
  }
  else
  {
    Serial.println(F("GP: Loi - Duong di cuoi cung khong du diem de ve."));
  }
  Serial.println(F("GP: Path generation and drawing function finished."));
}

void updateAndDrawTractorPositionHMI()
{
  // // Bước 1: Kiểm tra điều kiện để thực thi
  // if (!new_tractor_gps_data_received)
  // {
  //   return;
  // }
  // new_tractor_gps_data_received = false;

  // if (scale_factor_combined == 0 && field_vertices_gps.size() < 3)
  // {
  //   return;
  // }

  // // Bước 2: Chuyển đổi tọa độ GPS của máy cày sang tọa độ màn hình
  // Point new_tractor_center_screen = transformGpsToScreen(current_tractor_gps_actual);

  // // Bước 3: Xác định ID ảnh máy cày dựa trên g_yaw (0-359 độ)
  // int pic_id_to_draw_now;
  // double current_yaw = g_yaw;

  // // Chuẩn hóa yaw về khoảng [0.0, 360.0)
  // if (current_yaw >= 360.0)
  //   current_yaw = fmod(current_yaw, 360.0);
  // if (current_yaw < 0.0)
  //   current_yaw = fmod(current_yaw, 360.0) + 360.0;

  // pic_id_to_draw_now = static_cast<int>(std::round(current_yaw));

  // // Đảm bảo ID nằm trong khoảng 0-359 sau khi làm tròn
  // if (pic_id_to_draw_now >= 360)
  //   pic_id_to_draw_now = 0;
  // if (pic_id_to_draw_now < 0)
  //   pic_id_to_draw_now = 0; // Phòng trường hợp làm tròn số âm nhỏ

  // current_tractor_display_pic_id = pic_id_to_draw_now + 15;

  // // Bước 5: Tính tọa độ góc trên-trái (top-left) để vẽ ảnh MỚI
  // int new_pic_draw_x = static_cast<int>(std::round(new_tractor_center_screen.x - (double)TRACTOR_PIC_FIXED_WIDTH / 2.0));
  // int new_pic_draw_y = static_cast<int>(std::round(new_tractor_center_screen.y - (double)TRACTOR_PIC_FIXED_HEIGHT / 2.0));

  // // Bước 7: Vẽ ảnh MỚI của máy cày lên màn hình
  // for (size_t i = 0; i < field_vertices_screen.size(); ++i)
  // {
  //   hmi.drawPointMarker(field_vertices_screen[i], RED);
  // }
  //   hmi.drawPointMarker(g_final_path_points.front(), GREEN);
  // for (size_t i = 0; i < g_polygon_vertices.size(); ++i)
  // {
  //   hmi.drawLine(g_polygon_vertices[i], g_polygon_vertices[(i + 1) % g_polygon_vertices.size()], BLUE);
  // }
  // for (size_t i = 0; i < g_final_path_points.size() - 1; ++i)
  // {
  //   hmi.drawLine(g_final_path_points[i], g_final_path_points[i + 1], YELLOW);
  // }
  // hmi.drawPointMarker(g_final_path_points.back(), RED);

  // hmi.changePic1(new_pic_draw_x, new_pic_draw_y, current_tractor_display_pic_id);

  // // Bước 8: Cập nhật thông tin cho lần gọi hàm tiếp theo
  // previous_tractor_screen_actual = new_tractor_center_screen;
  // previous_tractor_display_pic_id = current_tractor_display_pic_id;
  // has_valid_previous_tractor_pos = true;
  if (scale_factor_combined == 0.0f || !origin_set)
  {
    // Serial.println(F("UADT: Scale factor or EKF origin not set. Skipping tractor draw."));
    return;
  }

  GpsPoint ekf_pos_as_gps;
  ekf_pos_as_gps.longitude = lon_origin + (ekf_x / meters_per_deg_lon);
  ekf_pos_as_gps.latitude = lat_origin + (ekf_y / METERS_PER_DEGREE_LATITUDE);

  Point new_tractor_center_screen = transformGpsToScreen(ekf_pos_as_gps);

  // Bước 3: Xác định ID ảnh máy cày dựa trên góc yaw từ EKF (ekf.x[4] là radian)
  double current_yaw_rad = ekf_theta;               // Góc yaw từ EKF (radian), đã được chuẩn hóa trong [-PI, PI]
  double current_yaw_deg = degrees(current_yaw_rad); // Chuyển sang độ, sẽ trong [-180, 180]

  // Chuẩn hóa yaw về khoảng [0.0, 360.0)
  if (current_yaw_deg < 0.0)
  {
    current_yaw_deg += 360.0;
  }
  if (current_yaw_deg >= 360.0)
  { // Xử lý trường hợp gần 360 hoặc > 360 do làm tròn/tính toán
    current_yaw_deg = fmod(current_yaw_deg, 360.0);
  }

  int pic_id_base = static_cast<int>(std::round(current_yaw_deg));

  // Đảm bảo ID nằm trong khoảng 0-359 sau khi làm tròn
  if (pic_id_base >= 360)
    pic_id_base = 0;
  // if (pic_id_base < 0) pic_id_base = 0; // Không nên xảy ra nếu chuẩn hóa đúng

  current_tractor_display_pic_id = pic_id_base + 15; // Giả sử offset 15 là đúng cho ID ảnh HMI

  // Bước 4: Tính tọa độ góc trên-trái (top-left) để vẽ ảnh MỚI
  int new_pic_draw_x = static_cast<int>(std::round(new_tractor_center_screen.x - (double)TRACTOR_PIC_FIXED_WIDTH / 2.0));
  int new_pic_draw_y = static_cast<int>(std::round(new_tractor_center_screen.y - (double)TRACTOR_PIC_FIXED_HEIGHT / 2.0));

  // Bước 5: Vẽ ảnh MỚI của máy cày
  // Bước 5: Vẽ lại các yếu tố tĩnh của bản đồ (nếu cần thiết mỗi lần)
  // Để tối ưu, phần này nên được vẽ một lần khi bản đồ được tạo.
  if (!field_vertices_screen.empty())
  { // Các đỉnh gốc của thửa ruộng
    for (size_t i = 0; i < field_vertices_screen.size(); ++i)
    {
      hmi.drawPointMarker(field_vertices_screen[i], RED);
    }
  }
  if (!g_polygon_vertices.empty())
  { // Đường biên thửa ruộng (có thể giống field_vertices_screen)
    for (size_t i = 0; i < g_polygon_vertices.size(); ++i)
    {
      hmi.drawLine(g_polygon_vertices[i], g_polygon_vertices[(i + 1) % g_polygon_vertices.size()], BLUE);
    }
  }
  if (g_final_path_points.size() >= 2)
  {                                                                  // Đường đi zigzag
    hmi.drawPointMarker(g_final_path_points.front(), GREEN);         // Điểm bắt đầu đường đi
    for (size_t i = 0; i < g_final_path_points.size() - 1; ++i)
    {
      hmi.drawLine(g_final_path_points[i], g_final_path_points[i + 1], YELLOW);
    }
    hmi.drawPointMarker(g_final_path_points.back(), RED); // Điểm kết thúc đường đi
  }

  // Bước 6: Vẽ ảnh MỚI của máy cày lên màn hình bằng cách cập nhật đối tượng 'p1'
  hmi.changePic1(new_pic_draw_x, new_pic_draw_y, current_tractor_display_pic_id);
}

// Hàm này cần được gọi trong loop()
// Bạn cần tự hoàn thiện phần đọc và phân tích NMEA từ module GPS thực tế
void readAndProcessGpsData()
{
  // --- PHẦN ĐỌC GPS THỰC TẾ CẦN THAY THẾ VÀO ĐÂY ---
  // Ví dụ: đọc từ một Serial khác nối với GPS, dùng thư viện TinyGPS++, NMEAparser, v.v.
  // if (gpsSerial.available() > 0) {
  //    char c = gpsSerial.read();
  //    if (gpsParser.encode(c)) { // Giả sử gpsParser là đối tượng của thư viện bạn dùng
  //        if (gpsParser.location.isValid() && gpsParser.location.isUpdated()) {
  //            current_tractor_gps_actual.latitude = gpsParser.location.lat();
  //            current_tractor_gps_actual.longitude = gpsParser.location.lng();
  //            new_tractor_gps_data_received = true;
  //            Serial.print(F("GPS_REAL: Lat=")); Serial.print(current_tractor_gps_actual.latitude, 6);
  //            Serial.print(F(", Lon=")); Serial.println(current_tractor_gps_actual.longitude, 6);
  //        }
  //    }
  // }
  // --- KẾT THÚC PHẦN CẦN THAY THẾ ---
  // 22.626860129985378, 120.2656864793982
  // === BẮT ĐẦU PHẦN MÔ PHỎNG ĐỂ TEST (XÓA KHI CÓ GPS THỰC) ===
  static unsigned long lastGpsSimTime = 0;
  if (millis() - lastGpsSimTime > 100 && !field_vertices_gps.empty())
  { // Cập nhật mô phỏng mỗi 3 giây
    lastGpsSimTime = millis();
    // Di chuyển ngẫu nhiên một chút từ điểm đầu tiên của thửa ruộng để mô phỏng
    current_tractor_gps_actual.latitude = field_vertices_gps[0].latitude + (double)(rand() % 60 - 50) / 200000.0;   // Thay đổi nhỏ
    current_tractor_gps_actual.longitude = field_vertices_gps[0].longitude + (double)(rand() % 60 - 50) / 200000.0; // Thay đổi nhỏ
    // current_tractor_gps_actual.latitude = g_latitude;                                                        // Thay đổi nhỏ
    // current_tractor_gps_actual.longitude = g_longitude;                                                        // Thay đổi nhỏ
    new_tractor_gps_data_received = true;
    g_yaw++;
    if (g_yaw >= 360.0)
      g_yaw = 0.0; // Quay vòng lại sau 360 độ
    Serial.printf(("GPS: New tractor pos: %.6f, %.6f, yaw=%.2f\n"),
                  current_tractor_gps_actual.latitude, current_tractor_gps_actual.longitude, g_yaw);
  }
  // === KẾT THÚC PHẦN MÔ PHỎNG ===
}

// Hàm trợ giúp để lấy kích thước của ảnh máy cày dựa trên ID ảnh
void getTractorPicDimensions(int pic_id, int &width, int &height)
{
  if (pic_id == TRACTOR_PIC_ID_UP || pic_id == TRACTOR_PIC_ID_DOWN)
  {
    width = TRACTOR_PIC_VERTICAL_WIDTH;
    height = TRACTOR_PIC_VERTICAL_HEIGHT;
  }
  else if (pic_id == TRACTOR_PIC_ID_LEFT || pic_id == TRACTOR_PIC_ID_RIGHT)
  {
    width = TRACTOR_PIC_HORIZONTAL_WIDTH;
    height = TRACTOR_PIC_HORIZONTAL_HEIGHT;
  }
  else
  {
    // Mặc định nếu ID không xác định (có thể là TRACTOR_PIC_ID_DEFAULT)
    // Nên kiểm tra TRACTOR_PIC_ID_DEFAULT thuộc loại nào
    if (TRACTOR_PIC_ID_DEFAULT == TRACTOR_PIC_ID_UP || TRACTOR_PIC_ID_DEFAULT == TRACTOR_PIC_ID_DOWN)
    {
      width = TRACTOR_PIC_VERTICAL_WIDTH;
      height = TRACTOR_PIC_VERTICAL_HEIGHT;
    }
    else
    {
      width = TRACTOR_PIC_HORIZONTAL_WIDTH;
      height = TRACTOR_PIC_HORIZONTAL_HEIGHT;
    }
  }
}