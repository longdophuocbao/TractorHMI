#ifndef GLOBALS_H
#define GLOBALS_H

#include <vector>
#include "Point.h"
#include "function.h"
#include "HMI_Display.h"
extern const unsigned long BAUDRATE;
extern const int8_t RX_PIN;  // Chân RX của Nextion
extern const int8_t TX_PIN;  // Chân TX của Nextion

// Đối tượng HMI_Display toàn cục
extern HMI_Display hmi;
// Biến toàn cục
extern std::vector<GpsPoint> field_vertices_gps; // Lưu tọa độ GPS gốc
extern std::vector<Point> field_vertices_screen; // Lưu tọa độ màn hình đã scale (Point double)

extern double min_lon_gps, max_lon_gps, min_lat_gps, max_lat_gps; // Biên GPS
extern float scale_factor_combined;                               // Hệ số scale chung từ độ sang pixel
extern float offset_x_for_screen, offset_y_for_screen;            // Offset để căn giữa trên màn hình
extern double average_latitude_rad;                               // Vĩ độ trung bình (radian) để tính toán chuyển đổi

extern ProgramState currentState; // Trạng thái chương trình
extern int StartPoint;            // Chỉ số đỉnh bắt đầu gần nhất
extern int StartDir;              // Chỉ số đỉnh bắt đầu của cạnh định hướng

// Định nghĩa màu Nextion
extern const int WHITE;
extern const int BLACK;
extern const int GREEN;
extern const int BLUE;
extern const int RED;
extern const int YELLOW;
extern const int SCREEN_BACKGROUND_COLOR;

// KÍCH THƯỚC MÀN HÌNH NEXTION VÀ PADDING
extern const int SCREEN_WIDTH_PX;
extern const int SCREEN_HEIGHT_PX;
extern const int SCREEN_PADDING_PX;

// Hằng số cho chuyển đổi GPS
extern const float METERS_PER_DEGREE_LATITUDE; // Hằng số gần đúng

// Chiều rộng làm việc thực tế (mét) và quy đổi sang pixel
extern float working_width_real_meters; // Ví dụ: nông cụ rộng 4 mét
extern int working_width_px;                // Chiều rộng làm việc đã scale sang pixel (sẽ được tính toán)

extern const double GEOMETRY_EPSILON;

extern const int TRACTOR_PIC_ID_UP;
extern const int TRACTOR_PIC_ID_RIGHT;
extern const int TRACTOR_PIC_ID_LEFT;
extern const int TRACTOR_PIC_ID_DOWN;
extern const int TRACTOR_PIC_WIDTH; 
extern const int TRACTOR_PIC_HEIGHT;

extern const int TRACTOR_PIC_VERTICAL_WIDTH;
extern const int TRACTOR_PIC_VERTICAL_HEIGHT;
extern const int TRACTOR_PIC_HORIZONTAL_WIDTH;
extern const int TRACTOR_PIC_HORIZONTAL_HEIGHT;

extern const int TRACTOR_PIC_ID_DEFAULT;
extern int current_tractor_display_pic_id; // ID ảnh máy cày sẽ được vẽ
extern int previous_tractor_display_pic_id;

extern GpsPoint current_tractor_gps_actual;         // Tọa độ GPS thực từ module
extern Point current_tractor_screen_actual;         // 
extern Point previous_tractor_screen_actual;         // Lưu vị trí màn hình trước đó để xóa
extern bool has_valid_previous_tractor_pos;   // Cờ cho biết có vị trí cũ hợp lệ để xóa không
extern bool new_tractor_gps_data_received;     // Cờ báo có dữ liệu GPS mới

#endif // GLOBALS_H