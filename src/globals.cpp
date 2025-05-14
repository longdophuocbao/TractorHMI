#include "globals.h"
#include "HMI_Display.h"

const unsigned long BAUDRATE = 115200; // Chân RX của Nextion
const int8_t RX_PIN = 18; // Chân RX của Nextion
const int8_t TX_PIN = 19; // Chân TX của Nextion

// Định nghĩa đối tượng HMI_Display toàn cục
HardwareSerial NextionSerial = Serial2; // Sử dụng Serial2 cho Nextion
HMI_Display hmi(NextionSerial);

// Biến toàn cục
std::vector<GpsPoint> field_vertices_gps; // Lưu tọa độ GPS gốc
std::vector<Point> field_vertices_screen; // Lưu tọa độ màn hình đã scale (Point double)

double min_lon_gps = 0.0, max_lon_gps = 0.0, min_lat_gps = 0.0, max_lat_gps = 0.0; // Biên GPS
float scale_factor_combined = 0.0f;                                               // Hệ số scale chung từ độ sang pixel
float offset_x_for_screen = 0.0f, offset_y_for_screen = 0.0f;                     // Offset để căn giữa trên màn hình
double average_latitude_rad = 0.0;                                               // Vĩ độ trung bình (radian) để tính toán chuyển đổi

ProgramState currentState = WAITING_FOR_POINTS; // Trạng thái chương trình
int StartPoint = 0;                             // Chỉ số đỉnh bắt đầu gần nhất
int StartDir = 0;                               // Chỉ số đỉnh bắt đầu của cạnh định hướng

// Định nghĩa màu Nextion
const int WHITE = 65535;
const int BLACK = 0;
const int GREEN = 2016;
const int BLUE = 31;
const int RED = 63488;
const int YELLOW = 65504;
const int SCREEN_BACKGROUND_COLOR = 17424;

// KÍCH THƯỚC MÀN HÌNH NEXTION VÀ PADDING
const int SCREEN_WIDTH_PX = 500;
const int SCREEN_HEIGHT_PX = 480;
const int SCREEN_PADDING_PX = 10;

// Hằng số cho chuyển đổi GPS
const float METERS_PER_DEGREE_LATITUDE = 111132.954f; // Hằng số gần đúng

// Chiều rộng làm việc thực tế (mét) và quy đổi sang pixel
float working_width_real_meters = 1.4f; // Ví dụ: nông cụ rộng 4 mét
int working_width_px = 0;               // Chiều rộng làm việc đã scale sang pixel (sẽ được tính toán)

const double GEOMETRY_EPSILON = 1e-9;

const int TRACTOR_PIC_ID_UP = 9;
const int TRACTOR_PIC_ID_RIGHT = 10;
const int TRACTOR_PIC_ID_LEFT = 11;
const int TRACTOR_PIC_ID_DOWN = 12;
const int TRACTOR_PIC_WIDTH = 22;  
const int TRACTOR_PIC_HEIGHT = 36;

const int TRACTOR_PIC_VERTICAL_WIDTH = 22;
const int TRACTOR_PIC_VERTICAL_HEIGHT = 36;
const int TRACTOR_PIC_HORIZONTAL_WIDTH = 36;
const int TRACTOR_PIC_HORIZONTAL_HEIGHT = 22;

const int TRACTOR_PIC_ID_DEFAULT = TRACTOR_PIC_ID_UP;
int current_tractor_display_pic_id = TRACTOR_PIC_ID_DEFAULT; // ID ảnh máy cày sẽ được vẽ
int previous_tractor_display_pic_id = TRACTOR_PIC_ID_DEFAULT;

// Biến cho vị trí máy cày thực tế
GpsPoint current_tractor_gps_actual;         // Tọa độ GPS thực từ module
Point current_tractor_screen_actual;         // Tọa độ màn hình tương ứng
Point previous_tractor_screen_actual;        // Lưu vị trí màn hình trước đó để xóa
bool has_valid_previous_tractor_pos = false; // Cờ cho biết có vị trí cũ hợp lệ để xóa không
bool new_tractor_gps_data_received = false;  // Cờ báo có dữ liệu GPS mới