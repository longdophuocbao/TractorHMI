#include "globals.h"
#include "HMI_Display.h"

const unsigned long BAUDRATE = 115200; // Chân RX của Nextion
const int8_t RX_PIN = 16;              // Chân RX của Nextion
const int8_t TX_PIN = 17;              // Chân TX của Nextion

// Định nghĩa đối tượng HMI_Display toàn cục
HardwareSerial NextionSerial = Serial2; // Sử dụng Serial2 cho Nextion
HMI_Display hmi(NextionSerial);

// Biến toàn cục
std::vector<GpsPoint> field_vertices_gps; // Lưu tọa độ GPS gốc
std::vector<Point> field_vertices_screen; // Lưu tọa độ màn hình đã scale (Point double)

double min_lon_gps = 0.0, max_lon_gps = 0.0, min_lat_gps = 0.0, max_lat_gps = 0.0; // Biên GPS
float scale_factor_combined = 0.0f;                                                // Hệ số scale chung từ độ sang pixel
float offset_x_for_screen = 0.0f, offset_y_for_screen = 0.0f;                      // Offset để căn giữa trên màn hình
double average_latitude_rad = 0.0;                                                 // Vĩ độ trung bình (radian) để tính toán chuyển đổi

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
const int SCREEN_BACKGROUND_COLOR = 17424; // 63289

// KÍCH THƯỚC MÀN HÌNH NEXTION VÀ PADDING
const int SCREEN_WIDTH_PX = 500;
const int SCREEN_HEIGHT_PX = 480;
const int SCREEN_PADDING_PX = 10;

// Hằng số cho chuyển đổi GPS
const float METERS_PER_DEGREE_LATITUDE = 111132.954f; // Hằng số gần đúng

// Chiều rộng làm việc thực tế (mét) và quy đổi sang pixel
float working_width_real_meters = 1.5f; // Ví dụ: nông cụ rộng 4 mét
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

const int TRACTOR_PIC_FIXED_WIDTH = 43;
const int TRACTOR_PIC_FIXED_HEIGHT = 43;

const int TRACTOR_PIC_ID_DEFAULT = TRACTOR_PIC_ID_UP;
int current_tractor_display_pic_id = TRACTOR_PIC_ID_DEFAULT; // ID ảnh máy cày sẽ được vẽ
int previous_tractor_display_pic_id = TRACTOR_PIC_ID_DEFAULT;

// Biến cho vị trí máy cày thực tế
GpsPoint current_tractor_gps_actual;         // Tọa độ GPS thực từ module
Point current_tractor_screen_actual;         // Tọa độ màn hình tương ứng
Point previous_tractor_screen_actual;        // Lưu vị trí màn hình trước đó để xóa
bool has_valid_previous_tractor_pos = false; // Cờ cho biết có vị trí cũ hợp lệ để xóa không
bool new_tractor_gps_data_received = false;  // Cờ báo có dữ liệu GPS mới

uint16_t g_svnum = 0;    // Số lượng vệ tinh GPS
float g_yaw = 0.0;       // Góc phương vị (Yaw) của máy cày (độ)
float g_roll = 0.0;      // Góc lăn (Roll) của máy cày (độ)
float g_pitch = 0.0;     // Góc nghiêng (Pitch) của máy cày (độ)
float g_longitude = 0.0; // Kinh độ (decimal degrees)
float g_latitude = 0.0;  // Vĩ độ (decimal degrees)
float g_pdop = 0.0;      // Độ chính xác vị trí (Position Dilution of Precision)
float g_hdop = 0.0;      // Độ chính xác ngang (Horizontal Dilution of Precision)
float g_vdop = 0.0;      // Độ chính xác dọc (Vertical Dilution of Precision)
float g_altitude= 0.0;          // Độ cao GPS tính bằng mét
float g_heading = 0.0;          // Hướng GPS tính bằng độ
float g_groundSpeed = 0.0;      // Tốc độ di chuyển trên mặt đất GPS tính bằng km/h
float g_accelX = 0.0;       // Gia tốc theo trục X (m/s^2)
float g_accelY = 0.0;       // Gia tốc theo trục Y (m/s^2)
float g_accelZ = 0.0;       // Gia tốc theo trục Z (m/s^2)
float g_angularVelX = 0.0;  // Vận tốc góc theo trục X (°/s)
float g_angularVelY = 0.0;  // Vận tốc góc theo trục Y (°/s)
float g_angularVelZ = 0.0;  // Vận tốc góc theo trục Z (°/s)

// Giá trị mặc định cho các tham số EKF
double g_sigma_accel_process = 0.0001; // m/s^2, giá trị ban đầu từ main.cpp
double g_sigma_omega_process = 0.0001; // rad/s, giá trị ban đầu từ main.cpp
double g_std_dev_gps_pos = 0.001;    // m, giá trị ban đầu từ main.cpp
double g_std_dev_gps_vel = 2.0;      // m/s, giá trị ban đầu từ main.cpp

// Biến toàn cục cho bộ lọc Kalman mở rộng (EKF)
_float_t ekf_x = 0.0; // Biến toàn cục cho trạng thái EKF
_float_t ekf_y = 0.0; // Biến toàn cục cho trạng thái EKF
_float_t ekf_vx = 0.0; // Biến toàn cục cho trạng thái EKF
_float_t ekf_vy = 0.0; // Biến toàn cục cho trạng thái
_float_t ekf_theta = 0.0; // Biến toàn cục cho trạng thái EKF (yaw)

bool origin_set = false; // Cờ cho biết đã đặt tọa độ gốc từ GPS hay chưa

_float_t lat_origin = 0.0; // Sẽ được đặt bởi điểm GPS đầu tiên
_float_t lon_origin = 0.0; // Sẽ được đặt bởi điểm GPS đầu tiên

const _float_t METERS_PER_DEG_LAT = 111132.954; // Mét xấp xỉ trên mỗi độ vĩ độ
_float_t meters_per_deg_lon = 0.0;              // Sẽ được tính sau khi lat_origin được đặt
