#include <Arduino.h>
#include "function.h"
#include "HMI_Display.h"
#include "globals.h"
#include <esp_now.h>
#include <WiFi.h>
#include "matrix_utils.h"

typedef struct struct_message
{
  float roll;
  float pitch;
  float yaw;
  float longitude;   // Kinh độ (decimal degrees)
  float latitude;    // Vĩ độ (decimal degrees)
  uint16_t svnum;    // Số lượng vệ tinh GPS
  float pdop;        // Độ chính xác vị trí (Position Dilution of Precision)
  float hdop;        // Độ chính xác ngang (Horizontal Dilution of Precision)
  float vdop;        // Độ chính xác dọc (Vertical Dilution of Precision)
  float altitude;    // Độ cao GPS tính bằng mét
  float heading;     // Hướng GPS tính bằng độ
  float groundSpeed; // Tốc độ di chuyển trên mặt đất GPS tính bằng km/h
  float accelX;      // Gia tốc theo trục X (m/s^2)
  float accelY;      // Gia tốc theo trục Y (m/s^2)
  float accelZ;      // Gia tốc theo trục Z (m/s^2)
  float angularVelX; // Vận tốc góc theo trục X (°/s)
  float angularVelY; // Vận tốc góc theo trục Y (°/s)
  float angularVelZ; // Vận tốc góc theo trục Z (°/s)
  int16_t fieldX;    // Giá trị từ trường theo trục X (đơn vị: LSB)
  int16_t fieldY;    // Giá trị từ trường theo trục Y (đơn vị: LSB)
  int16_t fieldZ;    // Giá trị từ trường theo trục Z (đơn vị: LSB)
} struct_message;

struct_message receivedData;

// ---------------- Biến toàn cục ----------------
ekf_t ekf;

bool revce_flag = false;

float g_ax_body = 0.0;    // Gia tốc theo trục X trong hệ tọa độ thân xe (m/s^2)
float g_ay_body = 0.0;    // Gia tốc theo trục Y trong hệ tọa độ thân xe
float g_omega_body = 0.0; // Vận tốc góc trong hệ tọa độ thân xe (độ/giây)

_float_t dt = 0.0;

_float_t gps_x = 0.0;  // Vị trí x từ GPS (tọa độ cục bộ)
_float_t gps_y = 0.0;  // Vị trí y từ GPS
_float_t gps_vx = 0.0; // Vận tốc vx từ GPS (hướng Đông)
_float_t gps_vy = 0.0; // Vận tốc vy từ GPS (hướng Bắc)

unsigned long last_time_us = 0; // Biến để lưu thời gian trước đó (microseconds)
// uint16_t Tiller_GroundAngle;
// int16_t Tiller_GroundAngle_filter;
// uint16_t Tiller_TractorAngle;
// int16_t Tiller_TractorAngle_filter;

void OnDataRecv(const uint8_t *mac, const uint8_t *incomingData, int len)
{
  // Serial.print("aaaaaaa");
  // Serial.println(sizeof(receivedData));
  if (len == sizeof(receivedData))
  {
    revce_flag = true;
    // Serial.println("Receiver!");
    memcpy(&receivedData, incomingData, sizeof(receivedData));
    // Tiller_GroundAngle = receivedData.tiller;
    // Tiller_GroundAngle_filter = simpleKalmanFilter.updateEstimate(Tiller_GroundAngle);
    // Tiller_TractorAngle = receivedData.angle;
    // Tiller_TractorAngle_filter = simpleKalmanFilter_2.updateEstimate(Tiller_TractorAngle);
    g_svnum = receivedData.svnum;
    g_yaw = receivedData.yaw;
    g_roll = receivedData.roll;               // Lưu góc lăn (Roll) của máy cày
    g_pitch = receivedData.pitch;             // Lưu góc nghiêng (Pitch) của máy cày
    g_longitude = receivedData.longitude;     // Lưu kinh độ
    g_latitude = receivedData.latitude;       // Lưu vĩ độ
    g_pdop = receivedData.pdop;               // Lưu độ chính xác vị trí
    g_hdop = receivedData.hdop;               // Lưu độ chính xác ngang
    g_vdop = receivedData.vdop;               // Lưu độ chính xác dọc
    g_altitude = receivedData.altitude;       // Lưu độ cao GPS
    g_heading = receivedData.heading;         // Lưu hướng GPS
    g_groundSpeed = receivedData.groundSpeed; // Lưu tốc độ di chuyển trên mặt đất GPS
    g_accelX = receivedData.accelX;           // Lưu gia tốc theo trục X
    g_accelY = receivedData.accelY;           // Lưu gia tốc theo trục Y
    g_accelZ = receivedData.accelZ;           // Lưu gia tốc theo trục Z
    g_angularVelX = receivedData.angularVelX; // Lưu vận tốc góc theo trục X
    g_angularVelY = receivedData.angularVelY; // Lưu vận tốc góc theo trục Y
    g_angularVelZ = receivedData.angularVelZ; // Lưu vận tốc góc theo trục Z
    g_ax_body = g_accelX;
    g_ay_body = g_accelY;
    g_omega_body = degreesToRadians(g_angularVelZ);
  }
  else
  {
    revce_flag = false;
    // Tiller_GroundAngle_filter = 0;
    // Tiller_TractorAngle_filter = 0;
    g_svnum = 0;
    g_yaw = 0;
    g_latitude = 0.0;  // Đặt vĩ độ về 0 nếu không nhận được dữ liệu
    g_longitude = 0.0; // Đặt kinh độ về 0 nếu không nhận được dữ liệu
    g_pdop = 0.0;      // Đặt độ chính xác vị trí về 0 nếu không nhận được dữ liệu
    g_hdop = 0.0;      // Đặt độ chính xác ngang về 0 nếu không nhận được dữ liệu
    g_vdop = 0.0;      // Đặt độ chính xác dọc về 0 nếu không nhận được dữ liệu
    Serial.println("Error_Receiver!");
  }
}

void initialize_ekf_parameters()
{
  Serial.println("Khoi tao cac tham so EKF...");
  // Ước tính trạng thái ban đầu (ví dụ: zeros hoặc từ các lần đọc cảm biến đầu tiên)
  // tinyekf_initialize sẽ đặt ekf.x về 0, nên chúng ta không cần đặt ở đây trước.
  do
  {
    Serial.printf("snow: %d, latitude: %.6f, longitude: %.6f\n", g_svnum, g_latitude, g_longitude);

  } while (g_svnum < 10 && g_latitude == 0.0 && g_longitude == 0.0);
  initYAW = radians(g_yaw);
  ekf.x[0] = 0.0; // x_pos
  ekf.x[1] = 0.0; // y_pos
  ekf.x[2] = 0.0; // vx
  ekf.x[3] = 0.0; // vy
  ekf.x[4] = 0.0; // theta (yaw) tính bằng radian

  lat_origin = g_latitude;
  lon_origin = g_longitude; // Lưu tọa độ gốc ban đầu từ GPS
  meters_per_deg_lon = 111320.0 * cos(radians(lat_origin));
  origin_set = true;        // Đặt cờ gốc đã được thiết lập
  _float_t pdiag[EKF_N] = {
      2.0, // P_x: phương sai của vị trí x (m^2)
      2.0, // P_y: phương sai của vị trí y (m^2)
      0.5, // P_vx: phương sai của vận tốc x ((m/s)^2)
      0.5, // P_vy: phương sai của vận tốc y ((m/s)^2)
      0.01   // P_theta: phương sai của góc yaw (radians^2)
  };
  ekf_initialize(&ekf, pdiag);
  delay(500);
  Serial.printf("EKF da khoi tao: ekf0:%f, ekf1:%f, ekf2:%f, ekf3:%f, ekf4:%f\n",
                ekf.x[0], ekf.x[1], ekf.x[2], ekf.x[3], ekf.x[4]);
  delay(5000); // Đợi 1 giây để ổn định
}

void calculate_state_transition_fx(
    _float_t fx[EKF_N],                 // Mảng lưu trạng thái dự đoán (output)
    const _float_t current_x[EKF_N],    // Trạng thái hiện tại
    _float_t acc_x_b, _float_t acc_y_b, // Gia tốc theo thân xe (body frame)
    _float_t yaw_rate,                  // Tốc độ góc quay (body frame)
    _float_t delta_t)                   // Khoảng thời gian dt
{
  // Lấy các giá trị từ trạng thái trước đó
  const _float_t prev_x_pos = current_x[0];
  const _float_t prev_y_pos = current_x[1];
  const _float_t prev_vx = current_x[2];
  const _float_t prev_vy = current_x[3];
  const _float_t prev_theta = current_x[4];

  // --- Bước 1: Dự đoán góc quay (Yaw) mới ---
  // Góc mới là góc cũ cộng với tốc độ góc nhân với thời gian.
  // Đây là cách dự đoán đúng trong EKF, thay vì gán trực tiếp từ cảm biến.
  _float_t next_theta = prev_theta + yaw_rate * delta_t;

  // Chuẩn hóa góc về khoảng [-PI, PI] để tránh giá trị góc quá lớn
  next_theta = fmod(next_theta + M_PI, 2.0 * M_PI) - M_PI;
  if (next_theta < -M_PI)
  {
    next_theta += 2.0 * M_PI;
  }
  fx[4] = next_theta;

  // --- Bước 2: Dự đoán vận tốc dọc thân xe (forward speed) mới ---
  // Tính vận tốc dọc thân xe hiện tại từ các thành phần vận tốc trong hệ tọa độ thế giới
  _float_t prev_v = prev_vx * cos(prev_theta) + prev_vy * sin(prev_theta);

  // Dự đoán vận tốc dọc thân xe tiếp theo bằng gia tốc
  _float_t next_v = prev_v + acc_x_b * delta_t;

  // --- Bước 3: Dự đoán vị trí mới (x, y) bằng mô hình động học chính xác ---
  // Mô hình này cho kết quả chính xác khi xe di chuyển theo cung tròn (khi yaw_rate != 0)
  // hoặc đường thẳng (khi yaw_rate == 0).
  if (fabs(yaw_rate) > 1e-6) // Trường hợp xe đang quay (chuyển động cong)
  {
    _float_t turn_radius = next_v / yaw_rate; // Bán kính của cung tròn
    fx[0] = prev_x_pos + turn_radius * (sin(next_theta) - sin(prev_theta));
    fx[1] = prev_y_pos - turn_radius * (cos(next_theta) - cos(prev_theta)); // Chú ý dấu trừ
  }
  else // Trường hợp xe đi thẳng
  {
    fx[0] = prev_x_pos + next_v * cos(prev_theta) * delta_t;
    fx[1] = prev_y_pos + next_v * sin(prev_theta) * delta_t;
  }

  // --- Bước 4: Dự đoán các thành phần vận tốc trong hệ tọa độ thế giới (vx, vy) ---
  // Vận tốc trong hệ tọa độ thế giới được tính trực tiếp từ vận tốc dọc thân xe (next_v) và góc yaw mới.
  // Điều này đảm bảo vector vận tốc luôn cùng hướng với xe, tuân thủ ràng buộc của xe vi sai.
  fx[2] = next_v * cos(next_theta);
  fx[3] = next_v * sin(next_theta);
}

void calculate_jacobian_F(
    _float_t F[EKF_N * EKF_N],          // Ma trận F (output)
    const _float_t current_x[EKF_N],    // Trạng thái hiện tại (để lấy theta)
    _float_t acc_x_b, _float_t acc_y_b, // Gia tốc body frame
    _float_t delta_t)                   // Khoảng thời gian dt
{
  _float_t prev_theta = current_x[4];
  _float_t c_th = cos(prev_theta);
  _float_t s_th = sin(prev_theta);

  // Đạo hàm riêng cho các phần tử ma trận F
  // d(ax_world)/d(theta) = -ax_body*sin(theta) - ay_body*cos(theta)
  _float_t d_ax_w_d_th = -acc_x_b * s_th - acc_y_b * c_th;
  // d(ay_world)/d(theta) =  ax_body*cos(theta) - ay_body*sin(theta)
  _float_t d_ay_w_d_th = acc_x_b * c_th - acc_y_b * s_th;

  // Khởi tạo F (là ma trận không hoặc gần ma trận đơn vị rồi điền các giá trị khác)
  memset(F, 0, EKF_N * EKF_N * sizeof(_float_t));

  // Hàng 1: dfx[0]/dx (đạo hàm của x_pos_pred)
  F[0 * EKF_N + 0] = 1.0;                                   // d(fx[0])/d(x_pos)
  F[0 * EKF_N + 2] = delta_t;                               // d(fx[0])/d(vx)
  F[0 * EKF_N + 4] = 0.5 * d_ax_w_d_th * delta_t * delta_t; // d(fx[0])/d(theta)

  // Hàng 2: dfx[1]/dx (đạo hàm của y_pos_pred)
  F[1 * EKF_N + 1] = 1.0;                                   // d(fx[1])/d(y_pos)
  F[1 * EKF_N + 3] = delta_t;                               // d(fx[1])/d(vy)
  F[1 * EKF_N + 4] = 0.5 * d_ay_w_d_th * delta_t * delta_t; // d(fx[1])/d(theta)

  // Hàng 3: dfx[2]/dx (đạo hàm của vx_pred)
  F[2 * EKF_N + 2] = 1.0;                   // d(fx[2])/d(vx)
  F[2 * EKF_N + 4] = d_ax_w_d_th * delta_t; // d(fx[2])/d(theta)

  // Hàng 4: dfx[3]/dx (đạo hàm của vy_pred)
  F[3 * EKF_N + 3] = 1.0;                   // d(fx[3])/d(vy)
  F[3 * EKF_N + 4] = d_ay_w_d_th * delta_t; // d(fx[3])/d(theta)

  // Hàng 5: dfx[4]/dx (đạo hàm của theta_pred)
  F[4 * EKF_N + 4] = 1.0; // d(fx[4])/d(theta)
}

void get_process_noise_Q(_float_t Q[EKF_N * EKF_N], _float_t delta_t)
{
  memset(Q, 0, EKF_N * EKF_N * sizeof(_float_t));
  // Nhiễu quá trình: sự không chắc chắn trong mô hình chuyển động.
  // Đây là ma trận Q đơn giản hóa. Một ma trận Q chặt chẽ hơn có thể được suy ra từ các mô hình nhiễu liên tục.
  // Các giá trị này cần được tinh chỉnh (tune) dựa trên hiệu suất hệ thống và độ tin cậy vào mô hình.

  // Giả sử độ lệch chuẩn của gia tốc không được mô hình hóa (unmodeled acceleration)
  _float_t sigma_accel_process = g_sigma_accel_process; // m/s^2
  // Giả sử độ lệch chuẩn của thay đổi tốc độ góc không được mô hình hóa
  _float_t sigma_omega_process = g_sigma_omega_process; // rad/s

  // Ma trận Q thường được xây dựng dựa trên công thức Q_discrete = G * Q_continuous * G^T * dt
  // Trong đó G là ma trận ánh xạ nhiễu vào trạng thái.
  // Với mô hình [pos, vel], và nhiễu là gia tốc ngẫu nhiên (white noise acceleration):
  // Q_block = [ dt^4/4  dt^3/2 ] * sigma_accel^2
  //           [ dt^3/2  dt^2   ]
  // Đối với theta, Q_theta = dt^2 * sigma_omega^2 (nếu omega_dot là white noise)
  // hoặc Q_theta = var_omega * dt (nếu omega là white noise, ảnh hưởng trực tiếp đến theta)
  // Vì omega_body là đầu vào, nhiễu ở đây có thể là nhiễu của chính omega_body hoặc thay đổi không lường trước.

  // Đặt các phương sai trên đường chéo (cách tiếp cận đơn giản để bắt đầu)
  // Tinh chỉnh các giá trị này!
  Q[0 * EKF_N + 0] = pow(0.5 * sigma_accel_process * delta_t * delta_t, 2); // Phương sai vị trí x
  Q[1 * EKF_N + 1] = pow(0.5 * sigma_accel_process * delta_t * delta_t, 2); // Phương sai vị trí y
  Q[2 * EKF_N + 2] = pow(sigma_accel_process * delta_t, 2);                 // Phương sai vận tốc vx
  Q[3 * EKF_N + 3] = pow(sigma_accel_process * delta_t, 2);                 // Phương sai vận tốc vy
  Q[4 * EKF_N + 4] = pow(sigma_omega_process * delta_t, 2);                 // Phương sai góc theta

  // Cân nhắc thêm các phần tử ngoài đường chéo nếu các thành phần nhiễu tương quan,
  // ví dụ: Q(0,2) và Q(1,3) như mô tả trong các tài liệu về EKF.
  // Q[0*EKF_N + 2] = (pow(delta_t, 3)/2.0) * pow(sigma_accel_process,2);
  // Q[2*EKF_N + 0] = Q[0*EKF_N + 2];
  // Q[1*EKF_N + 3] = (pow(delta_t, 3)/2.0) * pow(sigma_accel_process,2);
  // Q[3*EKF_N + 1] = Q[1*EKF_N + 3];
}

void calculate_measurement_prediction_hx(
    _float_t hx[EKF_M],              // Mảng lưu phép đo dự đoán (output)
    const _float_t current_x[EKF_N]) // Trạng thái dự đoán hiện tại
{
  // GPS đo trực tiếp x_pos, y_pos, vx, vy từ trạng thái
  hx[0] = current_x[0]; // x_pos dự đoán
  hx[1] = current_x[1]; // y_pos dự đoán
  hx[2] = current_x[2]; // vx dự đoán
  hx[3] = current_x[3]; // vy dự đoán
}

// Lấy Jacobian H của hàm đo lường
void get_jacobian_H(_float_t H[EKF_M * EKF_N])
{ // Ma trận H (output)
  memset(H, 0, EKF_M * EKF_N * sizeof(_float_t));
  // GPS đo x_pos, y_pos, vx, vy
  // Hàng 1: dhx[0]/dx (đạo hàm của phép đo x_pos)
  H[0 * EKF_N + 0] = 1.0; // d(hx[0])/d(x_pos)
  // Hàng 2: dhx[1]/dx (đạo hàm của phép đo y_pos)
  H[1 * EKF_N + 1] = 1.0; // d(hx[1])/d(y_pos)
  // Hàng 3: dhx[2]/dx (đạo hàm của phép đo vx)
  H[2 * EKF_N + 2] = 1.0; // d(hx[2])/d(vx)
  // Hàng 4: dhx[3]/dx (đạo hàm của phép đo vy)
  H[3 * EKF_N + 3] = 1.0; // d(hx[3])/d(vy)
}

void get_measurement_noise_R(_float_t R_matrix[EKF_M * EKF_M])
{
  memset(R_matrix, 0, EKF_M * EKF_M * sizeof(_float_t));
  // Nhiễu đo lường: sự không chắc chắn trong các показания GPS.
  // Đây là các phương sai (độ lệch chuẩn bình phương).
  // Các giá trị này nên được lấy từ datasheet của ZED-F9R hoặc ước tính thực nghiệm.
  _float_t std_dev_gps_pos = g_std_dev_gps_pos; // m (ví dụ: độ lệch chuẩn vị trí GPS là 0.5 mét)
  _float_t std_dev_gps_vel = g_std_dev_gps_vel; // m/s (ví dụ: độ lệch chuẩn vận tốc GPS là 0.1 m/s)

  R_matrix[0 * EKF_M + 0] = pow(std_dev_gps_pos, 2); // Phương sai của x_gps
  R_matrix[1 * EKF_M + 1] = pow(std_dev_gps_pos, 2); // Phương sai của y_gps
  R_matrix[2 * EKF_M + 2] = pow(std_dev_gps_vel, 2); // Phương sai của vx_gps
  R_matrix[3 * EKF_M + 3] = pow(std_dev_gps_vel, 2); // Phương sai của vy_gps
}

void handleSerialInput()
{
  // Đọc toàn bộ chuỗi cho đến khi gặp ký tự xuống dòng '\n'
  String inputString = Serial.readStringUntil('\n');
  inputString.trim(); // Loại bỏ các khoảng trắng thừa ở đầu và cuối chuỗi

  if (inputString.length() == 0)
  {
    return; // Bỏ qua nếu là chuỗi rỗng (ví dụ: chỉ nhấn Enter)
  }

  Serial.print("Đã nhận: [");
  Serial.print(inputString);
  Serial.println("]");

  float tempValues[4];    // Mảng tạm để lưu các giá trị đã phân tích
  int valueCount = 0;     // Đếm số lượng giá trị hợp lệ đã tìm thấy
  int currentIndex = 0;   // Vị trí bắt đầu tìm kiếm token tiếp theo
  int nextCommaIndex = 0; // Vị trí của dấu phẩy tiếp theo

  // Lặp qua chuỗi để tìm các token được phân tách bằng dấu phẩy
  while (valueCount < 4 && currentIndex < inputString.length())
  {
    nextCommaIndex = inputString.indexOf(',', currentIndex);

    String token;
    if (nextCommaIndex == -1)
    { // Nếu không tìm thấy dấu phẩy nào nữa
      // Lấy phần còn lại của chuỗi làm token cuối cùng
      token = inputString.substring(currentIndex);
      currentIndex = inputString.length(); // Đánh dấu đã xử lý hết chuỗi
    }
    else
    {
      // Lấy token từ vị trí hiện tại đến trước dấu phẩy
      token = inputString.substring(currentIndex, nextCommaIndex);
      currentIndex = nextCommaIndex + 1; // Di chuyển con trỏ qua dấu phẩy
    }

    token.trim(); // Loại bỏ khoảng trắng thừa trong token (ví dụ: " 0.4 ")

    if (token.length() > 0)
    {                                           // Chỉ xử lý nếu token không rỗng
      tempValues[valueCount] = token.toFloat(); // Chuyển đổi token sang float
      valueCount++;
    }
    else if (nextCommaIndex != -1 && currentIndex <= inputString.length())
    {
      // Trường hợp có dấu phẩy liên tiếp (ví dụ: "0.1,,0.3") hoặc chuỗi kết thúc bằng dấu phẩy
      // Bỏ qua token rỗng này, nhưng vẫn phải đảm bảo valueCount chính xác
    }
  }

  // Kiểm tra xem đã nhận đủ 4 giá trị chưa
  if (valueCount == 4)
  {
    // Cập nhật các biến toàn cục
    g_sigma_accel_process = tempValues[0];
    g_sigma_omega_process = tempValues[1];
    g_std_dev_gps_pos = tempValues[2];
    g_std_dev_gps_vel = tempValues[3];

    Serial.println("Cập nhật các tham số thành công!");
    // printCurrentParameters();
  }
  else
  {
    Serial.print("Lỗi: Định dạng hoặc số lượng tham số không đúng. Nhận được ");
    Serial.print(valueCount);
    Serial.println(" giá trị hợp lệ.");
    Serial.println("Vui lòng nhập đúng 4 giá trị float, phân tách bằng dấu phẩy.");
    Serial.println("Ví dụ: 0.01,0.01,0.01,1.0");
    Serial.println("Các tham số hiện tại chưa được thay đổi:");
    // printCurrentParameters();
  }
}

bool ExternKalmanFilter()
{
  // --- Tính toán dt và xử lý các giá trị không hợp lệ ---
  unsigned long current_time_us = micros();
  dt = (current_time_us - last_time_us) / 1000000.0; // Chuyển từ microseconds sang seconds
  last_time_us = current_time_us;

  // Tối ưu 1: Bỏ qua chu kỳ nếu dt quá nhỏ, tránh dự đoán với giá trị không đáng tin cậy.
  if (dt < 1e-5) // Ngưỡng nhỏ hơn một chút so với 1e-6 để ổn định hơn
  {
    return false; // Trả về false để báo hiệu chu kỳ này không được thực thi
  }

  // --- Bước Dự đoán (Luôn chạy) ---
  // Các ma trận này phụ thuộc vào trạng thái và dt, cần được tính toán mỗi lần.
  _float_t fx[EKF_N];
  _float_t F[EKF_N * EKF_N];
  _float_t Q[EKF_N * EKF_N];

  calculate_state_transition_fx(fx, ekf.x, g_ax_body, g_ay_body, g_omega_body, dt);
  calculate_jacobian_F(F, ekf.x, g_ax_body, g_ay_body, dt);
  get_process_noise_Q(Q, dt);

  ekf_predict(&ekf, fx, F, Q);

  // Chuẩn hóa góc yaw trong trạng thái về khoảng [-PI, PI]
  ekf.x[4] = fmod(ekf.x[4] + M_PI, 2.0 * M_PI) - M_PI;
  if (ekf.x[4] < -M_PI)
  {
    ekf.x[4] += 2.0 * M_PI;
  }

  // --- Bước Cập nhật (Chỉ chạy khi có dữ liệu GPS mới và hợp lệ) ---
  if (origin_set && revce_flag && g_svnum > 10)
  {
    revce_flag = false; // Xử lý cờ ngay lập tức để tránh thực hiện lại

    // --- Tối ưu 2: Sử dụng biến `static` cho các ma trận không thay đổi hoặc ít thay đổi ---
    // Các ma trận này sẽ chỉ được cấp phát bộ nhớ một lần duy nhất.
    static _float_t H_jac[EKF_M * EKF_N];
    static _float_t R_noise[EKF_M * EKF_M];
    static bool is_ekf_matrices_initialized = false;

    // Ma trận H là hằng số trong mô hình này, chỉ cần tính một lần.
    if (!is_ekf_matrices_initialized)
    {
      get_jacobian_H(H_jac);
      is_ekf_matrices_initialized = true;
    }

    // Ma trận R chỉ phụ thuộc vào các tham số có thể điều chỉnh.
    // Việc tính lại mỗi lần cập nhật là chấp nhận được và đảm bảo R luôn đúng.
    get_measurement_noise_R(R_noise);

    // --- Chuẩn bị vector đo lường Z ---
    gps_x = (g_longitude - lon_origin) * meters_per_deg_lon;
    gps_y = (g_latitude - lat_origin) * METERS_PER_DEG_LAT;
    _float_t ground_speed_mps = g_groundSpeed * 1000.0 / 3600.0;
    _float_t heading_rad = g_yaw * M_PI / 180.0;
    gps_vx = ground_speed_mps * sin(heading_rad);
    gps_vy = ground_speed_mps * cos(heading_rad);
    _float_t z[EKF_M] = {gps_x, gps_y, gps_vx, gps_vy};

    // --- Kiểm tra Mahalanobis để loại bỏ phép đo nhiễu ---
    _float_t hx[EKF_M];
    calculate_measurement_prediction_hx(hx, ekf.x);

    // Sử dụng các biến static cho các ma trận tạm thời trong tính toán Mahalanobis
    static _float_t y_innovation[EKF_M], S_inv[EKF_M * EKF_M];
    mat_sub(z, hx, y_innovation, EKF_M, 1);

    // Tính S = H * P_pred * H^T + R
    _float_t S_innovation_cov[EKF_M * EKF_M]; // Khai báo cục bộ vì nó thay đổi nhiều
    _float_t H_P_pred[EKF_M * EKF_N];
    _float_t HT_jac[EKF_N * EKF_M];
    mat_mul(H_jac, ekf.P, H_P_pred, EKF_M, EKF_N, EKF_N);
    mat_transpose(H_jac, HT_jac, EKF_M, EKF_N);
    mat_mul(H_P_pred, HT_jac, S_innovation_cov, EKF_M, EKF_N, EKF_M);
    mat_add(S_innovation_cov, R_noise, S_innovation_cov, EKF_M, EKF_M);

    if (!mat_inv_gj(S_innovation_cov, S_inv, EKF_M))
    {
      Serial.println("Mahalanobis: Khong the nghich dao ma tran S! Bo qua cap nhat.");
      return true; // Vẫn trả về true vì đã xử lý thành công chu kỳ, chỉ bỏ qua update
    }

    // Tính Mahalanobis distance squared: d_sq = y^T * S_inv * y
    _float_t S_inv_y[EKF_M];
    mat_mul_vec(S_inv, y_innovation, S_inv_y, EKF_M, EKF_M);
    _float_t mahalanobis_sq_dist = 0;
    for (int i = 0; i < EKF_M; ++i)
    {
      mahalanobis_sq_dist += y_innovation[i] * S_inv_y[i];
    }

    // So sánh với ngưỡng và thực hiện cập nhật
    if (mahalanobis_sq_dist < MAHALANOBIS_CHI2_THRESHOLD_4DOF_95PCT)
    {
      if (ekf_update(&ekf, z, hx, H_jac, R_noise))
      {
        // Cập nhật các biến toàn cục sau khi cập nhật thành công
        ekf_x = ekf.x[0];
        ekf_y = ekf.x[1];
        ekf_vx = ekf.x[2]; // Cập nhật cả vận tốc
        ekf_vy = ekf.x[3];
        ekf_theta = ekf.x[4];
      }
      else
      {
        Serial.println("EKF Update That bai (loi nghich dao ma tran ben trong)");
        // Tối ưu 3: Thay vì reset ngay, chỉ cần bỏ qua cập nhật.
        // Reset chỉ nên xảy ra nếu lỗi kéo dài.
      }
    }
    else
    {
      Serial.printf("Phep do GPS bi loai bo boi Mahalanobis (%.2f > %.2f)\n",
                    mahalanobis_sq_dist, MAHALANOBIS_CHI2_THRESHOLD_4DOF_95PCT);
      // Không cập nhật EKF với phép đo này.
      // Không cần reset EKF ở đây vì đây là trường hợp ngoại lệ dự kiến.
    }
  }
  return true; // Trả về true nếu hàm đã chạy xong một chu kỳ dự đoán
}

// Hàm setup và loop giữ nguyên
void setup()
{
  setCpuFrequencyMhz(240); // Đặt tần số CPU của ESP32 (nếu cần)
  delay(500);
  Serial.begin(115200);
  delay(500); // Chờ Serial Monitor mở (cho một số board)

  hmi.begin(BAUDRATE, RX_PIN, TX_PIN); // Khởi tạo giao tiếp Nextion
  delay(500);                          // Chờ HMI khởi động
  WiFi.mode(WIFI_STA);
  delay(500); // Đợi WiFi ổn định
  // Init ESP-NOW
  if (esp_now_init() != ESP_OK)
  {
    Serial.println("Error initializing ESP-NOW");
    return;
  }

  esp_now_register_recv_cb(esp_now_recv_cb_t(OnDataRecv));
  // resetAndClear(); // Xóa màn hình và reset trạng thái ban đầu
  Serial.println(F("--- May Nong Nghiep - Dieu Huong Zigzag (GPS) ---"));
  Serial.println(F("Man hinh: 800x480"));
  StartPoint = 0;
  StartDir = 0;
  currentState = WAITING_FOR_POINTS;
  pinMode(33, OUTPUT);       // Thiết lập chân LED tích hợp làm đầu ra
  hmi.sendCommand("page 1"); // Chuyển đến trang chính của HMI
  // digitalWrite(33, HIGH);    // Bật LED tích hợp
  // delay(200);             // Giữ LED sáng trong 1 giây
  // digitalWrite(33, LOW);  // Tắt LED tích hợp
  // 0.1,0.01,0.01,0.61
  // 0.1,0.01,0.01,0.02
  // 0.1,0.001,1.0,0.6
  g_sigma_accel_process = 0.1;
  g_sigma_omega_process = 0.01;
  g_std_dev_gps_pos = 5.0;
  g_std_dev_gps_vel = 0.61;

  hmi.fillRect(0, 0, SCREEN_WIDTH_PX, SCREEN_HEIGHT_PX, SCREEN_BACKGROUND_COLOR);
}
unsigned long previousMillis = 0;

// Khoảng thời gian bạn muốn (100 mili giây)
const long interval = 100;
void loop()
{
  unsigned long currentMillis = millis();
  // char recei = hmi.receiveCommand(); // Gọi hàm nhận lệnh
  // if (recei == 0x12) // Nếu có lệnh mới nhận được
  // {

  //   digitalWrite(33, HIGH); // Bật LED tích hợp
  //   delay(500);             // Giữ LED sáng trong 1 giây
  //   digitalWrite(33, LOW);  // Tắt LED tích hợp
  // }

  switch (currentState)
  {
  case WAITING_FOR_POINTS:
    handlePointInput();
    break;
  case POINTS_ENTERED_READY_TO_DRAW:
     // Khoảng cách mét trên mỗi độ kinh độ
    initialize_ekf_parameters();
    drawFieldAndPath();
    currentState = PATH_DISPLAYED;
    has_valid_previous_tractor_pos = false;
    Serial.println(F("Da ve xong. Go 'clear'/'change' de thay doi hoac nhap lai."));
    break;
  case PATH_DISPLAYED:
    if (Serial.available() > 0)
    {
      handleSerialInput();
    }
    // delay(100); // Đợi 100 mili giây trước khi tiếp tục
    // handlePointInput();                // Tiếp tục lắng nghe lệnh 'clear', 'change...'

    // readAndProcessGpsData();           // Đọc dữ liệu GPS (mô phỏng hoặc thực tế)
    Serial.printf("distan lat: %.6f, lon: %.6f\n", (g_latitude - lat_origin) * METERS_PER_DEG_LAT, (g_longitude - lon_origin) * meters_per_deg_lon);
    ExternKalmanFilter(); // Chạy bộ lọc Kalman mở rộng (EKF)
    Serial.printf("  T: %.3f, X: %.2f, Y: %.2f, Vx: %.2f, Vy: %.2f, Th(deg): %.1f\n",
                  millis() / 1000.0, ekf.x[0], ekf.x[1], ekf.x[2], ekf.x[3], ekf.x[4] * 180.0 / M_PI);

    // updateAndDrawTractorPositionHMI(); // Cập nhật vị trí máy cày trên HMI
    break;
  }
  // Serial.printf("g_svnum: %d, g_yaw: %.2f, g_latitude: %.6f, g_longitude: %.6f, g_pdop: %.2f, g_hdop: %.2f, g_vdop: %.2f, g_altitude: %.2f, g_heading: %.2f, g_groundSpeed: %.2f\n",
  //               g_svnum, g_yaw, g_latitude, g_longitude, g_pdop, g_hdop, g_vdop, g_altitude, g_heading, g_groundSpeed);
  // Serial.printf("g_svnum: %d ,g_longitude: %.6f, g_latitude: %.6f, g_groundSpeed: %.2f, g_heading: %.2f, g_yaw: %.2f, g_roll: %.2f, g_pitch: %.2f, g_ax_body: %.2f, g_ay_body: %.2f, g_omega_body: %.2f\n",
  //               g_svnum, g_longitude, g_latitude, g_groundSpeed, g_heading, g_yaw, g_roll, g_pitch, g_ax_body, g_ay_body, g_omega_body);
  // Tính toán dt (delta time)

  if (currentMillis - previousMillis >= interval)
  {
    // Lưu lại thời điểm cuối cùng khối mã này được thực thi
    previousMillis = currentMillis;
    updateAndDrawTractorPositionHMI(); // Cập nhật vị trí máy cày trên HMI
    String command = "x0.val=" + String(g_svnum);
    hmi.sendCommand(command); // Gửi lệnh cập nhật số lượng vệ tinh lên HMI
    command = "t11.txt=\"" + String(g_yaw, 2) + "\"";
    hmi.sendCommand(command); // Gửi lệnh cập nhật góc phương vị lên HMI
    command = "t10.txt=\"" + String(g_pitch, 2) + "\"";
    hmi.sendCommand(command); // Gửi lệnh cập nhật góc phương vị lên HMI
    command = "t9.txt=\"" + String(g_roll, 2) + "\"";
    hmi.sendCommand(command); // Gửi lệnh cập nhật góc phương vị lên HMI
    command = "t1.txt=\"" + String(g_latitude, 6) + "\"";
    hmi.sendCommand(command); // Gửi lệnh cập nhật vĩ độ lên HMI
    command = "t2.txt=\"" + String(g_longitude, 6) + "\"";
    hmi.sendCommand(command); // Gửi lệnh cập nhật kinh độ lên HMI
    command = "t3.txt=\"" + String(g_pdop, 4) + "\"";
    hmi.sendCommand(command); // Gửi lệnh cập nhật kinh độ lên HMI
    command = "t4.txt=\"" + String(g_hdop, 4) + "\"";
    hmi.sendCommand(command); // Gửi lệnh cập nhật kinh độ lên HMI
    command = "t5.txt=\"" + String(g_vdop, 4) + "\"";
    hmi.sendCommand(command); // Gửi lệnh cập nhật độ cao lên HMI
    command = "t0.txt=\"" + String(g_heading, 2) + "\"";
    hmi.sendCommand(command); // Gửi lệnh cập nhật hướng GPS lên HMI
    command = "t15.txt=\"" + String(g_groundSpeed, 2) + "\"";
    hmi.sendCommand(command); // Gửi lệnh cập nhật hướng GPS lên HMI
    command = "t16.txt=\"" + String(ekf.x[0], 2) + "\"";
    hmi.sendCommand(command); // Gửi lệnh cập nhật hướng GPS lên HMI
    command = "t19.txt=\"" + String(ekf.x[1], 2) + "\"";
    hmi.sendCommand(command); // Gửi lệnh cập nhật hướng GPS lên HMI
  }
}