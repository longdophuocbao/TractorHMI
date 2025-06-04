#include <Arduino.h>
#include "function.h"
#include "HMI_Display.h"
#include "globals.h"
#include <esp_now.h>
#include <WiFi.h>

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
} struct_message;

struct_message receivedData;

// uint16_t Tiller_GroundAngle;
// int16_t Tiller_GroundAngle_filter;
// uint16_t Tiller_TractorAngle;
// int16_t Tiller_TractorAngle_filter;

bool revce_flag = false;
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
    g_roll = receivedData.roll;         // Lưu góc lăn (Roll) của máy cày
    g_pitch = receivedData.pitch;       // Lưu góc nghiêng (Pitch) của máy cày
    g_longitude = receivedData.longitude; // Lưu kinh độ
    g_latitude = receivedData.latitude;   // Lưu vĩ độ
    g_pdop = receivedData.pdop;           // Lưu độ chính xác vị trí
    g_hdop = receivedData.hdop;           // Lưu độ chính xác ngang
    g_vdop = receivedData.vdop;           // Lưu độ chính xác dọc
    g_altitude = receivedData.altitude; // Lưu độ cao GPS
    g_heading = receivedData.heading;   // Lưu hướng GPS
    g_groundSpeed = receivedData.groundSpeed; // Lưu tốc độ di chuyển trên mặt đất GPS
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

// Hàm setup và loop giữ nguyên
void setup()
{
  delay(500);
  Serial.begin(115200);
  delay(500);  // Chờ Serial Monitor mở (cho một số board)

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
  hmi.sendCommand("page 0"); // Chuyển đến trang chính của HMI
  // digitalWrite(33, HIGH);    // Bật LED tích hợp
  // delay(200);             // Giữ LED sáng trong 1 giây
  // digitalWrite(33, LOW);  // Tắt LED tích hợp
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
    drawFieldAndPath();
    currentState = PATH_DISPLAYED;
    has_valid_previous_tractor_pos = false;
    Serial.println(F("Da ve xong. Go 'clear'/'change' de thay doi hoac nhap lai."));
    break;
  case PATH_DISPLAYED:
    delay(100); // Đợi 100 mili giây trước khi tiếp tục
    handlePointInput();                // Tiếp tục lắng nghe lệnh 'clear', 'change...'
    readAndProcessGpsData();           // Đọc dữ liệu GPS (mô phỏng hoặc thực tế)
    updateAndDrawTractorPositionHMI(); // Cập nhật vị trí máy cày trên HMI
    break;
  }
  // Serial.printf("g_svnum: %d, g_yaw: %.2f, g_latitude: %.6f, g_longitude: %.6f, g_pdop: %.2f, g_hdop: %.2f, g_vdop: %.2f, g_altitude: %.2f, g_heading: %.2f, g_groundSpeed: %.2f\n",
  //               g_svnum, g_yaw, g_latitude, g_longitude, g_pdop, g_hdop, g_vdop, g_altitude, g_heading, g_groundSpeed);
  if (currentMillis - previousMillis >= interval)
  {
    // Lưu lại thời điểm cuối cùng khối mã này được thực thi
    previousMillis = currentMillis;
    String command = "x0.val=" + String(g_svnum);
    hmi.sendCommand(command); // Gửi lệnh cập nhật số lượng vệ tinh lên HMI
    command = "t11.txt=\"" + String(g_yaw, 2) + "\"";
    hmi.sendCommand(command); // Gửi lệnh cập nhật góc phương vị lên HMI
    command = "t10.txt=\"" + String(g_pitch, 2) + "\"";
    hmi.sendCommand(command); // Gửi lệnh cập nhật góc phương vị lên HMI
    command = "t9.txt=\"" + String(g_roll, 2) + "\"";
    hmi.sendCommand(command); // Gửi lệnh cập nhật góc phương vị lên HMI
    command = "t1.txt=\"" + String(g_latitude,6) + "\"";
    hmi.sendCommand(command); // Gửi lệnh cập nhật vĩ độ lên HMI
    command = "t2.txt=\"" + String(g_longitude,6) + "\"";
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
  }
}