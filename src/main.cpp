#include <Arduino.h>
#include "function.h"
#include "HMI_Display.h"
#include "globals.h"
// Hàm setup và loop giữ nguyên
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
  StartPoint = 0;
  StartDir = 1;
}

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
    handlePointInput(); // Tiếp tục lắng nghe lệnh 'clear', 'change...'
    break;
  }
}