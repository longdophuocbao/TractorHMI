#ifndef HMI_DISPLAY_H
#define HMI_DISPLAY_H

#include <Arduino.h>
#include "Point.h" // Nếu `Point` được tách riêng, cần include file này

class HMI_Display
{
public:
    HMI_Display(HardwareSerial &serial);

    void begin(unsigned long BAUD_RATE, int8_t RX_PIN, int8_t TX_PIN);
    void drawPic(int x, int y, int pic_id);
    void visPic(int pic_id, int enable);
    void fillRect(int x, int y, int width, int height, int color);
    void drawLine(int x1, int y1, int x2, int y2, int color);
    void changePic1(int x, int y, int pic_id);
    void drawLine(const Point &p1, const Point &p2, int color); // Phiên bản nhận Point
    void clearScreen(int color);
    void drawPointMarker(int x, int y, int color);
    void drawPointMarker(const Point &p, int color); // Phiên bản nhận Point
    char receiveCommand();                           // Hàm chúng ta sẽ viết
    HardwareSerial *_serial;
    void sendCommand(String cmd);
    String _hmi_rx_buffer; // Bộ đệm để lưu trữ lệnh đang nhận
    int _hmi_ff_counter;   // Bộ đếm số byte 0xFF liên tiếp
};

#endif // HMI_DISPLAY_H