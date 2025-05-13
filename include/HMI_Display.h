#ifndef HMI_DISPLAY_H
#define HMI_DISPLAY_H

#include <Arduino.h>
#include "Point.h" // Nếu `Point` được tách riêng, cần include file này

class HMI_Display
{
public:
    HMI_Display(HardwareSerial &serial);

    void begin();
    void drawPic(int x, int y, int pic_id);
    void drawLine(int x1, int y1, int x2, int y2, int color);
    void drawLine(const Point &p1, const Point &p2, int color); // Phiên bản nhận Point
    void clearScreen(int color);
    void drawPointMarker(int x, int y, int color);
    void drawPointMarker(const Point &p, int color); // Phiên bản nhận Point

private:
    HardwareSerial *_serial;
    void sendCommand(String cmd);
};

#endif // HMI_DISPLAY_H