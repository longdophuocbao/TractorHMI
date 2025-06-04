#include "HMI_Display.h"
#include <cmath>

HMI_Display::HMI_Display(HardwareSerial &serial) : _serial(&serial), _hmi_rx_buffer(""), _hmi_ff_counter(0) {}

void HMI_Display::begin(unsigned long BAUD_RATE, int8_t RX_PIN, int8_t TX_PIN)
{
    _serial->begin(BAUD_RATE, SERIAL_8N1, RX_PIN, TX_PIN); // NEXTION_BAUD_RATE, NEXTION_RX_PIN, NEXTION_TX_PIN
    delay(100);                                            // Chờ serial ổn định
}

void HMI_Display::drawPic(int x, int y, int pic_id)
{
    String command = "pic " + String(x) + "," + String(y) + "," + String(pic_id);
    sendCommand(command);
}

void HMI_Display::visPic(int pic_id, int enable)
{
    String command = "vis " + String(pic_id) + "," + String(enable);
    sendCommand(command);
}
void HMI_Display::changePic1(int x, int y, int pic_id)
{
    String command = "p1.x=" + String(x);
    sendCommand(command);
    command = "p1.y=" + String(y);
    sendCommand(command);
    command = "p1.pic=" + String(pic_id);
    sendCommand(command);
}

void HMI_Display::fillRect(int x, int y, int width, int height, int color)
{
    String command = "fill " + String(x) + "," + String(y) + "," +
                     String(width) + "," + String(height) + "," + String(color);
    sendCommand(command);
    // delay(5); // sendCommand đã có delay, nhưng fill có thể cần thêm nếu phức tạp
}


void HMI_Display::drawLine(int x1, int y1, int x2, int y2, int color)
{
    String command = "line " + String(x1) + "," + String(y1) + "," + String(x2) + "," + String(y2) + "," + String(color);
    sendCommand(command);
}

void HMI_Display::drawLine(const Point &p1, const Point &p2, int color)
{
    drawLine(static_cast<int>(std::round(p1.x)), static_cast<int>(std::round(p1.y)),
             static_cast<int>(std::round(p2.x)), static_cast<int>(std::round(p2.y)),
             color);
}

void HMI_Display::clearScreen(int color)
{
    String command = "cls " + String(color);
    sendCommand(command);
    delay(50); // cls có thể cần delay lâu hơn
}

void HMI_Display::drawPointMarker(int x, int y, int color)
{
    const int radius = 4; // Bán kính điểm đánh dấu
    String command = "cirs " + String(x) + "," + String(y) + "," + String(radius) + "," + String(color);
    sendCommand(command);
}

void HMI_Display::drawPointMarker(const Point &p, int color)
{
    drawPointMarker(static_cast<int>(std::round(p.x)), static_cast<int>(std::round(p.y)), color);
}

void HMI_Display::sendCommand(String cmd)
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

char HMI_Display::receiveCommand()
{
    char incomingByte;
    while (_serial->available() > 0)
    {
        _serial->flush(); // Đảm bảo buffer được làm sạch trước khi đọc
        delay(5);         // Đợi một chút để đảm bảo dữ liệu đã sẵn sàng
        // Serial.print("Available bytes: ");
        // Serial.println(_serial->read(), HEX);      // In ra số byte có sẵn trong buffer
        char incomingByte = (char)_serial->read(); // Đọc một byte
        return incomingByte;
    }
    return '\0'; // Trả về ký tự null nếu không có dữ liệu
}
