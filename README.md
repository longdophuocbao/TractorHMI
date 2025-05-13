# Hệ Thống Tạo Đường Đi Ziczac Cho Máy Nông Nghiệp Điều Hướng Bằng GPS Trên ESP32 Với Màn Hình Nextion

Dự án này triển khai một hệ thống tạo đường đi ziczac (boustrophedon) tự động cho các ứng dụng nông nghiệp chính xác, sử dụng vi điều khiển ESP32, nhận đầu vào tọa độ GPS của thửa ruộng và hiển thị trực quan trên màn hình Nextion HMI. Đường đi được tạo ra đảm bảo tâm của nông cụ luôn cách biên của thửa ruộng một khoảng an toàn bằng một nửa chiều rộng làm việc.

![Hình ảnh minh họa đường đi](đường_dẫn_đến_ảnh_của_bạn.png)
*(Thay thế `đường_dẫn_đến_ảnh_của_bạn.png` bằng đường dẫn thực tế đến file ảnh `image_d3c7b1.png` hoặc một ảnh minh họa khác của dự án)*

## Tính Năng Chính

* **Nhập liệu GPS:** Cho phép người dùng nhập tọa độ GPS (kinh độ, vĩ độ) của các đỉnh thửa ruộng thông qua Serial Monitor.
* **Chuyển đổi tọa độ:** Tự động tính toán và chuyển đổi tọa độ GPS sang tọa độ pixel trên màn hình Nextion, tối ưu hóa tỷ lệ và căn giữa.
* **Tạo đường đi Ziczac (Boustrophedon):** Sinh ra một đường đi tối ưu để bao phủ diện tích đa giác của thửa ruộng.
* **Offset Biên An Toàn:** Đảm bảo đường tâm của nông cụ luôn cách biên thửa ruộng một khoảng bằng `working_width_px / 2`, giúp nông cụ không đè lên biên.
* **Tùy chỉnh Hướng Đường Đi:**
    * Người dùng có thể chọn điểm bắt đầu (`StartPoint`) gần đúng cho đường đi.
    * Người dùng có thể chọn hướng chính của đường đi (`StartDir`) để song song với cạnh trước hoặc cạnh sau của `StartPoint`.
* **Hiển thị Trực Quan:** Vẽ đa giác thửa ruộng và đường đi đã tạo lên màn hình Nextion HMI.
* **Giao Tiếp Serial:** Điều khiển và nhận phản hồi qua Serial Monitor (thêm điểm, hoàn tất, xóa, thay đổi tham số).

## Yêu Cầu Phần Cứng

* Vi điều khiển ESP32 (ví dụ: ESP32 DOIT DEVKIT V1).
* Màn hình Nextion HMI (trong dự án này, kích thước ví dụ là 800x480 pixels).
* Module GPS để cung cấp tọa độ (dự án hiện tại mô phỏng việc nhập liệu qua Serial).
* Kết nối dây phù hợp giữa ESP32 và màn hình Nextion.

## Yêu Cầu Phần Mềm & Thư Viện

* **Framework:** Arduino cho ESP32.
* **Môi trường phát triển:** PlatformIO (khuyến nghị) hoặc Arduino IDE.
* **Thư viện C++ chuẩn:** `<vector>`, `<algorithm>`, `<cmath>`, `<limits>`.
* **Thư viện Arduino:** `<HardwareSerial.h>`, `<String.h>`.
* **File HMI cho Nextion:** (Nếu có) File `.tft` đã được thiết kế và tải lên màn hình Nextion. Dự án này giao tiếp trực tiếp qua lệnh nối tiếp nên có thể không cần file HMI phức tạp.

## Cài Đặt và Thiết Lập

1.  **Clone Repository:**
    ```bash
    git clone [URL_REPOSITORY_CUA_BAN]
    cd [TEN_THU_MUC_DU_AN]
    ```
2.  **PlatformIO:**
    * Mở dự án bằng VS Code với PlatformIO extension.
    * File `platformio.ini` đã được cấu hình cho board `esp32doit-devkit-v1`. Bạn có thể cần điều chỉnh nếu sử dụng board ESP32 khác.
    * PlatformIO sẽ tự động tải các thư viện cần thiết.
3.  **Kết Nối Phần Cứng:**
    * **Nextion HMI với ESP32:**
        * `Nextion RX` <-> `ESP32 TX_PIN` (trong code là `GPIO19` cho `Serial2`)
        * `Nextion TX` <-> `ESP32 RX_PIN` (trong code là `GPIO18` cho `Serial2`)
        * `Nextion 5V` <-> `ESP32 VIN` (hoặc nguồn 5V phù hợp)
        * `Nextion GND` <-> `ESP32 GND`
    * **Module GPS (nếu sử dụng thực tế):** Kết nối với một cổng Serial khác của ESP32 và điều chỉnh code để đọc dữ liệu từ đó.
4.  **Cấu Hình Màn Hình Nextion:**
    * Đảm bảo tốc độ baud của màn hình Nextion được cài đặt là `9600` (hoặc giá trị `NEXTION_BAUD_RATE` bạn định nghĩa trong code).

## Hướng Dẫn Sử Dụng

1.  **Kết nối ESP32 với máy tính** và mở Serial Monitor (tốc độ baud 115200).
2.  **Nhập tọa độ GPS:**
    * Khi được yêu cầu, nhập tọa độ các đỉnh của thửa ruộng theo định dạng: `KinhĐộ,VĩĐộ` (ví dụ: `106.660235,10.776924`). Nhấn Enter sau mỗi điểm.
    * Cần nhập ít nhất 3 điểm.
3.  **Hoàn tất nhập liệu:**
    * Gõ `done` và nhấn Enter sau khi đã nhập tất cả các điểm.
    * Hệ thống sẽ tính toán, chuyển đổi tọa độ và vẽ thửa ruộng cùng đường đi lên màn hình Nextion.
4.  **Các lệnh khác qua Serial:**
    * `clear`: Xóa tất cả các điểm đã nhập và làm sạch màn hình, cho phép nhập lại từ đầu.
    * `change start {index}`: Thay đổi `StartPoint` (chỉ số đỉnh tham chiếu để bắt đầu đường đi). Ví dụ: `change start 2`.
    * `change dir`: Chuyển đổi `StartDir` giữa 0 (đường đi song song với cạnh TRƯỚC `StartPoint`) và 1 (đường đi song song với cạnh SAU `StartPoint`).
5.  **Thông số làm việc:**
    * `working_width_real_meters`: Chiều rộng làm việc thực tế của nông cụ (đơn vị mét). Giá trị này được định nghĩa trong code.
    * `working_width_px`: Được tự động tính toán dựa trên `working_width_real_meters` và tỷ lệ scale của thửa ruộng trên màn hình. Đây là chiều rộng làm việc (tính bằng pixel) được sử dụng để tạo khoảng cách giữa các đường đi ziczac.

## Cấu Trúc Code (Sau khi tách file)

*(Mô tả ngắn gọn cách bạn đã tổ chức các file, ví dụ:)*
* `main.cpp`: Chứa hàm `setup()`, `loop()`, logic chính điều khiển trạng thái và xử lý input.
* `HMI_Display.h/.cpp`: Định nghĩa lớp `HMI_Display` để giao tiếp với màn hình Nextion.
* `PathGenerator.h/.cpp`: Chứa hàm `generatePath` và các logic liên quan đến tạo đường đi.
* `GeometryUtils.h/.cpp`: Chứa các hàm hình học tiện ích như `Point`, `Segment`, `intersectLineSegment`, `clipLineWithPolygon`, `isInside`.
* `CoordinateTransformer.h/.cpp`: Chứa các hàm liên quan đến chuyển đổi tọa độ GPS sang màn hình.
* `Config.h` (Tùy chọn): Chứa các hằng số cấu hình.

## Logic Thuật Toán Chính

* **Chuyển đổi GPS sang Màn hình:**
    1.  Tìm hình chữ nhật bao nhỏ nhất (bounding box) của các điểm GPS.
    2.  Tính toán `average_latitude_rad` để điều chỉnh tỷ lệ kinh độ cho chính xác hơn.
    3.  Xác định `scale_factor_combined` để các điểm GPS vừa vặn trong kích thước màn hình (đã trừ `SCREEN_PADDING_PX`).
    4.  Tính toán `offset_x_for_screen` và `offset_y_for_screen` để căn giữa đa giác trên màn hình.
    5.  Áp dụng scale và offset để chuyển từng điểm GPS sang tọa độ `Point` (pixel) trên màn hình, lật trục Y.
    6.  Chuyển `working_width_real_meters` sang `working_width_px` dựa trên `scale_factor_combined` và `average_latitude_rad`.
* **Chọn Cạnh Định Hướng (`actualEdgeChoiceForOrientation`):**
    * Dựa vào `StartPoint` (là `startVertexIndex`) và `StartDir` (là `startDirGlobal`):
        * Nếu `StartDir = 0`: Chọn cạnh nối đỉnh `(StartPoint - 1)` và `StartPoint`.
        * Nếu `StartDir = 1`: Chọn cạnh nối đỉnh `StartPoint` và `(StartPoint + 1)`.
* **Tạo Đường Đi (`generatePath`):**
    1.  Xác định hướng chính (`primary_dir_normalized`) song song với cạnh đã chọn và hướng quét vuông góc vào trong (`inward_offset_dir`).
    2.  Bắt đầu từ cạnh chuẩn, tạo các đường quét song song dịch chuyển vào trong từng khoảng `workingWidth`.
    3.  Với mỗi đường quét, dùng `clipLineWithPolygon` để tìm các đoạn thẳng nằm bên trong đa giác.
    4.  **Co ngắn đoạn:** Các đoạn này sau đó được co ngắn từ cả hai đầu một khoảng `workingWidth/2` để đảm bảo tâm nông cụ cách biên. Chỉ những đoạn còn chiều dài dương đáng kể mới được giữ lại.
    5.  Sắp xếp tất cả các đoạn hợp lệ theo thứ tự quét.
    6.  Nối các đoạn đã sắp xếp theo thuật toán Boustrophedon (ziczac), cố gắng giữ các lượt rẽ ngắn nhất.
    7.  Điều chỉnh cuối cùng: Nếu điểm kết thúc của toàn bộ đường ziczac gần `StartPoint` hơn điểm bắt đầu, toàn bộ đường đi sẽ bị đảo ngược.

## Hạn Chế và Vấn Đề Hiện Tại

* **`clipLineWithPolygon` và `isInside`:** Các hàm hình học này, đặc biệt là `clipLineWithPolygon`, rất phức tạp để triển khai một cách hoàn toàn mạnh mẽ và chính xác trong mọi trường hợp (đa giác lõm sâu, các trường hợp biên, lỗi số thực). Phiên bản hiện tại đã được cải thiện nhưng có thể vẫn cần kiểm tra kỹ lưỡng hơn với các hình dạng đa giác phức tạp.
* **Độ chính xác số thực:** Tính toán với số thực luôn có sai số tiềm ẩn, có thể ảnh hưởng đến các phép so sánh và kiểm tra hình học.
* **Đa giác đơn giản:** Thuật toán hiện tại giả định đa giác đầu vào là đơn giản (không tự cắt).
* **Hiệu năng:** Với các đa giác có rất nhiều đỉnh hoặc `workingWidth` rất nhỏ (dẫn đến nhiều đường quét), hiệu năng trên ESP32 có thể cần được xem xét.

## Hướng Phát Triển Tương Lai (Tùy chọn)

* Sử dụng thư viện hình học chuyên dụng (nếu có thể trên ESP32) để tăng độ tin cậy của `clipLineWithPolygon` và hỗ trợ các phép toán offset đa giác phức tạp hơn.
* Hỗ trợ đa giác lõm một cách hoàn hảo.
* Khả năng định nghĩa các "đảo" hoặc chướng ngại vật bên trong thửa ruộng.
* Cải thiện giao diện người dùng, có thể là trực tiếp trên màn hình Nextion thay vì chỉ qua Serial.
* Tối ưu hóa thuật toán để cải thiện hiệu năng.

---

Hy vọng README này sẽ hữu ích cho dự án của bạn!
