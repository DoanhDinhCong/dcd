#!/usr/bin/env python3
"""
velocity_bridge.py - Cầu nối giao tiếp STM32 qua Serial
=======================================================
Node ROS2 giao tiếp với STM32 để điều khiển robot mecanum 4 bánh.

CHỨC NĂNG CHÍNH:
----------------
1. TX (Truyền): Gửi lệnh vận tốc "V vx vy wz" tới STM32 với tần số 50Hz
2. RX (Nhận): Đọc dữ liệu encoder "ENC ms=... T=... D=..." và publish JointState

PROTOCOL GIAO TIẾP:
------------------
TX (ROS2 → STM32):
  Format: "V vx vy wz\n"
  - vx: Vận tốc tuyến tính X (m/s) - tiến/lùi
  - vy: Vận tốc tuyến tính Y (m/s) - sang trái/phải (mecanum)
  - wz: Vận tốc góc Z (rad/s) - xoay
  Ví dụ: "V 0.500 0.000 0.785\n" → đi thẳng 0.5m/s, xoay 45°/s

RX (STM32 → ROS2):
  Format: "ENC ms=12345 T=100,200,300,400 D=5,6,7,8\n"
  - ms: Timestamp (milliseconds)
  - T: Total encoder ticks [FL, FR, RR, RL]
  - D: Delta ticks từ lần đọc trước [FL, FR, RR, RL]
  Ví dụ: "ENC ms=1000 T=100,150,120,130 D=10,15,12,13\n"

THÔNG SỐ QUAN TRỌNG:
--------------------
- serial_port: /dev/ttyACM0 (STM32 thường dùng ttyACM thay vì ttyUSB)
- baud: 115200 (tốc độ truyền STM32)
- rate_hz: 50 (tần số gửi lệnh - 50Hz = 20ms/lần)
- cmd_timeout_ms: 200 (timeout lệnh - nếu >200ms không có lệnh mới)
- ticks_per_rev: 6864 (số xung encoder trên 1 vòng bánh - xem datasheet motor)

✅ ĐÃ SỬA CÁC LỖI:
------------------
1. MAJOR #8: Watchdog spam serial - Chỉ gửi zero MỘT LẦN khi timeout
2. CRITICAL: js.n → js.name (lỗi thuộc tính JointState message)


"""

import time
import serial
import re
import threading
import math
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import String, Int32MultiArray
from sensor_msgs.msg import JointState

# =============================================================================
# REGEX PATTERN - Mẫu để phân tích chuỗi encoder
# =============================================================================
# Pattern để parse dòng encoder từ STM32
# Format: "ENC ms=12345 T=1,2,3,4 D=5,6,7,8"
# Groups:
#   1: ms (timestamp)
#   2-5: T values (total ticks 4 bánh)
#   6-9: D values (delta ticks 4 bánh)
ENC_PAT = re.compile(
    r"ENC\s+"                           # Từ khóa "ENC" + khoảng trắng
    r"ms=(\d+)\s+"                      # ms=<số> + khoảng trắng
    r"T=([-0-9]+),([-0-9]+),([-0-9]+),([-0-9]+)\s+"  # T=<4 số phân cách bởi dấu phẩy>
    r"D=([-0-9]+),([-0-9]+),([-0-9]+),([-0-9]+)"     # D=<4 số phân cách bởi dấu phẩy>
)


class VelocityBridgeVfmt(Node):
    """
    Node cầu nối vận tốc - Giao tiếp với STM32 điều khiển mecanum robot
    
    Kiến trúc:
    ---------
    ┌─────────────┐      ┌──────────────┐      ┌─────────────┐
    │   Nav2 /    │ cmd_vel│  Velocity   │  TX  │   STM32     │
    │  Teleop     │──────→│   Bridge    │─────→│  Firmware   │
    └─────────────┘       │             │      └─────────────┘
                          │   (Node)    │  RX         ↓
    ┌─────────────┐ joint │             │←─────  Encoders
    │robot_state_ │ state │             │
    │ publisher   │←──────┴──────────────┘
    └─────────────┘
    
    Luồng dữ liệu:
    -------------
    1. Subscribe /cmd_vel (geometry_msgs/Twist) từ Nav2/teleop
    2. Giới hạn vận tốc trong max_vx, max_vy, max_wz
    3. Gửi "V vx vy wz" tới STM32 qua Serial (50Hz)
    4. Đọc "ENC ..." từ STM32 (thread riêng)
    5. Tính position và velocity từ encoder ticks
    6. Publish sensor_msgs/JointState cho robot_state_publisher
    
    Tham số cấu hình:
    ----------------
    SERIAL & TX:
    - serial_port: Cổng serial STM32 (mặc định /dev/ttyACM0)
    - baud: Baudrate (mặc định 115200)
    - rate_hz: Tần số gửi lệnh (mặc định 50Hz)
    - cmd_timeout_ms: Timeout lệnh (mặc định 200ms)
    - max_vx/vy/wz: Giới hạn vận tốc
    - zero_on_timeout: Gửi zero khi timeout (mặc định False)
    
    ENCODER & JOINT STATE:
    - ticks_per_rev: Xung encoder/vòng (mặc định 6864)
    - wheel_joint_names: Tên các joint bánh xe
    - invert_wheels: Đảo chiều bánh nào [FL, FR, RR, RL]
    
    LOGGING:
    - echo_tx: Hiện log lệnh gửi đi (mặc định True)
    - echo_rx: Hiện log dữ liệu nhận về (mặc định False)
    - tx_log_on_change_only: Chỉ log khi vận tốc thay đổi
    """
    
    def __init__(self):
        super().__init__('velocity_bridge')
        
        # =====================================================================
        # THAM SỐ SERIAL VÀ TRUYỀN (TX)
        # =====================================================================
        
        # Cổng serial kết nối STM32
        # STM32 thường dùng /dev/ttyACM0 (CDC - Communications Device Class)
        # Arduino/ESP32 thường dùng /dev/ttyUSB0 (FTDI/CH340)
        self.declare_parameter('serial_port', '/dev/ttyACM0')
        
        # Tốc độ truyền (baudrate)
        # STM32 thường dùng 115200 hoặc 921600
        self.declare_parameter('baud', 115200)
        
        # Tần số gửi lệnh (Hz)
        # 50Hz = 20ms/lần - cân bằng giữa độ mượt và tải CPU
        # Cao hơn (100Hz) = mượt hơn nhưng tốn CPU
        # Thấp hơn (20Hz) = giật hơn nhưng nhẹ
        self.declare_parameter('rate_hz', 50.0)
        
        # Timeout lệnh (milliseconds)
        # Nếu >200ms không nhận cmd_vel mới → coi như timeout
        # Nếu zero_on_timeout=True → gửi "V 0 0 0" để dừng robot
        self.declare_parameter('cmd_timeout_ms', 200)
        
        # Giới hạn vận tốc tối đa (m/s và rad/s)
        # Dùng để clamp các lệnh vượt quá khả năng robot
        self.declare_parameter('max_vx', 1.0)   # Vận tốc X tối đa (m/s)
        self.declare_parameter('max_vy', 1.0)   # Vận tốc Y tối đa (m/s)
        self.declare_parameter('max_wz', 2.0)   # Vận tốc góc tối đa (rad/s)
        
        # Watchdog - Gửi zero khi timeout?
        # True: Gửi "V 0 0 0" khi timeout để dừng robot (an toàn hơn)
        # False: Không gửi gì, STM32 tự xử lý timeout
        self.declare_parameter('zero_on_timeout', False)
        
        # Hiển thị log TX/RX?
        self.declare_parameter('echo_tx', True)   # Hiện lệnh gửi đi
        self.declare_parameter('echo_rx', False)  # Hiện dữ liệu nhận về (spam!)
        
        # =====================================================================
        # THAM SỐ ENCODER VÀ JOINT STATE
        # =====================================================================
        
        # Số xung encoder trên 1 vòng bánh xe
        # Công thức: ticks_per_rev = encoder_resolution × gear_ratio
        # Ví dụ: 
        #   - Encoder 11 xung/vòng × Gear 1:62.4 = 686.4 xung/vòng motor
        #   - Nếu có thêm bánh răng giảm tốc 1:10 → 6864 xung/vòng bánh
        # ⚠️ CẦN ĐO CHÍNH XÁC: Cho bánh quay 1 vòng, đếm xung encoder
        self.declare_parameter('ticks_per_rev', 6864.0)
        
        # Tên các joint bánh xe trong URDF
        # Thứ tự: [Front-Left, Front-Right, Rear-Right, Rear-Left]
        # Phải khớp với URDF: <joint name="wheel_fl_joint" ...>
        self.declare_parameter('wheel_joint_names', [
            'wheel_fl_joint',  # Bánh trước trái
            'wheel_fr_joint',  # Bánh trước phải
            'wheel_rr_joint',  # Bánh sau phải
            'wheel_rl_joint'   # Bánh sau trái
        ])
        
        # Đảo chiều bánh xe nào?
        # [FL, FR, RR, RL] - True = đảo chiều, False = giữ nguyên
        # Dùng khi encoder đấu ngược hoặc motor quay ngược
        # ⚠️ CẦN KIỂM TRA: Cho robot đi thẳng, xem bánh nào quay ngược
        self.declare_parameter('invert_wheels', [True, True, True, True])

        # =====================================================================
        # ĐỌC GIÁ TRỊ THAM SỐ
        # =====================================================================
        
        port = self.get_parameter('serial_port').get_parameter_value().string_value
        baud = int(self.get_parameter('baud').value)
        self.rate = float(self.get_parameter('rate_hz').value)
        self.timeout_s = float(self.get_parameter('cmd_timeout_ms').value) / 1000.0
        self.max_vx = float(self.get_parameter('max_vx').value)
        self.max_vy = float(self.get_parameter('max_vy').value)
        self.max_wz = float(self.get_parameter('max_wz').value)
        self.zero_on_timeout = bool(self.get_parameter('zero_on_timeout').value)
        self.echo_tx = bool(self.get_parameter('echo_tx').value)
        self.echo_rx = bool(self.get_parameter('echo_rx').value)

        # =====================================================================
        # KIỂM SOÁT LOG TX (Tránh spam terminal)
        # =====================================================================
        
        # Chỉ log khi vận tốc thay đổi đáng kể?
        # True: Chỉ log khi |Δv| > epsilon (giảm spam)
        # False: Log mọi lần gửi (rất nhiều log!)
        self.declare_parameter('tx_log_on_change_only', True)
        
        # Ngưỡng thay đổi tối thiểu để log
        # epsilon_v: Thay đổi vận tốc tuyến tính (m/s)
        # epsilon_w: Thay đổi vận tốc góc (rad/s)
        self.declare_parameter('tx_log_epsilon_v', 0.01)   # 1cm/s
        self.declare_parameter('tx_log_epsilon_w', 0.02)   # ~1°/s
        
        # Keepalive: Log định kỳ dù không thay đổi
        # 0.0 = không log keepalive
        # >0 = log mỗi N giây để biết node vẫn chạy
        self.declare_parameter('tx_keepalive_sec', 0.0)
        
        self.tx_log_on_change_only = bool(self.get_parameter('tx_log_on_change_only').value)
        self.tx_log_eps_v = float(self.get_parameter('tx_log_epsilon_v').value)
        self.tx_log_eps_w = float(self.get_parameter('tx_log_epsilon_w').value)
        self.tx_keepalive_sec = float(self.get_parameter('tx_keepalive_sec').value)
        
        # Biến tracking cho log
        self._last_logged_cmd = None          # Lệnh cuối cùng được log
        self._last_log_wall = time.monotonic()  # Thời điểm log cuối (wall time)

        # =====================================================================
        # CHUYỂN ĐỔI ENCODER
        # =====================================================================
        
        tpr = float(self.get_parameter('ticks_per_rev').value)
        
        # Tính radian trên mỗi tick encoder
        # 1 vòng = 2π radian = ticks_per_rev ticks
        # → 1 tick = 2π / ticks_per_rev radian
        self.rad_per_tick = 2.0 * math.pi / tpr
        
        # Đọc tên joint và cờ đảo chiều
        self.names = [str(x) for x in self.get_parameter('wheel_joint_names').value]
        self.invert = [bool(x) for x in self.get_parameter('invert_wheels').value]

        # =====================================================================
        # ✅ SỬA LỖI MAJOR #8: CỜ CHỐNG SPAM ZERO
        # =====================================================================
        # LỖI CŨ: Gửi "V 0 0 0" liên tục mỗi 20ms khi timeout
        #         → Serial buffer STM32 bị tràn, robot lag
        # 
        # SỬA: Dùng flag để chỉ gửi zero MỘT LẦN khi timeout
        #      Sau đó không gửi gì nữa cho đến khi có cmd mới
        # 
        # Workflow:
        #   1. Nhận cmd_vel → reset flag → gửi bình thường
        #   2. Timeout → gửi zero MỘT LẦN → set flag
        #   3. Vẫn timeout → không gửi gì (flag = True)
        #   4. Nhận cmd_vel mới → reset flag → quay lại bước 1
        # =====================================================================
        self._already_sent_zero = False  # Flag: Đã gửi zero chưa?
        
        # =====================================================================
        # KẾT NỐI SERIAL
        # =====================================================================
        
        self.get_logger().info(f"Đang mở cổng serial: {port} @ {baud} baud")
        
        try:
            # Mở cổng serial
            # timeout=0.02: Đọc non-blocking, chờ tối đa 20ms
            self.ser = serial.Serial(
                port=port, 
                baudrate=baud, 
                timeout=0.02
            )
            
            # Đợi 200ms cho STM32 reset sau khi mở serial
            # Một số board STM32 tự reset khi DTR toggle
            time.sleep(0.2)
            
            self.get_logger().info("✅ Đã kết nối Serial thành công!")
            
        except serial.SerialException as e:
            self.get_logger().error(f"❌ Không thể mở cổng serial {port}")
            self.get_logger().error(f"   Lỗi: {e}")
            self.get_logger().error(f"   Kiểm tra:")
            self.get_logger().error(f"   1. STM32 đã cắm USB chưa?")
            self.get_logger().error(f"   2. Cổng đúng chưa: ls -l /dev/ttyACM*")
            self.get_logger().error(f"   3. Có quyền truy cập: sudo chmod 666 {port}")
            raise
            
        except Exception as e:
            self.get_logger().error(f"❌ Lỗi không mong đợi: {e}")
            raise

        # =====================================================================
        # PUBLISHERS - Xuất bản dữ liệu
        # =====================================================================
        
        # Xuất bản dòng encoder thô (debug)
        # Topic: /enc/line (std_msgs/String)
        # Nội dung: "ENC ms=12345 T=1,2,3,4 D=5,6,7,8"
        self.enc_line_pub = self.create_publisher(String, 'enc/line', 10)
        
        # Xuất bản encoder total ticks (debug)
        # Topic: /enc/total (std_msgs/Int32MultiArray)
        # Nội dung: [T_FL, T_FR, T_RR, T_RL] - Tổng ticks từ lúc bật nguồn
        self.enc_total_pub = self.create_publisher(Int32MultiArray, 'enc/total', 10)
        
        # Xuất bản encoder delta ticks (debug)
        # Topic: /enc/delta (std_msgs/Int32MultiArray)
        # Nội dung: [D_FL, D_FR, D_RR, D_RL] - Ticks thay đổi từ lần đọc trước
        self.enc_delta_pub = self.create_publisher(Int32MultiArray, 'enc/delta', 10)
        
        # Xuất bản trạng thái khớp bánh xe (QUAN TRỌNG!)
        # Topic: /joint_states (sensor_msgs/JointState)
        # Dùng bởi robot_state_publisher để cập nhật TF tree
        self.js_pub = self.create_publisher(JointState, 'joint_states', 20)

        # =====================================================================
        # SUBSCRIBER & TIMERS
        # =====================================================================
        
        # Thời điểm nhận lệnh cmd_vel cuối cùng
        self.last_cmd_time = self.get_clock().now()
        
        # Lệnh vận tốc hiện tại (vx, vy, wz)
        self.last_cmd = (0.0, 0.0, 0.0)
        
        # Subscribe topic /cmd_vel từ Nav2/teleop
        # QoS: 10 (hàng đợi 10 message)
        self.create_subscription(Twist, 'cmd_vel', self._on_cmd, 10)
        
        # Timer gửi lệnh vận tốc định kỳ
        # Period = max(2ms, 1/rate_hz)
        # Ví dụ: rate=50Hz → timer mỗi 20ms
        self.create_timer(max(0.002, 1.0/self.rate), self._tick)

        # =====================================================================
        # THREAD NHẬN DỮ LIỆU (RX)
        # =====================================================================
        
        # Event để dừng thread
        self._stop = threading.Event()
        
        # Tạo thread daemon đọc Serial
        # daemon=True: Thread tự tắt khi chương trình chính tắt
        self._thr = threading.Thread(target=self._rx_loop, daemon=True)
        self._thr.start()

        # =====================================================================
        # BIẾN CHO TÍNH TOÁN VELOCITY TỪ ENCODER
        # =====================================================================
        
        # Timestamp (ms) của lần đọc encoder trước
        # Dùng để tính dt (delta time) cho velocity
        self._last_ms = None
        
        # =====================================================================
        # LOG THÔNG TIN KHỞI ĐỘNG
        # =====================================================================
        
        self.get_logger().info("=" * 60)
        self.get_logger().info("✅ Velocity Bridge đã khởi động!")
        self.get_logger().info(f"   - Tần số gửi: {self.rate} Hz")
        self.get_logger().info(f"   - Timeout: {self.timeout_s} s")
        self.get_logger().info(f"   - Xung encoder/vòng: {tpr}")
        self.get_logger().info(f"   - Radian/tick: {self.rad_per_tick:.6f}")
        self.get_logger().info(f"   - Watchdog: {'BẬT' if self.zero_on_timeout else 'TẮT'}")
        self.get_logger().info("=" * 60)

    # =========================================================================
    # CLEANUP - Dọn dẹp khi tắt node
    # =========================================================================
    
    def destroy_node(self):
        """
        Hàm cleanup khi node bị hủy (Ctrl+C hoặc shutdown)
        
        Quy trình:
        ---------
        1. Set event để dừng thread RX
        2. Đợi thread kết thúc (tối đa 300ms)
        3. Đóng cổng serial
        4. Gọi destroy_node của parent class
        """
        self.get_logger().info("🛑 Đang tắt Velocity Bridge...")
        
        # Dừng thread RX
        self._stop.set()
        
        try:
            # Đợi thread kết thúc
            if self._thr.is_alive(): 
                self._thr.join(timeout=0.3)
        except Exception: 
            pass
        
        try: 
            # Đóng serial
            if hasattr(self, 'ser') and self.ser.is_open:
                self.ser.close()
                self.get_logger().info("🔌 Đã đóng cổng Serial")
        except Exception: 
            pass
        
        return super().destroy_node()

    # =========================================================================
    # TX - TRUYỀN LỆNH VẬN TỐC
    # =========================================================================
    
    def _on_cmd(self, msg: Twist):
        """
        Callback khi nhận message cmd_vel từ Nav2 hoặc teleop
        
        Args:
            msg (Twist): Message chứa vận tốc tuyến tính và góc
                        msg.linear.x: Vận tốc X (m/s) - tiến/lùi
                        msg.linear.y: Vận tốc Y (m/s) - sang trái/phải
                        msg.angular.z: Vận tốc góc Z (rad/s) - xoay
        
        Xử lý:
        -----
        1. Clamp (giới hạn) vận tốc trong khoảng cho phép
        2. Lưu lệnh vào self.last_cmd
        3. Cập nhật timestamp
        4. Reset flag _already_sent_zero nếu có chuyển động
        
        Ví dụ:
        ------
        Input: vx=1.5, vy=0.5, wz=0.3
        → Sau clamp: vx=1.0 (max_vx), vy=0.5, wz=0.3
        """
        # Lambda function để giới hạn giá trị trong [lo, hi]
        clamp = lambda v, lo, hi: lo if v < lo else hi if v > hi else v
        
        # Clamp các vận tốc trong giới hạn
        vx = clamp(msg.linear.x,  -self.max_vx, self.max_vx)
        vy = clamp(msg.linear.y,  -self.max_vy, self.max_vy)
        wz = clamp(msg.angular.z, -self.max_wz, self.max_wz)
        
        # Lưu lệnh mới
        self.last_cmd = (vx, vy, wz)
        self.last_cmd_time = self.get_clock().now()
        
        # =====================================================================
        # ✅ SỬA LỖI MAJOR #8: RESET FLAG KHI CÓ LỆNH CHUYỂN ĐỘNG MỚI
        # =====================================================================
        # Khi có lệnh chuyển động (khác 0) → reset flag
        # Điều này cho phép gửi zero lại nếu sau này timeout
        # =====================================================================
        if vx != 0.0 or vy != 0.0 or wz != 0.0:
            self._already_sent_zero = False

    def _send_v(self, vx, vy, wz):
        """
        Gửi lệnh vận tốc tới STM32 qua Serial
        
        Args:
            vx (float): Vận tốc X (m/s)
            vy (float): Vận tốc Y (m/s)
            wz (float): Vận tốc góc Z (rad/s)
        
        Protocol:
        --------
        Format: "V vx vy wz\n"
        - Prefix: "V " (chữ V + khoảng trắng)
        - vx, vy, wz: Float với 3 chữ số thập phân
        - Suffix: "\n" (newline)
        
        Ví dụ:
        ------
        _send_v(0.5, 0.0, 0.785)
        → Gửi: "V 0.500 0.000 0.785\n"
        
        Logging:
        -------
        - Chỉ log khi echo_tx=True
        - Có thể lọc theo tx_log_on_change_only
        - Có thể log định kỳ theo tx_keepalive_sec
        """
        # Tạo chuỗi lệnh theo protocol "V vx vy wz\n"
        # :.3f = Float với 3 chữ số thập phân
        line = f"V {vx:.3f} {vy:.3f} {wz:.3f}\n"
        
        try:
            # Gửi qua Serial (encode ASCII)
            self.ser.write(line.encode('ascii'))
            
            # ================================================================
            # LOGGING - Kiểm soát log để tránh spam
            # ================================================================
            if self.echo_tx:
                do_log = True  # Mặc định log
                
                # Kiểm tra: Chỉ log khi thay đổi?
                if self.tx_log_on_change_only:
                    if self._last_logged_cmd is None:
                        # Lần đầu tiên → log
                        do_log = True
                    else:
                        # So sánh với lệnh đã log trước
                        lvx, lvy, lwz = self._last_logged_cmd
                        
                        # Tính độ thay đổi lớn nhất
                        dv = max(abs(vx - lvx), abs(vy - lvy))  # Delta vận tốc tuyến tính
                        dw = abs(wz - lwz)                       # Delta vận tốc góc
                        
                        # Log nếu thay đổi > ngưỡng
                        do_log = (dv > self.tx_log_eps_v) or (dw > self.tx_log_eps_w)
                
                # Kiểm tra: Keepalive (log định kỳ dù không đổi)
                if not do_log and self.tx_keepalive_sec > 0.0:
                    now = time.monotonic()
                    if (now - self._last_log_wall) >= self.tx_keepalive_sec:
                        do_log = True
                
                # Thực hiện log nếu cần
                if do_log:
                    self.get_logger().info(f"📤 TX: {line.strip()}")
                    self._last_logged_cmd = (vx, vy, wz)
                    self._last_log_wall = time.monotonic()
                    
        except serial.SerialException as e:
            self.get_logger().warn(f'❌ Lỗi ghi Serial: {e}')
        except Exception as e:
            self.get_logger().warn(f'❌ Lỗi không mong đợi khi gửi: {e}')

    def _tick(self):
        """
        Timer callback - Gửi lệnh vận tốc định kỳ theo tần số rate_hz
        
        ✅ ĐÃ SỬA LỖI MAJOR #8: CHỐNG SPAM ZERO
        
        Logic hoạt động:
        ----------------
        1. Tính tuổi (age) của lệnh cuối = thời gian từ lần nhận cmd_vel cuối
        2. Nếu age > timeout:
           a. Nếu zero_on_timeout=True và chưa gửi zero:
              → Gửi zero MỘT LẦN
              → Set flag _already_sent_zero = True
           b. Nếu đã gửi zero rồi:
              → KHÔNG gửi gì nữa (tránh spam)
        3. Nếu age <= timeout:
           → Gửi lệnh bình thường (vx, vy, wz)
        
        Ví dụ timeline:
        --------------
        t=0ms:   Nhận cmd_vel(vx=0.5) → flag=False
        t=20ms:  Timer → age=20ms < 200ms → gửi "V 0.5 0 0"
        t=40ms:  Timer → age=40ms < 200ms → gửi "V 0.5 0 0"
        ...
        t=220ms: Timer → age=220ms > 200ms → gửi "V 0 0 0" MỘT LẦN → flag=True
        t=240ms: Timer → age=240ms > 200ms → KHÔNG gửi gì (flag=True)
        t=260ms: Timer → age=260ms > 200ms → KHÔNG gửi gì (flag=True)
        ...
        t=500ms: Nhận cmd_vel(vx=0.3) → flag=False → gửi lại bình thường
        """
        # Tính tuổi của lệnh cuối cùng
        now = self.get_clock().now()
        age = (now - self.last_cmd_time).nanoseconds * 1e-9  # Chuyển ns → s
        
        # =====================================================================
        # ✅ LOGIC MỚI: CHỈ GỬI ZERO MỘT LẦN KHI TIMEOUT
        # =====================================================================
        # So sánh với logic cũ (SPAM):
        # 
        # TRƯỚC (SAI):
        #   if timeout:
        #       send_zero()  # Gửi liên tục mỗi 20ms!
        # 
        # SAU (ĐÚNG):
        #   if timeout AND chưa_gửi_zero:
        #       send_zero()  # Chỉ gửi MỘT LẦN
        #       đánh_dấu_đã_gửi()
        # =====================================================================
        
        if self.zero_on_timeout and age > self.timeout_s:
            # Timeout! Không còn nhận cmd_vel mới
            
            # Chỉ gửi zero nếu CHƯA gửi lần nào
            if not self._already_sent_zero:
                # Gửi zero để dừng robot
                self._send_v(0.0, 0.0, 0.0)
                
                # Đánh dấu đã gửi
                self._already_sent_zero = True
                
                # Log cảnh báo
                self.get_logger().info(
                    f"⚠️ Cmd timeout ({age:.2f}s) - đã gửi ZERO một lần"
                )
            
            # Nếu đã gửi zero rồi → KHÔNG làm gì cả
            # (Không gửi lại zero, tránh spam)
            
        else:
            # Có lệnh trong thời hạn → gửi bình thường
            vx, vy, wz = self.last_cmd
            self._send_v(vx, vy, wz)

    # =========================================================================
    # RX - NHẬN DỮ LIỆU ENCODER
    # =========================================================================
    
    def _rx_loop(self):
        """
        Thread đọc dữ liệu encoder từ STM32
        
        Chạy trong thread riêng để không chặn main thread.
        Đọc liên tục từ Serial, parse dòng encoder, tính JointState.
        
        Protocol nhận:
        -------------
        Format: "ENC ms=12345 T=100,200,300,400 D=5,6,7,8\n"
        - ms: Timestamp từ STM32 (milliseconds)
        - T: Total ticks [FL, FR, RR, RL] - Tổng tích lũy từ lúc bật
        - D: Delta ticks [FL, FR, RR, RL] - Thay đổi từ lần đọc trước
        
        Xử lý:
        ------
        1. Đọc bytes từ Serial vào buffer
        2. Tìm dòng hoàn chỉnh (có '\n')
        3. Decode và parse bằng regex
        4. Publish dữ liệu thô (debug)
        5. Tính position và velocity từ encoder
        6. Publish JointState cho robot_state_publisher
        
        JointState calculation:
        ----------------------
        Position (rad): 
          pos = total_ticks × (2π / ticks_per_rev)
        
        Velocity (rad/s):
          vel = (delta_ticks / dt) × (2π / ticks_per_rev)
          dt = (ms - last_ms) / 1000
        
        Ví dụ:
        ------
        Input: "ENC ms=1000 T=6864,6864,6864,6864 D=686,686,686,686"
        → T=6864 ticks = 1 vòng = 2π rad = 6.28 rad
        → dt=0.1s, D=686 ticks = 0.1 vòng
        → vel = 686 / 6864 × 2π / 0.1 = 6.28 rad/s = 1 vòng/s
        """
        # Buffer lưu trữ bytes chưa xử lý
        buf = b''
        
        # Vòng lặp chính của thread
        while not self._stop.is_set():
            try:
                # ============================================================
                # BƯỚC 1: ĐỌC TỪ SERIAL
                # ============================================================
                # Đọc tối đa 256 bytes từ Serial
                # timeout=0.02s (set khi mở Serial)
                chunk = self.ser.read(256)
                buf += chunk
                
                # Nếu buffer chưa có dòng hoàn chỉnh → đợi thêm
                if b'\n' not in buf:
                    time.sleep(0.002)  # Ngủ 2ms để không spam CPU
                    continue
                
                # ============================================================
                # BƯỚC 2: TÁCH DÒNG
                # ============================================================
                # Split buffer thành các dòng
                # parts[-1] là phần dư chưa có '\n'
                parts = buf.split(b'\n')
                buf = parts[-1]  # Giữ lại phần dư
                
                # ============================================================
                # BƯỚC 3: XỬ LÝ TỪNG DÒNG
                # ============================================================
                for raw in parts[:-1]:  # Bỏ phần tử cuối (phần dư)
                    # Decode bytes → string
                    # errors='ignore': Bỏ qua bytes không hợp lệ
                    line = raw.decode(errors='ignore').strip()
                    
                    if not line:  # Dòng trống → skip
                        continue
                    
                    # Log dòng nhận được (nếu echo_rx=True)
                    if self.echo_rx: 
                        self.get_logger().info(f"📥 RX: {line}")
                    
                    # Publish dòng thô (cho debug)
                    self.enc_line_pub.publish(String(data=line))

                    # ========================================================
                    # BƯỚC 4: PARSE ENCODER DATA
                    # ========================================================
                    # Dùng regex để extract các số
                    m = ENC_PAT.match(line)
                    if not m:  # Không khớp pattern → skip
                        continue
                    
                    # Extract timestamp (ms)
                    ms = int(m.group(1))
                    
                    # Extract Total ticks (4 bánh)
                    # Groups 2-5: T values
                    T = [int(m.group(i)) for i in range(2, 6)]
                    # T = [T_FL, T_FR, T_RR, T_RL]
                    
                    # Extract Delta ticks (4 bánh)
                    # Groups 6-9: D values
                    d = [int(m.group(i)) for i in range(6, 10)]
                    # d = [D_FL, D_FR, D_RR, D_RL]
                    
                    


                    # ========================================================
                    # BƯỚC 5: PUBLISH DỮ LIỆU THÔ (DEBUG)
                    # ========================================================
                    # Publish total ticks
                    self.enc_total_pub.publish(Int32MultiArray(data=T))
                    
                    # Publish delta ticks
                    self.enc_delta_pub.publish(Int32MultiArray(data=d))
                    
                    # ========================================================
                    # BƯỚC 6: TÍNH JOINT STATE
                    # ========================================================
                    
                    # ------ Position từ total ticks ------
                    # Công thức: position (rad) = ticks × (2π / ticks_per_rev)
                    pos = [t * self.rad_per_tick for t in T]
                    # pos = [pos_FL, pos_FR, pos_RR, pos_RL] (rad)
                    
                    # ------ Velocity từ delta ticks ------
                    if self._last_ms is not None:
                        # Tính delta time (s)
                        dt = (ms - self._last_ms) / 1000.0
                        
                        # Kiểm tra dt hợp lệ (0 < dt < 0.5s)
                        # dt < 0: Timestamp bị lỗi hoặc STM32 reset
                        # dt > 0.5s: Quá lâu, có thể bị đứng
                        if 0.0 < dt < 0.5:
                            # Công thức: velocity = (delta_ticks / dt) × (2π / tpr)
                            vel = [di * self.rad_per_tick / dt for di in d]
                        else:
                            # dt không hợp lệ → velocity = 0
                            vel = [0.0] * 4
                    else:
                        # Lần đầu tiên → chưa có dt
                        vel = [0.0] * 4
                    
                    # Cập nhật timestamp cho lần sau
                    self._last_ms = ms
                    
                    # ------ Invert bánh xe nếu cần ------
                    # Dùng list comprehension với điều kiện
                    # Nếu invert[i]=True → đảo dấu
                    pos = [(-p if inv else p) for p, inv in zip(pos, self.invert)]
                    vel = [(-v if inv else v) for v, inv in zip(vel, self.invert)]
                    
                    # ========================================================
                    # BƯỚC 7: PUBLISH JOINT STATE
                    # ========================================================
                    # Tạo message JointState
                    js = JointState()
                    
                    # Header với timestamp
                    js.header.stamp = self.get_clock().now().to_msg()
                    
                    # ✅ SỬA LỖI CRITICAL: js.n → js.name
                    # LỖI CŨ: js.n = self.names  # ❌ Không có thuộc tính .n
                    # SỬA: js.name = self.names  # ✅ Thuộc tính đúng
                    js.name = self.names
                    
                    # Position và velocity
                    js.position = pos  # [rad, rad, rad, rad]
                    js.velocity = vel  # [rad/s, rad/s, rad/s, rad/s]
                    
                    # Publish
                    self.js_pub.publish(js)
                    
            except serial.SerialException as e:
                # Lỗi Serial (mất kết nối, timeout)
                self.get_logger().error(
                    f'❌ Lỗi Serial trong RX: {e}',
                    throttle_duration_sec=5.0
                )
                time.sleep(0.1)
                
            except Exception as e:
                # Lỗi khác (parse, decode, ...)
                self.get_logger().debug(
                    f'⚠️ Lỗi RX (non-critical): {e}',
                    throttle_duration_sec=5.0
                )
                time.sleep(0.01)


def main():
    """
    Hàm main - Điểm khởi động chương trình
    
    Quy trình:
    ---------
    1. Khởi tạo ROS2
    2. Tạo VelocityBridgeVfmt node
    3. Spin (chạy vòng lặp ROS2)
    4. Xử lý Ctrl+C
    5. Cleanup và shutdown
    """
    # Khởi tạo ROS2
    rclpy.init()
    
    try:
        # Tạo node
        node = VelocityBridgeVfmt()
        
        # Chạy node (vòng lặp xử lý callbacks)
        rclpy.spin(node)
        
    except KeyboardInterrupt:
        # Người dùng nhấn Ctrl+C
        print("\n⚠️ Đã nhận Ctrl+C, đang tắt...")
        
    except Exception as e:
        # Lỗi không mong đợi
        print(f"❌ Lỗi nghiêm trọng: {e}")
        import traceback
        traceback.print_exc()
        
    finally:
        # Cleanup
        if 'node' in locals():
            node.destroy_node()
        
        rclpy.shutdown()
        print("✅ Đã tắt Velocity Bridge")


if __name__ == '__main__':
    main()


# ==============================================================================
# HƯỚNG DẪN SỬ DỤNG VÀ KHẮC PHỤC SỰ CỐ
# ==============================================================================
"""
═══════════════════════════════════════════════════════════════════════════════
CÁCH CHẠY NODE
═══════════════════════════════════════════════════════════════════════════════

1. CHẠY RIÊNG (Standalone):
   
   ros2 run mecanum_robot_bringup velocity_bridge.py \\
       --ros-args \\
       -p serial_port:=/dev/ttyACM0 \\
       -p baud:=115200 \\
       -p rate_hz:=50.0 \\
       -p ticks_per_rev:=6864.0

2. CHẠY TRONG LAUNCH FILE:
   
   ros2 launch mecanum_robot_bringup robot_bringup.launch.py

3. KIỂM TRA HOẠT ĐỘNG:
   
   # Xem topics
   ros2 topic list | grep -E "cmd_vel|joint_states|enc"
   
   # Kiểm tra tần số
   ros2 topic hz /joint_states
   
   # Xem dữ liệu encoder
   ros2 topic echo /enc/line
   
   # Test gửi lệnh
   ros2 topic pub /cmd_vel geometry_msgs/Twist \\
       "{linear: {x: 0.2, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}"

═══════════════════════════════════════════════════════════════════════════════
KHẮC PHỤC SỰ CỐ
═══════════════════════════════════════════════════════════════════════════════

❌ LỖI 1: "Không thể mở cổng serial /dev/ttyACM0"
   Nguyên nhân:
   - STM32 chưa kết nối
   - Cổng USB sai
   - Không có quyền truy cập
   
   Giải pháp:
   ls -l /dev/ttyACM*                    # Kiểm tra cổng có tồn tại
   sudo chmod 666 /dev/ttyACM0           # Cấp quyền tạm thời
   sudo usermod -aG dialout $USER        # Cấp quyền vĩnh viễn (cần logout)

❌ LỖI 2: "Robot không chuyển động khi publish cmd_vel"
   Nguyên nhân:
   - STM32 firmware không nhận lệnh
   - Baudrate không khớp
   - Protocol không đúng
   
   Giải pháp:
   # Kiểm tra STM32 có nhận dữ liệu không:
   ros2 topic echo /enc/line
   # Nếu thấy "ENC ..." → STM32 hoạt động
   
   # Kiểm tra lệnh gửi đi:
   ros2 param set /velocity_bridge echo_tx true
   # Phải thấy "TX: V ..." trong log

❌ LỖI 3: "/joint_states không publish"
   Nguyên nhân:
   - STM32 không gửi dữ liệu encoder
   - Regex parse sai
   - Lỗi js.name (đã sửa)
   
   Giải pháp:
   # Kiểm tra dòng encoder thô:
   ros2 topic echo /enc/line
   # Phải thấy: "ENC ms=... T=... D=..."
   
   # Nếu không thấy → STM32 firmware có vấn đề
   # Nếu thấy nhưng không có /joint_states → check log node

❌ LỖI 4: "Bánh xe quay ngược chiều"
   Nguyên nhân:
   - Motor hoặc encoder đấu ngược
   - Cần invert bánh đó
   
   Giải pháp:
   # Sửa parameter invert_wheels:
   # [FL, FR, RR, RL] - True = đảo, False = giữ nguyên
   ros2 param set /velocity_bridge invert_wheels "[true, false, true, false]"
   
   # Hoặc sửa trong launch file

❌ LỖI 5: "Velocity tính toán sai"
   Nguyên nhân:
   - ticks_per_rev sai
   - dt tính sai
   - Delta ticks sai
   
   Giải pháp:
   # Đo chính xác ticks_per_rev:
   # 1. Đặt robot lên giá đỡ (bánh không chạm đất)
   # 2. Cho bánh quay CHẬM đúng 1 vòng
   # 3. Đếm total ticks thay đổi bao nhiêu
   # 4. Đó chính là ticks_per_rev
   
   ros2 topic echo /enc/total
   # Cho bánh quay 1 vòng, xem ticks tăng bao nhiêu

❌ LỖI 6: "Robot timeout liên tục"
   Nguyên nhân:
   - cmd_vel không được gửi
   - Timeout quá ngắn (200ms)
   
   Giải pháp:
   # Tăng timeout:
   ros2 param set /velocity_bridge cmd_timeout_ms 500
   
   # Hoặc tắt watchdog:
   ros2 param set /velocity_bridge zero_on_timeout false

═══════════════════════════════════════════════════════════════════════════════
KIỂM TRA VÀ XÁC NHẬN
═══════════════════════════════════════════════════════════════════════════════

✅ CHECKLIST:

□ 1. Kết nối phần cứng:
     - STM32 cắm USB
     - Motor có nguồn
     - Encoder nối đúng

□ 2. Kiểm tra cổng Serial:
     ls -l /dev/ttyACM*
     # Phải thấy ttyACM0 hoặc ttyACM1

□ 3. Kiểm tra quyền:
     groups | grep dialout
     # Nếu không có → sudo usermod -aG dialout $USER

□ 4. Chạy node:
     ros2 run mecanum_robot_bringup velocity_bridge.py

□ 5. Kiểm tra topics:
     ros2 topic hz /joint_states
     # Phải thấy tần số ổn định

□ 6. Test điều khiển:
     ros2 topic pub /cmd_vel geometry_msgs/Twist ...
     # Robot phải chuyển động

□ 7. Kiểm tra encoder:
     ros2 topic echo /enc/total
     # Ticks phải tăng khi bánh quay

═══════════════════════════════════════════════════════════════════════════════
THÔNG TIN THÊM
═══════════════════════════════════════════════════════════════════════════════

📖 Protocol chi tiết:
    TX: "V vx vy wz\n" - ASCII string
    RX: "ENC ms=... T=... D=...\n" - ASCII string

🔧 Baudrate phổ biến:
    - STM32: 115200 hoặc 921600
    - Arduino: 9600 hoặc 115200

📊 Encoder resolution:
    - Motor DC thường: 11-64 xung/vòng (motor shaft)
    - Sau hộp số: × gear_ratio
    - Ví dụ: 11 × 62.4 × 10 = 6864 xung/vòng bánh

⚡ Optimization:
    - rate_hz: 50Hz đủ cho hầu hết ứng dụng
    - Tăng lên 100Hz nếu cần mượt hơn (tốn CPU)
    - Giảm xuống 20Hz nếu CPU yếu

═══════════════════════════════════════════════════════════════════════════════
"""