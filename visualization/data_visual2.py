import sys
import os
import can
import threading
from PySide6.QtWidgets import QApplication, QMainWindow, QVBoxLayout, QWidget, QTableWidget, QTableWidgetItem
from PySide6.QtCore import QTimer
from matplotlib.figure import Figure
from matplotlib.backends.backend_qtagg import FigureCanvasQTAgg as FigureCanvas

# CAN 설정
can_interface = 'can0'
bus = None
lock = threading.Lock()  # 데이터 동기화를 위한 Lock

# 모터 상태 데이터
motor_data = {1: 0.0, 2: 0.0, 3: 0.0, 4: 0.0, 5: 0.0, 6: 0.0}  # 모터 ID와 포지션 값 저장


class MotorDashboard(QMainWindow):
    def __init__(self):
        super().__init__()
        self.initUI()

        # 타이머 설정 (100ms마다 GUI 업데이트)
        self.timer = QTimer(self)
        self.timer.timeout.connect(self.update_ui)
        self.timer.start(100)

    def initUI(self):
        self.setWindowTitle('Motor State Visualization')
        self.setGeometry(100, 100, 800, 600)

        # 메인 레이아웃 설정
        main_layout = QVBoxLayout()

        # 테이블 위젯 설정
        self.table = QTableWidget(6, 2)  # 6개의 모터, 2개의 컬럼(ID, Position)
        self.table.setHorizontalHeaderLabels(["Motor ID", "Position"])
        for i in range(6):
            self.table.setItem(i, 0, QTableWidgetItem(str(i + 1)))
            self.table.setItem(i, 1, QTableWidgetItem("0.0"))
        main_layout.addWidget(self.table)

        # 그래프 설정
        self.figure = Figure()
        self.canvas = FigureCanvas(self.figure)
        main_layout.addWidget(self.canvas)

        # 중앙 위젯 설정
        central_widget = QWidget(self)
        central_widget.setLayout(main_layout)
        self.setCentralWidget(central_widget)

    def update_ui(self):
        """GUI 업데이트: 테이블 및 그래프."""
        global motor_data

        # 테이블 업데이트
        with lock:
            for motor_id, position in motor_data.items():
                self.table.setItem(motor_id - 1, 1, QTableWidgetItem(f"{position:.1f}"))

        # 그래프 업데이트
        self.update_plot()

    def update_plot(self):
        """그래프 업데이트."""
        global motor_data

        # 그래프 초기화
        self.figure.clear()

        # 모터 포지션 데이터 시각화
        ax = self.figure.add_subplot(111)
        motor_ids = list(motor_data.keys())
        positions = list(motor_data.values())

        ax.bar(motor_ids, positions, tick_label=[f"Motor {id}" for id in motor_ids])
        ax.set_xlabel("Motor ID")
        ax.set_ylabel("Position")
        ax.set_title("Motor Position Visualization")
        ax.set_ylim(-500, 500)  # 포지션 값 범위 조정 (필요에 따라 변경)

        # 캔버스 업데이트
        self.canvas.draw()


def setup_can_interface(interface='can0', bitrate=1000000):
    """
    CAN 인터페이스를 초기화하고 설정합니다.
    """
    try:
        os.system(f"sudo ip link set {interface} down")
        os.system(f"sudo ip link set {interface} up type can bitrate {bitrate}")
        print(f"CAN interface {interface} is set with bitrate {bitrate}.")
    except Exception as e:
        print(f"Failed to setup CAN interface: {e}")
        exit(1)


def can_receive_thread():
    """CAN 메시지 수신을 위한 스레드."""
    global motor_data
    try:
        while True:
            msg = bus.recv(timeout=0.01)  # 10ms 타임아웃
            if msg is not None:
                motor_id = msg.arbitration_id & 0xFF
                if 1 <= motor_id <= 6:
                    position = int.from_bytes(msg.data[0:2], 'big', signed=True) * 0.1
                    with lock:  # 동기화
                        motor_data[motor_id] = position
    except can.CanError as e:
        print(f"CAN read error: {e}")


def main():
    global bus

    # CAN 인터페이스 설정
    setup_can_interface('can0', 1000000)

    # CAN 버스 초기화
    try:
        bus = can.interface.Bus(channel=can_interface, interface='socketcan')
        print("CAN bus initialized.")

        # CAN 메시지 수신 스레드 시작
        threading.Thread(target=can_receive_thread, daemon=True).start()

        # GUI 초기화
        app = QApplication(sys.argv)
        dashboard = MotorDashboard()

        dashboard.show()
        sys.exit(app.exec())

    except Exception as e:
        print(f"Error: {e}")
    finally:
        if bus is not None:
            bus.shutdown()
            print("CAN interface properly shut down.")


if __name__ == "__main__":
    main()
