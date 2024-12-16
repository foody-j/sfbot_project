import os
import can
import csv
import time
from datetime import datetime

# CAN 설정
can_interface = 'can0'
bus = None

# 모터 상태 데이터 저장용
motor_data = {
    1: {"position": 0.0, "speed": 0.0, "current": 0.0, "temperature": 0, "error": 0},
    2: {"position": 0.0, "speed": 0.0, "current": 0.0, "temperature": 0, "error": 0},
    3: {"position": 0.0, "speed": 0.0, "current": 0.0, "temperature": 0, "error": 0},
    4: {"position": 0.0, "speed": 0.0, "current": 0.0, "temperature": 0, "error": 0},
    5: {"position": 0.0, "speed": 0.0, "current": 0.0, "temperature": 0, "error": 0},
    6: {"position": 0.0, "speed": 0.0, "current": 0.0, "temperature": 0, "error": 0},
}

# CSV 파일 설정
csv_filename = datetime.now().strftime("%Y%m%d_%H%M") + "_motor_data.csv"

def setup_csv_file():
    """CSV 파일을 초기화합니다."""
    with open(csv_filename, mode="w", newline="") as file:
        writer = csv.writer(file)
        # CSV 헤더 작성
        writer.writerow(["Timestamp", "Motor ID", "Position", "Speed", "Current", "Temperature", "Error Code"])

def setup_can_interface(interface='can0', bitrate=1000000):
    """CAN 인터페이스를 초기화합니다."""
    try:
        os.system(f"sudo ip link set {interface} down")
        os.system(f"sudo ip link set {interface} up type can bitrate {bitrate}")
        print(f"CAN interface {interface} is set with bitrate {bitrate}.")
    except Exception as e:
        print(f"Failed to setup CAN interface: {e}")
        exit(1)

def read_can_message():
    """CAN 메시지를 수신하고 데이터를 업데이트합니다."""
    global motor_data

    try:
        msg = bus.recv(timeout=0.1)  # 100ms 타임아웃
        if msg is not None:
            motor_id = msg.arbitration_id & 0xFF
            if 1 <= motor_id <= 6:
                # CAN 데이터 파싱
                position = int.from_bytes(msg.data[0:2], "big", signed=True) * 0.1
                speed = int.from_bytes(msg.data[2:4], "big", signed=True) * 10.0
                current = int.from_bytes(msg.data[4:6], "big", signed=True) * 0.01
                temperature = msg.data[6]
                error = msg.data[7]

                # 데이터 업데이트
                motor_data[motor_id] = {
                    "position": position,
                    "speed": speed,
                    "current": current,
                    "temperature": temperature,
                    "error": error,
                }

                # 디버그 출력
                print(f"Motor ID: {motor_id} | Position: {position} | Speed: {speed} | "
                      f"Current: {current} | Temperature: {temperature} | Error: {error}")

                # 데이터 저장
                save_to_csv(motor_id, motor_data[motor_id])
    except can.CanError as e:
        print(f"CAN read error: {e}")

def save_to_csv(motor_id, data):
    """데이터를 CSV 파일에 저장합니다."""
    try:
        timestamp = datetime.now().strftime("%Y-%m-%d %H:%M:%S.%f")[:-3]  # 밀리초 포함한 타임스탬프
        with open(csv_filename, mode="a", newline="") as file:
            writer = csv.writer(file)
            # 데이터 저장
            writer.writerow([
                timestamp,
                motor_id,
                data["position"],
                data["speed"],
                data["current"],
                data["temperature"],
                data["error"],
            ])
    except Exception as e:
        print(f"Failed to write to CSV: {e}")

def main():
    global bus

    # CSV 파일 초기화
    setup_csv_file()

    # CAN 인터페이스 설정
    setup_can_interface('can0', 1000000)

    # CAN 버스 초기화
    try:
        bus = can.interface.Bus(channel=can_interface, interface='socketcan')
        print("CAN bus initialized.")

        # 데이터 수신 루프
        print("Monitoring CAN data...")
        while True:
            read_can_message()
            time.sleep(0.01)  # 10ms 간격으로 데이터 수신
    except Exception as e:
        print(f"Error: {e}")
    finally:
        if bus is not None:
            bus.shutdown()
            print("CAN interface properly shut down.")

if __name__ == "__main__":
    main()
