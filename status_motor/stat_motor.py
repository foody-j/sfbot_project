import os
import can
import time
import struct  # 데이터를 CAN 메시지 형식으로 변환

# CAN 설정
can_interface = 'can0'
bus = None

# 모터 CAN ID 배열 (1~6)
motor_can_ids = [1, 2, 3, 4, 5, 6]
# 모터 상태 데이터 저장용
motor_positions = [0.0] * 6
motor_speeds = [0.0] * 6
motor_currents = [0.0] * 6
motor_temps = [0] * 6
motor_errors = [0] * 6
# 제어 모드 정의
CAN_PACKET_SET_ORIGIN = 5  # 원점 설정 모드

def setup_can_interface(interface='can0', bitrate=1000000):
    """
    CAN 인터페이스를 초기화하고 설정합니다.
    """
    try:
        # CAN 인터페이스 활성화
        os.system(f"sudo ip link set {interface} down")  # 기존 설정 초기화
        os.system(f"sudo ip link set {interface} up type can bitrate {bitrate}")
        print(f"CAN interface {interface} is set with bitrate {bitrate}.")
    except Exception as e:
        print(f"Failed to setup CAN interface: {e}")
        exit(1)
def send_can_message(can_id, data):
    """
    CAN 메시지를 송신하는 함수.
    """
    global bus
    msg = can.Message(arbitration_id=can_id, data=data, is_extended_id=True)
    try:
        bus.send(msg)
        print(f"[DEBUG] Sent Message: ID=0x{can_id:X}, Data={data}")
    except can.CanError as e:
        print(f"CAN send error: {e}")

def set_origin_mode(can_id, set_origin_mode):
    """
    Set Origin Mode 설정 함수.
    """
    # 원점 설정 데이터 (1바이트)
    data = [set_origin_mode]

    # CAN ID와 Control Mode 결합
    can_id_with_mode = (CAN_PACKET_SET_ORIGIN << 8) | can_id

    # 디버그 출력
    print(f"[DEBUG] Sending Set Origin Mode: CAN ID=0x{can_id_with_mode:X}, Data={data}")

    # CAN 메시지 송신
    send_can_message(can_id_with_mode, data)

def read_can_message(bus):
    try:
        msg = bus.recv(timeout=1)  # 메시지 수신
        if msg is not None:
            print(f"Received message: ID=0x{msg.arbitration_id:X}, Data={msg.data}")

            # 데이터 길이 확인
            if len(msg.data) >= 7:  # 최소 7바이트인지 확인
                motor_id = msg.arbitration_id & 0xFF
                if 1 <= motor_id <= 6:
                    motor_index = motor_id - 1
                    pos = int.from_bytes(msg.data[0:2], 'big', signed=True) * 0.1
                    speed = int.from_bytes(msg.data[2:4], 'big', signed=True) * 10.0
                    current = int.from_bytes(msg.data[4:6], 'big', signed=True) * 0.01
                    temp = msg.data[6]  # 온도 데이터
                    error = msg.data[7] if len(msg.data) > 7 else 0  # 에러 데이터 (7바이트 초과 시 접근)

                    # 모터 상태 업데이트
                    motor_positions[motor_index] = pos
                    motor_speeds[motor_index] = speed
                    motor_currents[motor_index] = current
                    motor_temps[motor_index] = temp
                    motor_errors[motor_index] = error

                    # 디버그 출력
                    print(f"Motor ID: {motor_id} | Position: {pos} | Speed: {speed} | "
                          f"Current: {current} | Temp: {temp} | Error: {error}")
                else:
                    print(f"Invalid Motor ID: {motor_id}")
            else:
                print("Received message with insufficient data length.")
        else:
            print("No CAN message received.")
    except can.CanError as e:
        print(f"CAN read error: {e}")


def main():
    global bus

    # CAN 인터페이스 설정
    setup_can_interface('can0', 1000000)

    # CAN 버스 초기화
    try:
        bus = can.interface.Bus(channel='can0', interface='socketcan')  # interface로 수정
        print("CAN bus initialized.")
        # Set Origin Mode 테스트
        print("Sending Set Origin Mode command...")
        set_origin_mode(1, 0)  # CAN ID 1에 일시적 원점 설정
        # CAN 데이터 읽기 루프
        print("Monitoring CAN data...")
        while True:
            read_can_message(bus)
            time.sleep(0.001)
    except Exception as e:
        print(f"Error: {e}")
    finally:
        if bus is not None:
            bus.shutdown()
            print("CAN interface properly shut down.")

if __name__ == "__main__":
    main()