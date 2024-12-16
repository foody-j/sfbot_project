import os
import can
import time

# CAN 설정
can_interface = 'can0'
bus = None

# 제어 모드 정의
CAN_PACKET_SET_ORIGIN = 5  # 원점 설정 모드

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
    """
    CAN 메시지를 수신하고 처리합니다.
    """
    try:
        msg = bus.recv(timeout=1)
        if msg is not None:
            print(f"Received message: ID=0x{msg.arbitration_id:X}, Data={msg.data}")

            # 응답 메시지 처리
            if msg.arbitration_id == 0x2901:
                print(f"[DEBUG] Received response for Set Origin Mode: {msg.data}")
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
