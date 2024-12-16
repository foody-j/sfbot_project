import os
import can
import time
import struct  # 데이터를 CAN 메시지 형식으로 변환
import threading  # 쓰레딩을 위한 모듈


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
CAN_PACKET_SET_SPEED = 3  # Speed Loop Mode
CAN_PACKET_SET_ORIGIN = 5  # 원점 설정 모드
CAN_PACKET_SET_POS_SPD = 6     # Position-Speed Loop Mode

# 데이터 수신 쓰레드 종료 플래그
stop_thread = False

# 원점 설정 상태
origin_set = False

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

def set_speed_loop_mode(can_id, speed):
    """
    Speed Loop Mode 설정 함수.
    """
    speed_int = int(speed)
    data = [
        (speed_int >> 24) & 0xFF,
        (speed_int >> 16) & 0xFF,
        (speed_int >> 8) & 0xFF,
        speed_int & 0xFF,
    ]
    can_id_with_mode = (CAN_PACKET_SET_SPEED << 8) | can_id
    print(f"[DEBUG] Sending Speed Loop Mode: CAN ID=0x{can_id_with_mode:X}, Data={data}")
    send_can_message(can_id_with_mode, data)

def set_position_speed_loop(can_id, position, speed, acceleration):
    """
    Position-Speed Loop Mode 설정 함수.

    :param can_id: 모터 CAN ID (1~6)
    :param position: 설정할 위치 (float, 단위: degrees)
    :param speed: 설정할 속도 (int, 단위: 전기적 RPM)
    :param acceleration: 설정할 가속도 (int, 단위: RPM/s)
    """
    # 위치, 속도, 가속도를 정수 단위로 변환
    position_int = int(position * 10000)  # 0.0001° 단위
    speed_adjusted = int(speed / 10)      # 0.1 RPM 단위
    acceleration_adjusted = int(acceleration / 10)  # 0.1 RPM/s 단위

    # 데이터 포맷: [Position(32비트), Speed(16비트), Acceleration(16비트)]
    data = [
        (position_int >> 24) & 0xFF,
        (position_int >> 16) & 0xFF,
        (position_int >> 8) & 0xFF,
        position_int & 0xFF,
        (speed_adjusted >> 8) & 0xFF,
        speed_adjusted & 0xFF,
        (acceleration_adjusted >> 8) & 0xFF,
        acceleration_adjusted & 0xFF,
    ]

    # CAN ID와 Control Mode 결합
    can_id_with_mode = (CAN_PACKET_SET_POS_SPD << 8) | can_id

    # 디버그 출력
    print(f"[DEBUG] Sending Position-Speed Loop Mode: CAN ID=0x{can_id_with_mode:X}, Data={data}")

    # CAN 메시지 송신
    send_can_message(can_id_with_mode, data)

def read_can_message_thread():
    """
    CAN 데이터를 계속 읽는 쓰레드 함수.
    """
    global bus, stop_thread, origin_set

    try:
        while not stop_thread:
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
                        error = msg.data[7] if len(msg.data) > 7 else 0  # 에러 데이터

                        # 모터 상태 업데이트
                        motor_positions[motor_index] = pos
                        motor_speeds[motor_index] = speed
                        motor_currents[motor_index] = current
                        motor_temps[motor_index] = temp
                        motor_errors[motor_index] = error
                        # 전류값이 1.2 이상이고 아직 원점이 설정되지 않은 경우
                        if current > 1.2 and not origin_set:
                            print(f"[ALERT] Current threshold exceeded! Motor ID={motor_id}, Current={current}")
                            set_origin_mode(motor_id, 0)  # 현재 위치를 원점으로 설정
                            set_position_speed_loop(motor_id, 0, 500, 500)  # 위치를 0으로 고정
                            origin_set = True  # 원점이 설정되었음을 표시
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
    global bus, stop_thread

    # CAN 인터페이스 설정
    setup_can_interface('can0', 1000000)

    # CAN 버스 초기화
    try:
        bus = can.interface.Bus(channel='can0', interface='socketcan')  # interface로 수정
        print("CAN bus initialized.")
        # 데이터 읽기 쓰레드 시작
        stop_thread = False
        reader_thread = threading.Thread(target=read_can_message_thread, daemon=True)
        reader_thread.start()
        # Speed Loop Mode 실행
        print("Setting Speed Loop Mode...")
        set_speed_loop_mode(1, 500)  # Motor 1에 3000 RPM 설정

        # 메인 루프 실행
        while not stop_thread:
            if origin_set:
                print("[INFO] Origin has been set. Stopping main loop.")
                break
            time.sleep(0.1)

        stop_thread = True
        reader_thread.join()
        print("Reader thread stopped.")        
        


        # 쓰레드 종료
        stop_thread = True
        reader_thread.join()
        print("Reader thread stopped.")
    except Exception as e:
        print(f"Error: {e}")
    finally:
        if bus is not None:
            bus.shutdown()
            print("CAN interface properly shut down.")

if __name__ == "__main__":
    main()