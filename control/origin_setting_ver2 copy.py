import os
import can
import time
import struct
import threading

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
CAN_PACKET_SET_POS_SPD = 6  # Position-Speed Loop Mode

# 쓰레드 종료 플래그
stop_thread = False

# 원점 설정 상태
origin_set = False

# 모터 제어 명령 저장
motor_commands = {
    'mode': None,  # 현재 제어 모드
    'target_speed': 0,  # 목표 속도
    'target_position': 0,  # 목표 위치
    'target_acceleration': 0,  # 목표 가속도
}

def setup_can_interface(interface='can0', bitrate=1000000):
    """CAN 인터페이스를 초기화하고 설정합니다."""
    try:
        os.system(f"sudo ip link set {interface} down")
        os.system(f"sudo ip link set {interface} up type can bitrate {bitrate}")
        print(f"CAN interface {interface} is set with bitrate {bitrate}.")
    except Exception as e:
        print(f"Failed to setup CAN interface: {e}")
        exit(1)

def send_can_message(can_id, data):
    """CAN 메시지를 송신하는 함수."""
    global bus
    msg = can.Message(arbitration_id=can_id, data=data, is_extended_id=True)
    try:
        bus.send(msg)
        print(f"[DEBUG] Sent Message: ID=0x{can_id:X}, Data={data}")
    except can.CanError as e:
        print(f"CAN send error: {e}")

def set_origin_mode(can_id, set_origin_mode):
    """Set Origin Mode 설정 함수."""
    data = [set_origin_mode]
    can_id_with_mode = (CAN_PACKET_SET_ORIGIN << 8) | can_id
    print(f"[DEBUG] Sending Set Origin Mode: CAN ID=0x{can_id_with_mode:X}, Data={data}")
    send_can_message(can_id_with_mode, data)

def set_speed_loop_mode(can_id, speed):
    """Speed Loop Mode 설정 함수."""
    global motor_commands
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
    
    # 명령 저장
    motor_commands['mode'] = 'speed'
    motor_commands['target_speed'] = speed

def set_position_speed_loop(can_id, position, speed, acceleration):
    """Position-Speed Loop Mode 설정 함수."""
    global motor_commands
    position_int = int(position * 10000)
    speed_adjusted = int(speed / 10)
    acceleration_adjusted = int(acceleration / 10)

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

    can_id_with_mode = (CAN_PACKET_SET_POS_SPD << 8) | can_id
    print(f"[DEBUG] Sending Position-Speed Loop Mode: CAN ID=0x{can_id_with_mode:X}, Data={data}")
    send_can_message(can_id_with_mode, data)
    
    # 명령 저장
    motor_commands['mode'] = 'position'
    motor_commands['target_position'] = position
    motor_commands['target_speed'] = speed
    motor_commands['target_acceleration'] = acceleration

def command_sender_thread():
    """주기적으로 제어 명령을 전송하는 쓰레드 함수"""
    global stop_thread, motor_commands
    
    while not stop_thread:
        if motor_commands['mode'] == 'speed':
            for motor_id in motor_can_ids:
                set_speed_loop_mode(motor_id, motor_commands['target_speed'])
        elif motor_commands['mode'] == 'position':
            for motor_id in motor_can_ids:
                set_position_speed_loop(
                    motor_id,
                    motor_commands['target_position'],
                    motor_commands['target_speed'],
                    motor_commands['target_acceleration']
                )
        time.sleep(0.1)  # 100ms 주기로 명령 전송

def read_can_message_thread():
    """CAN 데이터를 계속 읽는 쓰레드 함수."""
    global bus, stop_thread, origin_set

    try:
        while not stop_thread:
            msg = bus.recv(timeout=1)
            if msg is not None:
                print(f"Received message: ID=0x{msg.arbitration_id:X}, Data={msg.data}")

                if len(msg.data) >= 7:
                    motor_id = msg.arbitration_id & 0xFF
                    if 1 <= motor_id <= 6:
                        motor_index = motor_id - 1
                        pos = int.from_bytes(msg.data[0:2], 'big', signed=True) * 0.1
                        speed = int.from_bytes(msg.data[2:4], 'big', signed=True) * 10.0
                        current = int.from_bytes(msg.data[4:6], 'big', signed=True) * 0.01
                        temp = msg.data[6]
                        error = msg.data[7] if len(msg.data) > 7 else 0

                        motor_positions[motor_index] = pos
                        motor_speeds[motor_index] = speed
                        motor_currents[motor_index] = current
                        motor_temps[motor_index] = temp
                        motor_errors[motor_index] = error

                        if current > 1.2 and not origin_set:
                            print(f"[ALERT] Current threshold exceeded! Motor ID={motor_id}, Current={current}")
                            set_origin_mode(motor_id, 0)
                            set_position_speed_loop(motor_id, 0, 500, 500)
                            origin_set = True

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

    setup_can_interface('can0', 1000000)

    try:
        bus = can.interface.Bus(channel='can0', interface='socketcan')
        print("CAN bus initialized.")

        # 데이터 읽기 쓰레드 시작
        stop_thread = False
        reader_thread = threading.Thread(target=read_can_message_thread, daemon=True)
        reader_thread.start()

        # 명령 전송 쓰레드 시작
        command_thread = threading.Thread(target=command_sender_thread, daemon=True)
        command_thread.start()

        # Speed Loop Mode 실행
        print("Setting Speed Loop Mode...")
        set_speed_loop_mode(1, 500)  # Motor 1에 500 RPM 설정

        # 메인 루프 실행
        while not stop_thread:
            if origin_set:
                print("[INFO] Origin has been set. Stopping main loop.")
            set_position_speed_loop(1,0,500,500)
            time.sleep(0.1)

        stop_thread = True
        reader_thread.join()
        command_thread.join()
        print("All threads stopped.")

    except Exception as e:
        print(f"Error: {e}")
    finally:
        if bus is not None:
            bus.shutdown()
            print("CAN interface properly shut down.")

if __name__ == "__main__":
    main()