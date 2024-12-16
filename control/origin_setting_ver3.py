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

# 각 모터별 원점 설정 상태
motor_origin_set = [False] * 6
# 모터 제어 명령 저장
motor_commands = {
    'mode': 'idle',  # 'idle', 'speed', 'position' 중 하나
    'target_speed': 0,
    'target_position': 0,
    'target_acceleration': 0,
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
def set_motor_idle():
    """모터를 idle 상태로 설정"""
    global motor_commands
    motor_commands['mode'] = 'idle'
    motor_commands['target_speed'] = 0
    motor_commands['target_position'] = 0
    motor_commands['target_acceleration'] = 0

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
        # idle 모드에서는 명령을 보내지 않음
        time.sleep(0.1)

def read_can_message_thread():
    """CAN 데이터를 계속 읽는 쓰레드 함수."""
    global bus, stop_thread, motor_origin_set

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

def check_motor_communication():
    """
    6개의 모터와의 통신 상태를 확인하는 함수
    Returns:
        bool: 모든 모터와의 통신이 정상이면 True, 아니면 False
    """
    global motor_positions, motor_speeds, motor_currents, motor_temps, motor_errors
    
    try:
        communication_timeout = 2.0  # 2초 타임아웃
        start_time = time.time()
        #motor_responded = [False] * 6
        motor_responded = [False]
        
        while time.time() - start_time < communication_timeout:
            # 각 모터의 응답 확인
            for i in range(1):
                if (motor_speeds[i] != 0.0 or 
                    motor_currents[i] != 0.0 or 
                    motor_temps[i] != 0 or 
                    motor_positions[i] != 0.0):
                    motor_responded[i] = True
            
            # 모든 모터가 응답했는지 확인
            if all(motor_responded):
                print("[SUCCESS] All motors are communicating properly")
                return True
                
            time.sleep(0.1)
        
        # 타임아웃 시 응답하지 않은 모터 출력
        for i in range(1):
            if not motor_responded[i]:
                print(f"[ERROR] Motor {i+1} is not responding")
        return False
        
    except Exception as e:
        print(f"[ERROR] Communication check failed: {e}")
        return False
def initialize_motor_origin():
    """
    모터의 원점을 설정하는 함수
    Returns:
        bool: 원점 설정 성공 여부
    """
    global motor_origin_set, motor_commands
    try:

        print("[INFO] Starting origin initialization for motor 1")
        
        # 1. 원점 설정 모드 활성화
        set_origin_mode(1, 0)
        time.sleep(0.5)
            
        # 2. 저속으로 원점 방향으로 이동
        set_speed_loop_mode(1, -100)  # 저속으로 역방향 회전
        # 3. 원점 감지 대기 (전류값 모니터링)
        timeout = time.time() + 20  # 10초 타임아웃
        while time.time() < timeout:
            if motor_currents[0] > 1.5:  # 전류 임계값 감지
                print(f"[INFO] Origin detected for motor 1, Current: {motor_currents[0]}")
                set_speed_loop_mode(1, 0)  # 모터 정지
                set_origin_mode(1, 0)  # 원점 위치 저장
                set_position_speed_loop(1,0,50,50)
                motor_origin_set[0] = True
                return True
            time.sleep(0.1)
        print("[ERROR] Origin initialization timeout")
        return False
        
    except Exception as e:
        print(f"[ERROR] Origin initialization failed: {e}")
        return False
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
        # 초기 상태는 idle
        set_motor_idle()
        # 모터 통신 상태 확인
        print("Checking motor communication...")
       
        if not check_motor_communication():
            print("[ERROR] Motor communication check failed")
            raise Exception("Motor communication error")
        print("Motor communication check completed")

        # 1번 모터 원점 초기화
        print("Initializing motor 1 origin...")
        if not initialize_motor_origin():
            print("[ERROR] Motor 1 origin initialization failed")
            raise Exception("Motor origin initialization error")
        print("Motor 1 origin initialization completed")

        while True:
            #set_position_speed_loop(1,-10,500,500)
            time.sleep(1)
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