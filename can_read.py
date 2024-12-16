import can

def receive_can_data(channel='can0', bitrate=1000000):
    # CAN 버스 설정
    bus = can.interface.Bus(channel=channel, bustype='socketcan')

    print(f"Listening on {channel}...")
    try:
        while True:
            # 메시지 수신
            message = bus.recv(timeout=1.0)  # 1초 대기
            if message:
                print(f"Received: ID={message.arbitration_id:X}, Data={message.data}")
    except KeyboardInterrupt:
        print("Stopped listening.")
    finally:
        bus.shutdown()

if __name__ == "__main__":
    receive_can_data()
