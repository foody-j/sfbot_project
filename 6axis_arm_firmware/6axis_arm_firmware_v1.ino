#include <SPI.h>
#include <mcp2515.h>


// 버튼 및 부저 핀 설정
#define BUZZER_PIN 3
// MCP2515 및 CAN 프레임 설정
struct can_frame canMsg;
MCP2515 mcp2515(10);
// 모터의 CAN ID 배열 (1~6)
const uint8_t motorCANIDs[6] = {1, 2, 3, 4, 5, 6};
// 각 모터의 현재 위치를 저장할 배열
float current_positions[6] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
// 컨트롤 모드
#define CAN_PACKET_SET_DUTY 0
#define CAN_PACKET_SET_CURRENT 1
#define CAN_PACKET_SET_CURRENT_BRAKE 2 // Current Break Mode
#define CAN_PACKET_SET_RPM 3 // Speed Mode
#define CAN_PACKET_SET_ORIGIN 5  // Set Origin Mode
#define CAN_PACKET_SET_POS_SPD 6 // Position-Speed Loop Mode

// 각 모터의 전류 및 속도 데이터를 저장할 배열
float motor_currents[6] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
float motor_positions[6] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
float motor_speeds[6] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
uint8_t motor_temps[6] = {0, 0, 0, 0, 0, 0};
uint8_t motor_errors[6] = {0, 0, 0, 0, 0, 0};

// Duty Cycle Mode 설정 함수
void setDutyCycleMode(uint8_t can_id, float duty) {
  uint8_t buffer[4];  // 4바이트의 데이터를 담는 버퍼
  
  // duty 값을 100,000 단위로 변환하여 int32_t로 처리 (0 ~ 1 범위)
  int32_t duty_int = (int32_t)(duty * 100000.0);  // 듀티 사이클을 0.00001 단위로 변환

  // 듀티 사이클 값의 범위를 제한 (0 ~ 0.95)
  if (duty_int < 0) duty_int = 0;           // 최소 0
  if (duty_int > 95000) duty_int = 95000;   // 최대 0.95 (95%)

  // 듀티 사이클(32비트) 설정
  buffer[0] = (duty_int >> 24) & 0xFF;
  buffer[1] = (duty_int >> 16) & 0xFF;
  buffer[2] = (duty_int >> 8) & 0xFF;
  buffer[3] = duty_int & 0xFF;

  // CAN 메시지 전송 (Control Mode 0: Duty Cycle Mode)
  uint32_t can_id_with_mode = (CAN_PACKET_SET_DUTY << 8) | can_id;
  comm_can_transmit_eid(can_id_with_mode, buffer, 4);

  // 메시지 전송 로그
  Serial.print("Duty Cycle Mode 메시지 전송 - CAN ID: 0x");
  Serial.println(can_id_with_mode, HEX);
  Serial.print(" 듀티 사이클: ");
  Serial.print(duty);
  Serial.println(" (0~1 범위)");
}

// Set Origin Mode 설정 함수
void setOriginMode(uint8_t can_id, uint8_t set_origin_mode) {
  uint8_t buffer[1];  // 1바이트의 데이터를 담는 버퍼
  buffer[0] = set_origin_mode;  // 원점 설정 모드 데이터 (0: 일시적 원점)

  // CAN 메시지 전송 (Control Mode 5: Set Origin Mode)
  uint32_t can_id_with_mode = (CAN_PACKET_SET_ORIGIN << 8) | can_id;

  comm_can_transmit_eid(can_id_with_mode, buffer, 1);

  // 메시지 전송 로그
  //Serial.print("Set Origin Mode 메시지 전송 - CAN ID: 0x");
  //Serial.println(can_id_with_mode, HEX);
}
// Position-Speed Loop 명령어 전송 함수
void positionSpeedLoop(uint8_t can_id, float position_deg, int16_t speed, int16_t acceleration) {
  uint8_t buffer[8];  // 8바이트의 데이터를 담는 버퍼

  // 포지션 값을 0.0001도 단위로 변환하여 처리 (-360000000 ~ 360000000 범위)
  int32_t position = (int32_t)(position_deg * 10000.0); // 0.0001° 단위로 변환
  // Speed와 Acceleration은 10으로 나누어 처리
  int16_t speed_adjusted = speed / 10;  // 스피드를 0.1 단위로 변환
  int16_t acceleration_adjusted = acceleration / 10;  // 가속도를 0.1 단위로 변환

  // Position(32비트) 설정 (4바이트)
  buffer[0] = (position >> 24) & 0xFF;
  buffer[1] = (position >> 16) & 0xFF;
  buffer[2] = (position >> 8) & 0xFF;
  buffer[3] = position & 0xFF;

  // Speed(16비트) 설정 (2바이트)
  buffer[4] = (speed_adjusted >> 8) & 0xFF;
  buffer[5] = speed_adjusted & 0xFF;

  // Acceleration(16비트) 설정 (2바이트)
  buffer[6] = (acceleration_adjusted >> 8) & 0xFF;
  buffer[7] = acceleration_adjusted & 0xFF;

  // CAN 메시지 전송 (Control Mode 6: Position-Speed Loop Mode)
  uint32_t can_id_with_mode = (CAN_PACKET_SET_POS_SPD << 8) | can_id;
  comm_can_transmit_eid(can_id_with_mode, buffer, 8);

  // 메시지 전송 로그
  //Serial.print("Position-Speed Loop 메시지 전송 - CAN ID: 0x");
  //Serial.println(can_id_with_mode, HEX);
}
// Current Brake Mode 설정 함수
void setCurrentBrakeMode(uint8_t can_id, float brake_current) {
  uint8_t buffer[4];  // 4바이트의 데이터를 담는 버퍼
  
  // brake_current 값을 0.001A 단위로 변환하여 int32_t로 처리
  int32_t brake_current_int = (int32_t)(brake_current * 1000.0);  // 0.001A 단위로 변환

  // 전류 값의 범위를 제한 (0 ~ 60000 mA, 즉 0 ~ 60A)
  if (brake_current_int < 0) brake_current_int = 0;      // 최소 0A
  if (brake_current_int > 60000) brake_current_int = 60000;  // 최대 60A
  
  // 제동 전류(32비트) 설정
  buffer[0] = (brake_current_int >> 24) & 0xFF;
  buffer[1] = (brake_current_int >> 16) & 0xFF;
  buffer[2] = (brake_current_int >> 8) & 0xFF;
  buffer[3] = brake_current_int & 0xFF;

  // CAN 메시지 전송 (Control Mode 2: Current Brake Mode)
  uint32_t can_id_with_mode = (CAN_PACKET_SET_CURRENT_BRAKE << 8) | can_id;
  comm_can_transmit_eid(can_id_with_mode, buffer, 4);

  // 메시지 전송 로그
  Serial.print("Current Brake Mode 메시지 전송 - CAN ID: 0x");
  Serial.println(can_id_with_mode, HEX);
  Serial.print(" 제동 전류: ");
  Serial.print(brake_current);
  Serial.println(" A");
}
// Current Loop Mode 설정 함수
void setCurrentLoopMode(uint8_t can_id, float current) {
  uint8_t buffer[4];  // 4바이트의 데이터를 담는 버퍼
  
  // current 값을 0.001A 단위로 변환하여 int32_t로 처리
  int32_t current_int = (int32_t)(current * 1000.0);  // 0.001A 단위로 변환

  // 전류 값의 범위를 제한 (-60A ~ 60A, 즉 -60000 ~ 60000 mA)
  if (current_int < -60000) current_int = -60000;  // 최소 -60A
  if (current_int > 60000) current_int = 60000;    // 최대 60A
  
  // 전류(32비트) 설정
  buffer[0] = (current_int >> 24) & 0xFF;
  buffer[1] = (current_int >> 16) & 0xFF;
  buffer[2] = (current_int >> 8) & 0xFF;
  buffer[3] = current_int & 0xFF;

  // CAN 메시지 전송 (Control Mode 1: Current Loop Mode)
  uint32_t can_id_with_mode = (CAN_PACKET_SET_CURRENT << 8) | can_id;
  comm_can_transmit_eid(can_id_with_mode, buffer, 4);

  // 메시지 전송 로그
  Serial.print("Current Loop Mode 메시지 전송 - CAN ID: 0x");
  Serial.println(can_id_with_mode, HEX);
  Serial.print(" 전류: ");
  Serial.print(current);
  Serial.println(" A");
}
// Speed Loop Mode 설정 함수
void setSpeedLoopMode(uint8_t can_id, float speed) {
  uint8_t buffer[4];  // 4바이트의 데이터를 담는 버퍼
  
  // speed 값을 전기적 RPM (Electrical RPM) 단위로 변환하여 int32_t로 처리
  int32_t speed_int = (int32_t)speed;

  // 속도 값의 범위를 제한 (-100,000 ~ 100,000 ERPM)
  if (speed_int < -100000) speed_int = -100000;  // 최소 -100,000 ERPM
  if (speed_int > 100000) speed_int = 100000;    // 최대 100,000 ERPM
  
  // 속도(32비트) 설정
  buffer[0] = (speed_int >> 24) & 0xFF;
  buffer[1] = (speed_int >> 16) & 0xFF;
  buffer[2] = (speed_int >> 8) & 0xFF;
  buffer[3] = speed_int & 0xFF;

  // CAN 메시지 전송 (Control Mode 3: Speed Loop Mode)
  uint32_t can_id_with_mode = (CAN_PACKET_SET_RPM << 8) | can_id;
  comm_can_transmit_eid(can_id_with_mode, buffer, 4);

  // 메시지 전송 로그
  Serial.print("Speed Loop Mode 메시지 전송 - CAN ID: 0x");
  Serial.println(can_id_with_mode, HEX);
  Serial.print(" 속도: ");
  Serial.print(speed);
  Serial.println(" ERPM");
}
// CAN 메시지 송신 함수 (can_frame을 이용)
void comm_can_transmit_eid(uint32_t id, const uint8_t* data, uint8_t len) {
  if (len > 8) len = 8;  // 최대 8바이트로 제한

  // can_frame 설정
  canMsg.can_id = id | CAN_EFF_FLAG;  // 확장 ID 플래그 설정
  canMsg.can_dlc = len;  // 데이터 길이 설정

  // 데이터 설정
  for (int i = 0; i < len; i++) {
    canMsg.data[i] = data[i];
  }

  // CAN 메시지 송신
  mcp2515.sendMessage(&canMsg);

  /*// 송신된 메시지 출력
  Serial.print("송신된 메시지 - ID: 0x");
  Serial.print(id, HEX);
  Serial.print(" 데이터: ");
  for (int i = 0; i < len; i++) {
    Serial.print(canMsg.data[i], HEX);
    Serial.print(" ");
  }
  Serial.println();*/
}
// CAN 메시지 수신 및 데이터 처리 함수
void readCANMessage() {
  if (mcp2515.readMessage(&canMsg) == MCP2515::ERROR_OK) {
    // 모터 ID 추출 (확장 ID에서 하위 8비트 추출)
    uint8_t motor_id = canMsg.can_id & 0xFF;
    
    // motor_id가 1~6 범위 내에 있는지 확인
    if (motor_id >= 1 && motor_id <= 6) {
      int motor_index = motor_id - 1; // 배열 인덱스를 0~5로 맞춤

      // 포지션 값 (16비트), -3200 ~ 3200 범위
      int16_t pos_int = (canMsg.data[0] << 8) | canMsg.data[1];
      motor_positions[motor_index] = pos_int * 0.1;

      // 속도 값 (16비트), -32000 ~ 32000 전기적 RPM
      int16_t speed_int = (canMsg.data[2] << 8) | canMsg.data[3];
      motor_speeds[motor_index] = speed_int * 10.0;

      // 전류 값 (16비트), -6000 ~ 6000mA 범위
      int16_t current_int = (canMsg.data[4] << 8) | canMsg.data[5];
      motor_currents[motor_index] = current_int * 0.01;

      // 모터 온도 (8비트), 0~127
      motor_temps[motor_index] = canMsg.data[6];

      // 모터 에러코드 (8비트)
      motor_errors[motor_index] = canMsg.data[7];

      // 수신된 데이터 출력
      Serial.print("모터 ID: ");
      Serial.print(motor_id);
      Serial.print(" | 포지션: ");
      Serial.print(motor_positions[motor_index]);
      Serial.print(" | 속도: ");
      Serial.print(motor_speeds[motor_index]);
      Serial.print(" | 전류: ");
      Serial.print(motor_currents[motor_index]);
      Serial.print(" | 온도: ");
      Serial.print(motor_temps[motor_index]);
      Serial.print(" | 에러 코드: ");
      Serial.println(motor_errors[motor_index]);
    } else {
      Serial.println("잘못된 모터 ID 수신");
    }
  } else {
    Serial.println("CAN 메시지 수신 오류 또는 메시지 없음");
  }
}
void setup() {
  Serial.begin(115200);
  // MCP2515 초기화
  mcp2515.reset();
  mcp2515.setBitrate(CAN_1000KBPS, MCP_8MHZ);  // CAN 속도 설정
  mcp2515.setNormalMode();

  Serial.println("CAN 통신 준비 완료.");
  // 각 모터와의 통신 상태 확인
  bool allMotorsResponding = true;
  unsigned long startTime = millis();
  const unsigned long timeout = 2000; // 2초 타임아웃
  bool motorResponded[6] = {false, false, false, false, false, false};
  Serial.println("모터 통신 상태 확인 중...");
  // 모터 응답 체크
  while(millis() - startTime < timeout) {
    struct can_frame canMsg;
    
    if (mcp2515.readMessage(&canMsg) == MCP2515::ERROR_OK) {
      // CAN ID에서 모터 ID 추출 (하위 8비트)
      uint8_t received_motor_id = canMsg.can_id & 0xFF;
      
      // 수신된 모터 ID 확인
      for(int i = 0; i < 6; i++) {
        uint8_t expected_motor_id = motorCANIDs[i] & 0xFF;  // 저장된 CAN ID의 하위 8비트와 비교
        if(received_motor_id == expected_motor_id) {
          if(!motorResponded[i]) {  // 처음 응답했을 때만 메시지 출력
            motorResponded[i] = true;
            Serial.print("모터 ");
            Serial.print(i + 1);
            Serial.println(" 응답 확인");
          }
        }
      }
      
      // 모든 모터가 응답했는지 확인
      bool allResponded = true;
      for(int i = 0; i < 6; i++) {
        if(!motorResponded[i]) {
          allResponded = false;
          break;
        }
      }
      
      if(allResponded) {
        break;
      }
    }
  }
  
  // 결과 출력
  for(int i = 0; i < 6; i++) {
    if(!motorResponded[i]) {
      Serial.print("경고: 모터 ");
      Serial.print(i + 1);
      Serial.println(" 응답 없음");
      allMotorsResponding = false;
    }
  }

  if(allMotorsResponding) {
    Serial.println("모든 모터 통신 정상");
    // 성공 멜로디
    tone(BUZZER_PIN, 262);
    delay(500);
    noTone(BUZZER_PIN);
    delay(100);
    tone(BUZZER_PIN, 330);
    delay(500);
    noTone(BUZZER_PIN);
    delay(100);
    tone(BUZZER_PIN, 392);
    delay(500);
    noTone(BUZZER_PIN);
    delay(100);
  } else {
    Serial.println("일부 모터 통신 실패");
    // 실패 알림음
    tone(BUZZER_PIN, 262);
    delay(1000);
    noTone(BUZZER_PIN);
  }
}
bool initialPositionSet = false;  // 초기 위치 설정 완료 여부를 추적하는 플래그

void loop() {
  static uint8_t state = 0;  // 상태 머신 변수
  
  switch(state) {
    case 0:  // 초기 원점 설정 상태
      if (!initialPositionSet) {
        Serial.println("모터 원점 설정 시작...");
        // 모든 모터의 원점 설정
        for(int i = 0; i < 6; i++) {
          setOriginMode(motorCANIDs[i], 0);  // 0: 일시적 원점 설정
          delay(50);  // 각 모터 간 약간의 딜레이
        }
        Serial.println("모든 모터 원점 설정 완료");
        state = 1;  // 다음 상태로 전환
        delay(500);  // 원점 설정 후 잠시 대기
      }
      break;

    case 1:  // 0도 위치로 이동
      if (!initialPositionSet) {
        Serial.println("모터를 0도 위치로 이동 중...");
        // 모든 모터를 0도 위치로 이동
        for(int i = 0; i < 6; i++) {
          // 위치: 0도, 속도: 1000(적당한 속도), 가속도: 1000
          positionSpeedLoop(motorCANIDs[i], 0.0, 1000, 1000);
          delay(50);  // 각 모터 간 약간의 딜레이
        }
        initialPositionSet = true;  // 초기 위치 설정 완료
        state = 2;  // 정상 동작 상태로 전환
        Serial.println("초기 위치 설정 완료");
      }
      break;

    case 2:  // 정상 동작 상태 (초기화 완료 후)
      // 모터 상태 모니터링
      readCANMessage();
      // 여기에 추가적인 로봇 제어 코드를 구현할 수 있습니다.
      break;
  }

  delay(10);  // 루프 딜레이
}
