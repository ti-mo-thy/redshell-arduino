//#include "./redshell-messages/include/redshell/encoder.h"
//#include "./redshell-messages/include/redshell/command.h"

#define A1 3
#define B1 2
#define A2 4
#define B2 5
#define EN_A 6
#define EN_B 9
#define IN_1 14
#define IN_2 15
#define IN_3 16
#define IN_4 17

volatile int32_t pos1_ticks = 0;
volatile int32_t pos2_ticks = 0;

// The ISR must be as short as possible
void pos1_update() {
  if (digitalRead(A1) != digitalRead(B1)) {
    pos1_ticks++;
  } else {
    pos1_ticks--;
  }
}

void pos2_update() {
  if (digitalRead(A2) != digitalRead(B2)) {
    pos2_ticks++;
  } else {
    pos2_ticks--;
  }
}

uint8_t get_duty_cycle(float duty) {
  return (uint8_t) (2.55 * duty);
}

void set_motor_val(uint8_t pin, uint8_t in_front, uint8_t in_back, float duty, const int8_t dir){
  switch (dir) {
        case 1: {
            digitalWrite(in_front, HIGH);
            digitalWrite(in_back, LOW);
            analogWrite(pin, get_duty_cycle(duty));
            break;
        }
        case -1: {
            digitalWrite(in_front, LOW);
            digitalWrite(in_back, HIGH);
            analogWrite(pin, get_duty_cycle(duty));
            break;
        }
        default: {
            digitalWrite(in_front, LOW);
            digitalWrite(in_back, LOW);
            analogWrite(pin, 0);
            break;
        }
    }
}


void set_motor_1(float duty, const int8_t dir){
  switch (dir) {
      case 1: {
            digitalWrite(IN_1, HIGH);
            digitalWrite(IN_2, LOW);
            analogWrite(EN_A, get_duty_cycle(duty));
            break;
        }
        case -1: {
            digitalWrite(IN_1, LOW);
            digitalWrite(IN_2, HIGH);
            analogWrite(EN_A, get_duty_cycle(duty));
            break;
        }
        default: {
            digitalWrite(IN_1, LOW);
            digitalWrite(IN_2, LOW);
            analogWrite(EN_A, 0);
            break;
        }
    }
}

void set_motor_2(float duty, const int8_t dir){
    switch (dir) {
        case 1: {
            digitalWrite(IN_3, HIGH);
            digitalWrite(IN_4, LOW);
            analogWrite(EN_B, get_duty_cycle(duty));
            break;
        }
        case -1: {
            digitalWrite(IN_3, LOW);
            digitalWrite(IN_4, HIGH);
            analogWrite(EN_B, get_duty_cycle(duty));
            break;
        }
        default: {
            digitalWrite(IN_3, LOW);
            digitalWrite(IN_4, LOW);
            analogWrite(EN_B, 0);
            break;
        }
    }
}

void updateSpeed(double &speed1_rpm, double &speed2_rpm) {
    static int32_t last_pos1_ticks = 0;
    static int32_t last_pos2_ticks = 0;
    static int32_t lastTime_ms = 0;

    static constexpr double ms_to_s = 1e-3;
    const int32_t currentTime_ms = millis();
    const double timeDelta_s = static_cast<double>(currentTime_ms - lastTime_ms) * ms_to_s;

    static constexpr int16_t ticksPerCycle = 48;
    const double positionDelta1_cycles = static_cast<double>(pos1_ticks - last_pos1_ticks) / ticksPerCycle;
    const double positionDelta2_cycles = static_cast<double>(pos2_ticks - last_pos2_ticks) / ticksPerCycle;

    static constexpr double rps_to_rpm = 60.0;
    speed1_rpm = (positionDelta1_cycles / timeDelta_s) * rps_to_rpm;
    speed2_rpm = (positionDelta2_cycles / timeDelta_s) * rps_to_rpm;

    last_pos1_ticks = pos1_ticks;
    last_pos2_ticks = pos2_ticks;
    lastTime_ms = currentTime_ms;
}

void setup() {
  Serial.begin(9600);// Set pin 2 as an input
  pinMode(A1, INPUT);
  pinMode(B1, INPUT);
  pinMode(A2, INPUT);
  pinMode(B2, INPUT);
  pinMode(EN_A, OUTPUT);
  pinMode(IN_1, OUTPUT);
  pinMode(IN_2, OUTPUT);
  pinMode(IN_3, OUTPUT);
  pinMode(IN_4, OUTPUT);
  attachInterrupt(digitalPinToInterrupt(A1), pos1_update, CHANGE);
  attachInterrupt(digitalPinToInterrupt(A2), pos2_update, CHANGE);
}


void loop() {
//  set_motor_val(EN_/A, IN_1, IN_2, 0, 0);
  //Serial.println(pos1_ticks);
  double speed1_rpm, speed2_rpm;
  updateSpeed(speed1_rpm, speed2_rpm);
  Serial.print("T: ");
  Serial.print(pos1_ticks);
  Serial.print(", S: ");
  Serial.println(speed1_rpm);
  set_motor_1(0, 1);
  set_motor_2(0, 1);
  delay(300);
}
