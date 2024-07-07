#ifndef REDSHELL_MESSAGES_H
#define REDSHELL_MESSAGES_H

#include <stdlib.h>
#include <stdint.h>

/* PACKET INTEGER FORMAT
    |START_BIT|ID|PACKET|CRC|
      where: 
        START_BIT is 1 byte
        ID is 1 byte
        PACKET is 8 bytes
        CRC is 2 bytes
*/

#define REDSHELL_START_BYTE 0x66 // Execute Order 66 :D
#define REDSHELL_CRC16 0x8005
#define REDSHELL_PAYLOAD_SIZE 8
#define REDSHELL_MESSAGE_SIZE (4 + REDSHELL_PAYLOAD_SIZE)

typedef struct {
    uint8_t start; 
    uint8_t id;
    uint8_t data[REDSHELL_PAYLOAD_SIZE];
    uint16_t crc;
} PacketInfo;

uint16_t gen_crc16(const uint8_t *data, uint16_t size){
    uint16_t out = 0;
    int bits_read = 0, bit_flag;
    /* Sanity check: */
    if(data == NULL){
        return 0;
    }
    while(size > 0){
        bit_flag = out >> 15;
        /* Get next bit: */
        out <<= 1;
        out |= (*data >> bits_read) & 1; // item a) work from the least significant bits
        /* Increment bit counter: */
        bits_read++;
        if(bits_read > 7){
            bits_read = 0;
            data++;
            size--;
        }
        /* Cycle check: */
        if(bit_flag){
            out ^= REDSHELL_CRC16;
        }
    }
    // item b) "push out" the last 16 bits
    int i;
    for (i = 0; i < 16; ++i) {
        bit_flag = out >> 15;
        out <<= 1;
        if(bit_flag)
            out ^= REDSHELL_CRC16;
    }
    // item c) reverse the bits
    uint16_t crc = 0;
    i = 0x8000;
    int j = 0x0001;
    for (; i != 0; i >>=1, j <<= 1) {
        if (i & out) crc |= j;
    }
    return crc;
}

void serialize(PacketInfo packet, uint8_t* serial_packet) {
    serial_packet[0] = packet.start;
    serial_packet[1] = packet.id;

    for (int i = 0; i < REDSHELL_PAYLOAD_SIZE; i++)
    {
        serial_packet[2 + i] = packet.data[i];
    }

    serial_packet[2 + REDSHELL_PAYLOAD_SIZE] = (packet.crc & 0xFF);
    serial_packet[2 + REDSHELL_PAYLOAD_SIZE + 1] = ((packet.crc >> 8) & 0xFF);
}

void deserialize(PacketInfo* packet, uint8_t* serial_packet) {
    packet->start = serial_packet[0];
    packet->id = serial_packet[1];

    for (int i = 0; i < REDSHELL_PAYLOAD_SIZE; i++)
    {
        packet->data[i] = serial_packet[2 + i];
    }

    packet->crc = serial_packet[2 + REDSHELL_PAYLOAD_SIZE] | (packet->crc << 8);
}
#endif 

#ifndef COMMAND_H
#define COMMAND_H

#define REDSHELL_MSG_ID_COMMAND 0x2

PacketInfo msg_command_encode(int32_t speed_left_pct, int32_t speed_right_pct) {
    PacketInfo result;
    result.start = REDSHELL_START_BYTE;
    result.id = REDSHELL_MSG_ID_COMMAND;
    
    const int speed_size_bytes = 4;
    const int byte_size = 8;
    for (int i = 0; i < speed_size_bytes; i++) {
        result.data[i] = (speed_left_pct >> (byte_size * i)) & 0xFF;
        result.data[i + speed_size_bytes] = (speed_right_pct >> (byte_size * i)) & 0xFF;
    }

    result.crc = gen_crc16(result.data, REDSHELL_PAYLOAD_SIZE);

    return result;
}

void msg_command_decode(PacketInfo packet, int32_t* speed_left_pct, int32_t* speed_right_pct) {
    *speed_left_pct = packet.data[0] | (packet.data[1] << 8) | (packet.data[2] << 16) | (packet.data[3] << 24);
    *speed_right_pct = packet.data[4] | (packet.data[5] << 8) | (packet.data[6] << 16) | (packet.data[7] << 24);
}

#endif

#ifndef ENCODER_H
#define ENCODER_H

#define REDSHELL_MSG_ID_ENCODER 0x1

PacketInfo msg_encoder_encode(int32_t speed_motor_left_rpm, int32_t speed_motor_right_rpm) {
    PacketInfo result;
    result.start = REDSHELL_START_BYTE;
    result.id = REDSHELL_MSG_ID_ENCODER;

    const int speed_size_bytes = 4;
    const int byte_size = 8;
    for (int i = 0; i < speed_size_bytes; i++) {
        result.data[i] = (speed_motor_left_rpm >> (byte_size * i)) & 0xFF;
        result.data[i + speed_size_bytes] = (speed_motor_right_rpm >> (byte_size * i)) & 0xFF;
    }
    
    // gen_crc16 currently bugged
    result.crc = 0;//gen_crc16(result.data, REDSHELL_PAYLOAD_SIZE);
    return result;
}

void msg_encoder_decode(PacketInfo packet, int32_t* speed_motor_left_rpm, int32_t* speed_motor_right_rpm) {
    *speed_motor_left_rpm = packet.data[0] | (packet.data[1] << 8) | (packet.data[2] << 16) | (packet.data[3] << 24);
    *speed_motor_right_rpm = packet.data[4] | (packet.data[5] << 8) | (packet.data[6] << 16) | (packet.data[7] << 24);
}

#endif





#define ENC_A1 2
#define ENC_A2 3
#define ENC_B1 4
#define ENC_B2 5
#define EN_A 6
#define EN_B 9
#define IN_1 16
#define IN_2 17
#define IN_3 14
#define IN_4 15

volatile int32_t pos1_ticks = 0;
volatile int32_t pos2_ticks = 0;

int32_t last_cmd_time_ms;

void pos1_update() {
  if (digitalRead(ENC_A1) != digitalRead(ENC_B1)) {
    pos1_ticks++;
  } else {
    pos1_ticks--;
  }
}

void pos2_update() {
  if (digitalRead(ENC_A2) != digitalRead(ENC_B2)) {
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

void updateSpeed(int32_t &speed1_rpm, int32_t &speed2_rpm) {
    static int32_t last_pos1_ticks = 0;
    static int32_t last_pos2_ticks = 0;
    static int32_t lastTime_ms = 0;

    const int32_t currentTime_ms = millis();
    const int32_t time_delta_ms = currentTime_ms - lastTime_ms;
    if (time_delta_ms == 0)
    {
        return;
    }

    // Unsure why this isn't being multiplied by 60 but seems to work
    static constexpr double ms_to_s = 1e-3;
    const double time_delta_min = static_cast<double>(time_delta_ms) * ms_to_s;

    static constexpr int16_t ticksPerCycle = 48;
    const double positionDelta1_cycles = static_cast<double>(pos1_ticks - last_pos1_ticks) / ticksPerCycle;
    const double positionDelta2_cycles = static_cast<double>(pos2_ticks - last_pos2_ticks) / ticksPerCycle;

    speed1_rpm = static_cast<int32_t>(round(positionDelta1_cycles / time_delta_min));
    speed2_rpm = static_cast<int32_t>(round(positionDelta2_cycles / time_delta_min));

    last_pos1_ticks = pos1_ticks;
    last_pos2_ticks = pos2_ticks;
    lastTime_ms = currentTime_ms;
}

void setup() {
  Serial.begin(19200);
  pinMode(ENC_A1, INPUT);
  pinMode(ENC_B1, INPUT);
  pinMode(ENC_A2, INPUT);
  pinMode(ENC_B2, INPUT);
  pinMode(EN_A, OUTPUT);
  pinMode(IN_1, OUTPUT);
  pinMode(IN_2, OUTPUT);
  pinMode(IN_3, OUTPUT);
  pinMode(IN_4, OUTPUT);
  attachInterrupt(digitalPinToInterrupt(ENC_A1), pos1_update, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENC_A2), pos2_update, CHANGE);
}


uint8_t encoder_msg[REDSHELL_MESSAGE_SIZE];

void send_encoder_data()
{
  int32_t speed1_rpm = 0;
  int32_t speed2_rpm = 0;
  updateSpeed(speed1_rpm, speed2_rpm);
  PacketInfo enc_packet = msg_encoder_encode(-speed2_rpm, -speed1_rpm);
  serialize(enc_packet, encoder_msg);
  Serial.write(encoder_msg, REDSHELL_MESSAGE_SIZE);
}

int8_t sign(int32_t x)
{
  return (x > 0) - (x < 0);
}

char command_buffer[REDSHELL_MESSAGE_SIZE];
uint8_t command_index{0};
bool reading_msg{false};

void check_command_data()
{
  while(Serial.available() > 0)
  {
    char incoming_byte = Serial.read();

    if (incoming_byte == REDSHELL_START_BYTE)
    {
      command_index = 0;
      reading_msg = true;
    }
  
    if (reading_msg)
    {
      command_buffer[command_index] = incoming_byte;
      command_index++;
  
      if (command_index >= REDSHELL_MESSAGE_SIZE)
      {
        PacketInfo incoming_packet;
        deserialize(&incoming_packet, (uint8_t*)(command_buffer));

        int32_t speed_left_pct, speed_right_pct;
        msg_command_decode(incoming_packet, &speed_left_pct, &speed_right_pct);


        set_motor_1(abs(speed_right_pct), -sign(speed_right_pct));
        set_motor_2(abs(speed_left_pct), -sign(speed_left_pct));
        
        reading_msg = false;
        last_cmd_time_ms = millis();
      }
    }
  }
}

void check_timeout()
{
  static constexpr int32_t timeout_ms = 200;
  if ((millis() - last_cmd_time_ms) > timeout_ms)
  {
    set_motor_1(0, 1);
    set_motor_2(0, 1);
  }
}

void loop() {
  send_encoder_data();
  check_command_data();
  check_timeout();
}
