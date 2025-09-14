/*  esp32_payload.ino — ESP32 Payload (SYSID=72)
 *  IMU (MPU6050, I2C) + GPS (UART1) + 4× Servos (LEDC)
 *  MAVLink over UART2 → SiK 433 MHz air radio
 *  Publishes HEARTBEAT/ATTITUDE/GPS_RAW_INT; Receives DO_SET_SERVO (1..4)
 */
#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <MadgwickAHRS.h>
#include <TinyGPSPlus.h>
extern "C" { #include "mavlink.h" }

HardwareSerial SerialMAV(2);
const int MAV_RX_PIN=26, MAV_TX_PIN=27; const uint32_t MAV_BAUD=57600;
HardwareSerial SerialGPS(1);
const int GPS_RX_PIN=16, GPS_TX_PIN=17; const uint32_t GPS_BAUD=9600;
const int I2C_SDA=21, I2C_SCL=22;
const int SERVO_PINS[4]={14,15,4,5}, SERVO_CH[4]={0,1,2,3};
const int SERVO_FREQ=50, SERVO_RES=16; const int PWM_CENTER=1500, PWM_MIN=1100, PWM_MAX=1900;
uint8_t sysid=72, compid=MAV_COMP_ID_PAYLOAD_CONTROLLER;
Adafruit_MPU6050 mpu; Madgwick filter; TinyGPSPlus gps;
float roll=0,pitch=0,yaw=0,p_rate=0,q_rate=0,r_rate=0;
double gps_lat_deg=0.0,gps_lon_deg=0.0; float gps_alt_m=0.0; uint8_t gps_fix=0,gps_sat=0;
uint32_t t_ms=0,last_hb=0,last_att=0,last_gps=0,last_cmd=0;
uint16_t us_to_duty(int us){ us = constrain(us,PWM_MIN,PWM_MAX); return (uint16_t)((us/20000.0f)*65535.0f); }
void set_servo_us(uint8_t i,int us){ if(i<4) ledcWrite(SERVO_CH[i], us_to_duty(us)); }
void neutral_servos(){ for(int i=0;i<4;i++) set_servo_us(i,PWM_CENTER); }
void mav_send_heartbeat(){ mavlink_message_t m; uint8_t b[MAVLINK_MAX_PACKET_LEN];
  mavlink_msg_heartbeat_pack(sysid,compid,&m,MAV_TYPE_GENERIC,MAV_AUTOPILOT_INVALID,MAV_MODE_FLAG_MANUAL_INPUT_ENABLED,0,MAV_STATE_ACTIVE);
  SerialMAV.write(b, mavlink_msg_to_send_buffer(b,&m)); }
void mav_send_attitude(){ sensors_event_t a,g,tmp; mpu.getEvent(&a,&g,&tmp);
  p_rate=g.gyro.x; q_rate=g.gyro.y; r_rate=g.gyro.z;
  static uint32_t last=micros(); uint32_t now=micros(); float dt=(now-last)*1e-6f; last=now; if(dt<=0) dt=1e-3f;
  float ax=a.acceleration.x/9.80665f, ay=a.acceleration.y/9.80665f, az=a.acceleration.z/9.80665f;
  filter.updateIMU(p_rate,q_rate,r_rate,ax,ay,az);
  roll=filter.getRoll()*DEG_TO_RAD; pitch=filter.getPitch()*DEG_TO_RAD; yaw=filter.getYaw()*DEG_TO_RAD;
  mavlink_message_t m; uint8_t b[MAVLINK_MAX_PACKET_LEN];
  mavlink_msg_attitude_pack(sysid,compid,&m, now, roll,pitch,yaw, p_rate,q_rate,r_rate);
  SerialMAV.write(b, mavlink_msg_to_send_buffer(b,&m)); }
void mav_send_gps(){ while(SerialGPS.available()) gps.encode(SerialGPS.read());
  if(gps.location.isValid()){ gps_lat_deg=gps.location.lat(); gps_lon_deg=gps.location.lng(); }
  if(gps.altitude.isValid()) gps_alt_m=gps.altitude.meters();
  if(gps.satellites.isValid()) gps_sat=gps.satellites.value();
  gps_fix = gps.location.isValid() ? (gps.altitude.isValid()?3:2) : 0;
  int32_t lat=(int32_t)(gps_lat_deg*1e7), lon=(int32_t)(gps_lon_deg*1e7), altmm=(int32_t)(gps_alt_m*1000.0f);
  mavlink_message_t m; uint8_t b[MAVLINK_MAX_PACKET_LEN]; uint32_t usec=micros();
  mavlink_msg_gps_raw_int_pack(sysid,compid,&m, usec,gps_fix,lat,lon,altmm,65535,65535,0,0,gps_sat);
  SerialMAV.write(b, mavlink_msg_to_send_buffer(b,&m)); }
void handle_command_long(const mavlink_command_long_t &c){
  if(c.command==MAV_CMD_DO_SET_SERVO){ int idx=(int)c.param1, pwm=(int)c.param2; if(1<=idx && idx<=4){ set_servo_us((uint8_t)(idx-1),pwm); last_cmd=millis(); } } }
void mav_read(){ mavlink_message_t m; mavlink_status_t s; while(SerialMAV.available()){ uint8_t c=SerialMAV.read();
  if(mavlink_parse_char(MAVLINK_COMM_0,c,&m,&s)){ if(m.msgid==MAVLINK_MSG_ID_COMMAND_LONG){ mavlink_command_long_t cmd; mavlink_msg_command_long_decode(&m,&cmd); if(cmd.target_system==sysid||cmd.target_system==0) handle_command_long(cmd); } } } }
void setup(){ Serial.begin(115200); Wire.begin(I2C_SDA,I2C_SCL);
  if(!mpu.begin()){ Serial.println("MPU6050 not found!"); while(1) delay(1000); }
  mpu.setAccelerometerRange(MPU6050_RANGE_8_G); mpu.setGyroRange(MPU6050_RANGE_500_DEG); mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);
  SerialGPS.begin(GPS_BAUD, SERIAL_8N1, GPS_RX_PIN, GPS_TX_PIN); SerialMAV.begin(MAV_BAUD, SERIAL_8N1, MAV_RX_PIN, MAV_TX_PIN);
  for(int i=0;i<4;i++){ ledcSetup(SERVO_CH[i],SERVO_FREQ,SERVO_RES); ledcAttachPin(SERVO_PINS[i],SERVO_CH[i]); } neutral_servos(); }
void loop(){ t_ms=millis();
  if(t_ms-last_hb>1000){ mav_send_heartbeat(); last_hb=t_ms; }
  if(t_ms-last_att>20){ mav_send_attitude(); last_att=t_ms; }
  if(t_ms-last_gps>200){ mav_send_gps(); last_gps=t_ms; }
  mav_read(); if(t_ms-last_cmd>500) neutral_servos(); }
