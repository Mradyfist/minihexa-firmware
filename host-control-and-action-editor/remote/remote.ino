#include "hiwonder_robot.h"
#include "uart_server.h"
#include "ble_server.h"
#include "wifi_server.h"
#include "cortex_config.h"
#include "pose_dispatch.h"

Robot minihexa;
UartServerManager uart;
BLEServerManager ble_server;
WiFiServerManager wifi_server;

int8_t read_val[18] = {0};
uint8_t servo_num;
uint8_t set_rgb[3] = {0,0,0};
uint8_t rgb[3] = {0,0,0};
uint8_t count = 0;
uint16_t dis;
uint16_t set_time;
uint32_t tick1_start = 0;
uint32_t tick2_start = 0;

ServoArg_t servo[18];
RecData_t message;
Vector_t read_offset;

Velocity_t vel = {0.0f,0.0f,0.0f};
Vector_t pos = {0.0f,0.0f,0.0f};
Euler_t att = {0.0f,0.0f,0.0f};

static uint32_t pose_move_ms(uint8_t raw) {
  if(raw == 0) {
    return 0;
  }
  if(raw == 1) {
    return 200;
  }
  return raw;
}

static bool velocity_is_idle(const Velocity_t &v) {
  return fabs(v.vx) < 0.01f && fabs(v.vy) < 0.01f && fabs(v.omega) < 0.01f;
}

static void apply_pose_message(const RecData_t &msg) {
  uint32_t pose_time = pose_move_ms(msg.data[6]);

  if(velocity_is_idle(vel)) {
    minihexa.apply_pose_command(
      (int8_t)msg.data[0], (int8_t)msg.data[1], (int8_t)msg.data[2],
      (int8_t)msg.data[3], (int8_t)msg.data[4], (int8_t)msg.data[5],
      msg.data[7], (int8_t)msg.data[8], msg.data[9],
      msg.data[10], (int8_t)msg.data[11],
      pose_time);
    return;
  }

  pos.z = fmap((float)(int8_t)msg.data[5], 0.0f, 30.0f, 0.0f, 3.0f);
  pos.x = fmap((float)(int8_t)msg.data[3], -50.0f, 50.0f, -3.0f, 3.0f);
  pos.y = fmap((float)(int8_t)msg.data[4], -50.0f, 50.0f, -3.0f, 3.0f);

  att.roll = fmap((float)(int8_t)msg.data[2], -50.0f, 50.0f, -20.0f, 20.0f);
  att.pitch = fmap((float)(int8_t)msg.data[1], -50.0f, 50.0f, -20.0f, 20.0f);
  att.yaw = fmap(-(float)(int8_t)msg.data[0], -50.0f, 50.0f, -60.0f, 60.0f);

  minihexa.pose_leg_lift_mask = msg.data[7];
  minihexa.pose_leg_lift_sign_flip = msg.data[9];
  minihexa.pose_leg_lift_z = fmap((float)(int8_t)msg.data[8], 0.0f, 30.0f, 0.0f, 5.5f);
  minihexa.pose_stance_mask = msg.data[10];
  minihexa.pose_stance_spread = fmap((float)(int8_t)msg.data[11], 0.0f, 30.0f, 0.0f, 1.0f);
  minihexa.move(&vel, &pos, &att);
}

static void apply_rgb_message(const RecData_t &msg) {
  set_rgb[0] = msg.data[0];
  set_rgb[1] = msg.data[1];
  set_rgb[2] = msg.data[2];

  rgb[0] = set_rgb[0];
  rgb[1] = set_rgb[1];
  rgb[2] = set_rgb[2];

  minihexa.sensor.set_ultrasound_rgb(RGB_WORK_SOLID_MODE, rgb, rgb);
}

static void reply_print(const char *frame) {
  if(ble_server.state == SWITCH_WIFI) {
    wifi_server.tcpClient.print(frame);
  }
  else {
    Serial.print(frame);
  }
}

void  uart_receive_callback() {
  uart.receive_message();
  if(uart.rec.mode == MINIHEXA_ACTION_GROUP_STOP) {
    minihexa.action_group_stop();
  }
}

void setup() {
  minihexa.begin();
  uart.begin();
  uart.on_receive(uart_receive_callback);
  set_pose_frame_handler(apply_pose_message);
  cortex_config.begin();
  ble_server.begin();
  WiFiServerManager::station_begin();
  String ssid = cortex_config.wifi_ssid();
  if(ssid.length() > 0) {
    WiFi.begin(ssid.c_str(), cortex_config.wifi_password().c_str());
  }
  else {
    WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  }
}

void loop() {
  switch(ble_server.state) {
    case SWITCH_UART:
      message = uart.rec;
      if(millis() - tick2_start > 1000) {
        Serial.printf("A|%d&", minihexa.board.bat_voltage);
        tick2_start = millis();
      }
      if(message.mode == MINIHEXA_AVOID && millis() - tick1_start > 200) {
        dis = minihexa.sensor.get_distance();
        tick1_start = millis();
      }
      if(wifi_server.begin() == true) {
        wifi_server.udp_server();
        wifi_server.tcp_server();
        if(wifi_server.is_cortex_active() == true) {
          ble_server.state = SWITCH_WIFI;
        }
      }
      break;

    case SWITCH_WIFI:
      if(wifi_server.is_cortex_active() == false) {
        ble_server.state = SWITCH_UART;
      }
      else {
        wifi_server.tcp_server();
        message = wifi_server.rec;
        if(millis() - tick2_start > 1000) {
          wifi_server.tcpClient.printf("A|%d&", minihexa.board.bat_voltage);
          tick2_start = millis();
        }  
      }
      break;

    case SWITCH_BLE:
      wifi_server.parameters_reset();
      message = ble_server.rec;
      if(millis() - tick1_start > 200) {
        dis = minihexa.sensor.get_distance();
        ble_server.send_message(minihexa.board.bat_voltage, dis);
        tick1_start = millis();
      }    
      break;
  }

  if(ble_server.state == SWITCH_UART) {
    drain_pose_frames();
    if(uart.rgb_stream_pending) {
      apply_rgb_message(uart.rgb_stream);
      uart.rgb_stream_pending = false;
    }
  }
  else if(ble_server.state == SWITCH_WIFI) {
    drain_pose_frames();
    if(wifi_server.rgb_stream_pending) {
      apply_rgb_message(wifi_server.rgb_stream);
      wifi_server.rgb_stream_pending = false;
    }
  }

  switch(message.mode) {

  case MINIHEXA_CRAWL_STATE:
    minihexa.crawl_state();
    break;

  case MINIHEXA_MOVING_CONTROL:
    clear_pose_frames();
    vel.vx = fmap((float)(int8_t)message.data[0], -50.0f, 50.0f, -3.0f, 3.0f);
    vel.vy = fmap((float)(int8_t)message.data[1], -50.0f, 50.0f, -3.0f, 3.0f);

    switch(message.data[2]) {
    case 0:
      vel.omega = 0.0f;
      break;

    case 1:
      if(vel.vx == 0.0f && vel.vy == 0.0f) {
        vel.omega = 2.5f;
      }
      else {
        vel.omega = 0.2f;
      }
      break;

    case 2:
      if(vel.vx == 0.0f && vel.vy == 0.0f) {
        vel.omega = -2.5f;
      }
      else {
        vel.omega = -0.2f;
      }
      break;
    
    default:
      break;
    }
    minihexa.gait_mode = message.data[3] ? Robot::GAIT_RIPPLE : Robot::GAIT_TRIPOD;
    if(message.data[4] <= 100) {
      minihexa.gait_smooth = message.data[4];
    }
    minihexa.move(&vel, &pos, &att);
    break;

  case MINIHEXA_POSE_CONTROL:
    apply_pose_message(message);
    break;

  case MINIHEXA_OFFSET_READ:
    minihexa.read_deviation(read_val);
    {
      char buf[256];
      snprintf(buf, sizeof(buf),
               "G|3|%d|%d|%d|%d|%d|%d|%d|%d|%d|%d|%d|%d|%d|%d|%d|%d|%d|%d&",
               read_val[0], read_val[1], read_val[2], read_val[3], read_val[4], read_val[5],
               read_val[6], read_val[7], read_val[8], read_val[9], read_val[10], read_val[11],
               read_val[12], read_val[13], read_val[14], read_val[15], read_val[16], read_val[17]);
      reply_print(buf);
    }
    break;

  case MINIHEXA_OFFSET_SAVE:
    if(minihexa.download_deviation()) {
      ledcWriteTone(LEDC_CHANNEL_0, 3000);
      delay(100);
      ledcWriteTone(LEDC_CHANNEL_0, 0);
      reply_print("G|2|1&");
    }
    else {
      reply_print("G|2|0&");
    }
    break;
 

  case MINIHEXA_LEG_OFFSET_SET:
    minihexa.set_deviation(message.data[0], (int8_t)message.data[1]);
    break;

  case MINIHEXA_ACTION_GROUP_RUN:
    switch(message.data[0]) {
    case 1:
      minihexa.twist(15.0f, 3, 15, COUNTER_CLOCKWISE);
      break;

    case 2:
      minihexa.twist(15.0f, 3, 15, CLOCKWISE);
      break;

    case 3:
      minihexa._wake_up();
      break;

    case 4:
      minihexa.wake_up();
      break;

    case 5:
      minihexa.acting_cute();
      break;

    case 6:
      minihexa.rear_up();
      break;

    default:
      minihexa.action_group_run(message.data[0]);
      break;
    }
    
    break;

  case MINIHEXA_ACTION_GROUP_DOWNLOAD:
    minihexa.action_group_download(message.data[0], message.data, sizeof(message.data));
    reply_print("K|3&");
    if(message.data[1] == message.data[2]) {
      ledcWriteTone(LEDC_CHANNEL_0, 3000);
      delay(100);
      ledcWriteTone(LEDC_CHANNEL_0, 0);
    }
    break;

  case MINIHEXA_ACTION_GROUP_ERASE:
    minihexa.action_group_erase(message.data[0]);
    ledcWriteTone(LEDC_CHANNEL_0, 3000);
    delay(100);
    ledcWriteTone(LEDC_CHANNEL_0, 0);
    reply_print("K&");
    break;

  case MINIHEXA_ACTION_GROUP_ALL_ERASE:
    for(uint8_t i = 0; i < 50; i++) {
       if(minihexa.action_group_erase(i) != true) {
          Serial.printf("ID: %d erase fail!\n", i);
          break;
       }
       if(i == 49) {
          Serial.printf("All action group is erase!\n");
          ledcWriteTone(LEDC_CHANNEL_0, 3000);
          delay(100);
          ledcWriteTone(LEDC_CHANNEL_0, 0);
          reply_print("K&");
       }
    }
    break;

  case MINIHEXA_SERVO_CONTROL:
    servo_num = message.data[0];
    set_time = BYTE_TO_HW(message.data[2], message.data[1]);
    for(uint8_t i = 0; i < servo_num; i++) {
      servo[i].id = message.data[3 + i * 3];
      servo[i].duty = BYTE_TO_HW(message.data[5 + i * 3], message.data[4 + i * 3]);
      switch(servo[i].id) {
        case 1:
          servo[i].duty += minihexa.leg1.joint_a.deviation;
          break;
          
        case 2:
          servo[i].duty += minihexa.leg1.joint_b.deviation;
          break;

        case 3:
          servo[i].duty += minihexa.leg1.joint_c.deviation;
          break;

        case 4:
          servo[i].duty += minihexa.leg2.joint_a.deviation;
          break;

        case 5:
          servo[i].duty += minihexa.leg2.joint_b.deviation;
          break;

        case 6:
          servo[i].duty += minihexa.leg2.joint_c.deviation;
          break;

        case 7:
          servo[i].duty += minihexa.leg3.joint_a.deviation;
          break;

        case 8:
          servo[i].duty += minihexa.leg3.joint_b.deviation;
          break;

        case 9:
          servo[i].duty += minihexa.leg3.joint_c.deviation;
          break;

        case 10:
          servo[i].duty += minihexa.leg4.joint_a.deviation;
          break;

        case 11:
          servo[i].duty += minihexa.leg4.joint_b.deviation;
          break;

        case 12:
          servo[i].duty += minihexa.leg4.joint_c.deviation;
          break;

        case 13:
          servo[i].duty += minihexa.leg5.joint_a.deviation;
          break;

        case 14:
          servo[i].duty += minihexa.leg5.joint_b.deviation;
          break;

        case 15:
          servo[i].duty += minihexa.leg5.joint_c.deviation;
          break;

        case 16:
          servo[i].duty += minihexa.leg6.joint_a.deviation;
          break;

        case 17:
          servo[i].duty += minihexa.leg6.joint_b.deviation;
          break;

        case 18:
          servo[i].duty += minihexa.leg6.joint_c.deviation;
          break;
      }
    }
    minihexa.multi_servo_control(servo, servo_num, set_time);
    break; 

  case MINIHEXA_SERVO_DUTY_READ:
    {
      char buf[384];
      snprintf(buf, sizeof(buf),
               "M|1|%d|2|%d|3|%d|4|%d|5|%d|6|%d|7|%d|8|%d|9|%d|10|%d|11|%d|12|%d|13|%d|14|%d|15|%d|16|%d|17|%d|18|%d&",
               minihexa.leg1.joint_a.duty, minihexa.leg1.joint_b.duty, minihexa.leg1.joint_c.duty,
               minihexa.leg2.joint_a.duty, minihexa.leg2.joint_b.duty, minihexa.leg2.joint_c.duty,
               minihexa.leg3.joint_a.duty, minihexa.leg3.joint_b.duty, minihexa.leg3.joint_c.duty,
               minihexa.leg4.joint_a.duty, minihexa.leg4.joint_b.duty, minihexa.leg4.joint_c.duty,
               minihexa.leg5.joint_a.duty, minihexa.leg5.joint_b.duty, minihexa.leg5.joint_c.duty,
               minihexa.leg6.joint_a.duty, minihexa.leg6.joint_b.duty, minihexa.leg6.joint_c.duty);
      reply_print(buf);
    }
    break;

  case MINIHEXA_DISTANCE_READ:
    {
      char buf[24];
      dis = minihexa.sensor.get_distance();
      snprintf(buf, sizeof(buf), "N|%u&", (unsigned)dis);
      reply_print(buf);
    }
    break;

  case MINIHEXA_RGB_ADJUST:
    apply_rgb_message(message);
    break;

  case MINIHEXA_AVOID:
    if(message.data[0] == 1) {
      if(dis > 220) {
        rgb[0] = 0;
        rgb[1] = 237;
        rgb[2] = 178;
      }
      else if(dis < 220){
        rgb[0] = 255;
        rgb[1] = 0;
        rgb[2] = 0;
      }
      minihexa.avoid(dis);
    }
    else {
      vel = {0.0f,0.0f,0.0f};
      rgb[0] = set_rgb[0];
      rgb[1] = set_rgb[1];
      rgb[2] = set_rgb[2];
      minihexa.move(&vel, &pos, &att);
      minihexa.avoid_state = minihexa.FORWARD;
    }
    minihexa.sensor.set_ultrasound_rgb(RGB_WORK_SOLID_MODE, rgb, rgb);
    break;

  case MINIHEXA_BALANCE:
    if(message.data[0] == 1) {
      minihexa.balance(true);
    }
    else {
      att = {0.0f,0.0f,0.0f};
      minihexa.balance(false);
      minihexa.move(&vel, &pos, &att);
    } 
    break;

  case MINIHEXA_RESET:
    vel = {0.0f,0.0f,0.0f};
    pos = {0.0f,0.0f,1.0f};
    att = {0.0f,0.0f,0.0f};
    minihexa.reset();
    break;

  case MINIHEXA_ARM_CONTROL:
    break;

  default:
    break;
  }

  if(message.mode != 0) {
    switch(ble_server.state) {
    case SWITCH_UART:
      if(message.mode != MINIHEXA_AVOID && message.mode != MINIHEXA_BALANCE) {
        memset(&uart.rec, 0, sizeof(uart.rec));
      }
      break;

    case SWITCH_WIFI:
      memset(&wifi_server.rec, 0, sizeof(wifi_server.rec));
      break;

    case SWITCH_BLE:
      if(message.mode != MINIHEXA_AVOID && message.mode != MINIHEXA_BALANCE) {
        memset(&ble_server.rec, 0, sizeof(ble_server.rec));
      }
      break;
    }
  }
}
