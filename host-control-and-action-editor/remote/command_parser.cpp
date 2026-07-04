#include "command_parser.h"

static String rec_data[62];

RecData_t parse_command_frame(const String &data) {
  String data_update;
  RecData_t rec;
  uint8_t index = 0;

  data_update = data;

  while (data_update.indexOf('|') != -1) {
    rec_data[index] = data_update.substring(0, data_update.indexOf('|'));
    data_update = data_update.substring(data_update.indexOf('|') + 1);
    index++;
  }

  rec_data[index] = data_update;

  if (rec_data[0] == "B") {
    rec.mode = MINIHEXA_CRAWL_STATE;
  }
  else if (rec_data[0] == "C") {
    rec.mode = MINIHEXA_MOVING_CONTROL;
    for (uint8_t i = 0; i < 3; i++) {
      rec.data[i] = (uint8_t)atoi(rec_data[i + 1].c_str());
    }
    rec.data[3] = (index >= 4) ? (uint8_t)atoi(rec_data[4].c_str()) : 0;
    rec.data[4] = (index >= 5) ? (uint8_t)atoi(rec_data[5].c_str()) : 255;
    rec.data[5] = (index >= 6) ? (uint8_t)atoi(rec_data[6].c_str()) : 255;
  }
  else if (rec_data[0] == "F") {
    rec.mode = MINIHEXA_POSE_CONTROL;
    for (uint8_t i = 0; i < 6; i++) {
      rec.data[i] = (uint8_t)atoi(rec_data[i + 1].c_str());
    }
    rec.data[6] = (index >= 7) ? (uint8_t)atoi(rec_data[7].c_str()) : 1;
    rec.data[7] = (index >= 8) ? (uint8_t)atoi(rec_data[8].c_str()) : 0;
    rec.data[8] = (index >= 9) ? (uint8_t)atoi(rec_data[9].c_str()) : 0;
    rec.data[9] = (index >= 10) ? (uint8_t)atoi(rec_data[10].c_str()) : 0;
    rec.data[10] = (index >= 11) ? (uint8_t)atoi(rec_data[11].c_str()) : 0;
    rec.data[11] = (index >= 12) ? (uint8_t)atoi(rec_data[12].c_str()) : 0;
  }
  else if (rec_data[0] == "G") {
    switch (atoi(rec_data[1].c_str())) {
    case 1:
      rec.mode = MINIHEXA_LEG_OFFSET_SET;
      for (uint8_t i = 0; i < 4; i++) {
        rec.data[i] = (uint8_t)atoi(rec_data[2 + i].c_str());
      }
      break;

    case 2:
      rec.mode = MINIHEXA_OFFSET_SAVE;
      rec.data[0] = (uint8_t)atoi(rec_data[2].c_str());
      break;

    case 3:
      rec.mode = MINIHEXA_OFFSET_READ;
      rec.data[0] = (uint8_t)atoi(rec_data[2].c_str());
      break;
    }
  }
  else if (rec_data[0] == "K") {
    switch (atoi(rec_data[1].c_str())) {
    case 1:
      rec.mode = MINIHEXA_ACTION_GROUP_RUN;
      rec.data[0] = (uint8_t)atoi(rec_data[2].c_str());
      break;

    case 2:
      rec.mode = MINIHEXA_ACTION_GROUP_STOP;
      break;

    case 3:
      rec.mode = MINIHEXA_ACTION_GROUP_DOWNLOAD;
      for (uint8_t i = 0; i < 60; i++) {
        rec.data[i] = (uint8_t)atoi(rec_data[2 + i].c_str());
      }
      break;

    case 4:
      rec.mode = MINIHEXA_ACTION_GROUP_ERASE;
      rec.data[0] = (uint8_t)atoi(rec_data[2].c_str());
      break;

    case 5:
      rec.mode = MINIHEXA_ACTION_GROUP_ALL_ERASE;
      break;
    }
  }
  else if (rec_data[0] == "D") {
    rec.mode = MINIHEXA_ARM_CONTROL;
  }
  else if (rec_data[0] == "H") {
    rec.mode = MINIHEXA_RGB_ADJUST;
    for (uint8_t i = 0; i < 3; i++) {
      rec.data[i] = (uint8_t)atoi(rec_data[1 + i].c_str());
    }
  }
  else if (rec_data[0] == "I") {
    rec.mode = MINIHEXA_AVOID;
    rec.data[0] = (uint8_t)atoi(rec_data[1].c_str());
  }
  else if (rec_data[0] == "J") {
    rec.mode = MINIHEXA_BALANCE;
    rec.data[0] = (uint8_t)atoi(rec_data[1].c_str());
  }
  else if (rec_data[0] == "L") {
    rec.mode = MINIHEXA_SERVO_CONTROL;
    for (uint8_t i = 0; i < 57; i++) {
      rec.data[i] = (uint8_t)atoi(rec_data[1 + i].c_str());
    }
  }
  else if (rec_data[0] == "M") {
    rec.mode = MINIHEXA_SERVO_DUTY_READ;
  }
  else if (rec_data[0] == "N") {
    rec.mode = MINIHEXA_DISTANCE_READ;
  }
  else if (rec_data[0] == "O") {
    rec.mode = MINIHEXA_RESET;
  }
  else {
    rec.mode = MINIHEXA_NULL;
    memset(rec.data, 0, sizeof(rec.data));
  }
  return rec;
}
