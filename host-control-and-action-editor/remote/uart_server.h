#ifndef UART_SERVER_H_
#define UART_SERVER_H_

#include "Arduino.h"
#include "global.h"

class UartServerManager
{
public:
  RecData_t rec;
  RecData_t pose_stream;
  RecData_t rgb_stream;
  bool pose_stream_pending;
  bool rgb_stream_pending;
  void begin();
  void receive_message();
  void on_receive(void (*callback)(void));
};
#endif
