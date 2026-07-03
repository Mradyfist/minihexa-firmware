#ifndef COMMAND_PARSER_H_
#define COMMAND_PARSER_H_

#include "global.h"
#include "Arduino.h"

RecData_t parse_command_frame(const String &data);

#endif
