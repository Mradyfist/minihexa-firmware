#ifndef POSE_DISPATCH_H_
#define POSE_DISPATCH_H_

#include "global.h"

typedef void (*PoseFrameHandler)(const RecData_t &msg);

void set_pose_frame_handler(PoseFrameHandler handler);
void dispatch_pose_frame(const RecData_t &msg);
void drain_pose_frames(void);
void clear_pose_frames(void);

#endif
