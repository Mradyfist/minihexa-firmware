#include "pose_dispatch.h"

#define POSE_QUEUE_SIZE 24

static RecData_t g_pose_queue[POSE_QUEUE_SIZE];
static volatile uint8_t g_pose_q_head = 0;
static volatile uint8_t g_pose_q_tail = 0;
static PoseFrameHandler g_pose_handler = nullptr;

void set_pose_frame_handler(PoseFrameHandler handler) {
  g_pose_handler = handler;
}

void dispatch_pose_frame(const RecData_t &msg) {
  uint8_t next = (uint8_t)((g_pose_q_head + 1) % POSE_QUEUE_SIZE);
  if(next == g_pose_q_tail) {
    g_pose_q_tail = (uint8_t)((g_pose_q_tail + 1) % POSE_QUEUE_SIZE);
  }
  g_pose_queue[g_pose_q_head] = msg;
  g_pose_q_head = next;
}

void drain_pose_frames(void) {
  if(g_pose_handler == nullptr) {
    g_pose_q_tail = g_pose_q_head;
    return;
  }
  while(g_pose_q_tail != g_pose_q_head) {
    g_pose_handler(g_pose_queue[g_pose_q_tail]);
    g_pose_q_tail = (uint8_t)((g_pose_q_tail + 1) % POSE_QUEUE_SIZE);
  }
}

void clear_pose_frames(void) {
  g_pose_q_tail = g_pose_q_head;
}
