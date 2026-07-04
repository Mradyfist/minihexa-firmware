#include "hiwonder_robot.h"
#include "Preferences.h"

static HW_Servo servo;

static void leg_servos_to_args(const RobotLeg &leg, ServoArg_t *out) {
  uint8_t base = (uint8_t)((leg.id - 1) * 3);
  out[base + 0] = {(uint8_t)leg.joint_a.id, (uint16_t)((int)leg.joint_a.duty + leg.joint_a.deviation)};
  out[base + 1] = {(uint8_t)leg.joint_b.id, (uint16_t)((int)leg.joint_b.duty + leg.joint_b.deviation)};
  out[base + 2] = {(uint8_t)leg.joint_c.id, (uint16_t)((int)leg.joint_c.duty + leg.joint_c.deviation)};
}

static void flush_leg_servos(RobotLeg &leg1, RobotLeg &leg2, RobotLeg &leg3,
                             RobotLeg &leg4, RobotLeg &leg5, RobotLeg &leg6,
                             uint16_t move_ms) {
  ServoArg_t servos[18];
  leg_servos_to_args(leg1, servos);
  leg_servos_to_args(leg2, servos);
  leg_servos_to_args(leg3, servos);
  leg_servos_to_args(leg4, servos);
  leg_servos_to_args(leg5, servos);
  leg_servos_to_args(leg6, servos);
  servo.multi_set(servos, 18, move_ms);
}
static Preferences preferences;      /* NVS non-volatile storage */

RobotJoint::RobotJoint() {}

void RobotJoint::attach_servo(uint8_t id) {
  this->id = id;
}

void RobotJoint::set_angle(bool dir, float angle, uint16_t time, bool is_ops) {
  uint16_t write_duty;

  if(this->id == 19 || this->id == 20 || this->id == 21) {
    angle = angle >  180.0f ? 180.0f : angle > 0.0f ? angle : 0.0f;
  }
  else {
    angle = angle >  280.0f ? 280.0f : angle > 0.0f ? angle : 0.0f;
  }

  this->dir = dir;
  this->angle = angle;
  this->time = time;
  
  if(this->dir == true) {
    if(this->id == 19 || this->id == 20 || this->id == 21) {
      this->duty = (uint16_t)(500 + (SERVO_ANGLE_180_FACTOR * angle));
    }
    else {
      this->duty = (uint16_t)(500 + (SERVO_ANGLE_280_FACTOR * angle));
    }
  }
  else {
    if(this->id == 19 || this->id == 20 || this->id == 21) {
      this->duty = (uint16_t)(2500 - (SERVO_ANGLE_180_FACTOR * angle));
    }
    else {
      this->duty = (uint16_t)(2500 - (SERVO_ANGLE_280_FACTOR * angle));
    }
  }
  
  write_duty = (uint16_t)((int)this->duty + this->deviation);
  if(is_ops == true)
  {
    servo.set(this->id, write_duty, this->time);
  }
}

void RobotJoint::set_duty(uint16_t duty, uint16_t time, bool is_ops) {
  uint16_t write_duty;

  this->duty = duty;
  this->time = time;
  write_duty = (uint16_t)((int)this->duty + this->deviation);
  if(is_ops == true)
  {
    servo.set(this->id, write_duty, this->time);
  }
}

void RobotJoint::set_deviation(int8_t val) {
  uint16_t write_duty;
  
  deviation = val > 100 ? 100 : val < -100 ? -100 : val;
  write_duty = (uint16_t)(1500 + this->deviation);
  servo.set(this->id, write_duty, 0);
}

int8_t RobotJoint::read_deviation() {
  int8_t read_val = 0; 

  String nvs_name = "knot" + String(id);
  if (!preferences.begin(nvs_name.c_str(), true)) {
    ESP_LOGI("Robot", "Failed to initialize NVS\n");
    preferences.begin(nvs_name.c_str(), false); // open in write mode
    preferences.putInt("value", 0);
    preferences.end();
    ESP_LOGI("Robot", "download deviation value: 0\n");
    return read_val;
  }
  read_val = (int8_t)preferences.getInt("value", 0);
  preferences.end();
  ESP_LOGI("Robot", "%d read deviation value: %d\n", id, read_val);
  return read_val;
}

bool RobotJoint::download_deviation() {
  String nvs_name = "knot" + String(id);
  if (!preferences.begin(nvs_name.c_str(), false)) {
    ESP_LOGE("Robot", "Failed to initialize NVS\n");
    return false;
  }
  preferences.putInt("value", (int)deviation); 
  preferences.end();
  ESP_LOGE("Robot", "download deviation value: %d\n", deviation);
  return true;
}

RobotLeg::RobotLeg() {}

static int pose_stance_hip_sign(uint8_t leg_id) {
  /* Widen support base: right-side hips +, left-side mirrored. */
  return (leg_id <= 3) ? 1 : -1;
}

static void apply_pose_leg_lift(
  RobotLeg &leg, uint8_t leg_id, uint8_t mask, float lift_z, uint8_t sign_flip_mask) {
  if(mask & (1u << (leg_id - 1u))) {
    /* Foot curl is applied after IK; skip Z shift to avoid fighting pose_control. */
    leg.pose_lift_curl = lift_z;
    leg.pose_lift_flip = (sign_flip_mask & (1u << (leg_id - 1u))) != 0;
  }
  else {
    leg.pose_lift_curl = 0.0f;
    leg.pose_lift_flip = false;
  }
}

static void apply_pose_stance(RobotLeg &leg, uint8_t leg_id, uint8_t mask, float spread_blend) {
  if(mask & (1u << (leg_id - 1u)) && spread_blend > 0.0f) {
    leg.pose_stance_hip = spread_blend;
  }
  else {
    leg.pose_stance_hip = 0.0f;
  }
}

static uint32_t gait_min_move_time(uint8_t gait_smooth) {
  return 400u + (uint32_t)gait_smooth * 8u;
}

static void apply_gait_smooth_params(Robot *robot, float *step_length, uint32_t *move_time) {
  if(robot->gait_smooth == 0) {
    return;
  }

  float smooth = robot->gait_smooth / 100.0f;

  *step_length *= 1.0f - 0.3f * smooth;
  *move_time = (uint32_t)((float)(*move_time) * (1.0f + 2.0f * smooth));

  uint32_t min_time = gait_min_move_time(robot->gait_smooth);
  if(*move_time < min_time) {
    *move_time = min_time;
  }
  if((*move_time / 4) % SAMPLE_INTERVAL != 0) {
    *move_time = ((*move_time / 4) / SAMPLE_INTERVAL) * 4 * SAMPLE_INTERVAL;
  }
}

static void clear_pose_leg_adjustments(Robot *robot) {
  robot->leg1.pose_lift_curl = 0.0f;
  robot->leg2.pose_lift_curl = 0.0f;
  robot->leg3.pose_lift_curl = 0.0f;
  robot->leg4.pose_lift_curl = 0.0f;
  robot->leg5.pose_lift_curl = 0.0f;
  robot->leg6.pose_lift_curl = 0.0f;
  robot->leg1.pose_lift_flip = false;
  robot->leg2.pose_lift_flip = false;
  robot->leg3.pose_lift_flip = false;
  robot->leg4.pose_lift_flip = false;
  robot->leg5.pose_lift_flip = false;
  robot->leg6.pose_lift_flip = false;
  robot->leg1.pose_stance_hip = 0.0f;
  robot->leg2.pose_stance_hip = 0.0f;
  robot->leg3.pose_stance_hip = 0.0f;
  robot->leg4.pose_stance_hip = 0.0f;
  robot->leg5.pose_stance_hip = 0.0f;
  robot->leg6.pose_stance_hip = 0.0f;
}

static void apply_all_pose_leg_lifts(Robot *robot) {
  uint8_t mask = robot->pose_leg_lift_mask;
  float lift_z = robot->pose_leg_lift_z;
  uint8_t stance_mask = robot->pose_stance_mask;
  float stance_spread = robot->pose_stance_spread;

  if((mask == 0 || lift_z <= 0.0f) && (stance_mask == 0 || stance_spread <= 0.0f)) {
    clear_pose_leg_adjustments(robot);
    return;
  }

  if(mask == 0 || lift_z <= 0.0f) {
    robot->leg1.pose_lift_curl = 0.0f;
    robot->leg2.pose_lift_curl = 0.0f;
    robot->leg3.pose_lift_curl = 0.0f;
    robot->leg4.pose_lift_curl = 0.0f;
    robot->leg5.pose_lift_curl = 0.0f;
    robot->leg6.pose_lift_curl = 0.0f;
    robot->leg1.pose_lift_flip = false;
    robot->leg2.pose_lift_flip = false;
    robot->leg3.pose_lift_flip = false;
    robot->leg4.pose_lift_flip = false;
    robot->leg5.pose_lift_flip = false;
    robot->leg6.pose_lift_flip = false;
  }
  else {
    apply_pose_leg_lift(robot->leg1, 1, mask, lift_z, robot->pose_leg_lift_sign_flip);
    apply_pose_leg_lift(robot->leg2, 2, mask, lift_z, robot->pose_leg_lift_sign_flip);
    apply_pose_leg_lift(robot->leg3, 3, mask, lift_z, robot->pose_leg_lift_sign_flip);
    apply_pose_leg_lift(robot->leg4, 4, mask, lift_z, robot->pose_leg_lift_sign_flip);
    apply_pose_leg_lift(robot->leg5, 5, mask, lift_z, robot->pose_leg_lift_sign_flip);
    apply_pose_leg_lift(robot->leg6, 6, mask, lift_z, robot->pose_leg_lift_sign_flip);
  }

  if(stance_mask == 0 || stance_spread <= 0.0f) {
    robot->leg1.pose_stance_hip = 0.0f;
    robot->leg2.pose_stance_hip = 0.0f;
    robot->leg3.pose_stance_hip = 0.0f;
    robot->leg4.pose_stance_hip = 0.0f;
    robot->leg5.pose_stance_hip = 0.0f;
    robot->leg6.pose_stance_hip = 0.0f;
  }
  else {
    apply_pose_stance(robot->leg1, 1, stance_mask, stance_spread);
    apply_pose_stance(robot->leg2, 2, stance_mask, stance_spread);
    apply_pose_stance(robot->leg3, 3, stance_mask, stance_spread);
    apply_pose_stance(robot->leg4, 4, stance_mask, stance_spread);
    apply_pose_stance(robot->leg5, 5, stance_mask, stance_spread);
    apply_pose_stance(robot->leg6, 6, stance_mask, stance_spread);
  }
}

bool RobotLeg::move(Vector_t point, uint32_t time, bool send_servo) {
  bool dir;
  float map_time;
  int32_t diff;

  if(fabs(goal_point.x - now_point.x) < 0.0001f &&
     fabs(goal_point.y - now_point.y) < 0.0001f &&
     fabs(goal_point.z - now_point.z) < 0.0001f) {
    is_busy = false;
    current_time = 0;
    goal_time = time;
    goal_point = point;
    start_point = now_point; 
  }
  else {
    is_busy = true;
    /* force-write the target value during motion */
    if(fabs(goal_point.x - point.x) > 0.0001f ||
       fabs(goal_point.y - point.y) > 0.0001f ||
       fabs(goal_point.z - point.z) > 0.0001f) {
      current_time = 0;
      goal_time = time;
      goal_point = point;
      start_point = now_point; 
    }

    diff = (int32_t)goal_time - (int32_t)current_time;
    if(goal_time == 0) {
      current_time = 0;
      map_time = 1;
    }
    else {
      current_time = abs(diff) < SAMPLE_INTERVAL ? goal_time : current_time;
      map_time = fmap((float)current_time, 0, (float)goal_time, 0, 1);
      current_time += SAMPLE_INTERVAL; 
    }

    now_point.x = start_point.x + map_time * (goal_point.x - start_point.x);
    now_point.y = start_point.y + map_time * (goal_point.y - start_point.y);
    now_point.z = start_point.z + map_time * (goal_point.z - start_point.z);      

    _result = ikine(&now_point);   

    if(this->id > 3) {
      dir = false;
    }
    else {
      dir = true;
    }

    this->joint_a.set_angle(true, KNOT_SERVO_MIDPOINT + _result.a, 0, false);
    this->joint_b.set_angle(dir, KNOT_SERVO_MIDPOINT - _result.b, 0, false);
    this->joint_c.set_angle(dir, 230.0f + _result.c, 0, false);

    this->joint_b.duty = this->joint_b.duty > 2143 ? 2143 : (this->joint_b.duty < 857 ? 857 : this->joint_b.duty);

    if(this->pose_lift_curl > 0.0f) {
      /* Mirror diagnostics leg lift: curl femur + foot so toe points up, not just Z shift. */
      static const float kMaxPoseLiftCm = 5.5f;
      static const int kFemurDelta = 220;
      static const int kFootDelta = 300;
      float blend = this->pose_lift_curl / kMaxPoseLiftCm;
      if(blend > 1.0f) {
        blend = 1.0f;
      }
      int femur_off, foot_off;
      if(this->id <= 3) {
        femur_off = -(int)(kFemurDelta * blend);
        foot_off = (int)(kFootDelta * blend);
      }
      else {
        femur_off = (int)(kFemurDelta * blend);
        foot_off = -(int)(kFootDelta * blend);
      }
      if(this->pose_lift_flip) {
        femur_off = -femur_off;
        foot_off = -foot_off;
      }
      this->joint_b.duty += femur_off;
      this->joint_c.duty += foot_off;
      this->joint_b.duty = this->joint_b.duty > 2143 ? 2143 : (this->joint_b.duty < 857 ? 857 : this->joint_b.duty);
      this->joint_c.duty = this->joint_c.duty > 2500 ? 2500 : (this->joint_c.duty < 500 ? 500 : this->joint_c.duty);
    }

    if(this->pose_stance_hip > 0.0f) {
      static const int kMaxHipSpread = 200;
      int hip_off = (int)(kMaxHipSpread * this->pose_stance_hip) * pose_stance_hip_sign(this->id);
      this->joint_a.duty += hip_off;
      this->joint_a.duty = this->joint_a.duty > 2500 ? 2500 : (this->joint_a.duty < 500 ? 500 : this->joint_a.duty);
    }

    ServoArg_t servos[3] = {{this->joint_a.id, (uint16_t)((int)this->joint_a.duty + this->joint_a.deviation)},
                            {this->joint_b.id, (uint16_t)((int)this->joint_b.duty + this->joint_b.deviation)},
                            {this->joint_c.id, (uint16_t)((int)this->joint_c.duty + this->joint_c.deviation)}};
    if(send_servo) {
      servo.multi_set(servos, 3, 0);
    }
  }
  return is_busy;
}

Vector_t RobotLeg::get_now_point() {
  return now_point;
}

RobotArm::RobotArm() {}

Robot::Robot() {
  leg1.id = 1;
  leg1.joint_a.attach_servo(1);
  leg1.joint_b.attach_servo(2);
  leg1.joint_c.attach_servo(3);

  leg2.id = 2;
  leg2.joint_a.attach_servo(4);
  leg2.joint_b.attach_servo(5);
  leg2.joint_c.attach_servo(6);

  leg3.id = 3;
  leg3.joint_a.attach_servo(7);
  leg3.joint_b.attach_servo(8);
  leg3.joint_c.attach_servo(9);

  leg4.id = 4;
  leg4.joint_a.attach_servo(10);
  leg4.joint_b.attach_servo(11);
  leg4.joint_c.attach_servo(12);

  leg5.id = 5; 
  leg5.joint_a.attach_servo(13);
  leg5.joint_b.attach_servo(14);
  leg5.joint_c.attach_servo(15);

  leg6.id = 6;
  leg6.joint_a.attach_servo(16);
  leg6.joint_b.attach_servo(17);
  leg6.joint_c.attach_servo(18);

  arm.joint_a.attach_servo(19);
  arm.joint_b.attach_servo(20);
  arm.joint_c.attach_servo(21);

  leg1.b_leg_start = {LEG1_START_X, LEG1_START_Y, 0};
  leg2.b_leg_start = {LEG2_START_X, LEG2_START_Y, 0};
  leg3.b_leg_start = {LEG3_START_X, LEG3_START_Y, 0};
  leg4.b_leg_start = {LEG4_START_X, LEG4_START_Y, 0};
  leg5.b_leg_start = {LEG5_START_X, LEG5_START_Y, 0};
  leg6.b_leg_start = {LEG6_START_X, LEG6_START_Y, 0};

  leg1.r_leg_start = rotation_trans(&leg2.b_leg_start, -45, AXIS_Z);
  leg2.r_leg_start = rotation_trans(&leg2.b_leg_start, 0, AXIS_Z);
  leg3.r_leg_start = rotation_trans(&leg2.b_leg_start, 45, AXIS_Z);
  leg4.r_leg_start = rotation_trans(&leg2.b_leg_start, 135, AXIS_Z);
  leg5.r_leg_start = rotation_trans(&leg2.b_leg_start, 180, AXIS_Z);
  leg6.r_leg_start = rotation_trans(&leg2.b_leg_start, -135, AXIS_Z);

  leg1.r_leg_end = rotation_trans(&leg1.start_point, -45, AXIS_Z);
  leg2.r_leg_end = rotation_trans(&leg2.start_point, 0, AXIS_Z);
  leg3.r_leg_end = rotation_trans(&leg3.start_point, 45, AXIS_Z);
  leg4.r_leg_end = rotation_trans(&leg4.start_point, 135, AXIS_Z);
  leg5.r_leg_end = rotation_trans(&leg5.start_point, 180, AXIS_Z);
  leg6.r_leg_end = rotation_trans(&leg6.start_point, -135, AXIS_Z);

  leg1.b_leg_end = vector_arg_ops(&leg1.r_leg_end, &leg1.b_leg_start, ADD);
  leg2.b_leg_end = vector_arg_ops(&leg2.r_leg_end, &leg2.b_leg_start, ADD);
  leg3.b_leg_end = vector_arg_ops(&leg3.r_leg_end, &leg3.b_leg_start, ADD);
  leg4.b_leg_end = vector_arg_ops(&leg4.r_leg_end, &leg4.b_leg_start, ADD);
  leg5.b_leg_end = vector_arg_ops(&leg5.r_leg_end, &leg5.b_leg_start, ADD);
  leg6.b_leg_end = vector_arg_ops(&leg6.r_leg_end, &leg6.b_leg_start, ADD);

  leg1.result = pose_control(1, &position, &euler, &leg1.r_leg_end, &leg1.b_leg_start);
  leg2.result = pose_control(2, &position, &euler, &leg2.r_leg_end, &leg2.b_leg_start);
  leg3.result = pose_control(3, &position, &euler, &leg3.r_leg_end, &leg3.b_leg_start);
  leg4.result = pose_control(4, &position, &euler, &leg4.r_leg_end, &leg4.b_leg_start);
  leg5.result = pose_control(5, &position, &euler, &leg5.r_leg_end, &leg5.b_leg_start);
  leg6.result = pose_control(6, &position, &euler, &leg6.r_leg_end, &leg6.b_leg_start); 

  leg1.start_result = leg1.result;
  leg2.start_result = leg2.result;
  leg3.start_result = leg3.result;
  leg4.start_result = leg4.result;
  leg5.start_result = leg5.result;
  leg6.start_result = leg6.result;

  leg1.trajectory_point = leg1.result;
  leg2.trajectory_point = leg2.result;
  leg3.trajectory_point = leg3.result;
  leg4.trajectory_point = leg4.result;
  leg5.trajectory_point = leg5.result;
  leg6.trajectory_point = leg6.result;
}

void Robot::update() {
  switch(func_state) {
    case CRAWL:
      leg1.result = pose_control(1, &position, &euler, &leg1.r_leg_end, &leg1.b_leg_start);
      leg2.result = pose_control(2, &position, &euler, &leg2.r_leg_end, &leg2.b_leg_start);
      leg3.result = pose_control(3, &position, &euler, &leg3.r_leg_end, &leg3.b_leg_start);
      leg4.result = pose_control(4, &position, &euler, &leg4.r_leg_end, &leg4.b_leg_start);
      leg5.result = pose_control(5, &position, &euler, &leg5.r_leg_end, &leg5.b_leg_start);
      leg6.result = pose_control(6, &position, &euler, &leg6.r_leg_end, &leg6.b_leg_start);  

      if(tick_count == move_time) {
        if(_step_num > 0) {
          _step_num--;
        }
      }

      if(_step_num == 0) {
        move_state = STOP;
        last_velocity.vx = 0.001f;
        last_velocity.vy = 0.001f;
        last_velocity.omega = 0.001f;
      }

      switch(move_state) {
        case MOVING: {
          leg1.set_time = 0;
          leg2.set_time = 0;
          leg3.set_time = 0;
          leg4.set_time = 0;
          leg5.set_time = 0;
          leg6.set_time = 0;

          if(gait_mode == GAIT_RIPPLE) {
            float t = tick_count / (float)move_time;
            uint32_t seg = move_time / 6;
            if(seg < SAMPLE_INTERVAL) {
              seg = SAMPLE_INTERVAL;
            }
            uint8_t swing_leg = (tick_count / seg) % 6 + 1;
            uint32_t local = tick_count % seg;
            float lift_sharp = _leg_lift * cosf(PI * local / (float)seg);
            float lift_smooth = _leg_lift * 0.5f * (1.0f - cosf(PI * local / (float)seg));
            float lift_blend = gait_smooth / 100.0f;
            float lift = lift_sharp * (1.0f - lift_blend) + lift_smooth * lift_blend;

            auto ripple_xy = [&](RobotLeg &leg, uint8_t id) {
              float phase = t + (id - 1) / 6.0f;
              float s = sinf(2 * PI * phase);
              leg.trajectory_point.x = leg.result.x + leg.amplitude.x * s;
              leg.trajectory_point.y = leg.result.y + leg.amplitude.y * s;
              leg.trajectory_point.z = (id == swing_leg) ? leg.result.z + lift : leg.result.z;
            };

            ripple_xy(leg1, 1);
            ripple_xy(leg2, 2);
            ripple_xy(leg3, 3);
            ripple_xy(leg4, 4);
            ripple_xy(leg5, 5);
            ripple_xy(leg6, 6);
          }
          else {
            float phase = 2 * PI * tick_count / (float)move_time;
            float cos_p = cosf(phase);
            float lift_blend = gait_smooth / 100.0f;
            bool mid_half = tick_count > (move_time / 4) && tick_count < (3 * move_time / 4);

            float lift_135_sharp = mid_half ? 0.0f : _leg_lift * cos_p;
            float lift_246_sharp = mid_half ? -_leg_lift * cos_p : 0.0f;
            float lift_135_smooth = _leg_lift * fmaxf(0.0f, cos_p);
            float lift_246_smooth = _leg_lift * fmaxf(0.0f, -cos_p);
            float lift_135 = lift_135_sharp * (1.0f - lift_blend) + lift_135_smooth * lift_blend;
            float lift_246 = lift_246_sharp * (1.0f - lift_blend) + lift_246_smooth * lift_blend;

            leg1.trajectory_point.x  = leg1.result.x + leg1.amplitude.x * sinf(phase);
            leg3.trajectory_point.x  = leg3.result.x + leg3.amplitude.x * sinf(phase);
            leg5.trajectory_point.x  = leg5.result.x + leg5.amplitude.x * sinf(phase);

            leg2.trajectory_point.x  = leg2.result.x - leg2.amplitude.x * sinf(phase);
            leg4.trajectory_point.x  = leg4.result.x - leg4.amplitude.x * sinf(phase);
            leg6.trajectory_point.x  = leg6.result.x - leg6.amplitude.x * sinf(phase);

            leg1.trajectory_point.y  = leg1.result.y + leg1.amplitude.y * sinf(phase);
            leg3.trajectory_point.y  = leg3.result.y + leg3.amplitude.y * sinf(phase);
            leg5.trajectory_point.y  = leg5.result.y + leg5.amplitude.y * sinf(phase);

            leg2.trajectory_point.y  = leg2.result.y - leg2.amplitude.y * sinf(phase);
            leg4.trajectory_point.y  = leg4.result.y - leg4.amplitude.y * sinf(phase);
            leg6.trajectory_point.y  = leg6.result.y - leg6.amplitude.y * sinf(phase);

            leg1.trajectory_point.z = leg1.result.z + lift_135;
            leg3.trajectory_point.z = leg3.result.z + lift_135;
            leg5.trajectory_point.z = leg5.result.z + lift_135;

            leg2.trajectory_point.z = leg2.result.z + lift_246;
            leg4.trajectory_point.z = leg4.result.z + lift_246;
            leg6.trajectory_point.z = leg6.result.z + lift_246;
          }
          tick_count = tick_count >= move_time ? 0 : tick_count + SAMPLE_INTERVAL;
          break;
        }

        case STOP:
          tick_count = 0;
          leg1.set_time = 300;
          leg2.set_time = 300;
          leg3.set_time = 300;  
          leg4.set_time = 300;
          leg5.set_time = 300;
          leg6.set_time = 300;  

          leg1.trajectory_point = leg1.result;
          leg2.trajectory_point = leg2.result;
          leg3.trajectory_point = leg3.result;
          leg4.trajectory_point = leg4.result;
          leg5.trajectory_point = leg5.result;
          leg6.trajectory_point = leg6.result;
          apply_all_pose_leg_lifts(this);

          if(fabs(leg1.trajectory_point.z - leg1.now_point.z) < 0.0001f) {
            move_state = REST;
          }
          break;

        case REST:
          leg1.set_time = move_time;
          leg2.set_time = move_time;
          leg3.set_time = move_time;  
          leg4.set_time = move_time;
          leg5.set_time = move_time;
          leg6.set_time = move_time;
          leg1.trajectory_point = leg1.result;
          leg2.trajectory_point = leg2.result;
          leg3.trajectory_point = leg3.result;
          leg4.trajectory_point = leg4.result;
          leg5.trajectory_point = leg5.result;
          leg6.trajectory_point = leg6.result;
          apply_all_pose_leg_lifts(this);
          break;

        default:
          break;
      }
      leg1.move(leg1.trajectory_point, leg1.set_time, false);
      leg2.move(leg2.trajectory_point, leg2.set_time, false);
      leg3.move(leg3.trajectory_point, leg3.set_time, false);
      leg4.move(leg4.trajectory_point, leg4.set_time, false);
      leg5.move(leg5.trajectory_point, leg5.set_time, false);
      leg6.move(leg6.trajectory_point, leg6.set_time, false);
      {
        uint16_t batch_time = leg1.set_time > 0 ? SAMPLE_INTERVAL : 0;
        flush_leg_servos(leg1, leg2, leg3, leg4, leg5, leg6, batch_time);
      }
      break;

    case ACTION_GROUP:
      break;
  }
  // Serial.printf("%.2f, %.2f, %.2f\n", leg1.trajectory_point.x, leg1.trajectory_point.y, leg1.trajectory_point.z);
}

static void pose_update_callback(TimerHandle_t xTimer) {
  Robot *self = static_cast<Robot *>(pvTimerGetTimerID(xTimer));
  self->update();
}

void Robot::begin() {
  uint8_t rgb[3] = {0,237,178};
  Velocity_t vel = {0.0f,0.0f,0.0f};
  Vector_t pos = {0.0f,0.0f,0.0f};
  Euler_t att = {0.0f,0.0f,0.0f};

  leg1.joint_a.deviation = leg1.joint_a.read_deviation();
  leg1.joint_b.deviation = leg1.joint_b.read_deviation();
  leg1.joint_c.deviation = leg1.joint_c.read_deviation();

  leg2.joint_a.deviation = leg2.joint_a.read_deviation();
  leg2.joint_b.deviation = leg2.joint_b.read_deviation();
  leg2.joint_c.deviation = leg2.joint_c.read_deviation();

  leg3.joint_a.deviation = leg3.joint_a.read_deviation();
  leg3.joint_b.deviation = leg3.joint_b.read_deviation();
  leg3.joint_c.deviation = leg3.joint_c.read_deviation();

  leg4.joint_a.deviation = leg4.joint_a.read_deviation();
  leg4.joint_b.deviation = leg4.joint_b.read_deviation();
  leg4.joint_c.deviation = leg4.joint_c.read_deviation();

  leg5.joint_a.deviation = leg5.joint_a.read_deviation();
  leg5.joint_b.deviation = leg5.joint_b.read_deviation();
  leg5.joint_c.deviation = leg5.joint_c.read_deviation();

  leg6.joint_a.deviation = leg6.joint_a.read_deviation();
  leg6.joint_b.deviation = leg6.joint_b.read_deviation();
  leg6.joint_c.deviation = leg6.joint_c.read_deviation();

  if(!SPIFFS.begin(true)) {
    ESP_LOGE("Robot", "SPIFFS Mount Failed\n");
    return;
  }

  servo.begin();

  timer = xTimerCreate("pose_update_timer", 
                       pdMS_TO_TICKS(SAMPLE_INTERVAL), 
                       pdTRUE, 
                       this, 
                       pose_update_callback);
 

  xTimerStart(timer, 0);
  board.begin();
  sensor.begin();
  sensor.set_ultrasound_rgb(RGB_WORK_SOLID_MODE, rgb, rgb);
  // list_action_group_dir();
}

void Robot::crawl_state() {
  if(func_state != CRAWL) {
    leg1.set_time = 1000;
    leg2.set_time = 1000;
    leg3.set_time = 1000;
    leg4.set_time = 1000;
    leg5.set_time = 1000;
    leg6.set_time = 1000;  
    leg1.trans_point = leg1.begin_point;
    leg2.trans_point = leg2.begin_point;
    leg3.trans_point = leg3.begin_point;
    leg4.trans_point = leg4.begin_point;
    leg5.trans_point = leg5.begin_point;
    leg6.trans_point = leg6.begin_point;
    delay(1000);
    func_state = CRAWL;
    move_state = REST;
  }
}

void Robot::cal_omni_move_end_point() {
  float m_radius, abs_vel, vx, vy, m_velocity, max_norm_circle2leg_end = 0;
  float circle2leg_theta[6];
  float norm_circle2leg_end[6];
  float ratio[6];
  float new_step[6];

  Velocity_t vel_90;
  Vector_t vector[6];
  Vector_t circle2leg_end[6];
  Vector_t circle2leg_trans_end[6];

  if(velocity.omega < 0) {
    vx = -(velocity.vx);
    vy = -(velocity.vy);        
  }
  else {
    vx = velocity.vx;
    vy = velocity.vy;         
  }

  abs_vel = (1 / invsqrt(pow(vx, 2) + pow(vy, 2)));
  m_radius =  abs_vel / fabs(velocity.omega);

  /* direction vector of the circle center relative to the body coordinate frame */
  vel_90.vx = -vy;
  vel_90.vy = vx;

  circle_center.x = vel_90.vx >= 0 ? 1 / invsqrt(pow(m_radius, 2) / (1 + pow(vx, 2) / pow(vy, 2))) : 
                                    -1 / invsqrt(pow(m_radius, 2) / (1 + pow(vx, 2) / pow(vy, 2)));

  circle_center.y = -circle_center.x * vx / vy;  

  if(fabs(velocity.vx - 0.001f) < 0.001f &&  fabs(velocity.vy - 0.001f) < 0.001f) {
    step_length = 1 / invsqrt(pow(velocity.omega, 2));
  }
  else {
    step_length = 1 / invsqrt(pow(velocity.vx, 2) + pow(velocity.vy, 2));
  }

  if(step_length > max_half_step_length) {
    move_time = move_time / (step_length / max_half_step_length);
    move_time = move_time < gait_min_move_time(gait_smooth) ? gait_min_move_time(gait_smooth) : move_time;
    step_length = max_half_step_length;
    if((move_time / 4) % SAMPLE_INTERVAL != 0) {
      move_time = ((move_time / 4) / SAMPLE_INTERVAL) * 4 * SAMPLE_INTERVAL;
    }
  }

  apply_gait_smooth_params(this, &step_length, &move_time);

  if(fabs(euler.pitch - 0.0f) < 0.001f && 
     fabs(euler.roll - 0.0f) < 0.001f &&
     fabs(euler.yaw - 0.0f) < 0.001f) { 
    if(position.z >= 0.0f && position.z < 1.0f) {
      _leg_lift = fmap(step_length, 0, max_half_step_length, 1.3f, 2.0f);
    }
    else if(position.z >= 1.0f && position.z < 2.0f) {
      _leg_lift = fmap(step_length, 0, max_half_step_length, 2.5f, 3.0f);
    }
    else if(position.z >= 2.0f && position.z <= 3.0f) {
      _leg_lift = fmap(step_length, 0, max_half_step_length, 3.0f, 3.5f);
    }  
  }
  else {
    if(position.z >= 0.0f && position.z < 1.0f) {
      _leg_lift = fmap(step_length, 0, max_half_step_length, 1.3f, 1.7f);
    }
    else if(position.z >= 1.0f && position.z < 2.0f) {
      _leg_lift = fmap(step_length, 0, max_half_step_length, 1.7f, 2.3f);
    }
    else if(position.z >= 2.0f && position.z <= 3.0f) {
      _leg_lift = fmap(step_length, 0, max_half_step_length, 2.0f, 3.0f);
    }  
  }

  circle2leg_end[0] = vector_arg_ops(&leg1.b_leg_end, &circle_center, SUB);
  circle2leg_end[1] = vector_arg_ops(&leg2.b_leg_end, &circle_center, SUB);
  circle2leg_end[2] = vector_arg_ops(&leg3.b_leg_end, &circle_center, SUB);
  circle2leg_end[3] = vector_arg_ops(&leg4.b_leg_end, &circle_center, SUB);
  circle2leg_end[4] = vector_arg_ops(&leg5.b_leg_end, &circle_center, SUB);
  circle2leg_end[5] = vector_arg_ops(&leg6.b_leg_end, &circle_center, SUB);

  for(uint8_t i = 0; i < 6; i++)
  {
    norm_circle2leg_end[i] = 1 / invsqrt(pow(circle2leg_end[i].x, 2) + pow(circle2leg_end[i].y, 2));      
    if(norm_circle2leg_end[i] > max_norm_circle2leg_end)
    {
        max_norm_circle2leg_end = norm_circle2leg_end[i];
    }
  }

  for(uint8_t i = 0; i < 6; i++)
  {
    ratio[i] = norm_circle2leg_end[i] / max_norm_circle2leg_end;
    new_step[i] = ratio[i] * step_length;
    if(velocity.omega >= 0)
    {
        circle2leg_theta[i] = RAD_TO_THETA(new_step[i] / norm_circle2leg_end[i]);
    }
    else
    {
        circle2leg_theta[i] = RAD_TO_THETA(-(new_step[i]) / norm_circle2leg_end[i]);
    }
    circle2leg_trans_end[i] = rotation_trans(&circle2leg_end[i], circle2leg_theta[i], AXIS_Z);
    vector[i] = vector_arg_ops(&circle_center, &circle2leg_trans_end[i], ADD);
  }

  vector[0] = vector_arg_ops(&vector[0], &leg1.b_leg_start, SUB);
  vector[1] = vector_arg_ops(&vector[1], &leg2.b_leg_start, SUB);
  vector[2] = vector_arg_ops(&vector[2], &leg3.b_leg_start, SUB);
  vector[3] = vector_arg_ops(&vector[3], &leg4.b_leg_start, SUB);
  vector[4] = vector_arg_ops(&vector[4], &leg5.b_leg_start, SUB);
  vector[5] = vector_arg_ops(&vector[5], &leg6.b_leg_start, SUB);

  leg1.omni_move_end_point = rotation_trans(&vector[0], 45, AXIS_Z);
  leg2.omni_move_end_point = rotation_trans(&vector[1], 0, AXIS_Z);
  leg3.omni_move_end_point = rotation_trans(&vector[2], -45, AXIS_Z);
  leg4.omni_move_end_point = rotation_trans(&vector[3], -135, AXIS_Z);
  leg5.omni_move_end_point = rotation_trans(&vector[4], -180, AXIS_Z);
  leg6.omni_move_end_point = rotation_trans(&vector[5], 135, AXIS_Z);
}

void Robot::move(Velocity_t *_velocity, Vector_t *_position, Euler_t *_euler, uint32_t time, int step_num) {
  _step_num = step_num;
  tick_count = _step_num == last_step_num ? tick_count : 0;
  last_step_num = _step_num;
  
  func_state = CRAWL;

  position.x = LIMIT(_position->x, -4.0f, 4.0f);
  position.y = LIMIT(_position->y, -4.0f, 4.0f);
  position.z = LIMIT(_position->z, -4.0f, 4.0f);

  euler.pitch = _euler->pitch;
  euler.roll = _euler->roll;
  euler.yaw = LIMIT(_euler->yaw, -20.0f, 20.0f);

  velocity.vx = _velocity->vx > 0.001f ?  _velocity->vx : (_velocity->vx < -0.001f ? _velocity->vx : 0.001f);
  velocity.vy = _velocity->vy > 0.001f ?  _velocity->vy : (_velocity->vy < -0.001f ? _velocity->vy : 0.001f);
  velocity.omega = _velocity->omega > 0.001f ?  _velocity->omega : (_velocity->omega < -0.001f ? _velocity->omega : 0.001f);

  if(fabs(velocity.vx - 0.001f) < 0.0001f && 
     fabs(velocity.vy - 0.001f) < 0.0001f && 
     fabs(velocity.omega - 0.001f) < 0.0001f) {

    if(fabs(last_velocity.vx - 0.001f) > 0.0001f || 
      fabs(last_velocity.vy - 0.001f) > 0.0001f || 
      fabs(last_velocity.omega - 0.001f) > 0.0001f) { 
        move_state = STOP;
    }
    else {
      move_state = REST;
      move_time = (time == 0) ? 0 : (time > SAMPLE_INTERVAL ? time : SAMPLE_INTERVAL);
    }
  }
    else {
    euler.roll = 0.0f;
    euler.yaw = 0.0f;
    if(gait_smooth > 0) {
      uint32_t min_time = gait_min_move_time(gait_smooth);
      move_time = time > min_time ? time : min_time;
    }
    else {
      move_time = time > SAMPLE_INTERVAL ? time : SAMPLE_INTERVAL;
    }
    move_state = MOVING;
    cal_omni_move_end_point();  
    leg1.amplitude = vector_arg_ops(&leg1.omni_move_end_point, &leg1.start_result, SUB);
    leg2.amplitude = vector_arg_ops(&leg2.omni_move_end_point, &leg2.start_result, SUB);
    leg3.amplitude = vector_arg_ops(&leg3.omni_move_end_point, &leg3.start_result, SUB);
    leg4.amplitude = vector_arg_ops(&leg4.omni_move_end_point, &leg4.start_result, SUB);
    leg5.amplitude = vector_arg_ops(&leg5.omni_move_end_point, &leg5.start_result, SUB);
    leg6.amplitude = vector_arg_ops(&leg6.omni_move_end_point, &leg6.start_result, SUB);
  }
  last_velocity = velocity;  
}

void Robot::reset(void) {
  pose_leg_lift_mask = 0;
  pose_leg_lift_sign_flip = 0;
  pose_leg_lift_z = 0.0f;
  pose_stance_mask = 0;
  pose_stance_spread = 0.0f;
  leg1.pose_lift_curl = 0.0f;
  leg2.pose_lift_curl = 0.0f;
  leg3.pose_lift_curl = 0.0f;
  leg4.pose_lift_curl = 0.0f;
  leg5.pose_lift_curl = 0.0f;
  leg6.pose_lift_curl = 0.0f;
  leg1.pose_lift_flip = false;
  leg2.pose_lift_flip = false;
  leg3.pose_lift_flip = false;
  leg4.pose_lift_flip = false;
  leg5.pose_lift_flip = false;
  leg6.pose_lift_flip = false;
  leg1.pose_stance_hip = 0.0f;
  leg2.pose_stance_hip = 0.0f;
  leg3.pose_stance_hip = 0.0f;
  leg4.pose_stance_hip = 0.0f;
  leg5.pose_stance_hip = 0.0f;
  leg6.pose_stance_hip = 0.0f;

  Velocity_t vel = {0.0f, 0.0f, 0.0f};
  Vector_t pos = {0.0f, 0.0f, 1.0f};
  Euler_t att = {0.0f, 0.0f, 0.0f};
  move(&vel, &pos, &att, 600);
  delay(600);
}

void Robot::avoid(uint16_t dis) {
  Velocity_t vel = {0.0f, 0.0f, 0.0f};
  Vector_t pos = {0.0f, 0.0f, 0.0f};
  Euler_t att = {0.0f, 0.0f, 0.0f};

  pos = position;
  switch(avoid_state) {
    case FORWARD:
      vel = {0.0f, 2.0f, 0.0f};
      move(&vel, &pos, &att, 800);      
      if(dis < 200 && dis > 100) {
        avoid_state = TURN;
      }
      else if(dis < 100) {
        avoid_state = BACK;
      }
      break;

    case BACK:
      vel = {0.0f, -2.0f, 0.0f};
      move(&vel, &pos, &att, 800);
      if(dis < 200 && dis > 100) {
        avoid_state = TURN;
      }
      else if(dis > 200) {
        avoid_state = FORWARD;
      }
      break;
      
    case TURN:
      vel = {0.0f, 0.0f, 1.8f};
      move(&vel, &pos, &att, 1000, 4);
      avoid_state = WAIT;
      break;
    
    case WAIT:
      if(_step_num == 0) {
        if(dis > 200) {
          avoid_state = FORWARD;
        }
        else if(dis < 200 && dis > 100) {
          avoid_state = TURN;
        }
        else if(dis < 100) {
          avoid_state = BACK;
        }        
      }
      break;
  }
}

void Robot::balance(bool state) {
  float euler[3];
  Velocity_t _velocity = {0.0f, 0.0f, 0.0f};
  Vector_t _position = {0.0f, 0.0f, 0.0f};
  Euler_t _euler = {0.0f, 0.0f, 0.0f};

  if(state == true) {
    if(millis() - balance_tick_start > 50) {
      board.imu_update(true);
      _position = position;
      board.get_imu_euler(euler);
      euler[0] = euler[0] > 18.0f ? 18.0f : (euler[0] < -18.0f ? -18.0f : euler[0]);
      euler[1] = euler[1] > 18.0f ? 18.0f : (euler[1] < -18.0f ? -18.0f : euler[1]);
      _euler = {-euler[0],-euler[1],0};
      move(&_velocity, &_position, &_euler, 100);
      balance_tick_start = millis();
    }
  }
  else {
    board.imu_update(false);
    balance_tick_start = 0;
  }
}

bool Robot::next_circular_point(CircularPath* path, float* x, float* y) {
    const uint16_t total_steps = path->steps_per_circle * path->total_circles;
    
    /* all steps completed (including returning to the origin)*/
    if (path->is_completed) {
        *x = 0.0f;
        *y = 0.0f;
        return true;
    }
    
    /* circular motion completed, but not yet returned to the origin */
    if (path->current_step == total_steps) {
        *x = 0.0f;
        *y = 0.0f;
        path->is_completed = true;
        return true;
    }
    
    /* normal circular motion */
    float angle = 2.0f * PI * ((float)path->current_step / path->steps_per_circle) * path->direction;
    *x = path->r * cosf(angle);
    *y = path->r * sinf(angle);
    
    path->current_step++;
    return false;
}

void Robot::twist(float radius, uint16_t circles, uint16_t steps_per_circle, RotationDirection dir) {
  CircularPath path;
  bool start_state = false;
  Velocity_t vel = {0.0f,0.0f,0.0f};
  Vector_t pos = {0.0f,0.0f,0.0f};
  Euler_t att = {0.0f,0.0f,0.0f};

  pos = position;
  path.r = radius;
  path.total_circles = circles;
  path.steps_per_circle = steps_per_circle;
  path.direction = dir;  
  path.current_step = 0;
  path.is_completed = false;

  while (!next_circular_point(&path, &att.roll, &att.pitch)) {
    if(start_state == false) {
      start_state = true;
      move(&vel, &pos, &att, 500);
      delay(500);
    }
    else {
      move(&vel, &pos, &att, 100);
      delay(100);
    }
  }

  att = {0.0f,0.0f,0.0f};
  move(&vel, &pos, &att, 500);
  delay(500);
}

void Robot::acting_cute() {
  Velocity_t vel = {0.0f,0.0f,0.0f};
  Vector_t pos = {0.0f,0.0f,0.0f};
  Euler_t att = {0.0f,0.0f,0.0f};

  pos = position;
  att = {0.0f,10.0f,0.0f};
  move(&vel, &pos, &att, 300);
  delay(300);
  att = {0.0f,-10.0f,0.0f};
  move(&vel, &pos, &att, 300);
  delay(300);
  att = {0.0f,10.0f,0.0f};
  move(&vel, &pos, &att, 300);
  delay(300);
  att = {0.0f,-10.0f,0.0f};
  move(&vel, &pos, &att, 300);
  delay(300);
  att = {0.0f,0.0f,0.0f};
  move(&vel, &pos, &att, 300);
  delay(300);
  att = {0.0f,0.0f,0.0f};
  pos = {2.0f,0.0f,position.z};
  move(&vel, &pos, &att, 200);
  delay(200);
  att = {0.0f,0.0f,0.0f};
  pos = {-2.0f,0.0f,position.z};  
  move(&vel, &pos, &att, 200);
  delay(200);
  att = {0.0f,0.0f,0.0f};
  pos = {2.0f,0.0f,position.z};
  move(&vel, &pos, &att, 200);
  delay(200);
  att = {0.0f,0.0f,0.0f};
  pos = {-2.0f,0.0f,position.z};  
  move(&vel, &pos, &att, 200);
  delay(200);
  att = {0.0f,0.0f,0.0f};
  pos = {0.0f,0.0f,position.z};  
  move(&vel, &pos, &att, 200);
  delay(200);
}

void Robot::apply_pose_command(
  int8_t yaw_p, int8_t pitch_p, int8_t roll_p,
  int8_t x_p, int8_t y_p, int8_t z_p,
  uint8_t lift_mask, int8_t lift_z_p, uint8_t lift_sign_flip,
  uint8_t stance_mask, int8_t stance_spread_p,
  uint32_t move_ms) {
  Velocity_t vel = {0.0f, 0.0f, 0.0f};
  Vector_t pos = {
    fmap((float)x_p, -50.0f, 50.0f, -3.0f, 3.0f),
    fmap((float)y_p, -50.0f, 50.0f, -3.0f, 3.0f),
    fmap((float)z_p, 0.0f, 30.0f, 0.0f, 3.0f),
  };
  Euler_t att;
  att.pitch = fmap((float)pitch_p, -50.0f, 50.0f, -20.0f, 20.0f);
  att.roll = fmap((float)roll_p, -50.0f, 50.0f, -20.0f, 20.0f);
  att.yaw = fmap(-(float)yaw_p, -50.0f, 50.0f, -60.0f, 60.0f);

  pose_leg_lift_mask = lift_mask;
  pose_leg_lift_sign_flip = lift_sign_flip;
  pose_leg_lift_z = lift_z_p > 0 ? fmap((float)lift_z_p, 0.0f, 30.0f, 0.0f, 5.5f) : 0.0f;
  pose_stance_mask = stance_mask;
  pose_stance_spread = stance_spread_p > 0 ? fmap((float)stance_spread_p, 0.0f, 30.0f, 0.0f, 1.0f) : 0.0f;

  func_state = CRAWL;
  move(&vel, &pos, &att, move_ms);
}

void Robot::wait_pose_ms(uint32_t ms) {
  if(ms == 0) {
    return;
  }

  xTimerStop(timer, portMAX_DELAY);
  uint32_t end = millis() + ms;
  while((int32_t)(millis() - end) < 0) {
    update();
    delay(SAMPLE_INTERVAL);
  }
  xTimerStart(timer, portMAX_DELAY);
}

static void rear_pose_step(
  Robot *robot,
  int8_t yaw_p,
  int8_t pitch_p,
  int8_t roll_p,
  int8_t x_p,
  int8_t y_p,
  int8_t z_p,
  int8_t lift_p,
  int8_t stance_p,
  uint32_t move_ms,
  uint32_t hold_ms) {
  /* Bit i => leg i+1. Front pair 3/4 lift; hind/mid support 1/2/5/6 spread. */
  static const uint8_t kLiftMask = (1u << 2) | (1u << 3);
  static const uint8_t kStanceMask = (1u << 0) | (1u << 1) | (1u << 4) | (1u << 5);
  /* No sign flip: leg 4 default left-side curl lifts outward in pitched rear-up. */
  static const uint8_t kSignFlip = 0;

  robot->apply_pose_command(
    yaw_p,
    pitch_p,
    roll_p,
    x_p,
    y_p,
    z_p,
    lift_p > 0 ? kLiftMask : 0,
    lift_p,
    lift_p > 0 ? kSignFlip : 0,
    stance_p > 0 ? kStanceMask : 0,
    stance_p,
    move_ms);
  robot->wait_pose_ms(hold_ms);
}

void Robot::rear_up(void) {
  static const int8_t kRearPitch = -63;
  static const int8_t kShiftBack = -33;
  static const int8_t kBodyZ = 12;
  static const int8_t kFrontRiseLift = 30;
  static const int8_t kFrontWaveLift = 40;
  static const int8_t kRearStanceSpread = 25;
  static const float kRollScale = 2.5f;
  static const float kRearRollDeg = 14.0f;
  static const float kYawWiggleDeg = 10.0f;
  static const uint16_t kRiseSteps = 4;
  static const uint16_t kWavePrepSteps = 2;
  static const uint16_t kWiggleCycles = 3;
  static const uint16_t kStepsPerCycle = 16;

  func_state = CRAWL;
  pose_leg_lift_mask = 0;
  pose_leg_lift_sign_flip = 0;
  pose_leg_lift_z = 0.0f;
  pose_stance_mask = 0;
  pose_stance_spread = 0.0f;
  clear_pose_leg_adjustments(this);

  rear_pose_step(this, 0, 0, 0, 0, 0, 0, 0, 0, 200, 300);

  for(uint16_t step = 1; step <= kRiseSteps; step++) {
    float t = (float)step / (float)kRiseSteps;
    rear_pose_step(
      this,
      0,
      (int8_t)lroundf((float)kRearPitch * t),
      0,
      0,
      (int8_t)lroundf((float)kShiftBack * t),
      (int8_t)lroundf((float)kBodyZ * t),
      (int8_t)lroundf((float)kFrontRiseLift * t),
      (int8_t)lroundf((float)kRearStanceSpread * t),
      400,
      400);
  }

  for(uint16_t step = 1; step <= kWavePrepSteps; step++) {
    float t = (float)step / (float)kWavePrepSteps;
    int8_t lift = (int8_t)lroundf(
      (float)kFrontRiseLift + t * (float)(kFrontWaveLift - kFrontRiseLift));
    rear_pose_step(
      this,
      0,
      kRearPitch,
      0,
      0,
      kShiftBack,
      kBodyZ,
      lift,
      kRearStanceSpread,
      350,
      350);
  }

  uint16_t total_steps = kWiggleCycles * kStepsPerCycle;
  for(uint16_t step = 0; step < total_steps; step++) {
    float angle = 2.0f * PI * (float)step / (float)kStepsPerCycle;
    int8_t roll = (int8_t)lroundf(kRearRollDeg * sinf(angle) * kRollScale);
    int8_t yaw = (int8_t)lroundf(-kYawWiggleDeg * sinf(2.0f * angle) * 50.0f / 60.0f);
    float lift_bob = 0.75f + 0.25f * (1.0f + sinf(2.0f * angle)) / 2.0f;
    int8_t lift = (int8_t)lroundf((float)kFrontWaveLift * lift_bob);
    rear_pose_step(
      this,
      yaw,
      kRearPitch,
      roll,
      0,
      kShiftBack,
      kBodyZ,
      lift,
      kRearStanceSpread,
      100,
      100);
  }

  rear_pose_step(this, 0, 0, 0, 0, 0, 0, 0, 0, 500, 500);
  pose_leg_lift_mask = 0;
  pose_leg_lift_sign_flip = 0;
  pose_leg_lift_z = 0.0f;
  pose_stance_mask = 0;
  pose_stance_spread = 0.0f;
  clear_pose_leg_adjustments(this);
}

void Robot::wake_up() {
  Velocity_t vel = {0.0f,0.0f,0.0f};
  Vector_t pos = {0.0f,0.0f,0.0f};
  Euler_t att = {0.0f,0.0f,0.0f};

  pos = position;
  move(&vel, &pos, &att, 200);
  delay(100);
  att = {-8.0f,0.0f,-15.0f};
  move(&vel, &pos, &att, 200);
  delay(1000);
  att = {-8.0f,0.0f,15.0f};
  move(&vel, &pos, &att, 200);
  delay(1000);
  vel = {0.0f,0.0f,0.0f};
  att = {0.0f,0.0f,0.0f};
  move(&vel, &pos, &att, 200);
  delay(600);
  vel = {0.0f,1.5f,0.0f};
  att = {0.0f,0.0f,0.0f};
  move(&vel, &pos, &att, 500, 10);
  delay(6000);
}

void Robot::_wake_up() {
  Velocity_t vel = {0.0f,0.0f,0.0f};
  Vector_t pos = {0.0f,0.0f,0.0f};
  Euler_t att = {0.0f,0.0f,0.0f};

  pos = position;
  att = {-8.0f,0.0f,-15.0f};
  move(&vel, &pos, &att, 200);
  delay(1000);
  att = {-8.0f,0.0f,15.0f};
  move(&vel, &pos, &att, 600);
  delay(1000);
  vel = {0.0f,0.0f,0.0f};
  att = {0.0f,0.0f,0.0f};
  move(&vel, &pos, &att, 200);
  delay(200);
}

void Robot::list_action_group_dir() {
  ESP_LOGI("Robot", "Listing directory: /\n");
  File root = SPIFFS.open("/");
  if(!root) {
    ESP_LOGE("Robot", "- failed to open directory\n");
    return;
  }
  if(!root.isDirectory()){
    ESP_LOGI("Robot", " - not a directory\n");
    return;
  }

  File file = root.openNextFile();
  while(file) {
    if(file.isDirectory()) {
      ESP_LOGI("Robot", "DIR:%s\n", file.name());
    } 
    else {
      ESP_LOGI("Robot", "FILE:%s SIZE: %d\n", file.name(), file.size());
    }
    file = root.openNextFile();
  }
}

void Robot::action_group_stop(void) {
  act_state = ACT_STOP;
} 

void Robot::action_group_run(uint8_t id) {
  uint8_t offset;
  uint8_t frame_index;
  uint8_t control_num;
  uint8_t count = 0;
  uint8_t buf[58] = {0}; /* frame_index|control_num|time_l|time_h|id|duty_l|duty_h|... */
  uint16_t move_time;
  ServoArg_t servos[18];

  func_state = ACTION_GROUP;
  act_state = READ_FRAME_NUM;
  File file = SPIFFS.open("/ActionGroup" + String(id) + ".rob", FILE_READ);
  if(!file) {
    ESP_LOGI("Robot", "Failed to open file for reading\n");
  }  

  while(file.available()) {
    switch(act_state) {
      case READ_FRAME_NUM: /*read the frame header*/
        act_read_frame_num = (uint8_t)file.read();
        act_state = READ_FRAME_DATA;
        break;

      case READ_FRAME_DATA:
        file.read(buf, sizeof(buf));
        control_num = buf[1];
        frame_index = buf[0];
        move_time = BYTE_TO_HW(buf[3], buf[2]); 
        if(id == 0) {
          Serial.printf("$$>%d<$$", frame_index);
        }
        ESP_LOGI("Robot", "id: %d frame_num: %d frame_index: %d control_num: %d move_time: %d\n", id, act_read_frame_num, frame_index, control_num, move_time);
        for(uint8_t i = 0; i < control_num; i++) {
          servos[i].id = buf[4 + i * 3];
          servos[i].duty = BYTE_TO_HW(buf[6 + i * 3], buf[5 + i * 3]); 
          ESP_LOGI("Robot", "id: %d duty: %d\n",  servos[i].id, servos[i].duty);
        }
        servo.multi_set(servos, control_num, move_time);
        delay(move_time);
        if(frame_index == act_read_frame_num) {
          act_state = ACT_STOP;
          if(id == 0) {
            Serial.printf("$$>end<$$");
          }
        }
        break;

      default:
        break;
    }  

    if(act_state == ACT_STOP) {
      break;
    }
  }

  file.close();
}

bool Robot::action_group_download(uint8_t id, uint8_t *data, size_t length) {
  size_t written;
  size_t len;
  uint8_t frame_index = data[2];

  if(id > 50) {
    ESP_LOGE("Robot", "id error!\n");
    return false;
  }

  if(frame_index > 254) {
    ESP_LOGE("Robot", "Frame index error!\n");
    return false;    
  }

  if(frame_index == 1) {
    File file = SPIFFS.open("/ActionGroup" + String(id) + ".rob", FILE_WRITE);
    if(!file) {
      ESP_LOGI("Robot", "- failed to open file for downloading\n");
      return false;
    }
    len = length - 1;
    written = file.write(&data[1], len);
    file.close();
  }
  else {
    File file = SPIFFS.open("/ActionGroup" + String(id) + ".rob", FILE_APPEND);
    if(!file) {
      ESP_LOGI("Robot", "- failed to open file for downloading\n");
      return false;
    }
    len = length - 2;
    written = file.write(&data[2], len);
    file.close();
  }

  if(written == len) {
    ESP_LOGI("Robot", "- %u bytes downloading\n", written);
  }
  else {
    ESP_LOGI("Robot", "- downloading failed, only wrote %u of %u bytes\n", written, length);
  }
  return true;
}

bool Robot::action_group_erase(uint8_t id) {
  if(id > 50) {
    ESP_LOGE("Robot", "ID ERROR!\n");
    return false;
  }

  File file = SPIFFS.open("/ActionGroup" + String(id) + ".rob", FILE_WRITE);
  if(!file) {
    ESP_LOGE("Robot", "- failed to open file for writing\n");
    return false;
  }
  file.close();
  return true;
}

void Robot::multi_servo_control(ServoArg_t* arg, uint16_t servo_num, uint16_t time) {
  for(uint8_t i = 0; i < servo_num; i++) {
    switch(arg[i].id) {
      case 1:
        leg1.joint_a.duty = arg[i].duty;
        break;

      case 2:
        leg1.joint_b.duty = arg[i].duty;
        break;

      case 3:
        leg1.joint_c.duty = arg[i].duty;
        break;

      case 4:
        leg2.joint_a.duty = arg[i].duty;
        break;

      case 5:
        leg2.joint_b.duty = arg[i].duty;
        break;

      case 6:
        leg2.joint_c.duty = arg[i].duty;
        break;

      case 7:
        leg3.joint_a.duty = arg[i].duty;
        break;

      case 8:
        leg3.joint_b.duty = arg[i].duty;
        break;

      case 9:
        leg3.joint_c.duty = arg[i].duty;
        break;

      case 10:
        leg4.joint_a.duty = arg[i].duty;
        break;

      case 11:
        leg4.joint_b.duty = arg[i].duty;
        break;

      case 12:
        leg4.joint_c.duty = arg[i].duty;
        break;

      case 13:
        leg5.joint_a.duty = arg[i].duty;
        break;

      case 14:
        leg5.joint_b.duty = arg[i].duty;
        break;

      case 15:
        leg5.joint_c.duty = arg[i].duty;
        break;

      case 16:
        leg6.joint_a.duty = arg[i].duty;
        break;

      case 17:
        leg6.joint_b.duty = arg[i].duty;
        break;

      case 18:
        leg6.joint_c.duty = arg[i].duty;
        break;
    }
  }
  servo.multi_set(arg, servo_num, time);
}

void Robot::set_deviation(uint8_t id, int8_t val) {
  switch(id) {
    case 1:
      leg1.joint_a.set_deviation(val);
      break;

    case 2:
      leg1.joint_b.set_deviation(val);
      break;

    case 3:
      leg1.joint_c.set_deviation(val);
      break;

    case 4:
      leg2.joint_a.set_deviation(val);
      break;

    case 5:
      leg2.joint_b.set_deviation(val);
      break;

    case 6:
      leg2.joint_c.set_deviation(val);
      break;

    case 7:
      leg3.joint_a.set_deviation(val);
      break;

    case 8:
      leg3.joint_b.set_deviation(val);
      break;

    case 9:
      leg3.joint_c.set_deviation(val);
      break;

    case 10:
      leg4.joint_a.set_deviation(val);
      break;

    case 11:
      leg4.joint_b.set_deviation(val);
      break;
  
    case 12:
      leg4.joint_c.set_deviation(val);
      break;

    case 13:
      leg5.joint_a.set_deviation(val);
      break;

    case 14:
      leg5.joint_b.set_deviation(val);
      break;

    case 15:
      leg5.joint_c.set_deviation(val);
      break;

    case 16:
      leg6.joint_a.set_deviation(val);
      break;

    case 17:
      leg6.joint_b.set_deviation(val);
      break;

    case 18:
      leg6.joint_c.set_deviation(val);
      break;
  
    default:
      break;
  }
}

void Robot::read_deviation(int8_t *val) {
  val[0] = leg1.joint_a.deviation;
  val[1] = leg1.joint_b.deviation;
  val[2] = leg1.joint_c.deviation;

  val[3] = leg2.joint_a.deviation;
  val[4] = leg2.joint_b.deviation;
  val[5] = leg2.joint_c.deviation;

  val[6] = leg3.joint_a.deviation;
  val[7] = leg3.joint_b.deviation;
  val[8] = leg3.joint_c.deviation;

  val[9] = leg4.joint_a.deviation;
  val[10] = leg4.joint_b.deviation;
  val[11] = leg4.joint_c.deviation;

  val[12] = leg5.joint_a.deviation;
  val[13] = leg5.joint_b.deviation;
  val[14] = leg5.joint_c.deviation;

  val[15] = leg6.joint_a.deviation;
  val[16] = leg6.joint_b.deviation;
  val[17] = leg6.joint_c.deviation;
}

bool Robot::download_deviation(void) {
  uint8_t count = 0;

  for(uint8_t i = 0; i < 18; i++) {
    switch(count) {
      case 0:
        if(leg1.joint_a.download_deviation()){
          count++;
        }
        else {
          return false;
        }
        break;

      case 1:
        if(leg1.joint_b.download_deviation()){
          count++;
        }
        else {
          return false;
        }
        break;
      
      case 2:
        if(leg1.joint_c.download_deviation()){
          count++;
        }
        else {
          return false;
        }
        break;

      case 3:
        if(leg2.joint_a.download_deviation()){
          count++;
        }
        else {
          return false;
        }
        break;

      case 4:
        if(leg2.joint_b.download_deviation()){
          count++;
        }
        else {
          return false;
        }
        break;

      case 5:
        if(leg2.joint_c.download_deviation()){
          count++;
        }
        else {
          return false;
        }
        break;

      case 6:
        if(leg3.joint_a.download_deviation()){
          count++;
        }
        else {
          return false;
        }
        break;

      case 7:
        if(leg3.joint_b.download_deviation()){
          count++;
        }
        else {
          return false;
        }
        break;

      case 8:
        if(leg3.joint_c.download_deviation()){
          count++;
        }
        else {
          return false;
        }
        break;

      case 9:
        if(leg4.joint_a.download_deviation()){
          count++;
        }
        else {
          return false;
        }
        break;

      case 10:
        if(leg4.joint_b.download_deviation()){
          count++;
        }
        else {
          return false;
        }
        break;

      case 11:
        if(leg4.joint_c.download_deviation()){
          count++;
        }
        else {
          return false;
        }
        break;

      case 12:
        if(leg5.joint_a.download_deviation()){
          count++;
        }
        else {
          return false;
        }
        break;

      case 13:
        if(leg5.joint_b.download_deviation()){
          count++;
        }
        else {
          return false;
        }
        break;

      case 14:
        if(leg5.joint_c.download_deviation()){
          count++;
        }
        else {
          return false;
        }
        break;

      case 15:
        if(leg6.joint_a.download_deviation()){
          count++;
        }
        else {
          return false;
        }
        break;

      case 16:
        if(leg6.joint_b.download_deviation()){
          count++;
        }
        else {
          return false;
        }
        break;

      case 17:
        if(!leg6.joint_c.download_deviation()){
          return false;
        }
        break;
    }
  }


  return true;
}