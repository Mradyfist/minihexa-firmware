#ifndef __HIWONDER_ROBOT_H__
#define __HIWONDER_ROBOT_H__

#include "hiwonder_servo.h"
#include "hiwonder_board.h"
#include "hiwonder_sensor.h"
#include "kinematics.h"
#include "SPIFFS.h"



#ifdef __cplusplus
extern "C" 
{
#endif
#define SAMPLE_INTERVAL 10

#define KNOT_SERVO_MIDPOINT   140.0f

#define SERVO_ANGLE_180_FACTOR    11.11111111111111f
#define SERVO_ANGLE_280_FACTOR    7.142857142857143f

#define LEG1_START_X    3.15f
#define LEG1_START_Y   -5.65f

#define LEG2_START_X    4.34f
#define LEG2_START_Y    0.0f

#define LEG3_START_X    3.15f
#define LEG3_START_Y    5.65f

#define LEG4_START_X   -3.15f
#define LEG4_START_Y    5.65f

#define LEG5_START_X   -4.34f 
#define LEG5_START_Y    0.0f 

#define LEG6_START_X   -3.15f 
#define LEG6_START_Y   -5.65f 

typedef enum {
  CLOCKWISE = 1, 
  COUNTER_CLOCKWISE = -1
}RotationDirection;

typedef struct {
  float r;                      /* circle radius */
  uint16_t steps_per_circle;    /* steps per circle */
  uint16_t total_circles;       /* total number of circles */
  uint16_t current_step;        /* current step (starting from 0)*/
  bool is_completed;            /* whether the entire path has been completed */
  RotationDirection direction;  /* rotation direction */
}CircularPath;

#ifdef __cplusplus
} // extern "C"
#endif



class RobotJoint {
  public: 
    RobotJoint();

    bool dir;                 /* dir = true forward rotation, dir = false reverse rotation */
    int8_t deviation;
    uint8_t id;
    uint16_t duty;
    uint16_t time;
    float angle;

    /**
     * @brief Bind servo ID number
     * 
     * @param  id -assigned servo index
     */
    void attach_servo(uint8_t id);
    
    /**
     * @brief Control the servo by setting an angle
     * 
     * @param  dir    -true  forward rotation 
     *                -false reverse rotation
     * @param  angle  -angle value
     * @param  time   -run time
     * @param  is_ops -whether to directly control the servo response 
     *                -true  servo responds 
     *                -false servo does not respond
     * @attention  Servos 19, 20, 21 are bound as 180-degree servos by default
     */
    void set_angle(bool dir, float angle, uint16_t time, bool is_ops);

    /**
     * @brief Control the servo by setting the PWM pulse width
     * 
     * @param  duty   -PWM pulse width
     * @param  time   -run time
     * @param  is_ops -whether to directly control the servo response 
     *                -true  servo responds 
     *                -false servo does not respond
     */
    void set_duty(uint16_t duty, uint16_t time, bool is_ops);

    /**
     * @brief Set deviation
     * 
     * @param val  -deviation value
     */
    void set_deviation(int8_t val);

    /**
     * @brief Read deviation
     * 
     * @return int8_t -deviation
     */
    int8_t read_deviation(void);

    /**
     * @brief Download deviation
     * 
     * @return true   -download succeeded
     * @return false  -download failed
     */
    bool download_deviation(void);
};

class RobotLeg {
  public:
    RobotLeg();

    uint8_t id;
    uint32_t set_time = 0;
    volatile bool is_busy = false;

    RobotJoint joint_a, joint_b, joint_c;
    
    Theta_t _result;
    
    Vector_t offset;                                /* servo deviation value that was read */

    Vector_t b_leg_start = {0.0f, 0.0f, 0.0f};                           /* coordinate of the leg start point in the body coordinate frame */
    Vector_t r_leg_start = {0.0f, 0.0f, 0.0f};
    Vector_t r_leg_end = {0.0f, 0.0f, 0.0f};                             /* leg end coordinate after the rotation transform */
    Vector_t b_leg_end = {0.0f, 0.0f, 0.0f};                             /* coordinate of the leg end in the body coordinate frame */
    Vector_t omni_move_end_point = {0.0f, 0.0f, 0.0f};                   /* target foot landing position of the leg end during omnidirectional motion */  
    Vector_t trajectory_point = {0.0f, 0.0f, 0.0f};                      /* trajectory planning target position */ 
    Vector_t result = {0.0f, 0.0f, 0.0f};                                /* coordinate from the pose solving */
    Vector_t start_result = {0.0f, 0.0f, 0.0f};                          /* initial coordinate for the pose solving */
    Vector_t amplitude = {0.0f, 0.0f, 0.0f};                             /* leg motion amplitude */
    Vector_t last_point = {0.0f, 0.0f, 0.0f};                            /* previous coordinate of the leg end */    
    Vector_t now_point = {0.0f, 0.0f, 0.0f};                             /* leg end coordinate */
    Vector_t trans_point = {0.0f, 0.0f, 0.0f};
    Vector_t begin_point = {6.5f, 0.0f, -4.0f};
    Vector_t start_point = {6.5f, 0.0f, -4.0f};
    Vector_t goal_point = {6.5f, 0.0f, -4.0f};
    /** Per-leg pose lift amount (cm); drives femur/foot curl after IK. */
    float pose_lift_curl = 0.0f;
    /** Support-leg hip spread blend (0..1) after IK. */
    float pose_stance_hip = 0.0f;
    /**
     * @brief Move the leg to a specified coordinate point
     * 
     * @param  point  -target coordinate
     * @param  time   -run time
     * @return true   -running
     * @return false  -not running
     */
    bool move(Vector_t point, uint32_t time);

    /**
     * @brief Get the current coordinate point
     * 
     * @return Vector_t -current coordinate point
     */
    Vector_t get_now_point(void);
  private:
    uint32_t current_time = 0;
    uint32_t goal_time = 20;

    
};

class RobotArm {
  public:
    RobotArm();
    RobotJoint joint_a, joint_b, joint_c;
};

class Robot {
  public:
    Robot();

    HW_Board board;
    HW_Sensor sensor;
    RobotArm arm;
    RobotLeg leg1, leg2, leg3, leg4, leg5, leg6;

    enum Func_State {CALIBRATE, CRAWL, ACTION_GROUP};
    enum Move_State {MOVING, STOP, REST};
    enum Avoid_State {FORWARD, BACK, TURN, WAIT};
    enum Act_State {READ_FRAME_NUM, READ_FRAME_DATA, ACT_STOP};
    enum GaitMode {GAIT_TRIPOD = 0, GAIT_RIPPLE = 1};
    Func_State func_state = CRAWL;
    Move_State move_state = REST;
    Avoid_State avoid_state = FORWARD;
    Act_State act_state = READ_FRAME_NUM;
    GaitMode gait_mode = GAIT_TRIPOD;
    /** 0 = fastest / snappiest gait, 100 = slowest / smoothest. */
    uint8_t gait_smooth = 0;
    uint8_t pose_leg_lift_mask = 0;
    uint8_t pose_leg_lift_sign_flip = 0;
    float pose_leg_lift_z = 0.0f;
    uint8_t pose_stance_mask = 0;
    float pose_stance_spread = 0.0f;

    uint32_t tick_count = 0;
    /**
     * @brief Body initialization
     * 
     */
    void begin(void);
    
    /**
     * @brief Position update; updates every 10ms by default
     * 
     */
    void update(void);
    
    /**
     * @brief Motion pose
     * 
     */
    void crawl_state(void);

    /**
     * @brief Body movement
     * 
     * @param  _velocity    -body x, y and rotation speed about the z axis 
     * @param  _position    -x, y and z axis position relative to the body center
     * @param  _euler       -Euler angles relative to the body center
     * @param  time         -run time
     * @param  step_num     -number of steps
     */
    void move(Velocity_t *_velocity, Vector_t *_position, Euler_t *_euler, uint32_t time = 600, int step_num = -1);

    /**
     * @brief Body reset
     * 
     */
    void reset(void);

    /**
     * @brief Obstacle avoidance
     * 
     * @param  dis  -measured current distance
     */
    void avoid(uint16_t dis);

    /**
     * @brief Self-balancing
     * 
     */
    void balance(bool state);

    /**
     * @brief Wiggle action
     * 
     * @param  radius            -radius
     * @param  circles           -number of circles
     * @param  steps_per_circle  -number of sample points per circle
     * @param  dir               -direction
     */
    void twist(float radius, uint16_t circles, uint16_t steps_per_circle, RotationDirection dir);

    /**
     * @brief Act-cute action
     * 
     */
    void acting_cute(void);

    /**
     * @brief Wake-up action
     * 
     */
    void wake_up();

    /**
     * @brief Wake-up running action
     * 
     */
    void _wake_up();

    // void coor_action1();
    // void coor_action2();

    /**
     * @brief List the names and sizes of the currently existing action group files (LOG must be enabled)
     * 
     */
    void list_action_group_dir(void);  

    /**
     * @brief Action group reset
     * 
     */
    void action_group_stop(void);    

    /**
     * @brief Run action group
     * 
     * @param  id -id number of the action group to run
     */
    void action_group_run(uint8_t id);

    /**
     * @brief Download action group
     * 
     * @param  id       -action group id number
     * @param  buf      -action group data to download
     * @param  length   -length of the action group data to download
     * @return true     -download succeeded
     * @return false    -download failed
     */
    bool action_group_download(uint8_t id, uint8_t *buf, size_t length);

    /**
     * @brief Erase action group
     * 
     * @param  id       -action group id number
     * @return true     -erase succeeded
     * @return false     -erase failed
     */
    bool action_group_erase(uint8_t id);

    /**
     * @brief Multi-servo control
     * 
     * @param  arg       -pointer to the servo parameter struct
     * @param  servo_num -number of servos
     * @param  time      -run time
     */
    void multi_servo_control(ServoArg_t* arg, uint16_t servo_num, uint16_t time);

    /**
     * @brief Set deviation
     * 
     * @param id  -joint id
     * @param val  -deviation value
     */
    void set_deviation(uint8_t id, int8_t val);

    /**
     * @brief Read deviation
     * 
     * @param *val -pointer to the read deviation value
     */
    void read_deviation(int8_t *val);

    /**
     * @brief Download deviation
     * 
     * @return true   -download succeeded
     * @return false  -download failed
     */
    bool download_deviation(void);        

  private:
    bool swing_state = false;
    uint32_t balance_tick_start = 0;
    uint32_t move_time = 0;
    float step_length;
    int _step_num;
    int last_step_num;
    float _leg_lift = 3;
    uint8_t act_read_frame_num = 0;
    const float max_half_step_length = 3.0f;
    const float body_width      = 7.6f;
    const float body_length     = 14.0f;

    Velocity_t velocity      = {0.0f, 0.0f, 0.0f};
    Velocity_t last_velocity = {0.0f, 0.0f, 0.0f};
    Vector_t   position      = {0.0f, 0.0f, 0.0f};
    Euler_t    euler         = {0.0f, 0.0f, 0.0f};
    Vector_t   circle_center;

    File file;
    TimerHandle_t timer;
    TimerHandle_t event;

    /**
     * @brief Foot landing coordinate calculation for movement
     * 
     */
    void cal_omni_move_end_point(void);

    /**
     * @brief  Circle trajectory point calculation
     * 
     * @param  path   -pointer to the trajectory point struct
     * @param  x      -x coordinate of the circle center
     * @param  y      -y coordinate of the circle center
     * @return true   -motion complete
     * @return false  -motion not complete
     */
    bool next_circular_point(CircularPath* path, float* x, float* y);
};

#ifdef __cplusplus
extern "C" {
#endif

#ifdef __cplusplus
} /* extern "C" */
#endif

#endif