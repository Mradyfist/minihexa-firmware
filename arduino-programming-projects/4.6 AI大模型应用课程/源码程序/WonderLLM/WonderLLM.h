#ifndef __WONDERLLM_H
#define __WONDERLLM_H

#include "stdint.h"
#include <stdbool.h>
#include "WonderLLM_porting.h"

#define WONDERLLM_MAX_FRAME_SIZE 256

/*define the wonderllm reply message type*/
typedef enum{
	Frame_NULL = 0,
	Frame_move = 1,
	Frame_get_status_battery = 2,
	Frame_get_status_distance,
	Frame_get_status_running_mode,
	Frame_get_status_Speed_mode,
	Frame_get_status_Bodystate,
	Frame_get_status_poseture,
	Frame_set_running_mode,
	Frame_set_speed_mode,
	Frame_set_led_color,
	Frame_set_buzzer,
	Frame_vision_analysis,
	// Frame_stop,
	Frame_BaryCenterMove,
	Frame_InclinationAngleMove,
	Frame_ActionGroup,
	
}FrameMode;

/*define the wonderllm receive message struct*/
typedef struct{
		int16_t motion_target_step;         /*number of motion steps in normal motion*/
		int16_t motion_target_RunningTime;  /*motion duration in normal motion*/
		int16_t motion_target_RotationAngle;/*rotation angle in normal motion (only for in-place left/right turns)*/
		int16_t motion_target_distance;     /*motion distance in normal motion (used for translation in the eight directions)*/
		char running_mode;               		/*1-normal, 2-intelligent_avoidance, 3-intelligent_track*/
		uint16_t avoid_distance;         		/*minimum distance the robot keeps from the target; used in intelligent obstacle-avoidance and intelligent follow modes*/
		uint8_t rgb_left[3];             		/*left RGB light-intensity ratio of the illuminated ultrasonic module*/
		uint8_t rgb_right[3];            		/*right RGB light-intensity ratio of the illuminated ultrasonic module*/
		char buzzer_count;               		/*number of buzzer beeps*/
		char Speed_Mode;                 		/*1-Low_speed, 2-High_speed*/
		char Detection_WonderLLM;      		/*0-module not detected on the bus  1-module detected on the bus*/
		char Vision_Result;             		/*vision recognition result*/
		char incline_direction;         		/*robot tilt direction (rotation about the Euler angles) in normal motion: 1-lean forward, 2-lean back, 3-lean left, 4-lean right, 5-twist left, 6-twist right*/
		char BaryCenter_direction;       		/*robot center-of-gravity movement direction in normal motion: 1-front, 2-front-right, 3-right, 4-rear-right, 5-back, 6-rear-left, 7-left, 8-front-left, 9-up, 10-down*/
		char ActionNum;                     /*index of the action group to execute*/
		char ExecuteActionNum;              /*number of times to execute the action group*/
		char movement_direction;            /*robot translation direction in normal motion: 1-front, 2-front-right, 3-right, 4-rear-right, 5-back, 6-rear-left, 7-left, 8-front-left, 9-turn left in place, 10-turn right in place*/

		FrameMode Frame_mode;               /*current MCP command type*/
		char json_data_raw[256];            /*buffer that holds the raw MCP command*/
}WonderLLM_Info;

/*expose an interface to the wonderllm receive-message object*/
extern WonderLLM_Info WonderLLM_hiwonder;


/**
 * @brief Initialize the WonderLLM communication module.
 *        This function scans and waits for the slave to be ready, then sends the tool registration command to the slave.
 * @return bool true means initialization succeeded, false means it failed.
 */
bool WonderLLM_Init(void);

/**
 * @brief Polling and processing function called in the main loop.
 *        This function periodically requests commands from WonderLLM and processes the received commands.
 */
void WonderLLM_Info_Get(WonderLLM_Info *obj);

/**
 * @brief Call this function to notify WonderLLM when the action is complete.
 */
void WonderLLM_Send_Action_Finish(void);

/**
 * @brief Call this function when status data needs to be returned.
 * @param params_str a string in the format "[[\"key\",\"value\"],...]".
 */
void WonderLLM_Send_Status(const char* params_str);

/**
 * @brief Have WonderLLM use the camera.
 */
void WonderLLM_Request_Vision(const char* params_str);


#endif /* __WONDERLLM_H */