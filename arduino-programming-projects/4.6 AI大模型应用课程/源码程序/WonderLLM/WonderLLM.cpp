/**
 * @file WonderLLM.cpp
 * @brief WonderLLM module driver (abstraction-layer application)
 * @note  This file only handles the module's working-logic scheduling; for the specific low-level hardware operations see "WonderLLM_porting", which need not be modified when porting
 * @author ZhiYuan (Gilbert@hiwonder.com)
 */

#include "WonderLLM.h"
#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <Wire.h>

#define debug_mode 0

// --- internal static variables ---
WonderLLM_Info WonderLLM_hiwonder;

// --- static function prototypes ---
static int parse_command(WonderLLM_Info *obj,const char* json_str);
static bool send_frame(const uint8_t* data, uint16_t len);
static bool receive_frame(uint8_t* buffer, uint16_t* len);
static uint8_t calculate_checksum(const uint8_t* data, uint16_t len);
static bool register_tools(void);

/**
 * @brief Confirm whether the module exists and complete the custom MCP tool registration
 */
bool WonderLLM_Init(void) {
    uint32_t start_tick =  Get_time_now(); // record the start time

    while (1) {
        // 1. check whether the device is ready
        if (Detect_WonderLLM() == true) {
            // device found, run the initialization sequence
						IIC_Config_MCP_Transmit();	
            delay_ms(5);
            register_tools(); // call the MCP write tool	
						IIC_Config_normal_Transmit();

						return true;

         }else{		// 2. if the device is not found, check whether it has timed out

						if(Get_time_now() - start_tick > 3000){
								// device still not found after 5 seconds, initialization failed
								return false;						
						}else{
								// 3. short delay to avoid frantic polling that hogs the CPU and the I2C bus
				        delay_ms(100); 						
						}
				 
				 }  
		}				        
}


/**
 * @brief Poll for and receive commands from WonderLLM in the main loop
 */

void WonderLLM_Info_Get(WonderLLM_Info *obj) {

		obj->Frame_mode = Frame_NULL;
		if (Detect_WonderLLM() == true) {
				obj->Detection_WonderLLM = 1;

				uint16_t received_len = sizeof(obj->json_data_raw);
				//try to receive data from the IIC bus
				//copy the raw JSON string sent by wonderllm into json_data_raw as a backup
				if (receive_frame((uint8_t*)obj->json_data_raw, &received_len)) {
						if (received_len > 0) {
								//append '\0' to the end of the received message frame to form a string, so string functions like strstr and sscanf can be used
								obj->json_data_raw[received_len] = '\0';
								//parse the message frame
								obj->Frame_mode = (FrameMode)parse_command(obj,obj->json_data_raw);
							
						}
				}
		}else{
			obj->Detection_WonderLLM = 0;
	}
}

/**
 * @brief Send the response that the action executed successfully
 * @note System command, see the communication protocol
 */

void WonderLLM_Send_Action_Finish(void) {
    char json_str[72];
    snprintf(json_str, sizeof(json_str), "{\"command\":\"action_finish\",\"params\":\"true\"}");
    send_frame((uint8_t*)json_str, strlen(json_str));
}

/**
 * @brief Send status information
 * @note System command, see the communication protocol
 */
void WonderLLM_Send_Status(const char* params_str) {
    char json_str[128];
    snprintf(json_str, sizeof(json_str), "{\"command\":\"status\",\"params\":%s}", params_str);
    send_frame((uint8_t*)json_str, strlen(json_str));
}


/**
 * @brief Request a single vision recognition from the WonderLLM module
 * @param prompt a string describing what you want the large model to do
 * @note 1. System command, see the communication protocol
 *       2. The prompt string passed in must not contain double quotes, otherwise WonderLLM will fail to parse it
 *       3. When transferring long strings (especially Chinese), IIC_Config_MCP_Transmit must be called to set the IIC rate
 *         up to 400,000, otherwise WonderLLM will not be able to receive the data completely
 *       4. Before the module finishes network provisioning (the white progress bar disappears and the emoji interface appears), this function has no effect     
 */
void WonderLLM_Request_Vision(const char* prompt) {
    char json_str[256];
    
    // use snprintf to safely build the JSON string
    snprintf(json_str, sizeof(json_str), 
             "{\"tool_name\":\"mcu.request\",\"command\":\"vision\",\"params\":\"%s\"}", 
             prompt);
		
		IIC_Config_MCP_Transmit();				
						
    delay_ms(5);
		
    // call the existing send_frame function to send this request
    send_frame((uint8_t*)json_str, strlen(json_str));			

}

/**
 * @brief Parse the received JSON command string
 */

static int parse_command(WonderLLM_Info *obj,const char* json_str) {
	
		 /*match whether the message frame contains "move", to confirm whether it is a self.robot.move type message*/
		 if (strstr(json_str, "move") != NULL) {
				
        char* dist_ptr;
				//scan whether the message frame has a "move" field
        dist_ptr = strstr(json_str, "move");
				//extract the data of the move field
				if (dist_ptr) sscanf(dist_ptr, "%*[^:]:%hhd", &(obj->movement_direction));

				//scan whether the message frame has a "step_num" field
        dist_ptr = strstr(json_str, "step_num");
				//extract the data of the step_num field
				if (dist_ptr) sscanf(dist_ptr, "%*[^:]:%hd", &(obj->motion_target_step));		

        dist_ptr = strstr(json_str, "duration");
				//extract the data of the duration field
				if (dist_ptr) sscanf(dist_ptr, "%*[^:]:%hd", &(obj->motion_target_RunningTime));

        dist_ptr = strstr(json_str, "distance");
				//extract the data of the duration field
				if (dist_ptr) sscanf(dist_ptr, "%*[^:]:%hd", &(obj->motion_target_distance));

        dist_ptr = strstr(json_str, "angle");
				//extract the data of the duration field
				if (dist_ptr) sscanf(dist_ptr, "%*[^:]:%hd", &(obj->motion_target_RotationAngle));

				return Frame_move;
     }

		 /*match whether the message frame contains "status_name", to confirm whether it is a self.robot.get_status type message*/
		 /*Note 1: get_status type message detection must be performed before set_mode type message detection,
			       otherwise the get_status message frame that queries motion state, because it contains a running_mode field,
			       would also be mistaken for a set_mode type message*/
		 /*Note 2: get_status type message detection must be performed before BaryCenterMove type message detection, for the same reason as above*/
     else if (strstr(json_str, "status_name") != NULL) {
				 FrameMode mode = Frame_NULL; 
			 
         if (strstr(json_str, "battery")) {

						 mode = Frame_get_status_battery;
         }
				 if (strstr(json_str, "Speed_Mode")) {

						 mode = Frame_get_status_Speed_mode;
         }
				 if (strstr(json_str, "distance")) {

						 mode = Frame_get_status_distance;
         }
				 if (strstr(json_str, "running_mode")) {

						 mode = Frame_get_status_running_mode;
         }
				 if (strstr(json_str, "Bodystate")) {

						 mode = Frame_get_status_Bodystate;
         }
				 if (strstr(json_str, "poseture")) {
						 mode = Frame_get_status_poseture;
         }

				 return mode;
     }

		 /*match whether the message frame contains "pose", to confirm whether it is a self.robot.BaryCenterMove type message*/
		 else if (strstr(json_str, "pose") != NULL) {
				
        char* dist_ptr;
				//scan whether the message frame has a "pose" field
        dist_ptr = strstr(json_str, "pose");
				//extract the data of the pose field
				if (dist_ptr) {
					sscanf(dist_ptr, "%*[^:]:%hhd", &(obj->BaryCenter_direction));
				}
        
				return Frame_BaryCenterMove;
     }

		 /*match whether the message frame contains "Inclination", to confirm whether it is a self.robot.InclinationAngleMove type message*/
		 else if (strstr(json_str, "Inclination") != NULL) {
				
        char* dist_ptr;
				//scan whether the message frame has an "Inclination" field
        dist_ptr = strstr(json_str, "Inclination");
				//extract the data of the Inclination field
				if (dist_ptr) {
					sscanf(dist_ptr, "%*[^:]:%hhd", &(obj->incline_direction));
				}
        
				return Frame_InclinationAngleMove;
     }

		//  /*match whether the message frame contains "stop", to confirm whether it is a self.robot.stop type message*/
		//  if (strstr(json_str, "stop") != NULL) {
		// 		return Frame_stop;
		//  }

		 /*match whether the message frame contains "running_mode", to confirm whether it is a self.robot.set_SpeedMode type message*/
		 else if (strstr(json_str, "running_mode") != NULL) {
			 if (strstr(json_str, "normal")) {
					obj->running_mode = 1; 
					obj->avoid_distance = 0;
					
			 }else{
					if (strstr(json_str, "intelligent_avoidance")) {
							obj->running_mode = 2;
					} 
					
					if (strstr(json_str, "intelligent_track")) {
							obj->running_mode = 3;
					}

					char* dist_ptr;
					//scan whether the message frame has a "distance" field
					dist_ptr = strstr(json_str, "distance");
					//extract the data of the distance field
					if (dist_ptr) {
						sscanf(dist_ptr, "%*[^:]:%hu", &(obj->avoid_distance));
					}				 				
			 }
 

				 
			 return Frame_set_running_mode;
		   
     }

		 /*match whether the message frame contains "running_mode", to confirm whether it is a self.robot.set_SpeedMode type message*/
		 else if (strstr(json_str, "speed_mode") != NULL) {
			 if (strstr(json_str, "Low_speed")) {
					obj->Speed_Mode = 1; 
			 }  
			 
			 if (strstr(json_str, "High_speed")) {
					 obj->Speed_Mode = 2;
			 } 
				 
			 return Frame_set_speed_mode;
		   
     }
		
		 /*match whether the message frame contains "lr", to confirm whether it is a self.robot.set_led_color type message*/
		 else if (strstr(json_str, "\"lr\"") != NULL) {

  			char* ptr;
        
        ptr = strstr(json_str, "\"lr\"");
				if (ptr) sscanf(ptr, "%*[^:]:%hhu", &(obj->rgb_left[0]));

				ptr = strstr(json_str, "\"lg\"");
				if (ptr) sscanf(ptr, "%*[^:]:%hhu", &(obj->rgb_left[1]));

				ptr = strstr(json_str, "\"lb\"");
				if (ptr) sscanf(ptr, "%*[^:]:%hhu", &(obj->rgb_left[2]));

				ptr = strstr(json_str, "\"rr\"");
				if (ptr) sscanf(ptr, "%*[^:]:%hhu", &(obj->rgb_right[0]));

				ptr = strstr(json_str, "\"rg\"");
				if (ptr) sscanf(ptr, "%*[^:]:%hhu", &(obj->rgb_right[1]));

				ptr = strstr(json_str, "\"rb\"");
				if (ptr) sscanf(ptr, "%*[^:]:%hhu", &(obj->rgb_right[2]));

        // assign the parsed values to the global RGB array
        // note the range checking and type conversion
        obj->rgb_left[0] = (obj->rgb_left[0] > 255) ? 255 : ((obj->rgb_left[0] < 0) ? 0 : obj->rgb_left[0]);
        obj->rgb_left[1] = (obj->rgb_left[1] > 255) ? 255 : ((obj->rgb_left[1] < 0) ? 0 : obj->rgb_left[1]);
        obj->rgb_left[2] = (obj->rgb_left[2] > 255) ? 255 : ((obj->rgb_left[2] < 0) ? 0 : obj->rgb_left[2]);

        obj->rgb_right[0] = (obj->rgb_right[0] > 255) ? 255 : ((obj->rgb_right[0] < 0) ? 0 : obj->rgb_right[0]);
        obj->rgb_right[1] = (obj->rgb_right[1] > 255) ? 255 : ((obj->rgb_right[1] < 0) ? 0 : obj->rgb_right[1]);
        obj->rgb_right[2] = (obj->rgb_right[2] > 255) ? 255 : ((obj->rgb_right[2] < 0) ? 0 : obj->rgb_right[2]);

				return Frame_set_led_color;
    }
		
		 /*match whether the message frame contains "count", to confirm whether it is a self.robot.set_buzzer type message*/
		 else if (strstr(json_str, "count") != NULL) {				
		 	 char* ptr;			 
			 ptr = strstr(json_str, "\"count\"");		 
			 if (ptr){
			 	sscanf(ptr, "%*[^:]:%hhd", &(obj->buzzer_count));
			 } 
			 return Frame_set_buzzer;
     }

		 /*match whether the message frame contains "actionNum", to confirm whether it is a self.robot.ActionGroup type message*/
		 else if (strstr(json_str, "actionNum") != NULL) {
		 	 char* ptr;
			 ptr = strstr(json_str, "\"actionNum\"");
			 if (ptr) sscanf(ptr, "%*[^:]:%hhd", &(obj->ActionNum));

			 ptr = strstr(json_str, "\"executeNum\"");
			 if (ptr) sscanf(ptr, "%*[^:]:%hhd", &(obj->ExecuteActionNum));

			 return Frame_ActionGroup;
     }

		 /*match whether the message frame contains "vision", to confirm whether it is a system type message*/
		 else if (strstr(json_str, "vision") != NULL) {
				// if (strstr(json_str, "true") != NULL) {
				// 	obj->Vision_Result = 1;
				// }else{
				// 	obj->Vision_Result = 0;
				// }

			  return Frame_vision_analysis;
     }

		//  /*match whether the message frame contains "reply", to confirm whether it is a system type message*/
		//  else if (strstr(json_str, "reply") != NULL) {

		// 	 return Frame_reply;
    //  }		
		 return Frame_NULL;		 
}

/**
 * @brief Register all available tools (functions) with the WonderLLM module
 * @note  When transferring long strings (especially Chinese), IIC_Config_MCP_Transmit must be called to set the IIC rate
 *        up to 400,000, otherwise WonderLLM will not be able to receive the data completely
 */
static bool register_tools(void) {

/*
		register user-defined mcp tools
	  tool_name: the name of the registered MCP tool (keep the format starting with `self`; the name must be clear enough for the large model to know its purpose, and avoid abbreviations)
	
		command: guides the large model on when to use this tool and describes the parameters that need to be returned to the host
	
		params: the parameters contained in the message frame of this type that WonderLLM sends back to the host, in the format: [[param_name_1, param_type, min_value (optional, none for string type), max_value (optional, none for string type)], [param_2...]]
					(string means the specific content of this parameter is provided by the large model, decided from the conversation context)
					(the parameter type currently only supports string, int, bool)
	
		return: indicates whether the host needs to return data to WonderLLM
						true - required; when true, the block setting has no effect, because return is itself blocking
						     (uses the basic system command status, see WonderLLM_Send_Status)
						false - not required
	
		block: whether execution is blocking; if so, WonderLLM waits for the host to reply confirming the command finished before continuing
					 true - yes; after executing the command carried by WonderLLM's reply message frame, the host must reply to WonderLLM that the command has finished
								(uses the basic system command action_finish, see WonderLLM_Send_Action_Finish)
					 false - no; after WonderLLM sends a command to the host via a message frame it continues immediately, no longer caring about the command's execution result
	
*/


		if (!send_frame((uint8_t*)tool_buzzer, strlen(tool_buzzer))){
			return false;
		} 

		if (!send_frame((uint8_t*)tool_led, strlen(tool_led))){
			return false;
		}
		
		// if (!send_frame((uint8_t*)tool_stop, strlen(tool_stop))){
		// 	return false;
		// }

		if (!send_frame((uint8_t*)tool_SpeedMode, strlen(tool_SpeedMode))){
			return false;
		}
	
		if (!send_frame((uint8_t*)tool_RunningMode, strlen(tool_RunningMode))){
			return false;
		}
	
		if (!send_frame((uint8_t*)tool_status, strlen(tool_status))){
			return false;
		}

		if (!send_frame((uint8_t*)tool_move, strlen(tool_move))){
			return false;
		}

		if (!send_frame((uint8_t*)tool_BaryCenterMove, strlen(tool_BaryCenterMove))){
			return false;
		}

		if (!send_frame((uint8_t*)tool_InclinationAngleMove, strlen(tool_InclinationAngleMove))){
			return false;
		}

		if (!send_frame((uint8_t*)tool_ActionGroup, strlen(tool_ActionGroup))){
			return false;
		}

		/*system command - MCP registration complete command*/		
    // static const char* tool_finish = "{\"command\":\"mcp_setting\",\"params\":\"true\"}";	
		if (!send_frame((uint8_t*)tool_finish, strlen(tool_finish))){
			return false;
		}
    
    return true;
}

/**
 * @brief Send the data frame out over I2C
 */
static bool send_frame(const uint8_t* data, uint16_t len) {

	   // check whether the data and length are valid
    if (data == NULL || len == 0) {
        return false;
    }
		
		// directly send the memory region pointed to by data, with length len
    if(WonderLLM_Send_Data((uint8_t *)data,len) !=0){
			return false;
		} 

		return true;
		
}

/**
 * @brief Receive the 8-byte frame header
 */
static bool receive_frame_head(uint16_t* part_ID, uint16_t* part_num, uint16_t* data_len) {
		uint8_t header[8];

		/*step1 receive the "header + length" and validate it*/
    if(WonderLLM_Receive_Data(header, 8, true) != 0){
			return false;
		}

		/*step1.1 validate the frame header*/
    if (header[0] != 0xAA || header[1] != 0x55) return false;
		delay_ms(1);
    
		/*step1.2 parse the frame length and validate it*/
    *data_len = ((uint16_t)header[2] << 8) | header[3];

    if (*data_len == 0|| *data_len > 31){
			Serial.println(F("data abnormal"));
			delay_ms(100);
			return false;
		} 

		#if (debug_mode ==1)
			if(*data_len != 1){
				//toggle one pin high/low; using a logic analyzer to monitor the SCL/SDA pins and this pin at the same time makes it easy to locate when the JSON string data is received
				digitalWrite(6, HIGH);
				Serial.print("rec_dataLen:");
				Serial.println(*data_len);
				digitalWrite(6, LOW);
			}
		#endif

		/*step1.3 parse the total number of fragments and the current fragment ID*/
		*part_ID = ((uint16_t)header[5] << 8) | header[4]; 
		*part_num = ((uint16_t)header[7] << 8) | header[6];

		return true;
}

/**
 * @brief Receive one complete data frame from I2C
 */
static bool receive_frame(uint8_t* buffer, uint16_t* len) {
		uint16_t data_len = 0;
		uint16_t data_len_max = 0;
		uint16_t part_ID = 0;
		uint16_t part_num = 0;
		uint16_t buffer_index = 0;
		uint16_t part_ID_temp = 0;
		uint16_t part_num_temp = 0;

		data_len_max = *len;
		*len =0;

		/*step1 receive the frame header and validate it*/
		if(receive_frame_head(&part_ID, &part_num, &data_len)){
			*len += data_len;
		}else{
			Serial.println(F("receive_frame_fail"));
			return false;
		}

		/*step2 receive the "frame data + checksum byte"*/
		//if the currently received packet is not the first one, packets were lost earlier and the data received is incomplete, so exit directly
		if(part_ID != 1){
			Serial.println(F("receive_frame_not_first"));
			return false;

		}else{  //receive all the data continuously starting from the first packet

			uint8_t result =0;

				//data_len (data length) + 1 (checksum byte)
				//the checksum byte of each packet is overwritten when the next packet is received
				//the checksum byte of the last packet is overwritten by the '\0' character used later to build the string
			for(int i=1; i<=part_num; i++){

				delay_ms(10);

				result = WonderLLM_Receive_Data((buffer + buffer_index), data_len + 1, true);			

				//exception handling: determine whether the read was normal
				if(result != 0){
					Serial.print("rec_result:");
					Serial.println(result);
					Serial.println(F("receive error"));
					return false;
				}

				//confirm the validity of the checksum byte
				if (buffer[buffer_index + data_len] == calculate_checksum((buffer + buffer_index), data_len)){

				#if(debug_mode ==1)					
						Serial.print(F("data:"));
						for(uint8_t t=0;t < data_len;t++){
							Serial.print((int)buffer[buffer_index + t]);
							Serial.print(' ');
						}
						Serial.println();
				#endif

					buffer_index += data_len;

				}else{ //checksum failed; clear the already-stored data and exit

					Serial.println(F("calculate error"));

					#if(debuf_mode ==1)
						/*print for debugging*/
						for(int i=0;i<=(*len);i++){
							Serial.print(buffer[i]);
							Serial.print(' ');
						}
						Serial.println();
					#endif

					memset(buffer,0,sizeof(buffer));
					return false;
				}

				//receive the header of the next packet to get its length, fragment ID and other info, preparing to receive the next packet
				if(i < part_num){

					delay_ms(100);

					if(receive_frame_head(&part_ID_temp, &part_num_temp, &data_len)){
						*len += data_len;
					}else{
						return false;
					}

					/*verify the next packet's fragment ID and total fragment count*/
					//the next packet's fragment ID is not consecutive with this one, meaning packet loss occurred, so exit directly
					if(( (part_ID + 1) != part_ID_temp ) || (part_num_temp != part_num) ){
						Serial.println(part_ID);
						Serial.println(part_ID_temp);
						Serial.println(F("data packet dropout"));
						return false;
					}else{
						part_ID += 1;
					}

					/*overflow protection for the data, to avoid out-of-bounds writes*/
					if((buffer_index + data_len + 1) > data_len_max ){
						Serial.println(F("Insufficient buffer space"));
						return false;
					}
				
				}

			}

			return true;			

		}

    return false;
}

/**
 * @brief Compute the XOR checksum of the data
 */
static uint8_t calculate_checksum(const uint8_t* data, uint16_t len) {
    uint8_t checksum = 0;
    for (uint16_t i = 0; i < len; i++) {
        checksum ^= data[i];
    }
    return checksum;
}