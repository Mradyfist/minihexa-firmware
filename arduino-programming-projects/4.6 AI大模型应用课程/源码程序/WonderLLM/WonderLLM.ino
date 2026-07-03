#include "hiwonder_robot.h"
#include <string.h>
#include <Wire.h>
#include "WonderLLM.h"

#define Move_stride_LowSpeed_1 70       //single-step translation stride for forward/backward motion in low-speed mode (unit: mm)
#define Move_stride_HighSpeed_1 85      //single-step translation stride for forward/backward motion in high-speed mode (unit: mm)
#define Move_stride_LowSpeed_2 65       //single-step translation stride for left/right motion in low-speed mode (unit: mm)
#define Move_stride_HighSpeed_2 75      //single-step translation stride for left/right motion in high-speed mode (unit: mm)
#define Move_stride_LowSpeed_3 90       //single-step translation stride for diagonal motion in low-speed mode (unit: mm)
#define Move_stride_HighSpeed_3 90      //single-step translation stride for diagonal motion in high-speed mode (unit: mm)
#define Rotation_angle_LowSpeed 24      //single-step rotation angle in low-speed mode (unit: degrees)
#define Rotation_angle_HighSpeed 45     //single-step rotation angle in high-speed mode (unit: degrees)

extern WonderLLM_Info WonderLLM_hiwonder;
static char info[128];

Robot minihexa;

uint8_t val[4];
uint8_t rgb1[3] = {0,0,255};
uint8_t rgb2[3] = {0,0,255};

Velocity_t vel = {0.0f,0.0f,0.0f};
Vector_t pos = {0.0f,0.0f,0.0f};
Euler_t att = {0.0f,0.0f,0.0f};

int timer = 0;
char sustain_move_flag = 0;

void setup() {
  delay(1000);
  Serial.begin(115200);
  minihexa.begin();
  minihexa.sensor.set_ultrasound_rgb(RGB_WORK_SOLID_MODE, rgb1, rgb2);
  WonderLLM_Init(); //initialize the WonderLLM module	
  WonderLLM_hiwonder.Speed_Mode = 1; //initialize to low-speed motion mode

  //delay for a while to ensure the module has powered up and finished network provisioning (the emoji interface appears)
  delay(18000);

}
void loop() {
  //get WonderLLM data
  WonderLLM_Info_Get(&WonderLLM_hiwonder); 
  //Serial.printf("%d\r\n",WonderLLM_hiwonder.Speed_Mode);
  if(WonderLLM_hiwonder.Frame_mode != Frame_NULL){
      sprintf(info,"raw str:%s\r\n",WonderLLM_hiwonder.json_data_raw);

      memset(WonderLLM_hiwonder.json_data_raw,0,sizeof(WonderLLM_hiwonder.json_data_raw));
      // Serial.print(info); 
    
      switch(WonderLLM_hiwonder.Frame_mode){
        case Frame_move:{
          sprintf(info,"Frame_move:direction:%d,step:%d,time:%d,distance:%d,angle:%d\r\n",WonderLLM_hiwonder.movement_direction,\
                                                                                          WonderLLM_hiwonder.motion_target_step,\
                                                                                          WonderLLM_hiwonder.motion_target_RunningTime,\
                                                                                          WonderLLM_hiwonder.motion_target_distance,\
                                                                                          WonderLLM_hiwonder.motion_target_RotationAngle);
          Serial.print(info);
          
          /* execute the user-defined motion control API */
          if(WonderLLM_hiwonder.motion_target_RotationAngle == -1 && WonderLLM_hiwonder.motion_target_distance == -1){ //do not specify the motion distance or the motion angle

              if(WonderLLM_hiwonder.motion_target_step == -1 && WonderLLM_hiwonder.motion_target_RunningTime == -1){ //specify neither step count nor duration; by default keep walking without stopping
                sustain_move_flag = 1;

              }else if(WonderLLM_hiwonder.motion_target_step != -1 && WonderLLM_hiwonder.motion_target_RunningTime == -1){ //specify the number of motion steps but not the motion duration
                timer = 600 * WonderLLM_hiwonder.motion_target_step; //taking one step takes 600ms by default

              }else if(WonderLLM_hiwonder.motion_target_step == -1 && WonderLLM_hiwonder.motion_target_RunningTime != -1){ //specify the number of motion steps but not the motion duration
                timer = WonderLLM_hiwonder.motion_target_RunningTime;

              }else if(WonderLLM_hiwonder.motion_target_step != -1 && WonderLLM_hiwonder.motion_target_RunningTime != -1){ //both the number of steps and the duration are specified; the two conflict, so the duration takes precedence
                timer = WonderLLM_hiwonder.motion_target_RunningTime;
              }

          }else if(WonderLLM_hiwonder.motion_target_RotationAngle != -1){ //specify the motion angle

            //convert the motion angle into a number of steps, handling it as "do not specify distance or angle" -- "specify step count but not duration"
            if(WonderLLM_hiwonder.Speed_Mode == 1){  //low-speed mode
                WonderLLM_hiwonder.motion_target_step = WonderLLM_hiwonder.motion_target_RotationAngle / Rotation_angle_LowSpeed;

            }else if(WonderLLM_hiwonder.Speed_Mode == 2){ //high-speed mode
                WonderLLM_hiwonder.motion_target_step = WonderLLM_hiwonder.motion_target_RotationAngle / Rotation_angle_HighSpeed;
                
            }
            Serial.println(WonderLLM_hiwonder.motion_target_step);

            timer = 600 * WonderLLM_hiwonder.motion_target_step; //taking one step takes 600ms by default

          }else if(WonderLLM_hiwonder.motion_target_distance != -1){ //specify the motion distance

            //convert the motion distance into a number of steps, handling it as "do not specify distance or angle" -- "specify step count but not duration"
            if(WonderLLM_hiwonder.Speed_Mode == 1){  //low-speed mode
                switch(WonderLLM_hiwonder.movement_direction){
                  case 1: //move forward
                  case 5:{//move backward
                    WonderLLM_hiwonder.motion_target_step = WonderLLM_hiwonder.motion_target_distance / Move_stride_LowSpeed_1;
                    Serial.println(WonderLLM_hiwonder.motion_target_distance);
                    break;
                  }
                  case 3: //translate right
                  case 7:{//translate left
                    WonderLLM_hiwonder.motion_target_step = WonderLLM_hiwonder.motion_target_distance / Move_stride_LowSpeed_2;
                    break;
                  } 
                  case 2://front-right
                  case 4://rear-right
                  case 6://rear-left
                  case 8:{//front-left
                    WonderLLM_hiwonder.motion_target_step = WonderLLM_hiwonder.motion_target_distance / Move_stride_LowSpeed_3;
                    break;
                  }
                }
                

            }else if(WonderLLM_hiwonder.Speed_Mode == 2){ //high-speed mode
                switch(WonderLLM_hiwonder.movement_direction){
                  case 1: //move forward
                  case 5:{//move backward
                    WonderLLM_hiwonder.motion_target_step = WonderLLM_hiwonder.motion_target_distance / Move_stride_HighSpeed_1;
                    break;
                  }
                  case 3: //translate right
                  case 7:{//translate left
                    WonderLLM_hiwonder.motion_target_step = WonderLLM_hiwonder.motion_target_distance / Move_stride_HighSpeed_2;
                    break;
                  } 
                  case 2://front-right
                  case 4://rear-right
                  case 6://rear-left
                  case 8:{//front-left
                    WonderLLM_hiwonder.motion_target_step = WonderLLM_hiwonder.motion_target_distance / Move_stride_HighSpeed_3;
                    break;
                  }
                }

            }
            timer = 600 * WonderLLM_hiwonder.motion_target_step; //taking one step takes 600ms by default

          }else{ //both the motion distance and angle are specified; the two conflict, so the robot does not execute this round of commands
              WonderLLM_hiwonder.motion_target_step = 0;
              WonderLLM_hiwonder.motion_target_RunningTime = 0;
          }



          switch(WonderLLM_hiwonder.movement_direction){
            case 1:{ //straight ahead
              Serial.println(36);
              if(WonderLLM_hiwonder.motion_target_step == 0 && WonderLLM_hiwonder.motion_target_RunningTime == 0){ //the current command is "robot stop motion"
                sustain_move_flag = 0;
              }
              vel = {0.0f,2.0f*WonderLLM_hiwonder.Speed_Mode,0.0f};              
              break;
            }
            case 2:{ //front-right
              vel = {2.0f*WonderLLM_hiwonder.Speed_Mode,2.0f*WonderLLM_hiwonder.Speed_Mode,0.0f};
              break;
            }
            case 3:{ //straight right
              vel = {2.0f*WonderLLM_hiwonder.Speed_Mode,0.0f,0.0f};
              break;
            }
            case 4:{ //rear-right
              vel = {2.0f*WonderLLM_hiwonder.Speed_Mode,-2.0f*WonderLLM_hiwonder.Speed_Mode,0.0f};
              break;
            }
            case 5:{ //straight back
              vel = {0.0f,-2.0f*WonderLLM_hiwonder.Speed_Mode,0.0f};
              break;
            }
            case 6:{ //rear-left
              vel = {-2.0f*WonderLLM_hiwonder.Speed_Mode,-2.0f*WonderLLM_hiwonder.Speed_Mode,0.0f};
              break;
            }
            case 7:{ //straight left
              vel = {-2.0f*WonderLLM_hiwonder.Speed_Mode,0.0f,0.0f};
              break;
            }
            case 8:{ //front-left
              vel = {-2.0f*WonderLLM_hiwonder.Speed_Mode,2.0f*WonderLLM_hiwonder.Speed_Mode,0.0f};
              break;
            }
            case 9:{ //turn left in place
              vel = {0.0f,0.0f,2.0f*WonderLLM_hiwonder.Speed_Mode};
              break;
            }
            case 10:{ //turn right in place
              vel = {0.0f,0.0f,-2.0f*WonderLLM_hiwonder.Speed_Mode};
              break;
            }

          }
          pos = {0.0f,0.0f,0.0f};
          att = {0.0f,0.0f,0.0f};
          
          //Serial.printf("%f %f %f\r\n",vel.vx,vel.vy,vel.omega);
          minihexa.move(&vel, &pos, &att, 600, WonderLLM_hiwonder.motion_target_step);

          if(sustain_move_flag != 1){
            delay(timer);
            vel = {0.0f,0.0f,0.0f};
            minihexa.move(&vel, &pos, &att);
          } 
          
          //this kind of MCP tool executes in a blocking manner (block=true); when finished, the basic system command action_finish must be called to reply to WinderMind
          WonderLLM_Send_Action_Finish();							
          break;
        }

        case Frame_BaryCenterMove:{
          sprintf(info,"Frame_BaryCenterMove:direction:%d\r\n",WonderLLM_hiwonder.BaryCenter_direction);
          Serial.print(info);
          
          /* execute the user-defined motion control API */
          switch(WonderLLM_hiwonder.BaryCenter_direction){
            case 1:{ //straight ahead 
              pos = {0.0f,3.0f,0.0f};
              break;
            }
            case 2:{ //front-right
              pos = {3.0f,3.0f,0.0f};
              break;
            }
            case 3:{ //straight right
              pos = {3.0f,0.0f,0.0f};
              break;
            }
            case 4:{ //rear-right
              pos = {3.0f,-3.0f,0.0f};
              break;
            }
            case 5:{ //straight back
              pos = {0.0f,-3.0f,0.0f};
              break;
            }
            case 6:{ //rear-left
              pos = {-3.0f,-3.0f,0.0f};
              break;
            }
            case 7:{ //straight left
              pos = {-3.0f,0.0f,0.0f};
              break;
            }
            case 8:{ //front-left
              pos = {-3.0f,3.0f,0.0f};
              break;
            }
            case 9:{ //straight up
              pos = {0.0f,0.0f,3.0f};
              break;
            }
            case 10:{ //straight down
              pos = {0.0f,0.0f,-1.5f};
              break;
            }
            case 11:{ //restore the initial pose
              pos = {0.0f,0.0f,0.0f};
              break;
            }
          }
          vel = {0.0f,0.0f,0.0f};
          att = {0.0f,0.0f,0.0f};
          minihexa.move(&vel, &pos, &att);

          //this kind of MCP tool executes in a blocking manner (block=true); when finished, the basic system command action_finish must be called to reply to WinderMind
          WonderLLM_Send_Action_Finish();							
          break;
        }

        case Frame_InclinationAngleMove:{
          sprintf(info,"Frame_InclinationAngleMove:direction:%d\r\n",WonderLLM_hiwonder.incline_direction);
          Serial.print(info);
          
          /* execute the user-defined motion control API */
          switch(WonderLLM_hiwonder.incline_direction){
            case 1:{ //lean forward
              att = {12.0f,0.0f,0.0f};
              break;
            }
            case 2:{ //lean back
              att = {-12.0f,0.0f,0.0f};
              break;
            }
            case 3:{ //lean left
              att = {0.0f,15.0f,0.0f};
              break;
            }
            case 4:{ //lean right
              att = {0.0f,-15.0f,0.0f};
              break;
            }
            case 5:{ //twist left
              att = {0.0f,0.0f,-15.0f};
              break;
            }
            case 6:{ //twist right
              att = {0.0f,-0.0f,15.0f};
              break;
            }
          }
          vel = {0.0f,0.0f,0.0f};
          pos = {0.0f,0.0f,0.0f};
          minihexa.move(&vel, &pos, &att);

          //this kind of MCP tool executes in a blocking manner (block=true); when finished, the basic system command action_finish must be called to reply to WonderLLM
          WonderLLM_Send_Action_Finish();							
          break;
        }

        // case Frame_stop:{
        //   sprintf(info,"Frame_stop:robot reset\r\n");
        //   Serial.print(info);

        //   vel = {0.0f,0.0f,0.0f};
        //   pos = {0.0f,0.0f,0.0f};
        //   att = {0.0f,0.0f,0.0f};
        //   minihexa.move(&vel, &pos, &att);
        //   minihexa.reset();
        //   WonderLLM_hiwonder.running_mode = 1;

        //   //this kind of MCP tool executes in a blocking manner (block=true); when finished, the basic system command action_finish must be called to reply to WonderLLM
        //   WonderLLM_Send_Action_Finish();							
        //   break;          
        // }
        
        case Frame_get_status_battery:{
          sprintf(info, "[\"battery\",\"%d\",\"mV\"]", minihexa.board.bat_voltage);
          
          // //this kind of MCP tool must return parameters to WonderLLM (return=true); when finished, call the basic system command status to return the parameters in the specified format
          WonderLLM_Send_Status(info);
          break;
        }

        case Frame_get_status_Bodystate:{
          float BodyPosition[3];
          float euler_angle[3];
          minihexa.board.get_imu_euler(euler_angle);
          minihexa.read_position(BodyPosition);
          if(fabs(euler_angle[0]) > 150.0f){  //robot has tipped over; the absolute value of the roll angle is greater than 150 degrees
            sprintf(info, "[\"Bodystate\",\"2\"]");
          }else{
            sprintf(info, "[\"Bodystate\",\"%d\"]", (BodyPosition[2] < 0) ? 1 : 0);
          }
          
          
          // //this kind of MCP tool must return parameters to WonderLLM (return=true); when finished, call the basic system command status to return the parameters in the specified format
          WonderLLM_Send_Status(info);
          break;
        }

        case Frame_get_status_poseture:{
          float euler_angle[3];
          minihexa.board.get_imu_euler(euler_angle);
          sprintf(info, "[[\"roll\",\"%.1f\"],[\"pitch\",\"%.1f\"]]",euler_angle[0],euler_angle[1]);
          
          // //this kind of MCP tool must return parameters to WonderLLM (return=true); when finished, call the basic system command status to return the parameters in the specified format
          WonderLLM_Send_Status(info);
          break;
        }

        case Frame_get_status_Speed_mode:{
          sprintf(info, "[\"Speed_mode\",\"%d\"]", WonderLLM_hiwonder.Speed_Mode);
          
          //this kind of MCP tool must return parameters to WonderLLM (return=true); when finished, call the basic system command status to return the parameters in the specified format
          WonderLLM_Send_Status(info);						
          break;
        }
        
        case Frame_get_status_distance:{       
          sprintf(info, "[\"distance\",\"%d\",\"mm\"]", minihexa.sensor._get_distance());
      
          //this kind of MCP tool must return parameters to WonderLLM (return=true); when finished, call the basic system command status to return the parameters in the specified format
          WonderLLM_Send_Status(info);
          break;
        }
        
        case Frame_get_status_running_mode:{
          sprintf(info, "[\"running_mode\",\"%d\"]", WonderLLM_hiwonder.running_mode);
          
          //this kind of MCP tool must return parameters to WonderLLM (return=true); when finished, call the basic system command status to return the parameters in the specified format
          WonderLLM_Send_Status(info);						
          break;
        }

        case Frame_set_running_mode:{
          sprintf(info,"Frame_set_running_mode:%d,distance:%d\r\n",WonderLLM_hiwonder.running_mode,WonderLLM_hiwonder.avoid_distance);
          Serial.print(info);

          vel = {0.0f,0.0f,0.0f};
          pos = {0.0f,0.0f,0.0f};
          att = {0.0f,0.0f,0.0f};
          minihexa.move(&vel, &pos, &att); 

          //this kind of MCP tool executes in a blocking manner (block=true); when finished, the basic system command action_finish must be called to reply to WonderLLM
          WonderLLM_Send_Action_Finish();					
          break;
        }
        
        case Frame_set_led_color:{
          sprintf(info,"Frame_set_RGB: Left:%d,%d,%d,Right:%d,%d,%d\r\n",\
                WonderLLM_hiwonder.rgb_left[0],WonderLLM_hiwonder.rgb_left[1],WonderLLM_hiwonder.rgb_left[2],\
                WonderLLM_hiwonder.rgb_right[0],WonderLLM_hiwonder.rgb_right[1],WonderLLM_hiwonder.rgb_right[2]);
          Serial.print(info);
          
          /* execute the user-defined RGB light control API */
          minihexa.sensor.set_ultrasound_rgb(RGB_WORK_SOLID_MODE, WonderLLM_hiwonder.rgb_left, WonderLLM_hiwonder.rgb_right);
          
          //this kind of MCP tool executes in a blocking manner (block=true); when finished, the basic system command action_finish must be called to reply to WonderLLM
          WonderLLM_Send_Action_Finish();
          break;
        }
        
        case Frame_set_buzzer:{
          sprintf(info,"Frame_set_buzzer:%d\r\n",WonderLLM_hiwonder.buzzer_count);
          Serial.print(info);
          
          /* execute the user-defined buzzer control API */
          for(int i = 0; i < WonderLLM_hiwonder.buzzer_count; i++){
            ledcWriteTone(LEDC_CHANNEL_0, 3000);
            delay(200);
            ledcWriteTone(LEDC_CHANNEL_0, 0);
            delay(500);
          }
          
          //this kind of MCP tool executes in a blocking manner (block=true); when finished, the basic system command action_finish must be called to reply to WonderLLM
          WonderLLM_Send_Action_Finish();
          break;
        }

        case Frame_set_speed_mode:{ 
          sprintf(info,"Frame_set_speed_mode:%d\r\n",WonderLLM_hiwonder.Speed_Mode);
          Serial.print(info);

          //this kind of MCP tool executes in a blocking manner (block=true); when finished, the basic system command action_finish must be called to reply to WonderLLM
          WonderLLM_Send_Action_Finish();
          break;						
        }

        case Frame_ActionGroup:{ 
          sprintf(info,"Frame_ActionGroup:index%d time:%d\r\n",WonderLLM_hiwonder.ActionNum,WonderLLM_hiwonder.ExecuteActionNum);
          Serial.print(info);

          //to prevent the action group from taking too long, which would make WonderLLM misjudge the command as timed out after not getting a reply for a long time, send the reply first and then execute the action group
          //this kind of MCP tool executes in a blocking manner (block=true); when finished, the basic system command action_finish must be called to reply to WonderLLM
          WonderLLM_Send_Action_Finish();

          for(int i = 0; i < WonderLLM_hiwonder.ExecuteActionNum; i++){
            delay(1000);
            switch(WonderLLM_hiwonder.ActionNum){
              case 3:{ //wake-up action
                minihexa.wake_up();
                break;
              }
              case 4:{ //wake-up running action
                minihexa._wake_up();
                break;
              }
              case 5:{ //act-cute action
                minihexa.acting_cute();
                break;
              }
              case 6:   //obstacle-crossing action
              case 7:   //left-leg fighting action
              case 8:   //right-leg fighting action
              case 9:   //left-foot forward ball-kick action
              case 10:  //left-foot rightward ball-kick action
              case 11:  //right-foot forward ball-kick action
              case 12:  //right-foot leftward ball-kick action 
              case 13:  //push-door action
              case 14:{ //wave action
                minihexa.action_group_run(WonderLLM_hiwonder.ActionNum);
                break;
              }                                                                                                                          
            }

          }

          break;						
        }

        case Frame_vision_analysis:{
          // sprintf(info,"Frame_vision_analysis:result:%d\r\n",WonderLLM_hiwonder.Vision_Result);
          // Serial.print(info);

          // if(WonderLLM_hiwonder.Vision_Result == 1){
          //   minihexa.action_group_run(14);
          //   WonderLLM_hiwonder.Vision_Result = 0;
          // }

          //lower the IIC rate to 100W
          /*not strictly required to run; if all other IIC devices support the 400W rate, there is no need to switch back
            to the lower 100W; calling this function is for compatibility with other low-speed IIC devices*/
          IIC_Config_normal_Transmit();	
          break;
        }

        default:{ 
          //do nothing...
          break;						
        }
      }
  }
  
  if(WonderLLM_hiwonder.running_mode == 1){
    // loop_timer++;
    // if(loop_timer%50 == 0){
    //   WonderLLM_Request_Vision(vision_prompt);
    //   loop_timer = 0;
    // }

  }else if(WonderLLM_hiwonder.running_mode == 2){
    WonderLLM_hiwonder.avoid_distance = (WonderLLM_hiwonder.avoid_distance < 200) ?200 : WonderLLM_hiwonder.avoid_distance; 
    minihexa._avoid(minihexa.sensor._get_distance(), WonderLLM_hiwonder.avoid_distance);

  }else if(WonderLLM_hiwonder.running_mode == 3){
    WonderLLM_hiwonder.avoid_distance = (WonderLLM_hiwonder.avoid_distance < 200) ?200 : WonderLLM_hiwonder.avoid_distance;

    int dis = minihexa.sensor._get_distance();
    Serial.println(dis);
    if(dis > (WonderLLM_hiwonder.avoid_distance + 50)){
      vel = {0.0f,2.0f,0.0f};
    }else if(dis < (WonderLLM_hiwonder.avoid_distance - 50)){
      vel = {0.0f,-2.0f,0.0f};
    }else{
      vel = {0.0f,0.0f,0.0f};
    }
    pos = {0.0f,0.0f,0.0f};
    att = {0.0f,0.0f,0.0f};
    minihexa.move(&vel, &pos, &att,500,2);    

  }

  delay_ms(100);
  // Serial.println(loop_timer);
}

