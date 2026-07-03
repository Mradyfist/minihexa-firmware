import gc
import heapq
import json
import struct
import _thread
import time      # import the time module, used for delay control
import Hiwonder  # import the Hiwonder robot control library
import Hiwonder_DEV
from machine import ADC

Move_stride_LowSpeed_1 = 70       #single-step translation stride for forward/backward motion in low-speed mode (unit: mm)
Move_stride_HighSpeed_1 = 85      #single-step translation stride for forward/backward motion in high-speed mode (unit: mm)
Move_stride_LowSpeed_2 = 65       #single-step translation stride for left/right motion in low-speed mode (unit: mm)
Move_stride_HighSpeed_2 = 75      #single-step translation stride for left/right motion in high-speed mode (unit: mm)
Move_stride_LowSpeed_3 = 90       #single-step translation stride for diagonal motion in low-speed mode (unit: mm)
Move_stride_HighSpeed_3 = 90      #single-step translation stride for diagonal motion in high-speed mode (unit: mm)
Rotation_angle_LowSpeed = 24      #single-step rotation angle in low-speed mode (unit: degrees)
Rotation_angle_HighSpeed = 45     #single-step rotation angle in high-speed mode (unit: degrees)

# create the robot object
robot = Hiwonder.Robot()
beep = Hiwonder.Buzzer()
imu = Hiwonder.IMU()
i2csonar = Hiwonder_DEV.DEV_SONAR()
adc = ADC(33,ADC.ATTN_6DB)        #initialize the ADC; 6dB attenuation controls the measurement voltage range
adc.width(ADC.WIDTH_12BIT)

vel = [0,0,0] #the robot's motion speed along the x, y, z axes
pos = [0,0,0] #the 3D coordinate of the robot's center of gravity in the coordinate frame
att = [0,0,0] #the Euler angles of the robot's rotation about the x, y, z axes

euler = [0,0,0] #the robot's three-axis Euler angles

motion_target_step = 0        #number of motion steps in normal motion
motion_target_RunningTime = 0 #motion duration in normal motion
motion_target_RotationAngle = 0 #rotation angle in normal motion (only for in-place left/right turns)
motion_target_distance = 0    #motion distance in normal motion (used for translation in the eight directions)
move_direction = 1		      #robot translation direction in normal motion: 1-front, 2-front-right, 3-right, 4-rear-right, 5-back, 6-rear-left, 7-left, 8-front-left, 9-turn left in place, 10-turn right in place
running_mode = 1              #1-normal, 2-intelligent_avoidance, 3-intelligent_track
avoid_distance = 0         	  #minimum distance the robot keeps from the target; used in intelligent obstacle-avoidance and intelligent follow modes
rgb_left = [0,0,0]            #left RGB light-intensity ratio of the illuminated ultrasonic module
rgb_right = [0,0,0]           #right RGB light-intensity ratio of the illuminated ultrasonic module
buzzer_count = 0              #number of buzzer beeps
Speed_Mode = 1            	  #1-Low_speed, 2-BelowMiddle_speed, 3-Medium_speed, 4-High_speed
Detection_WonderMind = 0	  #0-module not detected on the bus  1-module detected on the bus
Vision_Result = 0        	  #vision recognition result
incline_direction = 0    	  #robot tilt direction (rotation about the Euler angles) in normal motion: 1-lean forward, 2-lean back, 3-lean left, 4-lean right, 5-twist left, 6-twist right
BaryCenter_direction = 0  	  #robot center-of-gravity movement direction in normal motion: 1-front, 2-front-right, 3-right, 4-rear-right, 5-back, 6-rear-left, 7-left, 8-front-left, 9-up, 10-down
ActionNum = 0                 #index of the action group to execute
ExecuteActionNum = 0          #number of times to execute the action group

sustain_move_flag = 0         #flag for the robot's continuous motion
timer = 0

func_follow_num = 0            #counter dedicated to the follow function
func_avoid_num = 0             #counter dedicated to the avoid function

class Controller:
    sleep_command_dict = {
      "command":     "sleep",
      "params":      "true",
    }

    abort_command_dict = {
      "command":     "abort",
      "params":      "true",
    }  

    vision_command_dict = {
      "command":     "vision",
      "params":      "text",
    }

    voice_command_dict = {
      "command":     "voice",
      "params":      "text",
    }


    status_command_dict = {
      "command":     "status",
      "params":      [],
    }

    action_finish_command_dict = {
      "command":     "action_finish",
      "params":      "true",
    }
    mcp_finish_setting_dict = {
      "command":     "mcp_setting",
      "params":      "params",
    } 

    status = [
        "unknown",
        "starting",
        "configuring",
        "idle",
        "connecting",
        "listening",
        "speaking",
        "upgrading",
        "activating",
        "audio_testing",
        "fatal_error",
        "invalid_state"
    ]


    def __init__(self, i2c):
        self.addr = 0x55 
        self.i2c = i2c
        self.data_heap = []  
        self.counter = 0    

        # receive queue (priority, count, data)
        self.recv_heap = [] # priority queue used to store received data, with elements of (priority, data)
        self.recv_counter = 0  # used to preserve the order of data with the same priority
        # send queue (priority, count, dict_data)
        self.send_heap = []
        self.send_counter = 0      
        self.running = True

        # Add heap size limits to prevent unbounded growth
        self.max_heap_size = 10

        self.lock = _thread.allocate_lock()


        # related to fragment data reassembly
        self.fragments = {}  # store fragment data, in the format {fragment_id: [data, total_fragments, received_fragments]}
        self.fragment_timeout = 2.0  # fragment timeout (seconds)
        self.last_fragment_time = 0  # the time the last fragment was received

        while True:
          devices = self.i2c.scan()
          addr = [hex(d) for d in devices]
          if '0x55' in addr:
            break
          time.sleep(0.01)   
        time.sleep(0.1)   

    def enqueue_send(self, dict_data, priority=0):
        """put the data to be sent into the send queue"""
        # Prevent heap from growing too large
        with self.lock:
            if len(self.send_heap) >= self.max_heap_size:
                print("Send heap full, discarding oldest message")
                heapq.heappop(self.send_heap)
            heapq.heappush(self.send_heap, (priority, self.send_counter, dict_data))
            self.send_counter += 1         

    def start(self):    
      time.sleep(0.1)
      self.send_message(self.mcp_finish_setting_dict) 
      _thread.start_new_thread(self.callback, ())

    def stop(self):    
      self.running = False

    def sleep(self):
      self.enqueue_send(self.sleep_command_dict)


    def abort(self):
      self.send_message(self.abort_command_dict)  

    def vision(self, text):
      vision_cmd = self.vision_command_dict.copy()
      vision_cmd['params'] = text
      self.send_message(vision_cmd) 

    def tts(self, text):
      tts_cmd = self.tts_command_dict.copy()
      tts_cmd['params'] = text
      self.send_message(tts_cmd) 

    def set_tts_model(self, model):
      tts_model_cmd = self.tts_model_command_dict.copy()
      tts_model_cmd['params'] = model
      self.send_message(tts_model_cmd) 

    def set_voice(self, voice):
      voice_cmd = self.voice_command_dict.copy()
      voice_cmd['params'] = self.voice[voice]
      self.send_message(voice_cmd) 

    def send_status(self, data):
      status_cmd = self.status_command_dict.copy()
      status_cmd['params'] = data
      self.send_message(status_cmd)

    def send_action_finish(self, data):
      action_cmd = self.action_finish_command_dict.copy()
      action_cmd['params'] = data
      self.send_message(action_cmd)



    def process_fragment(self, fragment_id, total_fragments, actual_data):

        """process fragment data; reassemble it once all fragments have been received"""

        current_time = time.time()



        # print(f"Processing fragment: ID={fragment_id}, total={total_fragments}, data_len={len(actual_data)}")

        # check for a timeout; if timed out, clear the fragment cache
        if self.last_fragment_time > 0 and current_time - self.last_fragment_time > self.fragment_timeout:
            self.fragments.clear()
            print("Fragment timeout, cleared all fragments")

        self.last_fragment_time = current_time

        # if it is single-fragment data, return directly
        if total_fragments == 1:
            # print("Single fragment data, returning directly")

            return actual_data



        # if it is multi-fragment data, it must be reassembled

        # use the current timestamp as the fragment group's key, so different data groups can be distinguished

        # only the first fragment (fragment_id == 1) creates a new fragment group
        if fragment_id == 1:
            # create a new fragment group, using the current timestamp as the key
            group_id = int(current_time * 1000)  # use a millisecond-level timestamp
            self.fragments[group_id] = {
                'data': [None] * total_fragments,
                'total_fragments': total_fragments,
                'received_fragments': 0,
                'start_time': current_time
            }
            # print(f"Created new fragment group {group_id} with {total_fragments} fragments")

        # if there is no fragment group, we missed the first fragment, so ignore the current fragment
        if not self.fragments:
            print("No fragment groups available, possibly missed first fragment")
            return None

        # get the newest fragment group (the most recently created one)
        group_id = max(self.fragments.keys())
        fragment_info = self.fragments[group_id]

        # print(f"Using fragment group {group_id}, received {fragment_info['received_fragments']}/{fragment_info['total_fragments']} fragments")



        # check whether the fragment ID is valid

        # fragment IDs start at 1, so subtract 1 to use as the array index

        if fragment_id >= 1 and fragment_id <= total_fragments:
            # fragment data, converted to a 0-based index
            fragment_index = fragment_id - 1
        else:
            print(f"Invalid fragment ID: {fragment_id}, total fragments: {total_fragments}")
            return None

        # check whether the total fragment count of the current fragment group matches
        if fragment_info['total_fragments'] != total_fragments:
            print(f"Fragment count mismatch: expected {fragment_info['total_fragments']}, got {total_fragments}")
            return None



        # only fragment data (fragment_id >= 1) needs to be stored

        if fragment_id >= 1:

            # store the fragment data

            if fragment_info['data'][fragment_index] is None:

                fragment_info['data'][fragment_index] = actual_data
                fragment_info['received_fragments'] += 1
                # print(f"Stored fragment {fragment_id} (index {fragment_index}), now have {fragment_info['received_fragments']}/{total_fragments}")
            else:
                print(f"Fragment {fragment_id} (index {fragment_index}) already received, ignoring duplicate")

            # check whether all fragments have been received
            if fragment_info['received_fragments'] == total_fragments:
                # print("All fragments received, reassembling data")
                # reassemble the data
                reassembled_data = b''.join(fragment_info['data'])
                # remove it from the fragment cache
                del self.fragments[group_id]
                # print(f"Reassembled data length: {len(reassembled_data)}")
                return reassembled_data

        return None

    def calculate_checksum(self, data):
        checksum = 0
        for byte in data:
            checksum ^= byte
        return checksum & 0xFF

    def send_message(self, dict_data):
        try:
            data = json.dumps(dict_data).encode("utf-8")
            # print('data len: %d, max data len:1024\n'%len(data))
            if len(data) > 1024:
                print("Warning: Data too large, truncating")
                return False
            self.i2c.writeto(self.addr, data)
            time.sleep(0.05)
            return True
        except Exception as e:
            print(f"Send error: {e}")
            return False

    def get_message(self):
        """take one item out of the priority queue; return None if the queue is empty"""
        with self.lock:
            if self.recv_heap:
                #print("2\r\n")
                priority, count, data = heapq.heappop(self.recv_heap)
                # data = json.dumps(data, ensure_ascii=False)
                return data
            else:
                return None

    def callback(self):
        gc_counter = 0  # Add counter for periodic garbage collection

        while self.running:
            try:
                # Periodic garbage collection every 100 iterations
                gc_counter += 1
                if gc_counter % 100 == 0:
                    gc.collect()
                    # print(f"GC: Free memory after collection")

                # send it, if there is any
                with self.lock:
                    while self.send_heap:
                        _, _, msg = heapq.heappop(self.send_heap)
                        try:
                            raw = json.dumps(msg).encode("utf-8")
                            self.i2c.writeto(self.addr, raw)
                        except Exception as e:
                            print(f"Send error in callback: {e}")
                            break  # Exit send loop on error

                # Limit receive heap size
                with self.lock:
                    if len(self.recv_heap) >= self.max_heap_size:
                        #print("1\r\n")
                        heapq.heappop(self.recv_heap)



                # read the header info: AA55 (frame header) + data length (2 bytes) + fragment ID (2 bytes) + total fragment count (2 bytes)

                try:

                    # add a delay to ensure the slave is ready to send data
                    time.sleep(0.01)
                    header_data = self.i2c.readfrom(self.addr, 8)
                    # print(f"Header data: {header_data.hex()}")

                    # check whether the header data is valid
                    if len(header_data) != 8:
                        print(f"Invalid header length: {len(header_data)}, expected 8")
                        time.sleep(0.1)
                        continue

                    # check whether the frame header is valid
                    flag = struct.unpack('>H', header_data[0:2])[0]
                    if flag != 0xAA55:
                        print(f"Invalid flag: 0x{flag:04X}, expected 0xAA55")
                        time.sleep(0.1)
                        continue
                except Exception as e:
                    print(f"Error reading header: {e}")
                    time.sleep(0.1)
                    continue

                data_len = struct.unpack('>H', header_data[2:4])[0]
                fragment_id = struct.unpack('<H', header_data[4:6])[0]
                total_fragments = struct.unpack('<H', header_data[6:8])[0]

                # print(f"Received: flag=0x{flag:04X}, data_len={data_len}, fragment_id={fragment_id}, total_fragments={total_fragments}")

                if data_len > 0:
                    # Add size limit check
                    if data_len > 8192:  # Reasonable limit
                        print(f"Data too large: {data_len} bytes, skipping")
                        continue

                    try:
                        # add a delay to ensure the slave is ready to send data
                        time.sleep(0.01)
                        data_with_checksum = self.i2c.readfrom(self.addr, data_len + 1)

                        # check whether the data length is correct
                        if len(data_with_checksum) != data_len + 1:
                            print(f"Invalid data length: {len(data_with_checksum)}, expected {data_len + 1}")
                            time.sleep(0.1)
                            continue

                        actual_data = data_with_checksum[:-1]
                        received_checksum = data_with_checksum[-1]
                        calculated_checksum = self.calculate_checksum(actual_data)

                        # print(f"Data: {actual_data.hex()}, received_checksum=0x{received_checksum:02X}, calculated_checksum=0x{calculated_checksum:02X}")
                    except Exception as e:
                        print(f"Error reading data: {e}")
                        time.sleep(0.1)
                        continue

                    if received_checksum == calculated_checksum:
                        # process the fragment data
                        reassembled_data = self.process_fragment(fragment_id, total_fragments, actual_data)

                        # if the data reassembly is complete (for single-fragment data it returns immediately)
                        if reassembled_data is not None:
                            if len(reassembled_data) == 1:
                                status_idx = struct.unpack('B', reassembled_data)[0]
                                # print(f"Status index: {status_idx}")
                                if status_idx < len(self.status):
                                    data = self.status[status_idx]
                                else:
                                    print(f"Invalid status index: {status_idx}")
                                    continue
                            else:
                                try:
                                    data = json.loads(reassembled_data.decode('utf-8'))
                                    # print(f"JSON data: {data}")
                                except Exception as e:
                                    print(f"JSON decode error: {e}")

                                    continue



                            # assign the data a priority here; assume all data has the same priority of 0

                            # you can customize the priority based on the data content

                            # use counter to preserve the order of data with the same priority
                            priority = 0
                            with self.lock:
                                #print(data)
                                heapq.heappush(self.recv_heap, (priority, self.recv_counter, data))
                                self.recv_counter += 1
                    else:
                        print(f"Checksum error: received=0x{received_checksum:02X}, calculated=0x{calculated_checksum:02X}")

            except Exception as e:
                print("Callback error:", e)
                # Add a longer sleep on error to prevent rapid error loops
                time.sleep(0.5)
                continue

            time.sleep(0.02)

# usage example

#set up the mcp; calling the mcp triggers the slave to return data in the params format, where the string is provided by the large model based on your conversation with it, e.g. if you ask for the current pose it might return pose
#note that command should clearly describe the available return options; return indicates whether data needs to be returned to the slave, and if so, the returned data is given as a list per the requirements of send_status

#e.g. [["pose", "0"], ["battery", 8000]]; after the slave receives it, it is returned to the large model as-is for it to judge or announce

tool_buzzer = {
	"tool_name": "self.robot.set_buzzer",
	"command": "控制机器人的蜂鸣器时调用这个工具。'count'是蜂鸣器响的次数",
	"params":[["count","int"]],
	"block": "true",
	"return": "false",
}

tool_led = {
	"tool_name":"self.robot.set_led_color",
	"command":"设置左右RGB灯颜色。lr,lg,lb是左灯RGB, rr,rg,rb是右灯RGB, 范围0-255。",
	"params":[["lr","int","0","255"],["lg","int","0","255"],["lb","int","0","255"],["rr","int","0","255"],["rg","int","0","255"],["rb","int","0","255"]],
	"block":"true",
	"return":"false",
}

tool_SpeedMode = {
	"tool_name": "self.robot.set_SpeedMode",
	"command": "切换机器人的速度模式时调用这个工具。可切换的速度模式从1到2档包括：'Low_speed', 'High_speed'。",
	"params":[["speed_mode","string"]],
	"block":"true",
	"return":"false",
}

tool_RunningMode = {
	"tool_name":"self.robot.set_RunningMode",
	"command":"切换机器人的运动模式时调用这个工具。distance为距离,默认为200,单位毫米,可切换的运动模式包括：'normal','intelligent_avoidance', 'intelligent_track'。",
	"params":[["running_mode","string"],["distance","int"]],
	"block":"true",
	"return":"false",
}

tool_ActionGroup = {
	"tool_name":"self.robot.ActionGroup",
	"command":"控制机器人执行动作组时调用这个工具。actionNum为动作组代号,3号为唤醒,4号为唤醒奔跑,5号为撒娇,6号为越障,7号为左腿战斗,8号为右腿战斗,9号为左脚向前踢球,10号为左脚向右踢球,11号为右脚向前踢球,12号为右脚向左踢球,13号为推门,14号为挥手,executeNum为动作组运行次数",
	"params":[["actionNum","int","3","14"],["executeNum","int"]],
	"block":"true",
	"return":"false",
}

tool_status = {
	"tool_name":"self.robot.get_status",
	"command":"获取机器人的实时状态时调用这个工具。可查询的状态包括：'battery', 'distance', 'poseture','running_mode','Speed_Mode'。",
	"params":[["status_name","string"]],
	"block":"true",
	"return":"true",
}

tool_move = {
	"tool_name":"self.robot.move",
	"command":"控制机器人平移时调用。move为运动方向,分别为前、右前、右、右后、后、左后、左、左前, 原地左转, 原地右转,分别使用代号1至10。step_num是运动步数默认为-1。duration是运动时长默认-1毫秒。distance是运动距离默认为-1厘米。angle参数是运动角度默认为-1度。",
	"params":[["move","int","1","10"],["step_num","int"],["duration","int"],["distance","int"],["angle","int"]],
	"block":"true",
	"return":"false",
}

tool_BaryCenterMove = {
	"tool_name":"self.robot.BaryCenterMove",
	"command":"控制机器人重心移动时调用这个工具。pose参数控制运动方向,分别为正前、右前、正右、右后、正后、左后、正左、左前、正上、正下,分别使用代号1至10,如需恢复初始姿态使用11。",
	"params":[["pose","int","1","11"]],
	"block":"true",
	"return":"false",
}

tool_InclinationAngleMove = {
	"tool_name":"self.robot.InclinationAngleMove",
	"command":"控制机器人姿态倾斜时调用这个工具。Inclination参数控制运动方向,分别为前倾、后倾、左倾、右倾、左扭、右扭,分别使用代号1至6。",
	"params":[["Inclination","int","1","6"]],
	"block":"true",
	"return":"false",
}


time.sleep(1)
control = Controller(Hiwonder_DEV.IIC())
control.send_message(tool_buzzer)
control.send_message(tool_led)
control.send_message(tool_SpeedMode) 
control.send_message(tool_RunningMode)
control.send_message(tool_ActionGroup)
control.send_message(tool_status)
control.send_message(tool_move)
control.send_message(tool_BaryCenterMove)
control.send_message(tool_InclinationAngleMove)

control.start()

i2csonar.setRGB(0,0,0,255)
beep.setVolume(100)
beep.playTone(1500, 200, True)

'''voltage acquisition interface'''
def battery_Voltage_detect():
  read_Voltage = adc.read()   #read the analog level; the return value ranges from 0 to 4095
  read_Voltage *= 3.3/4096  #convert the ADC voltage value into the analog voltage (the portion of the supply voltage divided across the divider resistor)
  read_Voltage *= 11 #the divider resistor gets 1/11 of the original voltage
  return read_Voltage

'''run the wake-up running action'''
def wake_up():
  vel = [0,0,0]
  robot.go(vel, 0, 0)
  pos = [0,0,0]
  robot.set_body_pose(pos, 200)
  att = [0,0,0]
  robot.set_body_angle(att, 200)

  time.sleep(0.1)
  att = [-8,0,-15]
  robot.set_body_angle(att, 200)

  time.sleep(1)
  att = [-8,0,15]
  robot.set_body_angle(att, 200)
  
  time.sleep(1)
  att = [0,0,0]
  robot.set_body_angle(att, 200)

  time.sleep(0.6)
  vel = [0,1.5,0]
  robot.go(vel, 10, 600)
  time.sleep(6)
  
'''run the wake-up action'''
def _wake_up():
  vel = [0,0,0]
  robot.go(vel, 0, 0)
  pos = [0,0,0]
  robot.set_body_pose(pos, 200)
  att = [0,0,0]
  robot.set_body_angle(att, 200)

  att = [-8,0,-15]
  robot.set_body_angle(att, 200)
  time.sleep(0.1)
  
  att = [-8,0,15]
  robot.set_body_angle(att, 600)
  time.sleep(0.1)
  
  att = [0,0,0]
  robot.set_body_angle(att, 200)
  time.sleep(0.2)
  
'''run the act-cute action'''
def acting_cute():
  vel = [0,0,0]
  robot.go(vel, 0, 0)
  pos = [0,0,0]
  robot.set_body_pose(pos, 200)
  att = [0,0,0]
  robot.set_body_angle(att, 200)
  
  att = [0,10,0]
  robot.set_body_angle(att, 200)
  time.sleep(0.3)
  
  att = [0,-10,0]
  robot.set_body_angle(att, 200)
  time.sleep(0.3)
  
  att = [0,10,0]
  robot.set_body_angle(att, 200)
  time.sleep(0.3)
  
  att = [0,-10,0]
  robot.set_body_angle(att, 200)
  time.sleep(0.3)
  
  att = [0,0,0]
  robot.set_body_angle(att, 200)
  time.sleep(0.3)
  
  pos = [2,0,0]
  robot.set_body_pose(pos, 200) 
  time.sleep(0.2)
  
  pos = [-2,0,0]  
  robot.set_body_pose(pos, 200) 
  time.sleep(0.2)
  
  pos = [2,0,0]
  robot.set_body_pose(pos, 200) 
  time.sleep(0.2)
  
  pos = [-2,0,0]  
  robot.set_body_pose(pos, 200) 
  time.sleep(0.2)
  
  pos = [0,0,0]
  robot.set_body_pose(pos, 200)  
  time.sleep(0.2)

'''dis_avoid: target spacing'''
def avoid(dis_avoid):
  global func_avoid_num #declare an external (global) variable
  
  func_avoid_num +=1
  #run the internal logic once every 5*0.02=0.1s, to avoid reading the ultrasonic module too frequently and getting abnormal data
  if(func_avoid_num % 5 == 0):
    _distance = i2csonar.getDistance() *10 #cm--->mm
    print(_distance)
    if (_distance<=100): #below the critical distance, make the robot move backward
      robot.go([0,-2,0],4,1000)
      time.sleep(4)
    else:
      if (_distance<=dis_avoid): #below the target spacing but above the critical distance, make the robot turn
        robot.go([0,0,1.8],4,1000)
        time.sleep(4)
      else: #greater than the target spacing, make the robot move forward
        robot.go([0,2,0],-1,800)

'''dis_avoid: target spacing'''
def follow(dis_avoid):
  global func_follow_num #declare an external (global) variable
  
  func_follow_num +=1
  #run the internal logic once every 5*0.02=0.1s, to avoid reading the ultrasonic module too frequently and getting abnormal data
  if(func_follow_num % 5 ==0):
    _distance = i2csonar.getDistance() *10
    print(_distance)
    if (_distance<=dis_avoid -50): #below the critical distance, make the robot move backward
      robot.go([0,-2,0],-1,500)
    else:
      if (_distance>=dis_avoid +50): #greater than the target spacing, make the robot move forward
        robot.go([0,2,0],-1,500)
      else:  #below the target spacing but above the critical distance, make the robot stop
        robot.go([0,0,0],0)


# ================== main loop ==================
robot_status = []
loop_counter = 0

try:
    while True:
        loop_counter += 1
        if loop_counter % 1000 == 0:
            gc.collect()

        data = control.get_message()
        if data:
            if data not in control.status:
                print("Received data from queue:", data)

                try:
                    if 'move' in data:          #tool_move tool command
                        move_direction = data.get('move', 0)
                        motion_target_step = data.get('step_num', 0)
                        motion_target_RunningTime = data.get('duration', 0)
                        motion_target_RotationAngle = data.get('angle', 0)
                        motion_target_distance = data.get('distance', 0)
					
                        if(motion_target_RotationAngle == -1 and motion_target_distance == -1): #do not specify the motion distance or the motion angle
          
                          #specify neither step count nor duration; by default keep walking without stopping
                          if(motion_target_step == -1 and motion_target_RunningTime == -1): 
                            sustain_move_flag = 1

                          #specify the number of motion steps but not the motion duration
                          elif(motion_target_step != -1 and motion_target_RunningTime == -1): 
                            timer = 600 * motion_target_step #taking one step takes 600ms by default

                          #specify the number of motion steps but not the motion duration
                          elif(motion_target_step == -1 and motion_target_RunningTime != -1):
                            timer = motion_target_RunningTime

                          #both the number of steps and the duration are specified; the two conflict, so the duration takes precedence
                          elif(motion_target_step != -1 and motion_target_RunningTime != -1): 
                            timer = motion_target_RunningTime
                            
                        elif(motion_target_RotationAngle != -1): #specify the motion angle
    
                          #convert the motion angle into a number of steps, handling it as "do not specify distance or angle" -- "specify step count but not duration"
                          if(Speed_Mode == 1):  #low-speed mode
                            motion_target_step = motion_target_RotationAngle / Rotation_angle_LowSpeed

                          elif(Speed_Mode == 2): #high-speed mode
                            motion_target_step = motion_target_RotationAngle / Rotation_angle_HighSpeed
            
                          timer = 600 * motion_target_step  #taking one step takes 600ms by default
                          
                        elif(motion_target_distance != -1):      #specify the motion distance

                          #convert the motion distance into a number of steps, handling it as "do not specify distance or angle" -- "specify step count but not duration"
                          if(Speed_Mode == 1):  #low-speed mode
                            if(move_direction == 1 or move_direction == 5): #forward/backward
                              motion_target_step = motion_target_distance / Move_stride_LowSpeed_1
                              
                            elif(move_direction == 3 or move_direction == 7): #left/right
                              motion_target_step = motion_target_distance / Move_stride_LowSpeed_2
 
                            elif(move_direction == 2 or move_direction == 4 or move_direction == 6 or move_direction == 8): #diagonal
                              motion_target_step = motion_target_distance / Move_stride_LowSpeed_3

                          elif(Speed_Mode == 2): #high-speed mode
                            if(move_direction == 1 or move_direction == 5): #forward/backward
                              motion_target_step = motion_target_distance / Move_stride_HighSpeed_1
                              
                            elif(move_direction == 3 or move_direction == 7): #left/right
                              motion_target_step = motion_target_distance / Move_stride_HighSpeed_2
 
                            elif(move_direction == 2 or move_direction == 4 or move_direction == 6 or move_direction == 8): #diagonal
                              motion_target_step = motion_target_distance / Move_stride_HighSpeed_3

                          timer = 600 * motion_target_step #taking one step takes 600ms by default
    
                        else:                                    #both the motion distance and angle are specified; the two conflict, so the robot does not execute this round of commands
                        
                          motion_target_step = 0
                          motion_target_RunningTime = 0
                          
                          
                        if (move_direction == 1):   #1-straight ahead
                          if(motion_target_step == 0 and motion_target_RunningTime == 0): #the current command is "robot stop motion"
                            sustain_move_flag = 0
             
                          vel = [0,2*Speed_Mode,0]
							
                        elif (move_direction == 2): #2-front-right
                          vel = [2*Speed_Mode,2*Speed_Mode,0]
							
                        elif (move_direction == 3): #3-straight right
                          vel = [2*Speed_Mode,0,0]
							
                        elif (move_direction == 4): #4-rear-right
                          vel = [2*Speed_Mode,-2*Speed_Mode,0]
							
                        elif (move_direction == 5): #5-straight back
                          vel = [0,-2*Speed_Mode,0]
							
                        elif (move_direction == 6):	#6-rear-left
                          vel = [-2*Speed_Mode,-2*Speed_Mode,0]
							
                        elif (move_direction == 7):	#7-straight left					
                          vel = [-2*Speed_Mode,0,0]
							
                        elif (move_direction == 8): #8-front-left
                          vel = [-2*Speed_Mode,2*Speed_Mode,0]
							
                        elif (move_direction == 9): #9-turn left in place
                          vel = [0,0,2*Speed_Mode]
							
                        elif (move_direction == 10):#10-turn right in place	
                          vel = [0,0,-2*Speed_Mode]
							
                        else:
                          vel = [0,0,0]
							
                        robot.go(vel, motion_target_step, 600)
						
                        if(sustain_move_flag != 1):
                          time.sleep(timer/1000) #the unit here is seconds, so divide by 1000 for the unit conversion
                          vel = [0,0,0]
                          robot.go(vel, 0, 0)
          
                        control.send_action_finish("true")

                    if 'Inclination' in data:   #tool_InclinationAngleMove tool command
                        incline_direction = data.get('Inclination', 0)
                        
                        if (incline_direction == 1):   #1-lean forward
                          att = [12,0,0]
							
                        elif (incline_direction == 2):   #2-lean back
                          att = [-12,0,0]
							
                        elif (incline_direction == 3):   #3-lean left
                          att = [0,15,0]
							
                        elif (incline_direction == 4):   #4-lean right
                          att = [0,-15,0]
							
                        elif (incline_direction == 5):   #5-twist left
                          att = [0,0,-15]
							
                        elif (incline_direction == 6):   #6-twist right
                          att = [0,0,15]
							
                        else:
                          att = [0,0,0]
							
                        robot.set_body_angle(att, 600)
                        control.send_action_finish("true")

                    '''note: get_status type message detection must be performed before set_mode type message detection,
                      otherwise the get_status message frame that queries motion state, because it contains a running_mode field,
                      would also be mistaken for a set_mode type message'''
                    '''get_status type message detection must be performed before BaryCenterMove type message detection, for the same reason as above'''
                    if 'status_name' in data:   #tool_status tool command
                      if data['status_name'] == 'battery':
                        value = Hiwonder.Battery_power()
                        print(f"Battery: {value}")
                        robot_status.append(["battery", str(value),"mv"])

                      if data['status_name'] == 'Speed_Mode':
                        robot_status.append(["Speed_mode", str(Speed_Mode)])

                      if data['status_name'] == 'distance':
                        value = i2csonar.getDistance() *10  #the function returns in cm; convert the unit to mm
                        robot_status.append(["distance", str(value),"mm"])

                      if data['status_name'] == 'running_mode':
                        robot_status.append(["running_mode", str(running_mode)])
	
                      if data['status_name'] == 'poseture':
                        euler = imu.read_angle()
                        robot_status.append([["roll", str(euler[0])],["pitch", str(euler[1])]])
  
                      if robot_status:
                        control.send_status(robot_status)
                        robot_status.clear()
							
                    if 'pose' in data:          #tool_BaryCenterMove tool command
                      BaryCenter_direction = data.get('pose', 0)
                        
                      if (BaryCenter_direction == 1):   #1-straight ahead
                        pos = [0,3,0]
							
                      elif (BaryCenter_direction == 2):   #2-front-right
                        pos = [3,3,0]
							
                      elif (BaryCenter_direction == 3):   #3-straight right
                        pos = [3,0,0]
							
                      elif (BaryCenter_direction == 4):   #4-rear-right
                        pos = [3,-3,0]
							
                      elif (BaryCenter_direction == 5):   #5-straight back
                        pos = [0,-3,0]
							
                      elif (BaryCenter_direction == 6):   #6-rear-left
                        pos = [-3,-3,0]
							
                      elif (BaryCenter_direction == 7):   #7-straight left
                        pos = [-3,0,0]
							
                      elif (BaryCenter_direction == 8):   #8-front-left
                        pos = [-3,3,0]
							
                      elif (BaryCenter_direction == 9):   #9-straight up
                        pos = [0,0,3]
							
                      elif (BaryCenter_direction == 10):  #10-straight down	
                        pos = [0,0,-1.5]
							
                      else: #11-restore the initial pose
                        robot.set_body_angle([0,0,0],600)
                        pos = [0,0,0]
							
                      robot.set_body_pose(pos, 600)
                      control.send_action_finish("true")
              
                    if 'running_mode' in data:  #tool_RunningMode tool command
                      if data['running_mode'] == 'normal':
                        running_mode = 1
                        avoid_distance = 0
						
                      if data['running_mode'] == 'intelligent_avoidance':
                        running_mode = 2
						
                      if data['running_mode'] == 'intelligent_track':
                        running_mode = 3
                       
                      if data['running_mode'] == 'distance': 
                        avoid_distance = data.get('distance', 0)
                      
                      robot.go([0, 0, 0], 0, 1000)
                      robot.set_body_pose([0, 0, 0], 600)
                      robot.set_body_angle([0, 0, 0], 600)						
                      control.send_action_finish("true")

                    if 'speed_mode' in data:    #tool_SpeedMode tool command
                      if data['speed_mode'] == 'Low_speed':
                        Speed_Mode = 1
						
                      if data['speed_mode'] == 'High_speed':
                        Speed_Mode = 2
							
                      control.send_action_finish("true")

                    if 'lr' in data:          	#tool_led tool command
                      rgb_left[0] = data.get('lr', 0)
                      rgb_left[1] = data.get('lg', 0)
                      rgb_left[2] = data.get('lb', 0)
                      rgb_right[0] = data.get('rr', 0)
                      rgb_right[1] = data.get('rg', 0)
                      rgb_right[2] = data.get('rb', 0)
						
                      #clamp operation, to guarantee no values outside 0-255
                      rgb_left[0] = 255 if(rgb_left[0] > 255) else (0 if(rgb_left[0] < 0) else rgb_left[0])
                      rgb_left[1] = 255 if(rgb_left[1] > 255) else (0 if(rgb_left[1] < 0) else rgb_left[1])
                      rgb_left[2] = 255 if(rgb_left[2] > 255) else (0 if(rgb_left[2] < 0) else rgb_left[2])
                      rgb_right[0] = 255 if(rgb_right[0] > 255) else (0 if(rgb_right[0] < 0) else rgb_right[0])
                      rgb_right[1] = 255 if(rgb_right[1] > 255) else (0 if(rgb_right[1] < 0) else rgb_right[1])
                      rgb_right[2] = 255 if(rgb_right[2] > 255) else (0 if(rgb_right[2] < 0) else rgb_right[2])						
						
                      i2csonar.setRGB(1,rgb_left[0],rgb_left[1],rgb_left[2])
                      i2csonar.setRGB(2,rgb_right[0],rgb_right[1],rgb_right[2])
                      #the following statement can also be used instead
                      #i2csonar.setRGB(0,rgb_left[0],rgb_left[1],rgb_left[2])
                      control.send_action_finish("true")

                    if 'count' in data:     	#tool_buzzer tool command
                      buzzer_count = data.get('count', 0)
                      for num in range(buzzer_count):
                        beep.playTone(3000,200,True)
                      control.send_action_finish("true")

                    if 'actionNum' in data:     #tool_ActionGroup tool command
                      ActionNum = data.get('actionNum', 0)
                      ExecuteActionNum = data.get('executeNum', 0)
                      
                      if(ActionNum == 3): #wake-up action
                        for num in ExecuteActionNum:
                          _wake_up()
                        
                      elif(ActionNum == 4): #wake-up running action
                        for num in ExecuteActionNum:
                          wake_up()
                        
                      elif(ActionNum == 5): #act-cute action
                        for num in ExecuteActionNum:
                          acting_cute()
                        
                      elif(6<= ActionNum <= 14):
                        robot.action_run(str(ActionNum),ExecuteActionNum)
                        time.sleep(0.1)   #wait 0.1s to leave other threads response time to handle the action-group run command and set the flag
                        #robot.__action_run is set to 1 while an action group runs, and 0 otherwise
                        while (robot.__action_run != 0):
                          time.sleep(0.001)
                      control.send_action_finish("true")

                except Exception as e:
                    print(f"Action execution error: {e}")
            else:
                pass
				
        else:
            pass
       	
        if(running_mode == 2): 
          avoid_distance = 200 if(avoid_distance <200) else avoid_distance #clamp to limits
          avoid(avoid_distance)
          
        elif(running_mode == 3):
          avoid_distance = 200 if(avoid_distance <200) else avoid_distance #clamp to limits
          follow(avoid_distance)
			
        #since the task that receives from the large-model module runs every 0.02s, the main-loop delay here is kept consistent to ensure the command processing rate matches the command receiving rate
        time.sleep(0.02)

except KeyboardInterrupt:
    print("Stopping...")
finally:
    control.stop()
    time.sleep(0.5)






