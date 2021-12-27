import sys
import time
import json

# Import the ROS related imports only in production mode
# if not "unittest" in sys.modules.keys():
import rospy
from central_control.srv import *

from src._gamlaction import GamlAction
from src._gamlprimitives import GamlState
from hibotlib.hibot_client import *

def move_and_check(hibot_client, motor_list, target_position, speed, accel, decel, block=True, timeout=60, tol=0.07):
	def check_list_len():
		flag = True
		number = len(motor_list)
		flag = flag and len(target_position) == number
		flag = flag and len(speed) == number
		flag = flag and len(accel) == number
		flag = flag and len(decel) == number
		return flag

	if not check_list_len():
		print("[Gaml: MOVE AND CHECK] All of the list type input argument must have same length.")
		return False

	movement_id_list = []
	target_consition_list = []
	static_condition_list = []
	for i in range(len(motor_list)):
		if target_position[i] == -99: continue
		res = hibot_client.motor_ptp(motor_list[i], target_position[i], speed[i], accel[i], decel[i])
		if isinstance(res, int): 
			print("[Gaml: MOVE AND CHECK] Call motor(id:%d) movement failed."%(motor_list[i]))
			print(res)
			return False
		else:
			movement_id_list.append(res)

		pos_condition = hibot_client.generate_condition(motor_list[i], condition_type_name="HIBOT_ID_POS_CONDITION", arg_list=["POS_NEAR", target_position[i], tol], dep_list=[res])
		target_consition_list.append(pos_condition)

		static_condition = hibot_client.generate_condition(motor_list[i], condition_type_name="HIBOT_ID_STATIC_TIME_CONDITION", arg_list=None, dep_list=[res])
		static_condition_list.append(static_condition)

	result = False
	final_target_condition = hibot_client.generate_condition(0, BLANK_CONDITION, dep_list=[cond.cmd_id for cond in target_consition_list])
	final_static_condition = hibot_client.generate_condition(0, BLANK_CONDITION, dep_list=[cond.cmd_id for cond in static_condition_list])

	todo_list = target_consition_list + static_condition_list + [final_target_condition, final_static_condition]
	result = hibot_client.receive_dequeue_action(todo_list, final_target_condition.cmd_id, [final_target_condition.cmd_id, final_static_condition.cmd_id])

	return result == 1

def call_homing(hibot_client, motor_list, homing_method_list=None):
	pass

def IO_control(act_obj, io_name, status):
	master = int(act_obj.io_data_dict[io_name]["master"])
	pin = int(act_obj.io_data_dict[io_name]["pin"])
	act_obj.module.hibot_client.io_set(master, pin, status)


def call_main_handler(module, command, data):
	rospy.wait_for_service('/from_gaml')
	flag = True
	msg = ""
	try:
		call_main = rospy.ServiceProxy('/from_gaml', GamlSignal)
		res = call_main(module, command, data)
		flag, msg = res.status, res.message
	except rospy.ServiceException as e:
		flag = False
		msg = str(e)
	if not flag:
		print("[Gaml]: call service failed:%s"%msg)
	return flag, msg

class DelayAction(GamlAction):
	"""
	Represent the delay action of the
	Gaml script. 
	"""
	def __init__(self, module, **configs):
		super(DelayAction, self).__init__(module, **configs)

	def setup(self, **params):
		self.delay_time = params.get("delay_time", 0.5)

	def execute(self):
		self.state = GamlState.Executing
		time.sleep(self.delay_time)
		self.state = GamlState.Finished
		self.finish_signal.set()

class BlockAction(GamlAction):
	"""
	Represent the block action of the
	Gaml script. 
	"""
	def __init__(self, module, **configs):
		super(BlockAction, self).__init__(module, **configs)

	def setup(self, **params):
		self.block_message = params["block_message"]

	def execute(self):
		self.state = GamlState.Executing
		
		flag = False
		while not flag and not rospy.is_shutdown():
			flag, msg = call_main_handler(self.module.name, "check_status", self.block_message)

		self.state = GamlState.Finished
		self.finish_signal.set()



class Move(GamlAction):
	"""
	Represent the move action of the
	Gaml script. 
	"""
	def __init__(self, module, **configs):
		super(Move, self).__init__(module, **configs)

	def setup(self, **params):
		self.axis_list = ["x", "y", "z"]
		self.actuator_mapping = params["actuator_mapping"]
		self.pos_data_dict = params["position_mapping"][self.module.name]
		self.position_str = params["position"] if isinstance(params["position"], str) else params["position"].id
		self.speed = 0.1
		self.acceleration = 0.5
		self.deceleration = 0.5
		self.speed_akribis = 20
		self.acceleration_akribis = 5000
		self.deceleration_akribis = 5000

	def execute(self):
		self.state = GamlState.Executing
		print(str(self.module)+" running Move action...")

		target_pos_str = self.position_str
		if target_pos_str in self.pos_data_dict:
			target_pos = self.pos_data_dict[target_pos_str]
		else:
			indicator = False
			while not indicator and not rospy.is_shutdown():
				indicator, msg = call_main_handler(self.module.name, "get_target_pos", json.dumps({"description": self.position_str}))
				time.sleep(2)
			target_pos_str = msg
			target_pos = self.pos_data_dict[msg]

		speed = [self.speed for i in range(3)]
		accel = [self.acceleration for i in range(3)]
		decel = [self.deceleration for i in range(3)]

		#speed = [2, 2, 0.2]
		#accel = [6, 6, 1]
		#decel = [6, 6, 1]

		if self.module.name == "AnpGantry" or self.module.name == "SortingGantry":
			speed[2] = self.speed_akribis
			accel[2] = self.acceleration_akribis
			decel[2] = self.deceleration_akribis
		move_and_check(self.module.hibot_client, [self.actuator_mapping[self.module.name+"_"+self.axis_list[i]] for i in range(len(self.axis_list))], target_pos, speed, accel, decel)

		self.module.position = target_pos_str
		self.state = GamlState.Finished
		self.finish_signal.set()

class MoveTray(GamlAction):
	"""
	Represent the move tray action of the
	Gaml script. 
	"""
	def __init__(self, module, **configs):
		super(MoveTray, self).__init__(module, **configs)

	def setup(self, **params):
		"""
		format of tray_data: [tray id]@[position name]
		"""
		self.tray_data = params["tray_id"] if isinstance(params["tray_id"], str) else params["tray_id"].id
		self.position_seq = params["position_seq"] if isinstance(params["position_seq"], str) else params["position_seq"].id
		self.io_data_dict = params["io_mapping"]
		self.pos_data_dict = params["position_mapping"][self.module.name]
		self.axis_list = ["x", "y", "z"]
		self.speed = [0.1, 0.1, 0.1]
		self.accel = [0.5, 0.5, 0.5]
		self.decel = [0.5, 0.5, 0.5]

	def execute(self):
		self.state = GamlState.Executing
		print(str(self.module)+" running MoveTray action...")

		# Step1: Decode the input arguments and transform the format
		# 			call rosservice if needed.

		# Handle tray_data [tray id]@[src pos]
		if "@" not in self.tray_data:
			indicator = False
			while not indicator and not rospy.is_shutdown():
				indicator, msg = call_main_handler(self.module.name, "get_transform_tray_data", self.tray_data)
				time.sleep(2)
			self.tray_data = msg

		data_list = self.tray_data.split("@")
		tray_id = data_list[0]
		src_pos_str = data_list[1] + "_position"
		src_pos = self.pos_data_dict[src_pos_str]

		# Handle target pos [target name]_position
		target_pos_str = self.position_seq if "_position" in self.position_seq else self.position_seq+"_position"
		if target_pos_str in self.pos_data_dict:
			target_pos = self.pos_data_dict[target_pos_str]
		else:
			indicator = False
			while not indicator and not rospy.is_shutdown():
				indicator, msg = call_main_handler(self.module.name, "get_target_pos", json.dumps({"description": self.position_seq, "tray_id": tray_id}))
				time.sleep(2)
			target_pos_str = msg
			target_pos = self.pos_data_dict[msg]
			
		# check the validation of this action based on hardware sensor: 
		# 	there should be a tray at the source position
		# 	there should NOT be any tray at the target position
		while not self.check_validation(src_pos_str, target_pos_str) and not rospy.is_shutdown():
			rospy.logfatal("[Gaml] Invalid tray movement from %s to %s" % (src_pos_str, target_pos_str))
			time.sleep(1)

		# Step2: Tell main_handler we are going to take the tray(tray_id, position) to update current_state
		# This will also check the validation based on software data
		if tray_id == "TestTray":
			indicator, msg = call_main_handler(self.module.name, "take_tray", json.dumps({"position": src_pos_str.replace("_position", ""), "tray_id":tray_id}))
			if not indicator:
				while not rospy.is_shutdown():
					pass

		# Step3: Move the tray to target position
		move_and_check(self.axis_list, self.module.motor_dict, src_pos, self.speed, self.accel, self.decel)
		self.take_tray(src_pos_str)
		move_and_check(self.axis_list, self.module.motor_dict, target_pos, self.speed, self.accel, self.decel)
		self.put_tray(target_pos_str)

		# Step4: Tell main_handler we have put the tray(tray_id, position) to update current_state
		if tray_id == "TestTray":
			indicator, msg = call_main_handler(self.module.name, "put_tray", json.dumps({"position": target_pos_str.replace("_position", ""), "tray_id":tray_id}))
			if not indicator:
				while not rospy.is_shutdown():
					pass
		self.module.position = target_pos_str

		self.state = GamlState.Finished
		self.finish_signal.set()

	def check_validation(self, src_position_str, target_position_str):
		"""
		This is the funciton to check if there is really a tray at the source position and is there is any tray at the target position.
		This function will check this with data from io module
		"""
		src_position_str = src_position_str.replace("_position", "")
		target_position_str = target_position_str.replace("_position", "")
		result = True
		
		if "TrayIO" in src_position_str:
			result = result and self.get_digital_IO(src_position_str+"_lower_tray_sensor")
		else:
			result = result and self.get_digital_IO(src_position_str+"_tray_sensor")

		if "TrayIO" in target_position_str:
			result = result and not self.get_digital_IO(target_position_str+"_higher_tray_sensor")
		else:
			result = result and not self.get_digital_IO(target_position_str+"_tray_sensor")

		return result

	def IO_control(self, io_name, status):
		master = int(self.io_data_dict[io_name]["master"])
		pin = int(self.io_data_dict[io_name]["pin"])
		self.module.io_dict[master].io_set(pin, status)

	def get_digital_IO(self, io_name):
		master = int(self.io_data_dict[io_name]["master"])
		pin = int(self.io_data_dict[io_name]["pin"])
		read_data = self.module.io_dict[master].io_monitor()["read_data"]
		print("[get_digital_IO] name: {0}, value:{1}".format(io_name, int(read_data[pin:pin+1])))
		return int(read_data[pin:pin+1])

	def open_tray_latch(self):
		self.IO_control("tray_click_push_dual1_on", 0)
		self.IO_control("tray_click_push_dual2_on", 0)
		self.IO_control("tray_click_push_dual1_off", 0)
		self.IO_control("tray_click_push_dual2_off", 0)
		self.IO_control("tray_click_push_dual1_on", 1)
		self.IO_control("tray_click_push_dual2_on", 1)
		time.sleep(2)

	def close_tray_latch(self):
		self.IO_control("tray_click_push_dual1_on", 0)
		self.IO_control("tray_click_push_dual2_on", 0)
		self.IO_control("tray_click_push_dual1_off", 0)
		self.IO_control("tray_click_push_dual2_off", 0)
		self.IO_control("tray_click_push_dual1_off", 1)
		self.IO_control("tray_click_push_dual2_off", 1)
		time.sleep(2)

	def take_tray(self, position_str):
		# Step1: Decode
		position_type = None
		if "TrayIO" in position_str:
			position_type = "TrayIO"
		elif "sorting" in position_str:
			position_type = "Sorting"
		else:
			position_type = "Anp"

		high_pos 	= self.pos_data_dict[position_type+"_high"]
		middle_pos 	= self.pos_data_dict[position_type+"_middle_get"]
		low_pos 	= self.pos_data_dict[position_type+"_low"]	

		# Step2: Execute
		# If the tray_position has a lock, unlock it first
		if position_type == "Sorting" or position_type == "Anp":
			lock_io_str 	= position_str.split("_position")[0]+"_lock_tray"
			unlock_io_str 	= position_str.split("_position")[0]+"_unlock_tray"
			self.IO_control(lock_io_str, 0)
			self.IO_control(unlock_io_str, 0)
			self.IO_control(unlock_io_str, 1)

		move_and_check(self.axis_list, self.module.motor_dict, high_pos, self.speed, self.accel, self.decel)
		self.open_tray_latch()
		move_and_check(self.axis_list, self.module.motor_dict, middle_pos, self.speed, self.accel, self.decel)
		self.close_tray_latch()
		move_and_check(self.axis_list, self.module.motor_dict, low_pos, self.speed, self.accel, self.decel)

	def put_tray(self, position_str):
		# Step1: Decode
		position_type = None
		if "TrayIO" in position_str:
			position_type = "TrayIO"
		elif "sorting" in position_str:
			position_type = "Sorting"
		else:
			position_type = "Anp"

		high_pos 	= self.pos_data_dict[position_type+"_high"]
		middle_pos 	= self.pos_data_dict[position_type+"_middle"]
		low_pos 	= self.pos_data_dict[position_type+"_low"]	

		# Step2: Execute
		# If the tray_position has a lock, unlock it first
		if position_type == "Sorting" or position_type == "Anp":
			lock_io_str 	= position_str.split("_position")[0]+"_lock_tray"
			unlock_io_str 	= position_str.split("_position")[0]+"_unlock_tray"
			self.IO_control(lock_io_str, 0)
			self.IO_control(unlock_io_str, 0)
			self.IO_control(unlock_io_str, 1)
			time.sleep(0.2)

		move_and_check(self.axis_list, self.module.motor_dict, middle_pos, self.speed, self.accel, self.decel)
		self.open_tray_latch()
		move_and_check(self.axis_list, self.module.motor_dict, high_pos, self.speed, self.accel, self.decel)
		self.close_tray_latch()
		move_and_check(self.axis_list, self.module.motor_dict, low_pos, self.speed, self.accel, self.decel)

		# If the tray_position has a lock, lock it after the execution
		if position_type == "Sorting" or position_type == "Anp":
			self.IO_control(lock_io_str, 0)
			self.IO_control(unlock_io_str, 0)
			self.IO_control(lock_io_str, 1)

class SortingAction(GamlAction):
	"""
	Represent the sorting action of the
	Gaml script. 
	"""
	def __init__(self, module, **configs):
		super(SortingAction, self).__init__(module, **configs)

	def setup(self, **params):
		self.axis_list = ["x", "y", "z"]
		self.pos_data_dict = params["position_mapping"][self.module.name]
		self.io_data_dict = params["io_mapping"]
		self.speed_num = 0.5
		self.acceleration = 0.5
		self.deceleration = 0.5
		self.speed_akribis = 60
		self.acceleration_akribis = 5000
		self.deceleration_akribis = 5000
		self.speed = (self.speed_num, self.speed_num, self.speed_akribis)
		self.accel = (self.acceleration, self.acceleration, self.acceleration_akribis)
		self.decel = (self.deceleration, self.deceleration, self.deceleration_akribis)

		self.suck_delay = 0.05
		self.blow_delay = 0.05

	def execute(self):
		self.state = GamlState.Executing
		print(str(self.module)+" running Sorting action...")

		# Step1: Get the sorting sequences and sorted tray_id from main_handler
		indicator, data = call_main_handler(self.module.name, "get_sorting_seq", "")
		data_dict = json.loads(data)
		sorting_seq = data_dict["action_sequence"]
		sorting_tray_id = data_dict["tray_id"]

		# Step2: Execute the sorting sequences 
		# The format of action sequence looks like
		# [{"src":{"position":"sorting1", "x":1, "y":2}, "target":{"position":"sorting3", "x":2, "y":4}},{}]
		for single_action in sorting_seq:
			self.execute_single_sort(single_action)

		# Step3: Finish sorting and Tell main_handler to remove the tray_id from the data
		# TODO: Handle if Gaml CANNOT finish all actions in the sequence because of some reasons
		# 		such as, HBin2 tray is full of devices
		indicator, msg = call_main_handler(self.module.name, "update_sorting", json.dumps({"tray_id":str(sorting_tray_id), "finished_action_number": len(sorting_seq)}))

		self.state = GamlState.Finished
		self.finish_signal.set()

	def execute_single_sort(self, single_action):
		"""
		single_action: {"src":{"position":"sorting1", "x":1, "y":2}, "target":{"position":"sorting3", "x":2, "y":4}}
		"""
		# Step1: Decode the message
		src_position_str = single_action["src"]["position"]
		src_position_str = src_position_str if "_position" in src_position_str else src_position_str+"_position"
		src_position = self.generate_pos(src_position_str, single_action["src"]["row"], single_action["src"]["column"])
		target_position_str = single_action["target"]["position"]	
		target_position_str = target_position_str if "_position" in target_position_str else target_position_str+"_position"		
		target_position = self.generate_pos(target_position_str, single_action["target"]["row"], single_action["target"]["column"])
		high_pos = self.pos_data_dict["sorting_high"]
		low_pos = self.pos_data_dict["sorting_low"]
		
		# Step2: Execute
		move_and_check(self.axis_list, self.module.motor_dict, src_position, self.speed, self.accel, self.decel)
		move_and_check(self.axis_list, self.module.motor_dict, low_pos, self.speed, self.accel, self.decel)
		self.IO_control("Sorting_suck", 1)
		time.sleep(self.suck_delay)
		move_and_check(self.axis_list, self.module.motor_dict, high_pos, self.speed, self.accel, self.decel)
		move_and_check(self.axis_list, self.module.motor_dict, target_position, self.speed, self.accel, self.decel)
		move_and_check(self.axis_list, self.module.motor_dict, low_pos, self.speed, self.accel, self.decel)
		self.IO_control("Sorting_suck", 0)
		self.IO_control("Sorting_blow", 1)
		time.sleep(self.blow_delay)
		move_and_check(self.axis_list, self.module.motor_dict, high_pos, self.speed, self.accel, self.decel)
		self.IO_control("Sorting_blow", 0)

	def generate_pos(self, base_pos_str, row_index, column_index):
		# TODO: Get the magic number from db(global recipe data) 
		base_pos = self.pos_data_dict[base_pos_str]
		x_dis = 7.5
		y_dis = 7.5
		y_offset = 1.3/38
		return (base_pos[0]+x_dis*column_index, base_pos[1]-y_dis*row_index+y_offset*column_index, base_pos[2])

	def IO_control(self, io_name, status):
		master = self.io_data_dict[io_name]["master"]
		pin = self.io_data_dict[io_name]["pin"]
		self.module.io_dict[master].io_set(pin, status)

class AnpGantryAction(GamlAction):
	"""
	Represent the sorting action of the
	Gaml script. 
	"""
	def __init__(self, module, **configs):
		super(AnpGantryAction, self).__init__(module, **configs)

	def setup(self, **params):
		self.axis_list = ["x", "y", "z"]
		self.pos_data_dict = params["position_mapping"][self.module.name]
		self.io_data_dict = params["io_mapping"]
		self.move_type = params["type"]

		self.speed_num = 0.5
		self.acceleration = 0.5
		self.deceleration = 0.5
		self.speed_akribis = 20
		self.acceleration_akribis = 5000
		self.deceleration_akribis = 5000
		self.speed = (self.speed_num, self.speed_num, self.speed_akribis)
		self.accel = (self.acceleration, self.acceleration, self.acceleration_akribis)
		self.decel = (self.deceleration, self.deceleration, self.deceleration_akribis)

		self.suck_delay = 0.05
		self.blow_delay = 0.05

	def execute(self):
		self.state = GamlState.Executing
		print(str(self.module)+" running AnpGantry action...")

		high_pos = self.pos_data_dict["anp_safety_move_position"]
		low_pos = self.pos_data_dict["anp_device_suction_position"]

		move_and_check(self.axis_list, self.module.motor_dict, low_pos, self.speed, self.accel, self.decel)
		
		command = ""
		# execute some suction io
		if self.move_type == "suck":
			command = "suck_device_from_tray"
		elif self.move_type == "release":
			command = "release_device_to_tray"

		move_and_check(self.axis_list, self.module.motor_dict, high_pos, self.speed, self.accel, self.decel)
		
		indicator, msg = call_main_handler(self.module.name, command, json.dumps({"position": self.module.position}))
		if not indicator:
			while not rospy.is_shutdown():
				pass
		self.state = GamlState.Finished
		self.finish_signal.set()

	def IO_control(self, io_name, status):
		master = int(self.io_data_dict[io_name]["master"])
		pin = int(self.io_data_dict[io_name]["pin"])
		self.module.io_dict[master].io_set(pin, status)

class RotateAction(GamlAction):
	"""
	Represent the rotation action of the
	Gaml script. 
	"""
	def __init__(self, module, **configs):
		super(RotateAction, self).__init__(module, **configs)

	def setup(self, **params): 
		self.pos_data_dict = params["position_mapping"][self.module.name]
		self.io_data_dict = params["io_mapping"]
		self.axis_list = ["x", "y", "z"]

	def execute(self):
		self.state = GamlState.Executing
		print(str(self.module)+" running rotation action...")

		# == Implement what you want to do here ==
		# e.g. control motor, control io
		pos = self.pos_data_dict["point1"] # pos = [-99,-99,-99]

		# move motor and get the result (non-blocking)
		result = move_and_check(self.axis_list, self.module.motor_dict, pos, self.speed, self.accel, self.decel, block=False)

		self.IO_control("anp1_lock_tray", 1)
		time.sleep(5)
		io_value = self.get_digital_IO("anp2_tray_sensor")
		self.IO_control("anp1_lock_tray", 0)

		# == End == 

		self.state = GamlState.Finished
		self.finish_signal.set()

	def IO_control(self, io_name, status):
		master = int(self.io_data_dict[io_name]["master"])
		pin = int(self.io_data_dict[io_name]["pin"])
		self.module.io_dict[master].io_set(pin, status)

	def get_digital_IO(self, io_name):
		master = int(self.io_data_dict[io_name]["master"])
		pin = int(self.io_data_dict[io_name]["pin"])
		read_data = self.module.io_dict[master].io_monitor()["read_data"]
		print("[get_digital_IO] name: {0}, value:{1}".format(io_name, int(read_data[pin:pin+1])))
		return int(read_data[pin:pin+1])


class IOAction(GamlAction):
	"""
	Represent the IO action of the
	Gaml script. 
	"""
	def __init__(self, module, **configs):
		super(IOAction, self).__init__(module, **configs)

	def setup(self, **params):
		self.io_data_dict = params["io_mapping"]
		self.move_type = params["type"]

	def execute(self):
		self.state = GamlState.Executing
		print(str(self.module)+" running ioAction action...")
		if self.move_type == "open_traylift_latch":
			self.open_traylift_latch()
		elif self.move_type == "close_traylift_latch":
			self.close_tray_latch()
		elif self.move_type == "SortingIO02_exist":
			self.get_digital_IO("SortingIO02_exist_tray_sensor_active")
		elif self.move_type == "suckDeviceFromTray":
			self.suckDeviceFromTray()
		elif self.move_type == "releaseDeviceToTray":
			self.releaseDeviceToTray()
		elif self.move_type == "clawVacuumSuctionOn":
			self.clawVacuumSuctionOn()
		elif self.move_type == "clawVacuumSuctionOff":
			self.clawVacuumSuctionOff()
		elif self.move_type == "airFloatationOn":
			self.clawVacuumSuctionOff()
		elif self.move_type == "airFloatationOff":
			self.clawVacuumSuctionOff()
		elif self.move_type == "sortingSuckOn":
			self.sortingSuckOn()
		elif self.move_type == "sortingSuckOff":
			self.sortingSuckOff()
		elif self.move_type == "sortingBlowOn":
			self.sortingBlowOn()
		elif self.move_type == "sortingBlowOff":
			self.sortingBlowOff()
		self.state = GamlState.Finished
		self.finish_signal.set()
		

	def IO_control(self, io_name, status):
		master = int(self.io_data_dict[io_name]["master"])
		pin = int(self.io_data_dict[io_name]["pin"])
		self.module.io_dict[master].io_set(pin, status)

	def get_digital_IO(self, io_name):
		master = int(self.io_data_dict[io_name]["master"])
		pin = int(self.io_data_dict[io_name]["pin"])
		read_data = self.module.io_dict[master].io_monitor()["read_data"]
		print("[get_digital_IO] name: {0}, value:{1}".format(io_name, int(read_data[pin:pin+1])))
		return int(read_data[pin:pin+1])

	def open_traylift_latch(self):
		self.IO_control("tray_click_push_dual1_on", 0)
		self.IO_control("tray_click_push_dual2_on", 0)
		self.IO_control("tray_click_push_dual1_off", 0)
		self.IO_control("tray_click_push_dual2_off", 0)
		self.IO_control("tray_click_push_dual1_on", 1)
		self.IO_control("tray_click_push_dual2_on", 1)
	
	def close_traylift_latch(self):
		self.IO_control("tray_click_push_dual1_on", 0)
		self.IO_control("tray_click_push_dual2_on", 0)
		self.IO_control("tray_click_push_dual1_off", 0)
		self.IO_control("tray_click_push_dual2_off", 0)
		self.IO_control("tray_click_push_dual1_off", 1)
		self.IO_control("tray_click_push_dual2_off", 1)
	
	def suckDeviceFromTray(self):
		self.IO_control("Air_valve_u202", 1)
	
	def releaseDeviceToTray(self):
		self.IO_control("Air_valve_u202", 0)

	def clawVacuumSuctionOn(self):
		self.IO_control("Claw_vacuum_suction", 1)

	def clawVacuumSuctionOff(self):
		self.IO_control("Claw_vacuum_suction",0)

	def airFloatationOn(self):
		self.IO_control("Air_floatation",1)

	def airFloatationOff(self):
		self.IO_control("Air_floatation",0)

	def sortingSuckOn(self):
		self.IO_control("Sorting_suck", 1)

	def sortingSuckOff(self):
		self.IO_control("Sorting_suck", 0)

	def sortingBlowOn(self):
		self.IO_control("Sorting_blow", 1)

	def sortingBlowOff(self):
		self.IO_control("Sorting_blow", 0)

class StopAction(GamlAction):
	"""
	Represent the Stop action of the
	Gaml script. 
	"""
	def __init__(self, module, **configs):
		super(StopAction, self).__init__(module, **configs)

	def setup(self, **params): 
		pass

	def execute(self):
		self.state = GamlState.Executing
		print(str(self.module)+" stop action...")

		call_main_handler(self.module.name, "pause", "")
		time.sleep(0.5)

		resume = False
		while not resume and not rospy.is_shutdown():
			resume, msg = call_main_handler(self.module.name, "wait_resume", "")
			time.sleep(0.5)

		self.state = GamlState.Finished
		self.finish_signal.set()
