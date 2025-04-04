import math
import rtde_control
import rtde_receive
import rtde_io


class MyRobot:

	def __init__(self) -> None:
		
		self._ROBOT_CON_	= None
		self._ROBOT_RECV_	= None
		self._ROBOT_IO_		= None

	def my_init(self, host:str) -> None:

		self._ROBOT_CON_	= rtde_control.RTDEControlInterface(host)
		self._ROBOT_RECV_	= rtde_receive.RTDEReceiveInterface(host)
		self._ROBOT_IO_		= rtde_io.RTDEIOInterface(host)

	def my_release(self) -> None:

		if self._ROBOT_CON_ is not  None:

			self._ROBOT_CON_.stopScript()
			self._ROBOT_CON_.disconnect()
		
		if self._ROBOT_RECV_ is not None:
			self._ROBOT_RECV_.disconnect()
		
		if self._ROBOT_IO_ is not None:
			self._ROBOT_IO_.disconnect()
		
	def my_get_joint(self) -> []:

		res	= self._ROBOT_RECV_.getActualQ()
		joint_deg	= []
		for rad in res:
			joint_deg.append(math.degrees(rad))

		return	joint_deg
	
	def my_get_position(self) -> any:

		res	= self._ROBOT_RECV_.getActualTCPPose()
		return	res

	def my_move_j(self, joint_degree=[0] * 6, speed=0.01, acceleration=0.05, asynchronous=False) -> None:

		joint_rad	= []
		for deg in joint_degree:
			joint_rad.append(math.radians(deg))

		self._ROBOT_CON_.moveJ(q=joint_rad, speed=speed, acceleration=acceleration, asynchronous=asynchronous)

	def my_move_j_ik(self, position, speed=0.01, acceleration=0.05, asynchronous=False) -> None:

		self._ROBOT_CON_.moveJ_IK()

	def my_move_j_stop(self, a = 2.0, asynchronous=False) -> None:

		self._ROBOT_CON_.stopJ(a, asynchronous)

	def my_move_speed(self, velocity) -> None:

		self._ROBOT_CON_.speedL(xd=velocity, acceleration=0.1, time=0)

	def my_move_speed_stop(self, acceleration=0.1) -> None:

		self._ROBOT_CON_.speedStop(a=acceleration)

	def my_is_joint_move(self) -> bool:
		
		res	= self._ROBOT_RECV_.getActualQd()
		print(f"getActualQd={res}")

		vel_max	= max(res)
		print(f"vel_max={vel_max}")

		return vel_max > 0.0001 or vel_max < -0.0001

	def my_io_digital_set(self, id:int, signal:bool):

		res	= self._ROBOT_IO_.setStandardDigitalOut(id, signal)

		return	res
