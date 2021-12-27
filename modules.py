import time
from actions import *
from Gaml._gamlmodule import GamlModule
from Gaml._gamlprimitives import GamlId

class AnpShuttle(GamlModule):
	"""
	Represent the AnpShuttle module of the
	Gaml script. 
	"""
	def __init__(self, action_seq=[], **configs):
		super(AnpShuttle, self).__init__(action_seq=action_seq, **configs)

	def delay(self):
		return DelayAction(self.__class__)

	def setup(self, **configs):
		pass

	def run(self):
		print("AnpShuttle start action sequence...")
		self.execute_sequence()

class RU(GamlModule):
	"""
	Represent the RU module of the
	Gaml script. 
	"""
	def __init__(self, action_seq=[], **configs):
		super(RU, self).__init__(action_seq=action_seq, **configs)

	def delay(self):
		return DelayAction(self.__class__)

	def rotate(self):
		return RotateAction(self.__class__)

	def setup(self, **configs):
		pass

	def run(self):
		print("RU start action sequence...")
		self.execute_sequence()

class TrayLift(GamlModule):
	"""
	Represent the TrayLift module of the
	Gaml script. 
	"""
	def __init__(self, action_seq=[], **configs):
		super(TrayLift, self).__init__(action_seq=action_seq, **configs)
		
	def moveSortTrays(self):
		return Move(self.__class__, position="Tray2", **TrayLift.configs)

	def handleFullTray(self):
		return DelayAction(self.__class__) # TODO

	def setup(self, **configs):
		pass

	def run(self):
		print("TrayLift start action sequence...")
		self.execute_sequence()

	def delay(self):
		return DelayAction(self.__class__)

	# The following functions are for testing only
	def delay_test(self, *args):
		for i in range(len(args)):
			print(i, ": ", args[i])
		return DelayAction(self.__class__)

class AnpGantry(GamlModule):
	"""
	Represent the AnpGantry module of the
	Gaml script. 
	"""
	def __init__(self, action_seq=[], **configs):
		self._start_idx = 0
		super(AnpGantry, self).__init__(action_seq=action_seq, **configs)

	def check_all_device_transferred(self):
		return IOAction(self.__class__, io_id="device_suction", status="on", **AnpGantry.configs)

	def setup(self, **configs):
		pass

	def run(self):
		print("AnpGantry start action sequence...")
		self.execute_sequence()

	def delay(self):
		return DelayAction(self.__class__, delay_time=3)

	def stop(self):
		return StopAction(self.__class__)

	def suckDeviceFromTray(self):
		return AnpGantryAction(self.__class__, type="suck", **AnpGantry.configs)

	def releaseDeviceToTray(self):
		return AnpGantryAction(self.__class__, type="release", **AnpGantry.configs)

	def homeY(self):
		return HomeAction(self.__class__,wait_time=10,home_method=17,axis="y",**AnpGantry.configs)

	def moveToShuttleSuction(self):
		return AnpZMoveAction(self.__class__,type="anp_shuttle_suction_position",**AnpGantry.configs)

	def moveToDeviceSuction(self):
		return AnpZMoveAction(self.__class__,type="anp_device_suction_position",**AnpGantry.configs)

class AnpShuttle(GamlModule):
	"""
	Represent the AnpShuttle module of the
	Gaml script. 
	"""
	def __init__(self, action_seq=[], **configs):
		super(AnpShuttle, self).__init__(action_seq=action_seq, **configs)

	def setup(self, **configs):
		pass

	def run(self):
		print("AnpShuttle start action sequence...")
		self.execute_sequence()
		
	def delay(self):
		return DelayAction(self.__class__)

class SortingGantry(GamlModule):
	"""
	Represent the SortingGantry module of the
	Gaml script. 
	"""
	def __init__(self, action_seq=[], **configs):
		super(SortingGantry, self).__init__(action_seq=action_seq, **configs)

	def performSort(self):
		return SortingAction(self.__class__, **SortingGantry.configs)

	def performSortAgain(self):
		return SortingAction(self.__class__, **SortingGantry.configs)

	def stop(self):
		return StopAction(self.__class__)

	def setup(self, **configs):
		pass

	def run(self):
		print("SortingGantry start action sequence...")
		self.execute_sequence()

	def delay(self):
		return DelayAction(self.__class__)

class RotationUnit(GamlModule):
	"""
	Represent the RotationUnit module of the
	Gaml script. 
	"""
	def __init__(self, action_seq=[], **configs):
		super(RotationUnit, self).__init__(action_seq=action_seq, **configs)

	def setup(self, **configs):
		pass

	def run(self):
		print("RotationUnit start action sequence...")
		self.execute_sequence()

class AnpGantryBlock(GamlModule):
	"""
	This is the class to wait until anp gantry has an empty position.
	"""
	def __init__(self, action_seq=[], **configs):
		super(AnpGantryBlock, self).__init__(action_seq=action_seq, **configs)
		
	def checkAnpEmptyPos(self):
		return BlockAction(self.__class__, block_message="AnpGantryEmpty", **AnpGantryBlock.configs)

	def setup(self, **configs):
		pass

	def run(self):
		print("AnpGantryBlock start action sequence...")
		self.execute_sequence()

class TestingBlock(GamlModule):
	"""
	This is the class to wait until testing is finished.
	"""
	def __init__(self, action_seq=[], **configs):
		super(TestingBlock, self).__init__(action_seq=action_seq, **configs)
		
	def waitUntilTestingFinish(self):
		return BlockAction(self.__class__, block_message="TestingFinish", **TestingBlock.configs)

	def setup(self, **configs):
		pass

	def run(self):
		print("TestingBlock start action sequence...")
		self.execute_sequence()

class AnpTray:
	"""
	Represent the AnpTray module of the
	Gaml script. 
	"""
	def __init__(self, **configs):
		pass

	def getANP1Tray(self):
		return "AnpTray1@anp1"

	def getANP2Tray(self):
		return "AnpTray2@anp2"
	
class IOTest(GamlModule):
	def __init__(self, action_seq=[], **configs):
		super(IOTest, self).__init__(action_seq=action_seq, **configs)
		
	def open_traylift_latch(self):
		return IOAction(self.__class__, type="open_traylift_latch", **IOTest.configs)
	
	def close_traylift_latch(self):
		return IOAction(self.__class__, type="close_traylift_latch", **IOTest.configs)
	
	def SortingIO02_exist(self):
		return IOAction(self.__class__, type="SortingIO02_exist", **IOTest.configs)
    
	def SuckDeviceFromTray(self):
		return IOAction(self.__class__, type="suckDeviceFromTray", **IOTest.configs)
	
	def ReleaseDeviceToTray(self):
		return IOAction(self.__class__, type="releaseDeviceToTray", **IOTest.configs)

	def Claw_vacuum_suction_on(self):
		return IOAction(self.__class__, type="clawVacuumSuctionOn", **IOTest.configs)

	def Claw_vacuum_suction_off(self):
		return IOAction(self.__class__, type="clawVacuumSuctionOff", **IOTest.configs)

	def Air_floatation_on(self):
		return IOAction(self.__class__, type="airFloatationOn", **IOTest.configs)

	def Air_floatation_on(self):
		return IOAction(self.__class__, type="airFloatationOff", **IOTest.configs)

	def Sorting_suck_on(self):
		return IOAction(self.__class__, type="sortingSuckOn", **IOTest.configs)

	def Sorting_suck_off(self):
		return IOAction(self.__class__, type="sortingSuckOff", **IOTest.configs)

	def Sorting_blow_on(self):
		return IOAction(self.__class__, type="sortingBlowOn", **IOTest.configs)

	def Sorting_blow_on(self):
		return IOAction(self.__class__, type="sortingBlowOff", **IOTest.configs)

	def Anp_vibration_on(self):
		return IOAction(self.__class__, type="anp_vibration_on", **IOTest.configs)

	def Anp_vibration_off(self):
		return IOAction(self.__class__, type="anp_vibration_off", **IOTest.configs)

	def Anp_Cylinder_on(self):
		return IOAction(self.__class__, type="anp_Cylinder_on", **IOTest.configs)

	def Anp_Cylinder_off(self):
		return IOAction(self.__class__, type="anp_Cylinder_off", **IOTest.configs)

	def Anp_Electromagnet_on(self):
		return IOAction(self.__class__, type="anp_Electromagnet_on", **IOTest.configs)

	def Anp_Electromagnet_off(self):
		return IOAction(self.__class__, type="anp_Electromagnet_off", **IOTest.configs)

	def Cs_Vacuumsuction1_on(self):
		return IOAction(self.__class__, type="cs_Vacuumsuction1_on", **IOTest.configs)

	def Cs_Vacuumsuction1_off(self):
		return IOAction(self.__class__, type="cs_Vacuumsuction1_off", **IOTest.configs)

	def Cs_Vacuumsuction2_on(self):
		return IOAction(self.__class__, type="cs_Vacuumsuction2_on", **IOTest.configs)

	def Cs_Vacuumsuction2_off(self):
		return IOAction(self.__class__, type="cs_Vacuumsuction2_off", **IOTest.configs)

	def Cs_carrier_forward_or_backward_on(self):
		return IOAction(self.__class__, type="cs_carrier_forward_or_backward_on", **IOTest.configs)

	def Cs_carrier_forward_or_backward_off(self):
		return IOAction(self.__class__, type="cs_carrier_forward_or_backward_off", **IOTest.configs)

	def Anp_air_valve_u201_on(self):
		return IOAction(self.__class__, type="anp_air_valve_u201_on", **IOTest.configs)

	def Anp_air_valve_u201_off(self):
		return IOAction(self.__class__, type="anp_air_valve_u201_off", **IOTest.configs)

	def delay(self):
		return DelayAction(self.__class__)

	def stop(self):
		return StopAction(self.__class__)
	
	def setup(self, **configs):
		pass
	
	def run(self):
		print("Plunger IO action sequence...")
		self.execute_sequence()

class CarrierShuttle(GamlModule):
	"""
	Represent the CarrierShuttle module of the
	Gaml script. 
	"""
	def __init__(self, action_seq=[], **configs):
		super(CarrierShuttle, self).__init__(action_seq=action_seq, **configs)

	def delay(self):
		return DelayAction(self.__class__)

	def setup(self, **configs):
		pass

	def run(self):
		print("CarrierShuttle start action sequence...")
		self.execute_sequence()

class Plunger(GamlModule):
	def __init__(self, action_seq=[], **configs):
		self._start_idx = 0
		super(Plunger, self).__init__(action_seq=action_seq, **configs)

	def setup(self, **configs):
		pass

	def MoveTrayToTest(self):
		return PlungerAction(self.__class__, **Plunger.configs)

	def delay(self):
		return DelayAction(self.__class__)

	def run(self):
		print("Plunger start action sequence...")
		self.execute_sequence()
class UTH:
	"""
	Represent the UTH module of the
	Gaml script. 
	"""
	def __init__(self, **configs):
		pass

	def getSteadyLoopCnt(self):
		return 3

	def getUntestedTray(self):
		return GamlId("UntestedTray")
		
	def getEmptyTray(self):
		return GamlId("EmptyTray")

	def getFinishTestingTray(self):
		return GamlId("FinishTestingTray")

	def getFinishSortingTray(self):
		return GamlId("FinishSortingTray")
		
	def getAnpEmptyPos(self):
		return GamlId("ANPEmptyPos")

	def getSortingEmptyPos(self):
		return GamlId("SortingEmptyPos")

	def getSortedTrayTrgtPos(self):
		return GamlId("SortedTrayTrgtPos")

	def getAnpUntestedPos(self):
		return GamlId("AnpUntestedPos")

	def getAnpTestedPos(self):
		return GamlId("AnpTestedPos")

	def TestTrayonAnp1(self):
		return GamlId("TestTray@anp1")

	def getBin2Pos(self):
		return GamlId("Bin2Pos")

	def getBin3Pos(self):
		return GamlId("Bin3Pos")

	def getBin4Pos(self):
		return GamlId("Bin4Pos")
