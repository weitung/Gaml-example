import Gaml
from gadgethiServerUtils.file_basics import read_config_yaml

configs = {}
all_actuators = read_config_yaml("/Users/weitung/Dropbox (MIT)/gadgetHitech/008-AutoHandler/software/Gaml/configs/actuators_mapping.yaml")
configs.update({"io_mapping": all_actuators["IO"]})
configs.update({"position_mapping": read_config_yaml("/Users/weitung/Dropbox (MIT)/gadgetHitech/008-AutoHandler/software/Gaml/configs/test_position.yaml")})
configs.update({"actuator_mapping": all_actuators["Motor"]})
gaml = Gaml.Gaml(gaml_path='/Users/weitung/Dropbox (MIT)/gadgetHitech/008-AutoHandler/software/Gaml/examples/test_suite/'+ 'test_f1.gaml',
	production=True, module_list=["TrayLift", "AnpGantry", "SortingGantry"], **configs)

gaml.start()