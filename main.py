from Gaml import Gaml
from gadgethiServerUtils.file_basics import read_config_yaml

# configs = {}
# all_actuators = read_config_yaml("configs/actuators_mapping.yaml")
# configs.update({"io_mapping": all_actuators["IO"]})
# configs.update({"position_mapping": read_config_yaml("configs/position.yaml")})
# configs.update({"actuator_mapping": all_actuators["Motor"]})

# gaml = Gaml(grammar_path='grammars/gaml.ebnf', 
# 		gaml_path='examples/tray_handling.gaml', 
# 		module_list=["TrayLift", "AnpGantry", "SortingGantry"], 
# 		production=True, 
# 		handler_setting_path="configs/handler_settings.yaml", **configs)

# gaml.start()


configs = {}
all_actuators = read_config_yaml("configs/actuators_mapping.yaml")
configs.update({"io_mapping": all_actuators["IO"]})
configs.update({"position_mapping": read_config_yaml("configs/test_position.yaml")})
configs.update({"actuator_mapping": all_actuators["Motor"]})
gaml = Gaml(gaml_path='examples/test_suite/'+ 'test_f1.gaml',
	production=False, module_list=["TrayLift", "AnpGantry", "SortingGantry"], **configs)

gaml.start()