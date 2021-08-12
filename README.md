# qt_rviz_panel

## Instructions

### 1) Adding the required files into the `rmf_visualization_rviz2_plugin/src` folder found in the `rmf` folder.

The following files have to be copied into the rmf workspace
1. ParseGraph.cpp
2. ParseGraph.hpp
3. RmfPanel.cpp
4. RmfPanel.hpp

### 2) Modify `StandardNames.hpp` found in `rmf_visualization_rviz2_plugin/src` folder, to cater to the newly added files.

Add the following lines into the `StandardNames.hpp`

```
const std::string LoopRequestServiceName = "submit_task";
const std::string FleetStateTopicName = "fleet_states";
```

### 3) Modify `rmf_visualization_rviz2_plugin`'s `CMakeLists` to include the new files.

Add the following lines into the CMakeLists under `add_library` section, line 38
```
src/RmfPanel.cpp
src/ParseGraph.cpp
```
Additionally,
1. add `rmf_task_msgs` & `rmf_fleet_msgs` is a required package at line 35.
2. add `${rmf_task_msgs_LIBRARIES}` & `${rmf_fleet_msgs_LIBRARIES}`, under `target_link_libraries`
3. add `${rmf_task_msgs_INCLUDE_DIRS}` & `${rmf_fleet_msgs_INCLUDE_DIRS}`, under `target_include_directories`

### 4) Modify `plugin_description.xml`, found in `rmf_visualization_rviz2_plugin` folder
Add the following lines
```
<class name="rmf_visualization_rviz2_plugins/RmfPanel"
    type="rmf_visualization_rviz2_plugins::RmfPanel"
    base_class_type="rviz_common::Panel">
  <description>
    A panel widget allowing users to send amr task requests and visualize states.
  </description>
</class>
```

### 5) Ensure that `rmf_visualization_rviz2_plugin` `package.xml` has `rmf_task_msgs` & `rmf_fleet_msgs`as a dependent

### 6) Modify your .rviz file of your deployment
	a) Under `Panel` section , add the following lines 
	```
	- Class: rmf_visualization_rviz2_plugins/RmfPanel
    	Name: RmfPanel
    	```
	b) Under 'Window Geometry', add the following lines
	```
	RmfPanel:
		collapsed: false
	```

