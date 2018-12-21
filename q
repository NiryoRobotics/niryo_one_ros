[1mdiff --git a/niryo_one_bringup/launch/gazebo_simulation.launch b/niryo_one_bringup/launch/gazebo_simulation.launch[m
[1mindex 371e848..452cca7 100644[m
[1m--- a/niryo_one_bringup/launch/gazebo_simulation.launch[m
[1m+++ b/niryo_one_bringup/launch/gazebo_simulation.launch[m
[36m@@ -1,15 +1,18 @@[m
 <launch>[m
[32m+[m
   <arg name="execute_on_niryo_one_raspberry_pi_image" value="false"/>[m
   <arg name="urdf_without_meshes" default="false" />[m
[31m-   <include file="$(find niryo_one_bringup)/launch/niryo_one_base.launch" pass_all_args="true"/>[m
[32m+[m[32m  <include file="$(find niryo_one_bringup)/launch/niryo_one_base.launch" pass_all_args="true"/>[m
[32m+[m[41m  [m
   <!-- these are the arguments you can pass this launch file, for example paused:=true -->[m
   <arg name="paused" default="false"/>[m
   <arg name="use_sim_time" default="true"/>[m
   <arg name="gui" default="true"/>[m
   <arg name="headless" default="false"/>[m
   <arg name="debug" default="false"/>[m
[31m-  <arg name="robot_name" default="conveyor"/>[m
[32m+[m[32m  <arg name="robot_name" default="niryo_one"/>[m
   <!-- We resume the logic in empty_world.launch, changing only the name of the world to be launched -->[m
[32m+[m[41m  [m
   <include file="$(find gazebo_ros)/launch/empty_world.launch">[m
     <arg name="world_name" value="$(find niryo_one_gazebo)/worlds/niryo_one.world"/>[m
     <arg name="verbose" value="true" />[m
[36m@@ -19,13 +22,23 @@[m
     <arg name="use_sim_time" value="$(arg use_sim_time)"/>[m
     <arg name="headless" value="$(arg headless)"/>   [m
   </include>[m
[32m+[m[41m  [m
   <!-- push robot_description to factory and spawn robot in gazebo -->[m
   <node name="niryo_one_spawn" pkg="gazebo_ros" type="spawn_model" output="screen" args="-urdf -param robot_description -model niryo_one" respawn="false" />	[m
[32m+[m[41m [m
   <include file="$(find niryo_one_bringup)/launch/controllers.launch">[m
     <arg name="simulation_mode" value="true" />[m
   </include>[m
[32m+[m[41m [m
   <include file="$(find niryo_one_bringup)/launch/robot_commander.launch">[m
     <arg name="simulation_mode" value="true" />[m
   </include>[m
[32m+[m[41m [m
[32m+[m[32m  <include file="$(find niryo_one_bringup)/launch/user_interface.launch">[m
[32m+[m[32m      <arg name="simulation_mode" value="true" />[m
[32m+[m[32m  </include>[m
[32m+[m[41m [m
[32m+[m[32m  <node name="tf2_web_republisher" pkg="tf2_web_republisher" type="tf2_web_republisher"/>[m
   <include file="$(find rosbridge_server)/launch/rosbridge_websocket.launch" />[m
[32m+[m
 </launch>[m
\ No newline at end of file[m
[1mdiff --git a/niryo_one_description/config/default_config.rviz b/niryo_one_description/config/default_config.rviz[m
[1mindex 64b7111..104e3ff 100644[m
[1m--- a/niryo_one_description/config/default_config.rviz[m
[1m+++ b/niryo_one_description/config/default_config.rviz[m
[36m@@ -6,9 +6,8 @@[m [mPanels:[m
       Expanded:[m
         - /Global Options1[m
         - /Status1[m
[31m-        - /RobotModel1[m
       Splitter Ratio: 0.5[m
[31m-    Tree Height: 413[m
[32m+[m[32m    Tree Height: 565[m
   - Class: rviz/Selection[m
     Name: Selection[m
   - Class: rviz/Tool Properties[m
[36m@@ -125,7 +124,6 @@[m [mVisualization Manager:[m
   Enabled: true[m
   Global Options:[m
     Background Color: 48; 48; 48[m
[31m-    Default Light: true[m
     Fixed Frame: base_link[m
     Frame Rate: 30[m
   Name: root[m
[36m@@ -147,7 +145,7 @@[m [mVisualization Manager:[m
   Views:[m
     Current:[m
       Class: rviz/Orbit[m
[31m-      Distance: 0.447220176[m
[32m+[m[32m      Distance: 1.01129115[m
       Enable Stereo Rendering:[m
         Stereo Eye Separation: 0.0599999987[m
         Stereo Focal Distance: 1[m
[36m@@ -162,18 +160,18 @@[m [mVisualization Manager:[m
       Invert Z Axis: false[m
       Name: Current View[m
       Near Clip Distance: 0.00999999978[m
[31m-      Pitch: -0.119601592[m
[32m+[m[32m      Pitch: 0.325398505[m
       Target Frame: <Fixed Frame>[m
       Value: Orbit (rviz)[m
[31m-      Yaw: 5.34860659[m
[32m+[m[32m      Yaw: 5.19358969[m
     Saved: ~[m
 Window Geometry:[m
   Displays:[m
     collapsed: false[m
[31m-  Height: 719[m
[32m+[m[32m  Height: 846[m
   Hide Left Dock: false[m
   Hide Right Dock: false[m
[31m-  QMainWindow State: 000000ff00000000fd00000004000000000000016a0000022cfc0200000008fb0000001200530065006c0065006300740069006f006e00000001e10000009b0000006100fffffffb0000001e0054006f006f006c002000500072006f007000650072007400690065007302000001ed000001df00000185000000a3fb000000120056006900650077007300200054006f006f02000001df000002110000018500000122fb000000200054006f006f006c002000500072006f0070006500720074006900650073003203000002880000011d000002210000017afb000000100044006900730070006c00610079007301000000410000022c000000d700fffffffb0000002000730065006c0065006300740069006f006e00200062007500660066006500720200000138000000aa0000023a00000294fb00000014005700690064006500530074006500720065006f02000000e6000000d2000003ee0000030bfb0000000c004b0069006e0065006300740200000186000001060000030c00000261000000010000010f0000022cfc0200000003fb0000001e0054006f006f006c002000500072006f00700065007200740069006500730100000041000000780000000000000000fb0000000a0056006900650077007301000000410000022c000000ad00fffffffb0000001200530065006c0065006300740069006f006e010000025a000000b200000000000000000000000200000490000000a9fc0100000001fb0000000a00560069006500770073030000004e00000080000002e10000019700000003000005560000003efc0100000002fb0000000800540069006d00650100000000000005560000030000fffffffb0000000800540069006d00650100000000000004500000000000000000000002d10000022c00000004000000040000000800000008fc0000000100000002000000010000000a0054006f006f006c00730100000000ffffffff0000000000000000[m
[32m+[m[32m  QMainWindow State: 000000ff00000000fd00000004000000000000016a000002c4fc0200000008fb0000001200530065006c0065006300740069006f006e00000001e10000009b0000006400fffffffb0000001e0054006f006f006c002000500072006f007000650072007400690065007302000001ed000001df00000185000000a3fb000000120056006900650077007300200054006f006f02000001df000002110000018500000122fb000000200054006f006f006c002000500072006f0070006500720074006900650073003203000002880000011d000002210000017afb000000100044006900730070006c0061007900730100000028000002c4000000dd00fffffffb0000002000730065006c0065006300740069006f006e00200062007500660066006500720200000138000000aa0000023a00000294fb00000014005700690064006500530074006500720065006f02000000e6000000d2000003ee0000030bfb0000000c004b0069006e0065006300740200000186000001060000030c00000261000000010000010f000002c4fc0200000003fb0000001e0054006f006f006c002000500072006f00700065007200740069006500730100000041000000780000000000000000fb0000000a005600690065007700730100000028000002c4000000b000fffffffb0000001200530065006c0065006300740069006f006e010000025a000000b200000000000000000000000200000490000000a9fc0100000001fb0000000a00560069006500770073030000004e00000080000002e10000019700000003000005990000003efc0100000002fb0000000800540069006d00650100000000000005990000030000fffffffb0000000800540069006d0065010000000000000450000000000000000000000314000002c400000004000000040000000800000008fc0000000100000002000000010000000a0054006f006f006c00730100000000ffffffff0000000000000000[m
   Selection:[m
     collapsed: false[m
   Time:[m
[36m@@ -182,6 +180,6 @@[m [mWindow Geometry:[m
     collapsed: false[m
   Views:[m
     collapsed: false[m
[31m-  Width: 1366[m
[31m-  X: 0[m
[31m-  Y: 27[m
[32m+[m[32m  Width: 1433[m
[32m+[m[32m  X: 104[m
[32m+[m[32m  Y: 61[m
