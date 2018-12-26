# Niryo One Gazebo

This packages provides the world and models description for gazebo simulation.

## Save camera images in Gazebo 

Gazebo can save camera images into your laptop by adding camera to your world. To do ,Uncomment commented lines in the worlds/niryo_one_world file. 
 In this part : 

```
<sensor name='my_camera' type='camera'>
          <camera>
            <save enabled="true">
              <path>/home/my_camera_save</path>  
            </save> 
            <horizontal_fov>1.047</horizontal_fov>
            <image>
              <width>1920</width>
              <height>1080</height>
            </image>
            <clip>
              <near>0.1</near>
              <far>100</far>
            </clip>
          </camera>
          <always_on>1</always_on>
          <update_rate>30</update_rate>
        </sensor> 
```
you can change the child tag **path** to the directory in which you want the camera images be saved. If the directory does not exist, gazebo will try to create it. 

**width** and **height** set the resolution of the images : This camera will output images with 1920x1080 resolution at 30 frames per second. 

After runnig gazebo simulation launch file, check the saved images. You can conevert those images to a video with [ffmpeg](https://ffmpeg.org/ffmpeg.html) or others converters.

## Add a conveyor Model in Gazebo 

 You can load thers URDF models into your simulation. You can find a some urdf models [here](https://github.com/NiryoRobotics/niryo_one_ros/tree/gazebo-simulator/niryo_one_description/urdf/gazebo) 

Copy the following lines into [gazebo_simulation.launch](https://github.com/NiryoRobotics/niryo_one_ros/tree/gazebo-simulator/niryo_one_bringup/launch/gazebo_simulation.launch)

```
 <node name="niryo_one_conv_spawn" pkg="gazebo_ros" type="spawn_model" output="screen" args="-urdf -param conveyor_description -model conv" respawn="false" />	
   <arg name="conveyor_urdf_file" default="$(find xacro)/xacro --inorder '$(find niryo_one_description)/urdf/gazebo/conveyor.urdf.xacro'" />
   <param name="conveyor_description" command="$(arg conveyor_urdf_file)" />
```

You can find more GAzebo Tutorials in [here](http://gazebosim.org/tutorials)  