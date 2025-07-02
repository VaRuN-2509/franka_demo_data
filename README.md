# Franka_Data
This repo contains the required data needed for training diffusion policy on the real franka hardware.
Step by Step procedure to collect Data from Human demonstration for the PushT task.

# Diffusion Policy

[[Project page]](https://diffusion-policy.cs.columbia.edu/)
[[Paper]](https://diffusion-policy.cs.columbia.edu/#paper)
[[Data]](https://diffusion-policy.cs.columbia.edu/data/)
[[Colab (state)]](https://colab.research.google.com/drive/1gxdkgRVfM55zihY9TFLja97cSVZOZq2B?usp=sharing)
[[Colab (vision)]](https://colab.research.google.com/drive/18GIHeOQ5DyjMN8iIRZL2EKZ0745NLIpg?usp=sharing)


[Cheng Chi](http://cheng-chi.github.io/)<sup>1</sup>,
[Siyuan Feng](https://www.cs.cmu.edu/~sfeng/)<sup>2</sup>,
[Yilun Du](https://yilundu.github.io/)<sup>3</sup>,
[Zhenjia Xu](https://www.zhenjiaxu.com/)<sup>1</sup>,
[Eric Cousineau](https://www.eacousineau.com/)<sup>2</sup>,
[Benjamin Burchfiel](http://www.benburchfiel.com/)<sup>2</sup>,
[Shuran Song](https://www.cs.columbia.edu/~shurans/)<sup>1</sup>


## Camera access and functioning:-
1. we use 3 Brio cameras for this purpose
2. use **cv2_enemurate libraryy** to loop through camera_index
3. refer to the file - [human_demo_video](https://github.com/VaRuN-2509/Franka_Data/blob/main/human_demo_video.py)
   

#**2.Topic to gather data from**
1. run the ros2 launch file - franka.bringup to start the hardware
2. ros2 topic list :
  * /dynamic_joint_states
  * /franka_robot_state_broadcaster/current_pose
  * /franka_robot_state_broadcaster/desired_end_effector_twist
  * /franka_robot_state_broadcaster/desired_joint_states
  * /franka_robot_state_broadcaster/external_joint_torques
  * /franka_robot_state_broadcaster/external_wrench_in_base_frame
  * /franka_robot_state_broadcaster/external_wrench_in_stiffness_frame
  * /franka_robot_state_broadcaster/last_desired_pose
  * /franka_robot_state_broadcaster/measured_joint_states
  * /franka_robot_state_broadcaster/robot_state
  * /franka_robot_state_broadcaster/transition_event
  * /joint_state_broadcaster/transition_event
  * /joint_states
  * /parameter_events
  * /robot_description
  * /rosout
  * /tf
  * /tf_static
    
3.subscribe to :
* /franka_robot_state_broadcaster/current_pose
* /franka_robot_state_broadcaster/measured_joint_states
* /franka_robot_state_broadcaster/desired_end_effector_twist

4.refer to the file -> [human_demo_pose](https://github.com/VaRuN-2509/Franka_Data/blob/main/human_demo_pose.py)


## File structure of collected data
We collected the data in zarr file format
 ```
 . 
 ├── data
 │   ├── eef_pose (1384, 7) float32
 │   ├── eef_velocity (1384, 6) float32
 │   ├── joint_pose (1384, 7) float32
 │   ├── joint_velocity (1384, 7) float32
 │   └── timestamp (1384,) float64
 └── meta
     └── episode_ends (14,) float32
```


