## Note about these scripts
1.  `custom_robot.launch` need to replace the file under `~/linorobot2_ws/src/linorobot2/linorobot2_bringup/launch`  
    You also need to clone and build the `vdcs_receiver_node`, that project include the receiver program you need to use!  
        
    Launch the linorobot_bringup with parameter:  
    ```
    ros2 launch linorobot2_bringup bringup.launch.py custom_robot:=true
    ```