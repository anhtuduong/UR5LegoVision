1) clone ros_impedance_controller into **dls_ws** 

2) git submodule update --init --recursive

3) hyqmake

4) roslaunch ros_impedance_controller ros_impedance_controller.launch

by default the controller runs at 1 khz, you can specify custom frequencies setting task_period:=XX

by default the PD control is set, the gains are in the config/ros_impedance_controller_pd.yaml

only_torque allows to close the loop directly in torque (PD gains are zero) 

other params are:

robot_name

5) run /matlab_model_hyq/roscontroller.m to send joint position/velocity references **q_des**, **qd_des** and feed-forward torques **tau_ffwd**

6) for the python controller run python_controller/python_controller.py