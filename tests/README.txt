 ========================================================================================
|											 |
|         BRIEF DESCRIPTION OF THE TEST PROGRAMS					 |
|  	  --------------------------------------					 |
|           Oscar Efrain Ramos Ponce                                                     |
|           Programa de Pos Graduacao em Engenharia Eletrica                             |
|           Universidade Federal do Rio Grande do Sul, Brasil 2011                       |
									 		 |
 ========================================================================================


These programs have been created for testing different parts of the program and are not a 
final or complete version.
Each of the following programs contains a main function. That is, they are mutually exclusive 
and only one can exist at a time in the MakeFile.


- rob01ShowModel.cpp:
	This program only shows the robot model with all the angles
        fixed to 0 degrees (canonical position). There is no motion.

- rob02TestForwardKinematics:
	This program tests the forward kinematics. 
	The angle for each joint can be changed using the keyboard. 
	It displays the target joint angles (entered by the keyboard), the current
	angles (due to the P control law) and the pose (position and orientation) 
        of the foot wrt the body {C}. 

- rob03TestInverseKinematics:
	This program tests the inverse kinematics. 
	The position and orientation (x,y,z,roll,pitch,yaw) of each foot wrt the 
	body {C} can be changed using the keyboard.
        It displays:
            - Pose: the desired pose (entered by keyboard) [m] and [deg]
            - Pose: the obtained pose (using inverse kinematics)
            - Pose: the current pose (obtained with the control).
            - Angle: the angles obtained with the inverse kinematics
            - Angle: the current value of the angles of the robot
        
- rob04TestSimpleTrajectory:
	This program generates a simple trajectory for the leg (wrt the origin 
	of the system {C}):
          It moves one leg linearly in x and z (there is no equilibrium
          and it falls down)

- rob05TestTrajectoryHips:
	This program moves the hips (and the floating leg) specifying a
        pre-defined trajectory with respect to the Fixed Leg

- rob06SimpleInitialStep:
	This program computes and displays the very initial step of the robot using the 
	right leg as the moving leg (the fixed lef is the left one)

- robSimpleSteps:
	This program computes the trajectory of the feet (and hips) for 4 simple steps
	that are consecutively executed.



