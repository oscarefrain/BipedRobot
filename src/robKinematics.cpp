/*==========================================================================
    Functions for robot kinematics
    ------------------------------
     	Oscar E. Ramos Ponce
    	Estudante do Programa de Pós Graduação em Engenharia Elétrica
    	Universidade Federal do Rio Grande do Sul, Brasil 2011
==========================================================================*/


#include "../include/robFunctions.h"
#include "../include/robModel.h"


/******************** INVERSE KINEMATICS **********************************************************/

/* Inputs: theta_target[6] : initial (current) joint angles
           target_pose[6]  : desired position and orientation of end effector (foot): x,y,z,r,p,y
           ID              : RIGHT for right leg and LEFT for left leg

   Outputs: current_pose[6]: obtained positions and orientations
            theta_target[6]: obtained angles for the joints

   Note: theta_target is both an input and an output, it is the value of the angles that is passed as
         initial value and then modified in the computation of the inverse kinematics (passed by reference)
*/

int Kinematics::rInvKinematics(double theta_target[6],double current_pose[6], double target_pose[6], bool ID)
{
   /*Example:
      double theta_target[6] = {0,1,4,0,0,0};
      double target_pose[6]  = {0,-0.486,0.05,0,0,0};
      double current_pose[6];
      rInvKinematics(theta_target,current_pose, target_pose,LEFT);
      for (int i=0; i<6; i++)
	  {   printf(" %6.4f    %6.4f  \n",theta_target[i],current_pose[i]); }
   */

    dReal LAMBDA = 0.001;               // Constant LAMBDA for angle variation
    int error_indic = 0;                // Indicates error in achieving the desired pose (1: pose not achieved, 0:otherwise)
    int n_loops = 0; 		        // Counter of number of iterations in main loop
    int ALLOWED_NUM_ITERATIONS = 25;    // Allowed (max) number of iterations in main loop
    int out = 0;                        // Forces exit of main loop (1: break the main loop, 0: do iterations in main loop)

    double error_pose[6];               // Vector for error in pose (position and orientation)
    double theta_initial[6];            // To store the very initial values of the joint angles (thetas) -> it is returned if no solution is found
    double theta[6];                    // To store the values of the joint angles (thetas) -> it will be modified during the program
    double abs_sum_error_pos, abs_sum_error_or; // Sum of absolute errors (position and orientation)
    Math::TMatrix6 lambda_eye6, J, J1;          // 6x6 Matrices for Jacobian and accesories

    bool InWorkspace;                   // To verify if the point is in the workspace
    InWorkspace = Kinematics::rCheckWorkspace(target_pose, ID);
    if (!InWorkspace)                   // If the desired pose is not in the workspace, quit the function
        return(-1);

    // Create a 6x6 diagonal matrix with lambda^2 in the diagonal
    for (int i=0; i<6; i++)
	{   for (int j=0; j<6; j++)
		{   if(i==j) lambda_eye6.a[i][j] = LAMBDA*LAMBDA;
		    else lambda_eye6.a[i][j]=0;
		}
	}

    // Copy the initial (input) angle
    for(int i=0;i<6;i++)
    {   theta_initial[i] = theta_target[i];
        theta[i]         = theta_target[i];
    }

    /* Calculate Jacobian and forward kinematics. Input: target_pos_or (target position) */
    while (out ==0)
    {
        abs_sum_error_pos = 0.0;                        // Initial error is "0" for position
        abs_sum_error_or  = 0.0;                        // Initial error is "0" for position

        /* Jacobian of kinematic chain of the leg (Ti)
	   Inputs:  theta (joint angles), ID (RIGHT or LEFT)
                Outputs: J (jacobian), current_pose (forward kinematics of theta)   */
        J = Kinematics::rJacobian(current_pose,theta,ID);

        // Compute sum of errors in position and orientation (between current and desired)
        for(int i=0;i<6;i++)
        {
            error_pose[i] = target_pose[i] - current_pose[i]; // Error in position and orientation
            if (i<3) abs_sum_error_pos += fabs(error_pose[i]);    // Sum of Absolute value of error in position
            else     abs_sum_error_or  += fabs(error_pose[i]);    // Sum of Absolute value of error in orientation
        }

        // If the errors are "small", then finish
        if ((abs_sum_error_pos < 0.001) && (abs_sum_error_or < 0.01))  // Have position and orientation been reached?
        {   out = 1;
            break;
        }

        // If errors are not "small", update the angles
        J1 = trans(J) * inv( J*trans(J) + lambda_eye6 );    // Levenberg-Marquardt computation: damped least squares

        for(int i=0;i<6;i++)                                // Increase in angle:
        {   for(int j=0;j<6;j++)                            //    theta = theta + J1*[error_pos_or]
            {   theta[i] += J1.a[i][j]*error_pose[j];}
            theta[i] -= round(theta[i]/(2*M_PI))*2*M_PI;    // Standarization of angles btw -pi and pi
        }

        Kinematics::rLimitAngle(theta);                     // Limit angles to allowed values
        n_loops++;                                          // Increase the number of loops

        if (n_loops >= ALLOWED_NUM_ITERATIONS)              // If more than allowed number of iterations
        {   printf("Error, excess of  iterations\n");       // exit the loop and indicate error in achieving the target
            out = 1;
            error_indic = 1;
        }
    }

    for(int i=0;i<6;i++)
    {
        if(error_indic==1)  theta_target[i] = theta_initial[i]; // If pose not achieved, return the original angles (no modification)
        else                theta_target[i] = theta[i];         // If pose is achieved, return the new angles
    }

    return(0);
}

/******************** JACOBIAN (AND FORWARD KINEMATICS INSIDE) *****************************************************/

/* Inputs:  theta[6]: set of joint angles [rad]
            ID:       RIGHT for right leg and LEFT for left leg

   Output: PosOri[6]: Position [m] and orientation [rad] obtained by FORWARD KINEMATICS using the joint angles (theta)
           J:         Jacobian of the current configuration
*/

Math::TMatrix6 Kinematics::rJacobian(double PosOri[6], double theta[6], bool ID)
{
    /* Example:
         Math::TMatrix6 J;
         theta[6] = {0,0,0,0,0,0}, double PosOri[6];
         J = Kinematics::rJacobian(PosOri, theta, RIGHT);
         printf("\n%3.3f  %3.3f  %3.3f  %3.3f\n", PosOri[0],PosOri[1],PosOri[2],PosOri[3]);
         for (int i=0; i<6; i++)
    	 {   for (int j=0; j<6; j++)
             {   printf(" %6.2f ",J.a[i][j]);}
             printf("\n");
    	 }
    */

    double th1,th2,th3,th4,th5,th6;          // Variables for storing the joint angles
    
    //Angles values obtained from input theta
    th1=theta[0];  th2=theta[1];  th3=theta[2];
    th4=theta[3];  th5=theta[4];  th6=theta[5];

    //============= FORWARD KINEMATICS =======================

    //DH Homogeneous Matrices (4x4)
    Math::TMatrix4 AC0,A01,A12,A23,A34,A45,A56,A6F,Cad;

    if(ID==LEFT)         // For the left leg
    {   AC0 = Kinematics::rDenhar( M_PI_2, -L2,  L1, 0);
    }
    else if(ID==RIGHT)    // For the right leg
    {   AC0 = Kinematics::rDenhar( M_PI_2, -L2, -L1, 0);
    }

    A01 = Kinematics::rDenhar(        th1, -L3,  0,  M_PI_2);
    A12 = Kinematics::rDenhar( th2-M_PI_2,   0, L4,  M_PI_2);
    A23 = Kinematics::rDenhar(        th3,   0, L5,       0);
    A34 = Kinematics::rDenhar(        th4,   0, L6,       0);
    A45 = Kinematics::rDenhar(        th5,   0, L7, -M_PI_2);
    A56 = Kinematics::rDenhar( th6+M_PI_2,   0,  0, -M_PI_2);
    A6F = Kinematics::rDenhar(    -M_PI_2, -L8,  0,       0);

    double rpy[3];
    Cad = AC0*A01*A12*A23*A34*A45*A56*A6F;  // Complete kinematical chain
    Kinematics::rRPY(rpy,Cad);       // Orientation: RPY angles

    //Current position and orientation
    PosOri[0] = Cad.a[0][3];                // Position: x
    PosOri[1] = Cad.a[1][3];                //           y
    PosOri[2] = Cad.a[2][3];                //           z
    PosOri[3] = rpy[0];                     // Orientation: Roll  (about x)
    PosOri[4] = rpy[1];                     //              Pitch (about y)
    PosOri[5] = rpy[2];                     //              Yaw   (about z)

    //============= JACOBIAN =================================

    Math::TMatrix4 ACC,AC1,AC2,AC3,AC4,AC5,AC6;         // Homogeneous matrices for increments
    double zCC[3],zC1[3],zC2[3],zC3[3],zC4[3],zC5[3];
    double pC6[3],p16[2],p26[3],p36[3],p46[3],p56[3];
    double cross0[3],cross1[3],cross2[3],cross3[3],cross4[3],cross5[3];
    Math::TMatrix6 Jg,Ja,E;                     // Jacobians and E

    ACC = Math::eye(4);
    AC1 = AC0*A01;
    AC2 = AC1*A12;
    AC3 = AC2*A23;
    AC4 = AC3*A34;
    AC5 = AC4*A45;
    AC6 = AC5*A56*A6F;

    for(int i=0;i<3;i++)
      {
	zCC[i] = ACC.a[i][2];
	zC1[i] = AC1.a[i][2];
	zC2[i] = AC2.a[i][2];
	zC3[i] = AC3.a[i][2];
	zC4[i] = AC4.a[i][2];
	zC5[i] = AC5.a[i][2];

	pC6[i] = AC6.a[i][3] - ACC.a[i][3];
	p16[i] = AC6.a[i][3] - AC1.a[i][3];
	p26[i] = AC6.a[i][3] - AC2.a[i][3];
	p36[i] = AC6.a[i][3] - AC3.a[i][3];
	p46[i] = AC6.a[i][3] - AC4.a[i][3];
	p56[i] = AC6.a[i][3] - AC5.a[i][3];
      }
    
    Math::rCross(cross0,zCC,pC6);
    Math::rCross(cross1,zC1,p16);
    Math::rCross(cross2,zC2,p26);
    Math::rCross(cross3,zC3,p36);
    Math::rCross(cross4,zC4,p46);
    Math::rCross(cross5,zC5,p56);

    // Fill the Geometric Jacobian
    for(int i=0;i<3;i++)
    { 
      Jg.a[i][0] = cross0[i];
      Jg.a[i][1] = cross1[i];
      Jg.a[i][2] = cross2[i];
      Jg.a[i][3] = cross3[i];
      Jg.a[i][4] = cross4[i];
      Jg.a[i][5] = cross5[i];
    }
    for(int i=3;i<6;i++)
    { 
      Jg.a[i][0] = zCC[i-3];
      Jg.a[i][1] = zC1[i-3];
      Jg.a[i][2] = zC2[i-3];
      Jg.a[i][3] = zC3[i-3];
      Jg.a[i][4] = zC4[i-3];
      Jg.a[i][5] = zC5[i-3];
    }

    // Matrix E
    for (int i=0;i<6;i++)
      {  for(int j=0;j<6;j++)
	  {  E.a[i][j] = 0;
	  }
      }
    E.a[0][0]=1; E.a[1][1]=1; E.a[2][2]=1; E.a[5][5]=1;
    E.a[3][3] = cos(rpy[0])/cos(rpy[1]);
    E.a[4][3] = -sin(rpy[0]);
    E.a[5][3] = cos(rpy[0])*tan(rpy[1]);
    E.a[3][4] = sin(rpy[0])/cos(rpy[1]);
    E.a[4][4] = cos(rpy[0]);
    E.a[5][4] = tan(rpy[1])*sin(rpy[0]);

    Ja = E*Jg;

    return (Jg);
}


/******************** FORWARD KINEMATICS ***************************************************************************/

/* Inputs:  theta[6]: set of joint angles (rad)
            LR      : RIGHT for right leg and LEFT for left leg
*/

void Kinematics::rFwdKinematics(double theta[6], bool LR)
{
    // Angles
    double th1,th2,th3,th4,th5,th6;
    th1 = theta[0]; th2 = theta[1]; th3 = theta[2];
    th4 = theta[3]; th5 = theta[4]; th6 = theta[5];

    // DH Homogeneous Matrices (4x4)
    Math::TMatrix4 AC0,A01,A12,A23,A34,A45,A56,A6F,Cad;

    if(LR==LEFT)
    {   AC0 = Kinematics::rDenhar( M_PI_2, -L2,  L1, 0);
        //printf("\nForward Kinematics for Left leg:\n");
    }
    else if(LR==RIGHT)
    {   AC0 = Kinematics::rDenhar( M_PI_2, -L2, -L1, 0);
        //printf("\n*Forward Kinematics for Right leg:\n");
    }

    A01 = Kinematics::rDenhar(        th1, -L3,  0,  M_PI_2);
    A12 = Kinematics::rDenhar( th2-M_PI_2,   0, L4,  M_PI_2);
    A23 = Kinematics::rDenhar(        th3,   0, L5,       0);
    A34 = Kinematics::rDenhar(        th4,   0, L6,       0);
    A45 = Kinematics::rDenhar(        th5,   0, L7, -M_PI_2);
    A56 = Kinematics::rDenhar( th6+M_PI_2,   0,  0, -M_PI_2);
    A6F = Kinematics::rDenhar(    -M_PI_2, -L8,  0,       0);

    double ang[3];
    Cad = AC0*A01*A12*A23*A34*A45*A56*A6F;
    Kinematics::rRPY(ang,Cad);               // Orientation: RPY angles

    //Current position and orientation
    double pos_ori[6];
    pos_ori[0] = Cad.a[0][3];           // Position: X
    pos_ori[1] = Cad.a[1][3];           //           Y
    pos_ori[2] = Cad.a[2][3];           //           Z
    pos_ori[3] = ang[0];                // Orientation: Roll  R
    pos_ori[4] = ang[1];                //              Pitch P
    pos_ori[5] = ang[2];                //              Yaw   Y

    printf(" Real Pose:     X=%5.2f, Y=%5.2f, Z=%5.2f, ",pos_ori[0], pos_ori[1], pos_ori[2]);
    printf("R=%5.1f, P=%5.1f, Y=%5.1f\n",pos_ori[3]/M_PI*180, pos_ori[4]/M_PI*180, pos_ori[5]/M_PI*180);
}


/*************** STANDARD DENAVIT-HARTENBERG MATRIX ****************************************************************/

/* Inputs: alpha, a, th, d
   Output: DHmatrix (using the standard DH convention)                       */

Math::TMatrix4 Kinematics::rDenhar(double th, double d, double a, double alpha)
{   /* Example of use:
          Math::TMatrix4 Mat1;
      	  Mat1 = rDenhar(0.1, 3, 1, 5);
          printf("\n%3.3f  %3.3f  %3.3f  %3.3f\n",
                Mat1.a[0][0],Mat1.a[0][1],Mat1.a[2][2],Mat1.a[3][3]);
    */

    Math::TMatrix4 dh;
    dh.a[0][0] =  cos(th);
    dh.a[0][1] = -cos(alpha)*sin(th);
    dh.a[0][2] =  sin(alpha)*sin(th);
    dh.a[0][3] =  a*cos(th);
    dh.a[1][0] =  sin(th);
    dh.a[1][1] =  cos(th)*cos(alpha);
    dh.a[1][2] = -sin(alpha)*cos(th);
    dh.a[1][3] =  a*sin(th);
    dh.a[2][0] =  0;
    dh.a[2][1] =  sin(alpha);
    dh.a[2][2] =  cos(alpha);
    dh.a[2][3] =  d;
    dh.a[3][0] =  0;
    dh.a[3][1] =  0;
    dh.a[3][2] =  0;
    dh.a[3][3] =  1;
    return (dh);
}

/*************** ROLL PITCH AND YAW ANGLES *****************************************************************/

/* Input:   Math::TMatrix4 : Transformation matrix
   Output:  rpy[3]         : roll, pitch, yaw angles

   Note that the definitions are 'roll' about X axis, 'pitch' about Y axis and 'yaw' about Z axis,
   that is: M = rot(z,y)*rot(y,p)*rot(x,r), where y,p,r are the yaw,pitch,roll angles
*/

void Kinematics::rRPY(double rpy[3], struct Math::TMatrix4 T)
{  	/* Example:
         double rpy[3];
	 Math::TMatrix4 Mat1 = rDenhar(0.1, 3, 1, 5);
	 Kinematics::rRPY(rpy,Mat1);
  	     printf("\n%3.3f  %3.3f  %3.3f\n", rpy[0], rpy[1], rpy[2]);
	*/

    double th_r, th_p, th_y;

    //Compute Pitch
    th_p = atan2( -T.a[2][0], sqrt( pow(T.a[2][1],2) + pow(T.a[2][2],2) ) );
    if (th_p < -M_PI_2 || th_p > M_PI_2 )
    {    th_p = atan2( -T.a[2][0], sqrt( pow(T.a[2][1],2) + pow(T.a[2][2],2) ) );   }

    //If pitch is in the interval <-pi/2, pi/2>, obtain roll and yaw directly
    if (th_p > -M_PI_2 && th_p < M_PI_2 )
    {
        th_y = atan2( T.a[1][0], T.a[0][0] );
        th_r = atan2( T.a[2][1], T.a[2][2] );
    }

    //If pitch = +pi/2, only the difference can be recovered
    else if (th_p == M_PI_2)
    {
        th_y = 0;
        th_r = atan2( T.a[0][1], T.a[1][1] );
    }

    //If pitch = -pi/2, only the sum can be recovered
    else if (th_p == -M_PI_2)
    {
        th_y = 0;
        th_r = -atan2( T.a[0][1], T.a[1][1] );
    }

    rpy[0] = th_r; rpy[1] = th_p; rpy[2] = th_y;
}


/*************** ACCESSORY FUNCTIONS *******************************************************************************/

//Function to Limit the angle to the allowed values
void Kinematics::rLimitAngle(double ang[6])
{
    for(int i=0;i<6;i++)
    {   if(i==3)
        {
            if (ang[i] < LOW_ANG_KNEE*M_PI/180) ang[i] = LOW_ANG_KNEE*M_PI/180;
            if (ang[i] > HI_ANG_KNEE*M_PI/180)  ang[i] = HI_ANG_KNEE*M_PI/180;
        }
        else
        {
            if (ang[i] < LOW_ANG*M_PI/180) ang[i] = LOW_ANG*M_PI/180;
            if (ang[i] > HI_ANG*M_PI/180)  ang[i] = HI_ANG*M_PI/180;
        }
    }
}


//Function to verify if a pose is in the robot's workspace.
bool Kinematics::rCheckWorkspace(double Pose[6], bool ID)
{
    // - If Pose is inside the workspace, it returns the same pose
    // - If it is not, it returns the closest pose in the workspace (not yet implemented)

    //dReal xr=Pose[0], yr=Pose[1], zr=Pose[2];         // Current position
    dReal xr=Pose[0], zr=Pose[2];                       // Current position
    bool InWorkspace;                                   // To indicate if it is inside the workspace

    //--------------------- Verify workspace in xz plane ----------------------

    dReal Rad = L4 + L5 + L6 + L7 + L8;
    dReal zo = -(L2 + L3);

    //It is inside the circunference and z is in the down part
    if ( ( pow(zr-zo,2) + pow(xr,2) - pow(Rad,2) )<=0 && zr<=zo )
    {
        //PoseOut = PoseIn;
        InWorkspace = true;
    }
    else
    {
        //PoseOut = PoseIn;
        InWorkspace = false;
        printf("Workspace note: (x=%5.2f,z=%5.2f) is out of reachable space\n", xr, zr);
    }

    return (InWorkspace);

}
