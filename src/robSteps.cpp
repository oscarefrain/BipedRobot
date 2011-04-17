/*==========================================================================
    Functions to move the robot
    ---------------------------
     	Oscar E. Ramos Ponce
    	Estudante do Programa de Pós Graduação em Engenharia Elétrica
    	Universidade Federal do Rio Grande do Sul, Brasil 2011
==========================================================================*/


#include "../include/robFunctions.h"
#include "../include/robModel.h"


/*=====================================================================================================================
                      PRE COMPUTE THE INITIAL STEP
=====================================================================================================================*/

/* Inputs: InitialAngleL[6]: Initial Angles for the left leg
           InitialAngleR[6]: Initial Angles for the right leg
           StepLen         : Length of the initial step
           FixedLeg        : Indication of which leg is fixed (LEFT or RIGHT)

   Outputs: TotalAngleL[][6]: Precomputed angles for left leg
            TotalAngleR[][6]: Precomputed angles for right leg
*/

void Traj::rPreComputeInitialStep(double TotalAngleL[][6], double TotalAngleR[][6],
                                  double InitialAngleL[6], double InitialAngleR[6],
                                  double StepLen, bool FixedLeg)
{
    double t0 = 0.0, t1 = 2.0;                      // Time for interpolation (initial & final)
    Math::TPoint3D Pq1, Pq2, Pf1, Pf2;              // Initial and final points for the Trajectories
    double Hq = (L3+L4+L5+L6+L7+L8);                // High of the hips (Q:quartil) wrt the ground
    double initial_angleL[6], initial_angleR[6];    // Initial angles for the computation

    double ComputeAngleR1[TLENG][6], ComputeAngleL1[TLENG][6];  // Variables to store the values of the
    double ComputeAngleR2[TLENG][6], ComputeAngleL2[TLENG][6];  // computed angles for the different
    double ComputeAngleR3[TLENG][6], ComputeAngleL3[TLENG][6];  // parts of the trajectory
    double ComputeAngleR4[TLENG][6], ComputeAngleL4[TLENG][6];

    //********** FIRST: Move Hips (COG) over Fixed Leg (and a little down) *****************************

    Pq1.x=0.0; Pq1.z=Hq;      Pq1.vx=0; Pq1.vy=0; Pq1.vz=0;     // Hips wrt Fixed Leg: Initial Point
    Pq2.x=0.0; Pq2.z=Hq-L1/4; Pq2.vx=0; Pq2.vy=0; Pq2.vz=0;     // Hips wrt Fixed Leg: Final Point

    Pf1.x=0.0; Pf1.z=0;       Pf1.vx=0; Pf1.vy=0; Pf1.vz=0;     // Floating Leg wrt Fixed Leg: Initial Point
    Pf2.x=0.0; Pf2.z=0;       Pf2.vx=0; Pf2.vy=0; Pf2.vz=0;     // Floating Leg wrt Fixed Leg: Final Point

    // Y points vary depending on which is the fixed leg
    if(FixedLeg == LEFT)       { Pq1.y=0.0; Pq2.y=L1;  Pf1.y=-2*L1; Pf2.y=-2*L1; }
    else if(FixedLeg == RIGHT) { Pq1.y=0.0; Pq2.y=-L1; Pf1.y=2*L1;  Pf2.y=2*L1;  }

    Traj::rPreComputeBasicTrajectory(ComputeAngleL1, ComputeAngleR1, InitialAngleL, InitialAngleR,
                                     Pq1,Pq2,Pf1,Pf2,t0,t1,TLENG,FixedLeg);

    // Assign Angles to the output variables
    for(int i=0;i<TLENG;i++)
    {   for(int ang=0;ang<MAX_JOINTS;ang++)
        {   TotalAngleL[i][ang] = ComputeAngleL1[i][ang];
            TotalAngleR[i][ang] = ComputeAngleR1[i][ang];
        }
    }
    printf("\nInitial Step: 1st part angles were pre-computed \n");

    //*********** SECOND: Raise Floating Leg (straight up) ***************************************

    Pq1.x=0.0; Pq1.z=Hq-L1/4; Pq1.vx=0; Pq1.vy=0; Pq1.vz=0; // Hips wrt Fixed Leg: Initial Point
    Pq2.x=0.0; Pq2.z=Hq-L1/4; Pq2.vx=0; Pq2.vy=0; Pq2.vz=0; // Hips wrt Fixed Leg: Final Point

    Pf1.x=0.0; Pf1.z=0;       Pf1.vx=0; Pf1.vy=0; Pf1.vz=0; // Floating Leg wrt Fixed Leg: Initial Point
    Pf2.x=0.0; Pf2.z=0.1;     Pf2.vx=0; Pf2.vy=0; Pf2.vz=0; // Floating Leg wrt Fixed Leg: Final Point

    // Y points vary depending on which is the fixed leg
    if(FixedLeg == LEFT)       { Pq1.y=L1;  Pq2.y=L1;  Pf1.y=-2*L1; Pf2.y=-2*L1; }
    else if(FixedLeg == RIGHT) { Pq1.y=-L1; Pq2.y=-L1; Pf1.y=2*L1;  Pf2.y=2*L1;  }

    // Get the initial angles for the next trajectory computation
    for(int i=0;i<MAX_JOINTS;i++)
    {   initial_angleL[i] = ComputeAngleL1[TLENG-1][i];
        initial_angleR[i] = ComputeAngleR1[TLENG-1][i];
    }

    Traj::rPreComputeBasicTrajectory(ComputeAngleL2, ComputeAngleR2, initial_angleL, initial_angleR,
                                     Pq1,Pq2,Pf1,Pf2,t0,t1,TLENG,FixedLeg);

    // Assign Angles to the output variables
    for(int i=TLENG;i<2*TLENG;i++)
    {   for(int ang=0;ang<MAX_JOINTS;ang++)
        {   TotalAngleL[i][ang] = ComputeAngleL2[i-TLENG][ang];
            TotalAngleR[i][ang] = ComputeAngleR2[i-TLENG][ang];
        }
    }
    printf("Initial Step: 2nd part angles were pre-computed \n");

    //*********** THIRD: Move Floating Leg Forward and down ***************************************

    Pq1.x=0.0;       Pq1.z=Hq-L1/4;   Pq1.vx=0; Pq1.vy=0; Pq1.vz=0;     // Hips wrt Fixed Leg: Initial Point
    Pq2.x=StepLen/2; Pq2.z=Hq-2*L1/4; Pq2.vx=0; Pq2.vy=0; Pq2.vz=0;     // Hips wrt Fixed Leg: Final Point

    Pf1.x=0.0;       Pf1.z=0.1;       Pf1.vx=0; Pf1.vy=0; Pf1.vz=0;     // Floating Leg wrt Fixed Leg: Initial Point
    Pf2.x=StepLen;   Pf2.z=0.0;       Pf2.vx=0; Pf2.vy=0; Pf2.vz=0;     // Floating Leg wrt Fixed Leg: Final Point

    // Y points vary depending on which is the fixed leg
    if(FixedLeg == LEFT)       { Pq1.y=L1;  Pq2.y=L1;  Pf1.y=-2*L1; Pf2.y=-2*L1; }
    else if(FixedLeg == RIGHT) { Pq1.y=-L1; Pq2.y=-L1; Pf1.y=2*L1;  Pf2.y=2*L1;  }

    // Get the initial angles for the next trajectory computation
    for(int i=0;i<MAX_JOINTS;i++)
    {   initial_angleL[i] = ComputeAngleL2[TLENG-1][i];
        initial_angleR[i] = ComputeAngleR2[TLENG-1][i];
    }

    Traj::rPreComputeBasicTrajectory(ComputeAngleL3, ComputeAngleR3, initial_angleL, initial_angleR,
                                     Pq1,Pq2,Pf1,Pf2,t0,t1,TLENG,FixedLeg);

    // Assign Angles to the output variables
    for(int i=2*TLENG;i<3*TLENG;i++)
    {   for(int ang=0;ang<MAX_JOINTS;ang++)
        {   TotalAngleL[i][ang] = ComputeAngleL3[i-2*TLENG][ang];
            TotalAngleR[i][ang] = ComputeAngleR3[i-2*TLENG][ang];
        }
    }
    printf("Initial Step: 3rd part angles were pre-computed \n");

    //*********** FOURTH: Move Hips (COG) back to the middle (in the sagital plane) ***********************

    Pq1.x=StepLen/2; Pq1.z=Hq-2*L1/4; Pq1.vx=0; Pq1.vy=0; Pq1.vz=0; // Hips wrt Fixed Leg: Initial Point
    Pq2.x=StepLen/2; Pq2.z=Hq-2*L1/4; Pq2.vx=0; Pq2.vy=0; Pq2.vz=0; // Hips wrt Fixed Leg: Final Point

    Pf1.x=StepLen;   Pf1.z=0.0;       Pf1.vx=0; Pf1.vy=0; Pf1.vz=0;      // Floating Leg wrt Fixed Leg: Initial Point
    Pf2.x=StepLen;   Pf2.z=0.0;       Pf2.vx=0; Pf2.vy=0; Pf2.vz=0;    // Floating Leg wrt Fixed Leg: Final Point

    // Y points vary depending on which is the fixed leg
    if(FixedLeg == LEFT)       { Pq1.y=L1;  Pq2.y=0.0; Pf1.y=-2*L1; Pf2.y=-2*L1; }
    else if(FixedLeg == RIGHT) { Pq1.y=-L1; Pq2.y=0.0; Pf1.y=2*L1;  Pf2.y=2*L1;  }

    // Get the initial angles for the next trajectory computation
    for(int i=0;i<MAX_JOINTS;i++)
    {   initial_angleL[i] = ComputeAngleL3[TLENG-1][i];
        initial_angleR[i] = ComputeAngleR3[TLENG-1][i];
    }

    Traj::rPreComputeBasicTrajectory(ComputeAngleL4, ComputeAngleR4, initial_angleL, initial_angleR,
                                     Pq1,Pq2,Pf1,Pf2,t0,t1,TLENG,FixedLeg);

    // Assign Angles to the output variables
    for(int i=3*TLENG;i<4*TLENG;i++)
    {   for(int ang=0;ang<MAX_JOINTS;ang++)
        {   TotalAngleL[i][ang] = ComputeAngleL4[i-3*TLENG][ang];
            TotalAngleR[i][ang] = ComputeAngleR4[i-3*TLENG][ang];
        }
    }
    printf("Initial Step: 4th part angles were pre-computed \n\n");

}


/*=====================================================================================================================
                      PRE COMPUTE THE NORMAL STEPS
=====================================================================================================================*/

/* Inputs: InitialAngleL[6]: Initial Angles for the left leg
           InitialAngleR[6]: Initial Angles for the right leg
           StepLen         : Length of the initial step
           FixedLeg        : Indication of which leg is fixed (LEFT or RIGHT)

   Outputs: TotalAngleL[][6]: Precomputed angles for left leg
            TotalAngleR[][6]: Precomputed angles for right leg
*/
void Traj::rPreComputeNormalStep(double TotalAngleL[][6], double TotalAngleR[][6],
                                 double InitialAngleL[6], double InitialAngleR[6],
                                 double StepLen, bool FixedLeg)
{
    double t0 = 0.0, t1 = 2.0;                      // Time for interpolation (initial & final)
    Math::TPoint3D Pq1, Pq2, Pf1, Pf2;              // Initial and final points for the Trajectories
    double Hq = (L3+L4+L5+L6+L7+L8);                // High of the hips (Q:quartil) wrt the ground
    double initial_angleL[6], initial_angleR[6];    // Initial angles for the computation

    double ComputeAngleR1[TLENG][6], ComputeAngleL1[TLENG][6];  // Variables to store the values of the
    double ComputeAngleR2[TLENG][6], ComputeAngleL2[TLENG][6];  // computed angles for the different
    double ComputeAngleR3[TLENG][6], ComputeAngleL3[TLENG][6];  // parts of the trajectory
    double ComputeAngleR4[TLENG][6], ComputeAngleL4[TLENG][6];

    //********** FIRST: Move Hips (COG) over Fixed leg in Y (Sagital plane) ************************

    Pq1.x=-StepLen/2; Pq1.z=Hq-2*L1/4; Pq1.vx=0; Pq1.vy=0; Pq1.vz=0;    // Hips wrt Fixed Leg: Initial Point
    Pq2.x=-StepLen/2; Pq2.z=Hq-2*L1/4; Pq2.vx=0; Pq2.vy=0; Pq2.vz=0;    // Hips wrt Fixed Leg: Final Point

    Pf1.x=-StepLen;   Pf1.z=0.0;       Pf1.vx=0; Pf1.vy=0; Pf1.vz=0;    // Floating Leg wrt Fixed Leg: Initial Point
    Pf2.x=-StepLen;   Pf2.z=0.0;       Pf2.vx=0; Pf2.vy=0; Pf2.vz=0;    // Floating Leg wrt Fixed Leg: Final Point

    // Y points vary depending on which is the fixed leg
    if     (FixedLeg == LEFT) { Pq1.y=0.0; Pq2.y=L1;  Pf1.y=-2*L1; Pf2.y=-2*L1; }
    else if(FixedLeg == RIGHT){ Pq1.y=0.0; Pq2.y=-L1; Pf1.y=2*L1;  Pf2.y=2*L1;  }

    Traj::rPreComputeBasicTrajectory(ComputeAngleL1, ComputeAngleR1, InitialAngleL, InitialAngleR,
                                     Pq1,Pq2,Pf1,Pf2,t0,t1,TLENG,FixedLeg);
    
    // Assign Angles to the output variables
    for(int i=0;i<TLENG;i++)
    {   for(int ang=0;ang<MAX_JOINTS;ang++)
        {   TotalAngleL[i][ang] = ComputeAngleL1[i][ang];
            TotalAngleR[i][ang] = ComputeAngleR1[i][ang];
        }
    }
    printf("Step: 1st Part Angles were Pre-computed \n");

    //*********** SECOND: Move Floating Leg forward and up next to fixed leg (1st half of step) *****

    Pq1.x=-StepLen/2; Pq1.z=Hq-2*L1/4; Pq1.vx=0; Pq1.vy=0; Pq1.vz=0;    // Hips wrt Fixed Leg: Initial Point
    Pq2.x=0.0;        Pq2.z=Hq-2*L1/4; Pq2.vx=0; Pq2.vy=0; Pq2.vz=0;    // Hips wrt Fixed Leg: Final Point

    Pf1.x=-StepLen;   Pf1.z=0.0;       Pf1.vx=0; Pf1.vy=0; Pf1.vz=0;    // Floating Leg wrt Fixed Leg: Initial Point
    Pf2.x=0.0;        Pf2.z=0.1;       Pf2.vx=0; Pf2.vy=0; Pf2.vz=0;    // Floating Leg wrt Fixed Leg: Final Point

    // Y points vary depending on which is the fixed leg
    if(FixedLeg == LEFT)       { Pq1.y=L1;  Pq2.y=L1;  Pf1.y=-2*L1; Pf2.y=-2*L1; }
    else if(FixedLeg == RIGHT) { Pq1.y=-L1; Pq2.y=-L1; Pf1.y=2*L1;  Pf2.y=2*L1;  }

    // Get the initial angles for the next trajectory computation
    for(int i=0;i<MAX_JOINTS;i++)
    {   initial_angleL[i] = ComputeAngleL1[TLENG-1][i];
        initial_angleR[i] = ComputeAngleR1[TLENG-1][i];
    }

    Traj::rPreComputeBasicTrajectory(ComputeAngleL2, ComputeAngleR2, initial_angleL, initial_angleR,
                                     Pq1,Pq2,Pf1,Pf2,t0,t1,TLENG,FixedLeg);

    // Assign Angles to the output variables
    for(int i=TLENG;i<2*TLENG;i++)
    {   for(int ang=0;ang<MAX_JOINTS;ang++)
        {   TotalAngleL[i][ang] = ComputeAngleL2[i-TLENG][ang];
            TotalAngleR[i][ang] = ComputeAngleR2[i-TLENG][ang];
        }
    }
    printf("Step: 2nd Part Angles were Pre-computed \n");

    //*********** THIRD: Move Floating Leg forward and down (2nd half of step) ***********************

    Pq1.x=0.0;       Pq1.z=Hq-2*L1/4; Pq1.vx=0; Pq1.vy=0; Pq1.vz=0;     // Hips wrt Fixed Leg: Initial Point
    Pq2.x=StepLen/2; Pq2.z=Hq-2*L1/4; Pq2.vx=0; Pq2.vy=0; Pq2.vz=0;     // Hips wrt Fixed Leg: Final Point

    Pf1.x=0.0;       Pf1.z=0.1;       Pf1.vx=0; Pf1.vy=0; Pf1.vz=0;     // Floating Leg wrt Fixed Leg: Initial Point
    Pf2.x=StepLen;   Pf2.z=0.0;       Pf2.vx=0; Pf2.vy=0; Pf2.vz=0;     // Floating Leg wrt Fixed Leg: Final Point

    // Y points vary depending on which is the fixed leg
    if(FixedLeg == LEFT)       { Pq1.y=L1;  Pq2.y=L1;  Pf1.y=-2*L1; Pf2.y=-2*L1; }
    else if(FixedLeg == RIGHT) { Pq1.y=-L1; Pq2.y=-L1; Pf1.y=2*L1;  Pf2.y=2*L1;  }

    // Get the initial angles for the next trajectory computation
    for(int i=0;i<MAX_JOINTS;i++)
    {   initial_angleL[i] = ComputeAngleL2[TLENG-1][i];
        initial_angleR[i] = ComputeAngleR2[TLENG-1][i];
    }

    Traj::rPreComputeBasicTrajectory(ComputeAngleL3, ComputeAngleR3, initial_angleL, initial_angleR,
                                     Pq1,Pq2,Pf1,Pf2,t0,t1,TLENG,FixedLeg);

    // Assign Angles to the output variables
    for(int i=2*TLENG;i<3*TLENG;i++)
    {   for(int ang=0;ang<MAX_JOINTS;ang++)
        {   TotalAngleL[i][ang] = ComputeAngleL3[i-2*TLENG][ang];
            TotalAngleR[i][ang] = ComputeAngleR3[i-2*TLENG][ang];
        }
    }
    printf("Step: 3rd Part Angles were Pre-computed \n");

    //*********** FOURTH: Move Hips back to the Center (in sagital plane) *****************************

    Pq1.x=StepLen/2; Pq1.z=Hq-2*L1/4; Pq1.vx=0; Pq1.vy=0; Pq1.vz=0;     // Hips wrt Fixed Leg: Initial Point
    Pq2.x=StepLen/2; Pq2.z=Hq-2*L1/4; Pq2.vx=0; Pq2.vy=0; Pq2.vz=0;     // Hips wrt Fixed Leg: Final Point

    Pf1.x=StepLen;   Pf1.z=0.0;       Pf1.vx=0; Pf1.vy=0; Pf1.vz=0;     // Floating Leg wrt Fixed Leg: Initial Point
    Pf2.x=StepLen;   Pf2.z=0.0;       Pf2.vx=0; Pf2.vy=0; Pf2.vz=0;     // Floating Leg wrt Fixed Leg: Final Point

    // Y points vary depending on which is the fixed leg
    if(FixedLeg == LEFT)       { Pq1.y=L1;  Pq2.y=0.0; Pf1.y=-2*L1; Pf2.y=-2*L1; }
    else if(FixedLeg == RIGHT) { Pq1.y=-L1; Pq2.y=0.0; Pf1.y=2*L1;  Pf2.y=2*L1;  }

    // Get the initial angles for the next trajectory computation
    for(int i=0;i<MAX_JOINTS;i++)
    {   initial_angleL[i] = ComputeAngleL3[TLENG-1][i];
        initial_angleR[i] = ComputeAngleR3[TLENG-1][i];
    }

    Traj::rPreComputeBasicTrajectory(ComputeAngleL4, ComputeAngleR4, initial_angleL, initial_angleR,
                                     Pq1,Pq2,Pf1,Pf2,t0,t1,TLENG,FixedLeg);

    // Assign Angles to the output variables
    for(int i=3*TLENG;i<4*TLENG;i++)
    {   for(int ang=0;ang<MAX_JOINTS;ang++)
        {   TotalAngleL[i][ang] = ComputeAngleL4[i-3*TLENG][ang];
            TotalAngleR[i][ang] = ComputeAngleR4[i-3*TLENG][ang];
        }
    }
    printf("Step: 4th Part Angles were Pre-computed \n\n");

}



/*=================================================================================================================
                     PRE COMPUTE THE FINAL STEP 
===================================================================================================================*/

/* Inputs: InitialAngleL[6]: Initial Angles for the left leg
           InitialAngleR[6]: Initial Angles for the right leg
           StepLen         : Length of the initial step
           FixedLeg        : Indication of which leg is fixed (LEFT or RIGHT)

   Outputs: TotalAngleL[][6]: Precomputed angles for left leg
            TotalAngleR[][6]: Precomputed angles for right leg
*/
void Traj::rPreComputeFinalStep(double TotalAngleL[][6], double TotalAngleR[][6],
                                double InitialAngleL[6], double InitialAngleR[6],
                                double StepLen, bool FixedLeg)
{
    double t0 = 0.0, t1 = 2.0;                      // Time for interpolation (initial & final)
    Math::TPoint3D Pq1, Pq2, Pf1, Pf2;              // Initial and final points for the Trajectories
    double Hq = (L3+L4+L5+L6+L7+L8);                // High of the hips (Q:quartil) wrt the ground
    double initial_angleL[6], initial_angleR[6];    // Initial angles for the computation

    double ComputeAngleR1[TLENG][6], ComputeAngleL1[TLENG][6];  // Variables to store the values of the
    double ComputeAngleR2[TLENG][6], ComputeAngleL2[TLENG][6];  // computed angles for the different
    double ComputeAngleR3[TLENG][6], ComputeAngleL3[TLENG][6];  // parts of the trajectory
    double ComputeAngleR4[TLENG][6], ComputeAngleL4[TLENG][6];

    //********** FIRST: Move Hips (COG) over Fixed leg in Y (Sagital plane) ************************

    Pq1.x=-StepLen/2; Pq1.z=Hq-2*L1/4; Pq1.vx=0; Pq1.vy=0; Pq1.vz=0;    // Hips wrt Fixed Leg: Initial Point
    Pq2.x=-StepLen/2; Pq2.z=Hq-2*L1/4; Pq2.vx=0; Pq2.vy=0; Pq2.vz=0;    // Hips wrt Fixed Leg: Final Point

    Pf1.x=-StepLen;   Pf1.z=0.0;       Pf1.vx=0; Pf1.vy=0; Pf1.vz=0;    // Floating Leg wrt Fixed Leg: Initial Point
    Pf2.x=-StepLen;   Pf2.z=0.0;       Pf2.vx=0; Pf2.vy=0; Pf2.vz=0;    // Floating Leg wrt Fixed Leg: Final Point

    // Y points vary depending on which is the fixed leg
    if     (FixedLeg == LEFT) { Pq1.y=0.0; Pq2.y=L1;  Pf1.y=-2*L1; Pf2.y=-2*L1; }
    else if(FixedLeg == RIGHT){ Pq1.y=0.0; Pq2.y=-L1; Pf1.y=2*L1;  Pf2.y=2*L1;  }

    Traj::rPreComputeBasicTrajectory(ComputeAngleL1, ComputeAngleR1, InitialAngleL, InitialAngleR,
                                     Pq1,Pq2,Pf1,Pf2,t0,t1,TLENG,FixedLeg);

    // Assign Angles to the output variables
    for(int i=0;i<TLENG;i++)
    {   for(int ang=0;ang<MAX_JOINTS;ang++)
        {   TotalAngleL[i][ang] = ComputeAngleL1[i][ang];
            TotalAngleR[i][ang] = ComputeAngleR1[i][ang];
        }
    }
    printf("Final step: 1st Part Angles were Pre-computed \n");

    //*********** SECOND: Move Floating Leg forward and up next to fixed leg (1st half of step) *****

    Pq1.x=-StepLen/2; Pq1.z=Hq-2*L1/4; Pq1.vx=0; Pq1.vy=0; Pq1.vz=0;    // Hips wrt Fixed Leg: Initial Point
    Pq2.x=0.0;        Pq2.z=Hq-2*L1/4; Pq2.vx=0; Pq2.vy=0; Pq2.vz=0;    // Hips wrt Fixed Leg: Final Point

    Pf1.x=-StepLen;   Pf1.z=0.0;       Pf1.vx=0; Pf1.vy=0; Pf1.vz=0;    // Floating Leg wrt Fixed Leg: Initial Point
    Pf2.x=0.0;        Pf2.z=0.1;       Pf2.vx=0; Pf2.vy=0; Pf2.vz=0;    // Floating Leg wrt Fixed Leg: Final Point

    // Y points vary depending on which is the fixed leg
    if(FixedLeg == LEFT)       { Pq1.y=L1;  Pq2.y=L1;  Pf1.y=-2*L1; Pf2.y=-2*L1; }
    else if(FixedLeg == RIGHT) { Pq1.y=-L1; Pq2.y=-L1; Pf1.y=2*L1;  Pf2.y=2*L1;  }

    // Get the initial angles for the next trajectory computation
    for(int i=0;i<MAX_JOINTS;i++)
    {   initial_angleL[i] = ComputeAngleL1[TLENG-1][i];
        initial_angleR[i] = ComputeAngleR1[TLENG-1][i];
    }

    Traj::rPreComputeBasicTrajectory(ComputeAngleL2, ComputeAngleR2, initial_angleL, initial_angleR,
                                     Pq1,Pq2,Pf1,Pf2,t0,t1,TLENG,FixedLeg);

    // Assign Angles to the output variables
    for(int i=TLENG;i<2*TLENG;i++)
    {   for(int ang=0;ang<MAX_JOINTS;ang++)
        {   TotalAngleL[i][ang] = ComputeAngleL2[i-TLENG][ang];
            TotalAngleR[i][ang] = ComputeAngleR2[i-TLENG][ang];
        }
    }
    printf("Final step: 2nd Part Angles were Pre-computed \n");

    //*********** THIRD: Move Floating Leg down next to the fixed leg ********************************

    Pq1.x=0.0; Pq1.z=Hq-2*L1/4; Pq1.vx=0; Pq1.vy=0; Pq1.vz=0;     // Hips wrt Fixed Leg: Initial Point
    Pq2.x=0.0; Pq2.z=Hq-L1/4; Pq2.vx=0; Pq2.vy=0; Pq2.vz=0;     // Hips wrt Fixed Leg: Final Point

    Pf1.x=0.0; Pf1.z=0.1;       Pf1.vx=0; Pf1.vy=0; Pf1.vz=0;     // Floating Leg wrt Fixed Leg: Initial Point
    Pf2.x=0.0; Pf2.z=0.0;       Pf2.vx=0; Pf2.vy=0; Pf2.vz=0;     // Floating Leg wrt Fixed Leg: Final Point

    // Y points vary depending on which is the fixed leg
    if(FixedLeg == LEFT)       { Pq1.y=L1;  Pq2.y=L1;  Pf1.y=-2*L1; Pf2.y=-2*L1; }
    else if(FixedLeg == RIGHT) { Pq1.y=-L1; Pq2.y=-L1; Pf1.y=2*L1;  Pf2.y=2*L1;  }

    // Get the initial angles for the next trajectory computation
    for(int i=0;i<MAX_JOINTS;i++)
    {   initial_angleL[i] = ComputeAngleL2[TLENG-1][i];
        initial_angleR[i] = ComputeAngleR2[TLENG-1][i];
    }

    Traj::rPreComputeBasicTrajectory(ComputeAngleL3, ComputeAngleR3, initial_angleL, initial_angleR,
                                     Pq1,Pq2,Pf1,Pf2,t0,t1,TLENG,FixedLeg);

    // Assign Angles to the output variables
    for(int i=2*TLENG;i<3*TLENG;i++)
    {   for(int ang=0;ang<MAX_JOINTS;ang++)
        {   TotalAngleL[i][ang] = ComputeAngleL3[i-2*TLENG][ang];
            TotalAngleR[i][ang] = ComputeAngleR3[i-2*TLENG][ang];
        }
    }
    printf("Final step: 3rd Part Angles were Pre-computed \n");

    //*********** FOURTH: Move Hips back to the initial position **************************************

    Pq1.x=0.0; Pq1.z=Hq-L1/4; Pq1.vx=0; Pq1.vy=0; Pq1.vz=0;     // Hips wrt Fixed Leg: Initial Point
    Pq2.x=0.0; Pq2.z=Hq; Pq2.vx=0; Pq2.vy=0; Pq2.vz=0;     // Hips wrt Fixed Leg: Final Point

    Pf1.x=0.0; Pf1.z=0.0;       Pf1.vx=0; Pf1.vy=0; Pf1.vz=0;     // Floating Leg wrt Fixed Leg: Initial Point
    Pf2.x=0.0; Pf2.z=0.0;       Pf2.vx=0; Pf2.vy=0; Pf2.vz=0;     // Floating Leg wrt Fixed Leg: Final Point

    // Y points vary depending on which is the fixed leg
    if(FixedLeg == LEFT)       { Pq1.y=L1;  Pq2.y=0.0;  Pf1.y=-2*L1; Pf2.y=-2*L1; }
    else if(FixedLeg == RIGHT) { Pq1.y=-L1; Pq2.y=0.0; Pf1.y=2*L1;  Pf2.y=2*L1;  }

    // Get the initial angles for the next trajectory computation
    for(int i=0;i<MAX_JOINTS;i++)
    {   initial_angleL[i] = ComputeAngleL3[TLENG-1][i];
        initial_angleR[i] = ComputeAngleR3[TLENG-1][i];
    }

    Traj::rPreComputeBasicTrajectory(ComputeAngleL4, ComputeAngleR4, initial_angleL, initial_angleR,
                                     Pq1,Pq2,Pf1,Pf2,t0,t1,TLENG,FixedLeg);

    // Assign Angles to the output variables
    for(int i=3*TLENG;i<4*TLENG;i++)
    {   for(int ang=0;ang<MAX_JOINTS;ang++)
        {   TotalAngleL[i][ang] = ComputeAngleL4[i-3*TLENG][ang];
            TotalAngleR[i][ang] = ComputeAngleR4[i-3*TLENG][ang];
        }
    }
    printf("Final step: 4th Part Angles were Pre-computed \n\n");

}



/*=================== PRE COMPUTE THE BASIC TRAJECTORY: MOTION OF HIPS AND FLOATING LEG =======================*/

/* Inputs: InitialAngleL[6]: Initial Angles for the left leg
           InitialAngleR[6]: Initial Angles for the right leg
           Pq1: First point for the Hips        Pq2: End point for the Hips
           Pf1: First point for Floating leg    Pf2: End point for Floating leg
           to:  Initial time                    tf: Final time                      TLength: [int] Number of elements in time
           ID:  Leg that is going to be fixed, RIGHT for fixed right leg and LEFT for fixed left leg

   Outputs: TotalAngleL[][6]: Precomputed angles for left leg
            TotalAngleR[][6]: Precomputed angles for right leg
*/

//void Traj::rComputeHipsMotion(double TotalAngleL[][6], double TotalAngleR[][6],
void Traj::rPreComputeBasicTrajectory(double TotalAngleL[][6], double TotalAngleR[][6],
                                      double InitialAngleL[6], double InitialAngleR[6],
                                      Math::TPoint3D Pq1, Math::TPoint3D Pq2,
                                      Math::TPoint3D Pf1, Math::TPoint3D Pf2,
                                      double to, double tf, int TLength, bool ID)
{
    double t[TLength],Xq[TLength],Yq[TLength],Zq[TLength],Xf[TLength],Yf[TLength],Zf[TLength];  //Variables for time, x,y,z
    double APF[DOF][TLENG], APQ[DOF][TLENG], CPA[DOF][TLENG], CPF[DOF][TLENG];                  //Variables for changing references
    Math::TMatrix4 CPQ;                                                                         //Matrix for CPQ

    double TargetAngleR[MAX_JOINTS], TargetAngleL[MAX_JOINTS];
    double ObtainedPose[DOF];                                                                   //Obtained with inverse kinematics

    //Initialize the TargetAngles to the initial Angles
    for(int i=0;i<MAX_JOINTS;i++)
    {   TargetAngleR[i] = InitialAngleR[i];
        TargetAngleL[i] = InitialAngleL[i];
    }

    //Generate the trajectory using cubic interpolation (Swinging leg wrt Fixed leg)
    Math::rLinspace(t,to,tf,TLength);                       // time: linear space from to,tf with TLength of points

    Math::rOffLineCubicInterp(Xq, Pq1.x, Pq2.x, Pq1.vx, Pq2.vx, t, TLength);     // Interpolation for Xq
    Math::rOffLineCubicInterp(Yq, Pq1.y, Pq2.y, Pq1.vy, Pq2.vy, t, TLength);     // Interpolation for Yq
    Math::rOffLineCubicInterp(Zq, Pq1.z, Pq2.z, Pq1.vz, Pq2.vz, t, TLength);     // Interpolation for Zq

    Math::rOffLineCubicInterp(Xf, Pf1.x, Pf2.x, Pf1.vx, Pf2.vx, t, TLength);     // Interpolation for Xf
    Math::rOffLineCubicInterp(Yf, Pf1.y, Pf2.y, Pf1.vy, Pf2.vy, t, TLength);     // Interpolation for Yf
    Math::rOffLineCubicInterp(Zf, Pf1.z, Pf2.z, Pf1.vz, Pf2.vz, t, TLength);     // Interpolation for Zf

    // Write the files containing the positions of the feet and the hip
    FILE *outXq, *outYq, *outZq, *outXf, *outYf, *outZf;
    outXq = fopen("/tmp/Xq.dat","a");
    outYq = fopen("/tmp/Yq.dat","a");
    outZq = fopen("/tmp/Zq.dat","a");
    outXf = fopen("/tmp/Xf.dat","a");
    outYf = fopen("/tmp/Yf.dat","a");
    outZf = fopen("/tmp/Zf.dat","a");
    for(int i=0;i<TLength;i++)
      {
	fprintf(outXq,"%d\t%f \n",ID,Xq[i]);
	fprintf(outYq,"%d\t%f \n",ID,Yq[i]);
	fprintf(outZq,"%d\t%f \n",ID,Zq[i]);
	fprintf(outXf,"%d\t%f \n",ID,Xf[i]);
	fprintf(outYf,"%d\t%f \n",ID,Yf[i]);
	fprintf(outZf,"%d\t%f \n",ID,Zf[i]);
      }
    fclose(outXq);
    fclose(outYq);
    fclose(outZq);
    fclose(outXf);
    fclose(outYf);
    fclose(outZf);

    /*Left Leg on ground (fixed), right leg swinging
         F:Flutuante(Swinging), A:Apoio(Fixed), C:Corpo(Origen), Q:Quartil(Hips)    */

    // Initialize the matrices
    for(int i=0;i<TLength;i++)
    {
        //Trajectory of Swinging leg (F:flutuante) wrt Fixed leg (A:Apoio)
        APF[0][i] = Xf[i]; APF[1][i] = Yf[i]; APF[2][i] = Zf[i];
        APF[3][i] = 0;     APF[4][i] = 0;     APF[5][i] = 0;

        //Trajectory of Hips (Q:quartil) wrt Fixed leg (A:apoio)
        //APQ[0][i] = Xc[i]; APQ[1][i] = L5+L6+L7+L8; APQ[2][i] = 0;
        APQ[0][i] = Xq[i]; APQ[1][i] = Yq[i]; APQ[2][i] = Zq[i];
        APQ[3][i] = 0;     APQ[4][i] = 0;     APQ[5][i] = 0;

        //Fixed leg (A:apoio) wrt Origin (C:corpo): initialize to 0, to be used for inv kin
        CPA[0][i] = 0; CPA[1][i] = 0; CPA[2][i] = 0;
        CPA[3][i] = 0; CPA[4][i] = 0; CPA[5][i] = 0;

        //Swinging leg (F:flutuante) wrt Origin (C:corpo): initialize to 0, to be used for inv kin
        CPF[0][i] = 0; CPF[1][i] = 0; CPF[2][i] = 0;
        CPF[3][i] = 0; CPF[4][i] = 0; CPF[5][i] = 0;
    }


    // Print
    FILE *outPoseFl, *outCPF, *outPoseH, *outCPA;
    outPoseFl = fopen("/tmp/outPoseFloating.dat","a");
    outPoseH  = fopen("/tmp/outPoseHips.dat","a");
    outCPF    = fopen("/tmp/CPF.dat","a");
    outCPA    = fopen("/tmp/CPA.dat","a");

    //Compute the Trajectory relations and the Inverse kinematics
    for (int i=0;i<TLength;i++)
    {
        //Hips(Q) wrt Origin(C): Forward kinematics
        if(ID==LEFT)            // Left leg is fixed (apoio)
            CPQ = Kinematics::rDenhar( M_PI_2, -L2,  L1, 0);
        else if(ID==RIGHT)      // Right leg is fixed (apoio)
           CPQ = Kinematics::rDenhar( M_PI_2, -L2, -L1, 0);

        //************* Motion of Hips wrt Fixed Leg **********************************************
        //CPA: Fixed leg (A) wrt Origin (C) - Desired pose in inv kin
        CPA[0][i] = CPQ.a[0][3] - APQ[0][i];
        CPA[1][i] = CPQ.a[1][3] - APQ[1][i];
        CPA[2][i] = CPQ.a[2][3] - APQ[2][i];
        double CPA_temp[6] = {CPA[0][i],CPA[1][i],CPA[2][i],CPA[3][i],CPA[4][i],CPA[5][i]};
        if(ID==LEFT)
            Kinematics::rInvKinematics(TargetAngleL, ObtainedPose, CPA_temp, ID);
        else if(ID==RIGHT)
            Kinematics::rInvKinematics(TargetAngleR, ObtainedPose, CPA_temp, ID);
        //printf("%d: Motion for hips precomputed\n\n", i);
	// Print
	fprintf(outPoseH,"%d\t%f\t%f\t%f\n",ID,ObtainedPose[0],ObtainedPose[1],ObtainedPose[2]);
	fprintf(outCPA,"%d\t%f\t%f\t%f\n",ID,CPA_temp[0],CPA_temp[1],CPA_temp[2]);

        //************* Motion of Floating leg wrt Fixed Leg ***************************************
        //CPF: Swinging leg (F) wrt Origin (C) - Desired pose in inv kin
        CPF[0][i] = CPA[0][i] + APF[0][i];
        CPF[1][i] = CPA[1][i] + APF[1][i];
        CPF[2][i] = CPA[2][i] + APF[2][i];
        double CPF_temp[6]={CPF[0][i],CPF[1][i],CPF[2][i],CPF[3][i],CPF[4][i],CPF[5][i]};
        if(ID==LEFT)
            Kinematics::rInvKinematics(TargetAngleR, ObtainedPose, CPF_temp, !ID);
        else if(ID==RIGHT)
            Kinematics::rInvKinematics(TargetAngleL, ObtainedPose, CPF_temp, !ID);
        //printf("%d:Motion for floating leg precomputed\n\n",i);

	// Print
	fprintf(outPoseFl,"%d\t%f\t%f\t%f\n",ID,ObtainedPose[0],ObtainedPose[1],ObtainedPose[2]);
	fprintf(outCPF,"%d\t%f\t%f\t%f\n",ID,CPF_temp[0],CPF_temp[1],CPF_temp[2]);

	for(int j=0;j<MAX_JOINTS;j++)
        {
            TotalAngleR[i][j] = TargetAngleR[j];    // Angles [rad] for right leg
            TotalAngleL[i][j] = TargetAngleL[j];    // Angles [rad] for left leg
        }
    }

    fclose(outPoseFl);
    fclose(outCPF);
    fclose(outPoseH);
    fclose(outCPA);

}


/******************** MOVE (RAISE) LEG (WITH LINEAR TRAJECTORY): IT FALLS *********************************/

/* Inputs: P1: First point      P2: End point
           to: Initial time     tf: Final time      TLength: Number of elements in time
           ID: RIGHT for right leg and LEFT for left leg

   Outputs: TotalAngle[][6]: Precomputed angles
*/

void Traj::rComputeSimpleLegMotion(double TotalAngle[][6], Math::TPoint3D P1, Math::TPoint3D P2,
                                   double to, double tf, int TLength, bool ID)
{
    double t[TLength], X[TLength], Y[TLength], Z[TLength];  // variables for time (t), X,Y,Z
    Math::rLinspace(t,to,tf,TLength);                       // time: linear space from to,tf with TLength of points

    Math::rOffLineCubicInterp(X, P1.x, P2.x, P1.vx, P2.vx, t, TLength);     // Interpolation for X
    Math::rOffLineCubicInterp(Y, P1.y, P2.y, P1.vy, P2.vy, t, TLength);     // Interpolation for Y
    Math::rOffLineCubicInterp(Z, P1.z, P2.z, P1.vz, P2.vz, t, TLength);     // Interpolation for Z

    double target_pose[6] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};                 // Variable for desired pose
    double target_angle[6] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
    double ObtainedPose[6];

    for(int i=0; i<TLength; i++)
    {
        target_pose[0]=X[i]; target_pose[1]=Y[i]; target_pose[2]=Z[i];      // Desired pose for the leg wrt origin (C)
        Kinematics::rInvKinematics(target_angle, ObtainedPose, target_pose, ID);
        //printf("Position z: %f\n", target_pose[2]);

        for(int j=0;j<MAX_JOINTS;j++)
        {   TotalAngle[i][j] = target_angle[j];                             // Assign angles to the output variable
        }
    }
}
