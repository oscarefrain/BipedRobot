/*=============================================================================
    Funtions to create and draw the robot on the screen
    ----------------------------------------------------
       Oscar Efrain Ramos Ponce
       Estudante do Programa de Pós Graduação em Engenharia Elétrica
       Universidade Federal do Rio Grande do Sul, Brasil 2011
===============================================================================*/


#include <ode/ode.h>
#include <drawstuff/drawstuff.h>
#include "../include/robModel.h"

// Links for the model
Model::RobPart LinkR[MAX_LINK_LEG];       // Links for Right Leg
Model::RobPart LinkL[MAX_LINK_LEG];       // Links for Left Leg
Model::RobPart footR;                     // Link for Right foot
Model::RobPart footL;                     // Link for Left foot
Model::RobPart hips2legs;                 // Link for the hips
Model::RobPart body;                      // Link for the upper body


void Model::rCreateRobot(const dWorldID &world, const dSpaceID &space, dJointID *jointR, dJointID *jointL)
{
    Model::rCreateParts(world, space);
    Model::rCreateJoints(world, jointR, jointL);
}


/**************************************************************************************************
 **************** CREATE THE PARTS OF THE ROBOT ***************************************************
 **************************************************************************************************/

void Model::rCreateParts(const dWorldID &world, const dSpaceID &space)
{
    //--------- Create Body -----------------------------------------------------------
    Primitives::rCreateBox(world, space, &body, 0, 0, L8+L7+L6+L5+L4+L3+L2,
                           2*(r+0.02), 2*L1+2*r, 2*L2, MBODY, COLOR_BLUE, COLLIS);

    //--------- Create Hips -----------------------------------------------------------
    Primitives::rCreateCapsule(world, space, &hips2legs, 0.0, 0.0, L8+L7+L6+L5+L4+L3,
                               L2HIPS+r, r/3, 90, M2HIPS, COLOR_BLUE, COLLIS);

    //--------- Create Legs -----------------------------------------------------------
    dReal mass[MAX_LINK_LEG]   = {M3, M4, M5, M6, M7};                  // Mass of the leg links
    dReal length[MAX_LINK_LEG] = {L3, L4, L5, L6, L7};                  // Lengths of leg links
    dReal xg[MAX_LINK_LEG]     = {0.0, 0.0, 0.0, 0.0, 0.0};             // COG: Center of Gravity for legs
    dReal yg[MAX_LINK_LEG]     = {-L1, -L1, -L1, -L1, -L1};             //      for Right leg (for left leg, it should be +)
    dReal zg[MAX_LINK_LEG]     = {L8+L7+L6+L5+L4+L3/2, L8+L7+L6+L5+L4/2,
                                  L8+L7+L6+L5/2, L8+L7+L6/2, L8+L7/2};
    dReal leg_color[5][3] = {{COLOR_YELLOW},{COLOR_ORANGE},{COLOR_GREEN},{COLOR_BLUE},{COLOR_ORANGE}};
    //dReal leg_color[5][3] = {COLOR_YELLOW,COLOR_ORANGE,COLOR_GREEN,COLOR_BLUE,COLOR_ORANGE};

    for(int i=0; i<MAX_LINK_LEG; i++)
    {
        if(i==MAX_LINK_LEG-1 || i==0 || i==1)
        {
            // Right leg
            Primitives::rCreateCylinder(world, space, &LinkR[i], xg[i], yg[i], zg[i],
                                        length[i], r, 0, mass[i], leg_color[i][0], leg_color[i][1], leg_color[i][2], COLLIS);
            // Left leg
            Primitives::rCreateCylinder(world, space, &LinkL[i], xg[i], -yg[i], zg[i],
                                        length[i], r, 0, mass[i], leg_color[i][0], leg_color[i][1], leg_color[i][2], COLLIS);
        }
        else
        {
            // Right leg
            Primitives::rCreateCapsule(world, space, &LinkR[i], xg[i], yg[i], zg[i],
                                       length[i], r, 0, mass[i], leg_color[i][0], leg_color[i][1], leg_color[i][2], COLLIS);
            // Left leg
            Primitives::rCreateCapsule(world, space, &LinkL[i], xg[i], -yg[i], zg[i],
                                       length[i], r, 0, mass[i], leg_color[i][0], leg_color[i][1], leg_color[i][2], COLLIS);
        }
    }

    //--------- Create Feet -----------------------------------------------------------
    dReal cogFoot[3] = {FOOT_L/10, -L1, L8/2};      // COG: Center of Gravity for right feet (for left, + in Y)
    dReal sidesF[3]  = {FOOT_L,FOOT_W,L8};          // Sides of foot
    // Rigth foot
    Primitives::rCreateBox(world, space, &footR, cogFoot[0], cogFoot[1], cogFoot[2],
                           sidesF[0], sidesF[1], sidesF[2], MFOOT, COLOR_BLUE, COLLIS);
    // Left foot
    Primitives::rCreateBox(world, space, &footL, cogFoot[0], -cogFoot[1], cogFoot[2],
                           sidesF[0], sidesF[1], sidesF[2], MFOOT, COLOR_BLUE, COLLIS);
}


/****************************************************************************************************
 **************** CREATE THE JOINTS *****************************************************************
 ****************************************************************************************************/

void Model::rCreateJoints(const dWorldID &world, dJointID *jointR, dJointID *jointL)
{
    dReal x_anchor[MAX_JOINTS] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};        // Anchors for joints
    dReal y_anchor[MAX_JOINTS] = {-L1, -L1, -L1, -L1, -L1, -L1};        // (associated with the right leg)
    dReal z_anchor[MAX_JOINTS] = {L8+L7+L6+L5+L4+L3, L8+L7+L6+L5+L4,
                                  L8+L7+L6+L5, L8+L7+L6, L8+L7, L8};

    // Joint Body to Hips
    dJointID jointfixed;
    Primitives::rCreateJoint( world, &jointfixed, &hips2legs, &body, 0, 0, L3+L4+L5+L6+L7+L8,
                              "+z", LOW_ANG, HI_ANG);

    // Joint Hips to Right leg
    Primitives::rCreateJoint( world, &jointR[0], &LinkR[0], &hips2legs, x_anchor[0], y_anchor[0], z_anchor[0],
                              "+z", LOW_ANG, HI_ANG);

    // Joint Hips to Left leg
    Primitives::rCreateJoint( world, &jointL[0], &LinkL[0], &hips2legs, x_anchor[0], -y_anchor[0], z_anchor[0],
                              "+z", LOW_ANG, HI_ANG);

    // Joints 1-4
    for(int i=1;i<(MAX_JOINTS-1);i++)
    {
        char axis[6][3] = {"+z","+x","-y","-y","-y","+x"};
        if(i==3){
            Primitives::rCreateJoint( world, &jointR[i], &LinkR[i], &LinkR[i-1], x_anchor[i], y_anchor[i], z_anchor[i],
                                      axis[i], LOW_ANG_KNEE, HI_ANG_KNEE);
            Primitives::rCreateJoint( world, &jointL[i], &LinkL[i], &LinkL[i-1], x_anchor[i], -y_anchor[i], z_anchor[i],
                                      axis[i], LOW_ANG_KNEE, HI_ANG_KNEE);
        }
        else{
            Primitives::rCreateJoint( world, &jointR[i], &LinkR[i], &LinkR[i-1], x_anchor[i], y_anchor[i], z_anchor[i],
                                      axis[i], LOW_ANG, HI_ANG);
            Primitives::rCreateJoint( world, &jointL[i], &LinkL[i], &LinkL[i-1], x_anchor[i], -y_anchor[i], z_anchor[i],
                                      axis[i], LOW_ANG, HI_ANG);
        }
    }

    // Joint for Right foot
    Primitives::rCreateJoint( world, &jointR[MAX_JOINTS-1], &footR, &LinkR[MAX_LINK_LEG-1],
                              x_anchor[MAX_JOINTS-1], y_anchor[MAX_JOINTS-1], z_anchor[MAX_JOINTS-1],
                              "+x", LOW_ANG, HI_ANG);

    // Joint for Left foot
    Primitives::rCreateJoint( world, &jointL[MAX_JOINTS-1], &footL, &LinkL[MAX_LINK_LEG-1],
                              x_anchor[MAX_JOINTS-1], -y_anchor[MAX_JOINTS-1], z_anchor[MAX_JOINTS-1],
                              "+x", LOW_ANG, HI_ANG);

}



/***************************************************************************************************************
 ********************* DRAW THE ROBOT: using the given position and orientation of links ***********************
 ***************************************************************************************************************/

void Model::rDrawRobot()
{
    // Right and Left Feet
	Primitives::rDrawGeom(footR.geom, footR.color);
    Primitives::rDrawGeom(footL.geom, footL.color);

    // Right and Left leg
    for(int i=0; i<MAX_LINK_LEG; i++)
    {
        Primitives::rDrawGeom(LinkR[i].geom, LinkR[i].color);
        Primitives::rDrawGeom(LinkL[i].geom, LinkL[i].color);
    }

    // Hips
    Primitives::rDrawGeom(hips2legs.geom, hips2legs.color);

    // Body
    Primitives::rDrawGeom(body.geom, body.color);
}
