/*=========================================================================
    Header for the Robot Model
    --------------------------
     	Oscar E. Ramos Ponce
    	Estudante do Programa de Pós Graduação em Engenharia Elétrica
    	Universidade Federal do Rio Grande do Sul, Brasil 2011
=========================================================================*/


#ifndef _ROBMODEL_H_
#define _ROBMODEL_H_

#include <ode/ode.h>
#include <drawstuff/drawstuff.h>


/** ***************** MEASUREMENTS *************************************** */

// Lengths [m]
#define L1 (0.12f)
#define L2HIPS (2*L1)
#define L2 (0.15f)
#define L3 (0.03f)
#define L4 (0.03f)
#define L5 (0.45f)
#define L6 (0.45f)
#define L7 (0.03f)
#define L8 (0.07f)
#define FOOT_W (0.20f)
#define FOOT_L (0.35f)
#define r (0.1f)

// Masses of links and feet [kg]
#define MBODY  (0.1f)
#define M2HIPS (0.1f)
#define M3 (0.5f)
#define M4 (0.5f)
#define M5 (7.0f)
#define M6 (7.0f)
#define M7 (0.5f)
#define MFOOT (0.15f)

/** *********************************************************************** */

// Limits of joint's angles
#define LOW_ANG (-90.0f)
#define HI_ANG  (90.0f)
#define LOW_ANG_KNEE (-135.0f)
#define HI_ANG_KNEE  (0.01f)

// 5 Links, 6 DOF (joints)
#define MAX_LINK_LEG 5
#define MAX_JOINTS (MAX_LINK_LEG+1)
#define DOF 6

// Necessary for double precission
#ifdef dDOUBLE
#define dsDrawBox      dsDrawBoxD
#define dsDrawSphere   dsDrawSphereD
#define dsDrawCylinder dsDrawCylinderD
#define dsDrawCapsule  dsDrawCapsuleD
#define dsDrawLine     dsDrawLineD
#endif

// Definition of colors
#define COLOR_WHITE 1.0,1.0,1.0
#define COLOR_BLACK 0.3,0.3,0.3
#define COLOR_YELLOW 1.0,1.0,0.0
#define COLOR_BLUE 0.0,0.0,1.0
#define COLOR_GREEN 0.0,0.8,0.0
#define COLOR_ORANGE 1.0, 0.5, 0.0
#define COLOR_RED 1.0, 0.0, 0.0

// Definition for collision
#define COLLIS 1
#define NO_COL 0

/** *********************************************************************** */
 namespace Model
{
    // Structure to define an object
    typedef struct
    {
        dBodyID body;       // For dynamics: Body
        dGeomID geom;       // For Collision Detection: Geometry
        dReal   color[3];   // Color of the part
    }RobPart;

    // Defined in robModel.cpp
    void rCreateRobot(const dWorldID &world, const dSpaceID &space, dJointID *jointR, dJointID *jointL);
    void rCreateParts(const dWorldID &world, const dSpaceID &space);
    void rCreateJoints(const dWorldID &world, dJointID *jointR, dJointID *jointL);
    void rDrawRobot();
}

namespace Primitives
{
    // Defined in robPrimitives.cpp
    void rCreateCapsule( const dWorldID &world, const dSpaceID &space, Model::RobPart *obj,
                           const dReal posx, const dReal posy, const dReal posz,
                           const dReal length, const dReal radius, const dReal angle,
                           const dReal mass,
                           const dReal colr, const dReal colg, const dReal colb,
                           bool collision );

    void rCreateCylinder( const dWorldID &world, const dSpaceID &space, Model::RobPart *obj,
                            const dReal posx, const dReal posy, const dReal posz,
                            const dReal length, const dReal radius, const dReal angle,
                            const dReal mass,
                            const dReal colr, const dReal colg, const dReal colb,
                            bool collision );

    void rCreateBox( const dWorldID &world, const dSpaceID &space, Model::RobPart *obj,
                       const dReal posx, const dReal posy, const dReal posz,
                       const dReal lenx, const dReal leny, const dReal lenz,
                       const dReal mass,
                       const dReal colr, const dReal colg, const dReal colb,
                       bool collision );

    void rCreateJoint( const dWorldID &world, dJointID *joint,
						 Model::RobPart *obj1, Model::RobPart *obj2,
						 const dReal posx, const dReal posy, const dReal posz,
                         const char *jaxis, const dReal L_ang, const dReal H_ang);

    void rDrawGeom(dGeomID g, dReal color[3]);

    void rSetColor(dReal* col, const dReal R, const dReal G, const dReal B);
}

#endif
