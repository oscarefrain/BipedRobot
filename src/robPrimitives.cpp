/*=============================================================================
    Funtions that crate and draw the robot primitive elements in the screen
    -----------------------------------------------------------------------
     	Oscar E. Ramos Ponce
    	Estudante do Programa de Pós Graduação em Engenharia Elétrica
    	Universidade Federal do Rio Grande do Sul, Brasil 2011
===============================================================================*/

#include "../include/robModel.h"


/******** CREATE A CAPSULE (CYLINDER WITH SPHERICAL BORDERS) ***************************************************************/

void Primitives::rCreateCapsule( const dWorldID &world, const dSpaceID &space, Model::RobPart *obj,
                                 const dReal posx, const dReal posy, const dReal posz,
                                 const dReal length, const dReal radius, const dReal angle,
                                 const dReal mass,
                                 const dReal colr, const dReal colg, const dReal colb,
                                 bool collision )
{
    // Create a rigid body in the "world"
    obj->body = dBodyCreate(world);

    // Assign mass
    dMass m;
    dMassSetZero(&m);              		            //Initialize mass parameters
    dMassSetCapsuleTotal(&m, mass, 3, radius, length);      //    on z axis (3=z) with radius and length
    dBodySetMass(obj->body,&m);                             //Set mass parameter to body

    // Position and orientation
    dBodySetPosition(obj->body, posx, posy, posz);          //Set a position (x,y,z) according to the COG
    dMatrix3 R;                                             //Rotation matrix
    dRFromAxisAndAngle(R, 1.0, 0.0, 0.0, angle*M_PI/180);   //Rot matrix R "angle"[deg] about x axis (90 to make it horizontal)
    dBodySetRotation(obj->body, R);                         //Set rotation

    // Geometry
    obj->geom = dCreateCapsule(space, radius, length);      //Creates a geometry for the body with radius and length
    dGeomSetBody(obj->geom, obj->body);                     //Attach geometry with body

    // Color and Enable/Disable Collision
    Primitives::rSetColor(obj->color, colr, colg, colb);    //Set a color for the cylinder
  	if (!collision) dGeomDisable(obj->geom);            //If collision==0, disable collision

}

/******** CREATE A BOX ****************************************************************************************/

void Primitives::rCreateBox( const dWorldID &world, const dSpaceID &space, Model::RobPart *obj,
                             const dReal posx, const dReal posy, const dReal posz,
                             const dReal lenx, const dReal leny, const dReal lenz,
                             const dReal mass,
                             const dReal colr, const dReal colg, const dReal colb,
                             bool collision )
{
    // Create a rigid body in the "world"
    obj->body = dBodyCreate(world);

    // Assign mass
    dMass m;
    dMassSetZero(&m);              		             //Initialize mass parameters
    dMassSetBoxTotal(&m, mass, lenx, leny, lenz);
    dBodySetMass(obj->body, &m);                             //Set mass parameter to body

    // Position
    dBodySetPosition(obj->body, posx, posy, posz);           //Set a position (x,y,z) according to the COG

    // Geometry
    obj->geom = dCreateBox(space, lenx, leny, lenz);         //Creates a geometry for the body with radius and length
    dGeomSetBody(obj->geom, obj->body);                      //Attach geometry with body

    // Color and Enable/Disable Collision
    Primitives::rSetColor(obj->color, colr, colg, colb);     //Set a color for the cylinder
  	if (!collision) dGeomDisable(obj->geom);             //If collision==0, disable collision
}

/******** CREATE A CYLINDER **************************************************************************************/

void Primitives::rCreateCylinder( const dWorldID &world, const dSpaceID &space, Model::RobPart *obj,
                                  const dReal posx, const dReal posy, const dReal posz,
                                  const dReal length, const dReal radius, const dReal angle,
                                  const dReal mass,
                                  const dReal colr, const dReal colg, const dReal colb,
                                  bool collision )
{
    // Create a rigid body in the "world"
    obj->body = dBodyCreate(world);

    // Assign mass
    dMass m;
    dMassSetZero(&m);              		            //Initialize mass parameters
    dMassSetCylinderTotal(&m, mass, 3, radius, length);     //    on z axis (3=z) with radius and length
    dBodySetMass(obj->body,&m);                             //Set mass parameter to body

    // Position and orientation
    dBodySetPosition(obj->body, posx, posy, posz);          //Set a position (x,y,z) according to the COG
    dMatrix3 R;                                             //Rotation matrix
    dRFromAxisAndAngle(R, 1.0, 0.0, 0.0, angle*M_PI/180);   //Rot matrix R "angle"[deg] about x axis (90 to make it horizontal)
    dBodySetRotation(obj->body, R);                         //Set rotation

    // Geometry
    obj->geom = dCreateCylinder(space, radius, length);     //Creates a geometry for the body with radius and length
    dGeomSetBody(obj->geom, obj->body);                     //Attach geometry with body

    // Color and Enable/Disable Collision
    Primitives::rSetColor(obj->color, colr, colg, colb);    //Set a color for the cylinder
  	if (!collision) dGeomDisable(obj->geom);            //If collision==0, disable collision
}



/******** CREATE THE JOINTS ******************************************************************************/

void Primitives::rCreateJoint( const dWorldID &world, dJointID *joint,
							   Model::RobPart *obj1, Model::RobPart *obj2,
                               const dReal posx, const dReal posy, const dReal posz,
                               const char *jaxis, const dReal L_ang, const dReal H_ang)
{
   	*joint = dJointCreateHinge (world,0);
    dJointAttach (*joint, obj1->body, obj2->body);

    dJointSetHingeAnchor(*joint, posx, posy, posz);              //Set a joint anchor (origin of coordinates for joint)

    int axis[3]={0, 0, 0};
    if     (!strcmp(jaxis,"+x")) axis[0] = 1;
    else if(!strcmp(jaxis,"+y")) axis[1] = 1;
    else if(!strcmp(jaxis,"+z")) axis[2] = 1;
    else if(!strcmp(jaxis,"-x")) axis[0] = -1;
    else if(!strcmp(jaxis,"-y")) axis[1] = -1;
    else if(!strcmp(jaxis,"-z")) axis[2] = -1;


    dJointSetHingeAxis(*joint, axis[0], axis[1], axis[2]);       //Set hinge axis (axis of rotation in world frame)

    dJointSetHingeParam(*joint, dParamLoStop, L_ang*M_PI/180);   //Lowest angle for hips joint in rad
    dJointSetHingeParam(*joint, dParamHiStop, H_ang*M_PI/180);   //Highest angle for hips joint in rad

}

/******** DRAW GEOMETRIES ***************************************************************************/

void Primitives::rDrawGeom (dGeomID g, dReal color[3])
{
    dsSetColor(color[0], color[1], color[2]);

    const dReal *pos = dGeomGetPosition(g);                     // Get the position of the element
    const dReal *R   = dGeomGetRotation(g);                     // Get the rotation of the element

    int type = dGeomGetClass(g);
    if (type == dBoxClass)                                      // Box
    {
        dVector3 sides;
        dGeomBoxGetLengths(g, sides);
        dsDrawBoxD(pos, R, sides);
    }
    else if (type == dSphereClass)                              // Sphere
    {
        dsDrawSphereD(pos, R, dGeomSphereGetRadius(g) );
    }
    else if (type == dCCylinderClass)                           // Capsule
    {
        dReal radius, length;
        dGeomCCylinderGetParams(g, &radius, &length);
        dsDrawCappedCylinder(pos, R, length, radius);
    }
    else if (type == dCylinderClass)                            // Cylinder
    {
        dReal radius,length;
        dGeomCylinderGetParams(g, &radius, &length);
        dsDrawCylinder(pos, R, length, radius);
  }

}


/******** AUXILIAR FUNCTIONS ************************************************************************/

void Primitives::rSetColor(dReal* col, const dReal R, const dReal G, const dReal B)
{
	col[0] = R;
	col[1] = G;
	col[2] = B;
}
