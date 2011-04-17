/*=========================================================================
    Header for Robot Kinematics and Trajectory Planning
    ---------------------------------------------------
     	Oscar E. Ramos Ponce
    	Estudante do Programa de Pós Graduação em Engenharia Elétrica
    	Universidade Federal do Rio Grande do Sul, Brasil 2011
==========================================================================*/


#include <math.h>

#ifndef _ROBFUNCTIONS_H_
#define _ROBFUNCTIONS_H_

#define DELTA (0.00001f)
//TLENG will be the number of "iterations" that the program will have
#define TLENG 20
#define NINIT_T 4
#define NSTEP_T 4
#define NFINAL_T 4

#define LEFT  0
#define RIGHT 1


/*======= Functions for Matrices: defined in robMathAid.cpp ====================*/

namespace Math
{
  //Structure for 4x4 matrices (Homogeneous Transformation Matrices)
  //   Only the * operator is overloaded for this structure
  struct TMatrix4
  {   double a[4][4];
  };
  TMatrix4 operator*(TMatrix4 mat1,TMatrix4 mat2);
  TMatrix4 eye(double s);

  // Structure for 6x6 matrices. For this structure:
  //   the operators * + are overloaded, the inverse and transpose are defined
  struct TMatrix6
  {   double a[6][6];
  };
  TMatrix6 inv(struct TMatrix6 matrix);
  TMatrix6 trans(struct TMatrix6 matrix);
  TMatrix6 operator*(TMatrix6 mat1,TMatrix6 mat2);
  TMatrix6 operator+(TMatrix6 mat1,TMatrix6 mat2);

  // Function to create a linear space
  void rLinspace(double x_vector[], double xi, double xf, int n);
  // Function to create a cubic interpolation
  void rOffLineCubicInterp(double X[], double xo, double x1, double vo, double v1, double t[], int n);
  // Function to perform cross product
  void rCross(double res[], double v1[], double v2[]);

  struct TPoint3D
  { 
    double x;
    double y;
    double z;
    double vx;
    double vy;
    double vz;
  };

}

/*============== Defined in robKinematics.cpp =================================*/

namespace Kinematics
{
  int rInvKinematics(double theta_target[6],double current_pose[6], double target_pose[6], bool ID);
  void rFwdKinematics(double theta[6], bool LR);

  Math::TMatrix6 rJacobian(double pos_ori[6], double theta[6], bool ID);
  Math::TMatrix4 rDenhar(double th, double d, double a, double alpha);

  void rRPY(double rpy[3], struct Math::TMatrix4 T);
  void rLimitAngle(double ang[6]);
  bool rCheckWorkspace(double Pose[6], bool ID);

}

/*============== Defined in robSteps.cpp =================================*/


//Structure to store the 3D coordinates of the trajectories of Hips or Legs
namespace Traj
{
    void rComputeSimpleLegMotion(double TotalAngle[][6], Math::TPoint3D P1, Math::TPoint3D P2,
                                 double to, double tf, int T_NUM, bool ID);

    void rPreComputeBasicTrajectory(double TotalAngleL[][6], double TotalAngleR[][6],
                                    double InitialAngleL[6], double InitialAngleR[6],
                                    Math::TPoint3D Pq1, Math::TPoint3D Pq2,
                                    Math::TPoint3D Pf1, Math::TPoint3D Pf2,
                                    double to, double tf, int TLength, bool ID);

    void rPreComputeInitialStep(double TotalAngleL[][6], double TotalAngleR[][6],
                                double InitialAngleL[6], double InitialAngleR[6],
                                double StepLen, bool FixedLeg);

    void rPreComputeNormalStep(double TotalAngleL[][6], double TotalAngleR[][6],
                               double InitialAngleL[6], double InitialAngleR[6],
                               double StepLen, bool FixedLeg);

    void rPreComputeFinalStep(double TotalAngleL[][6], double TotalAngleR[][6],
                              double InitialAngleL[6], double InitialAngleR[6],
                              double StepLen, bool FixedLeg);

}


#endif
