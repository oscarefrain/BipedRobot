/*================================================================================
    Functions for mathematical definitions of matrices and operations
    -----------------------------------------------------------------
     	Oscar E. Ramos Ponce
    	Estudante do Programa de Pós Graduação em Engenharia Elétrica
    	Universidade Federal do Rio Grande do Sul, Brasil 2011
==================================================================================*/


#include "../include/robFunctions.h"


/******************** MATRICES OF 4X4 *****************************************************/

//Product of 4x4 matrices of type TMatrix4
Math::TMatrix4 Math::operator*(TMatrix4 mat1,TMatrix4 mat2)
{
    TMatrix4 prod;
    for(int i=0;i<4;i++)
    {   for (int j=0;j<4;j++)
        {
            prod.a[i][j]=0;                                 // Initialize elements to 0
            for(int k=0;k<4;k++)
                prod.a[i][j] += mat1.a[i][k]*mat2.a[k][j];  // Multiply cols and rows
        }
    }
    return (prod);
}

Math::TMatrix4 Math::eye(double s)
{
  TMatrix4 res;
  for(int i=0;i<4;i++)
    {   
      for (int j=0;j<4;j++)
	{
          if (i==j)
	    res.a[i][j]=1;                                
	  else
	    res.a[i][j]=0;
        }
    }
  return (res);
}


/******************** MATRICES OF 6X6 ****************************************************/

//Product of 6x6 matrices of type TMatrix6
Math::TMatrix6 Math::operator*(TMatrix6 mat1,TMatrix6 mat2)
{
    TMatrix6 prod;
    for(int i=0;i<6;i++)
    {   for (int j=0;j<6;j++)
        {   prod.a[i][j]=0;                                 // Initialize elements to 0
            for(int k=0;k<6;k++)
                prod.a[i][j] += mat1.a[i][k]*mat2.a[k][j];  // Multiply cols and rows
        }
    }
    return (prod);
}

//Sum of 6x6 matrices of type TMatrix6
Math::TMatrix6 Math::operator+(TMatrix6 mat1,TMatrix6 mat2)
{
    TMatrix6 sum;
    for(int i=0;i<6;i++)
    {   for (int j=0;j<6;j++)
        {   sum.a[i][j] = mat1.a[i][j] + mat2.a[i][j]; }   // Sum element by element
    }
    return (sum);
}

//Inverse of a 6x6 matrix of type TMatrix6 using Gauss Jordan reduction
Math::TMatrix6 Math::inv(struct TMatrix6 matrix)
{
	TMatrix6 inverse;
	int sizemat = 6;
	double mat[sizemat][2*sizemat];                     // Augmented matrix
	double divisor, multiplicador;

    for(int i=0;i<sizemat;i++)
    {   for(int j=0;j<2*sizemat;j++)
        {
            if(j<sizemat) mat[i][j] = matrix.a[i][j];     // Copy Matrix
            else mat[i][j] = 0;                           // Initialize rest to 0
        }
    }

    for (int i=0; i<sizemat; i++)                         // Put some 1's to
    {   mat[i][sizemat+i] = 1.0;}                         // make augmented mtx as an identity matrix

	for (int i=0; i<sizemat; i++)  // For each row
	{
        divisor = mat[i][i];                            // Gets aii as pivot
        for (int j=0; j<2*sizemat; j++)                 // and divides the row by the pivot
        {   mat[i][j] = (mat[i][j]/divisor);}

		for(int f0=0; f0<sizemat; f0++)                 // Make elements in col = 0, except pivot
		{
			if (f0==i) continue;
            multiplicador = mat[f0][i]*(-1);
            for (int j=0; j<2*sizemat; j++)
                mat[f0][j] = mat[f0][j] + multiplicador*mat[i][j];
		}
	}
	for (int i=0; i<sizemat; i++)
	{	for (int j=0; j<sizemat; j++)
			inverse.a[i][j] = mat[i][j+sizemat];
	}
	return (inverse);
}

//Transpose of a 6x6 matrix of type TMatrix6
Math::TMatrix6 Math::trans(struct TMatrix6 matrix)
{
	TMatrix6 transpose;
    for (int i=0; i<6; i++)
	{	for (int j=0; j<6; j++)
			transpose.a[i][j] = matrix.a[j][i];
	}
	return (transpose);
}


/**************** MATHEMATICAL AID FUNCTIONS FOR TRAJECTORY GENERATION **************************/

//Function to generate a linear space between xi and xf using n points
//    Output: x_vector,
//    Input:  xi, xf, n
void Math::rLinspace(double x_vector[], double xi, double xf, int n)
{
    /* Example:
        double x_vect[40];
        Math::linspace(x_vect,0,2,40);
        for(int i=0;i<40;i++) printf(" %6.4f  ",x_vect[i]);
    */

    double inc = (xf-xi)/(n-1);
    x_vector[0] = xi;
    for(int i=1;i<n;i++)
    {   x_vector[i] = x_vector[i-1] + inc;
    }
}


//Function to generate a Cubic Interpolation from (xo,vo) to (x1,v1) in sampled time t[]
//  Output: X[]
//  Inputs: xo,x1,vo,v1,n,t[]
void Math::rOffLineCubicInterp(double X[], double xo, double x1, double vo, double v1, double t[], int n)
{
    /* Example:
         int T_NUM = 40;
         double t[T_NUM], X[T_NUM];
         rLinspace(t,0,2,T_NUM);        // time: linear space from 0 to 2 with T_NUM of points
         rOffLineCubicInterp(X, 0, 5, 0, 0, t, T_NUM);
         for(int i=0;i<T_NUM;i++)
         {   printf(" %6.4f  ",X[i]);}
    */

    double Ti;
    Ti = t[n-1] - t[0];
    for(int i=0;i<n;i++)
    {
        X[i] = xo + vo*t[i] + (3*(x1-xo) - Ti*(2*vo-v1)) / pow(Ti,2) * pow(t[i],2) +
               (2*(xo-x1) + Ti*(vo+v1)) / pow(Ti,3) * pow(t[i],3);
    }
}

void Math::rCross(double res[], double v1[], double v2[])
{
  res[0] = v1[1]*v2[2] - v1[2]*v2[1];
  res[1] = v1[2]*v2[0] - v1[0]*v2[2];
  res[2] = v1[0]*v2[1] - v1[1]*v2[0];
}




