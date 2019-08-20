#include <mex.h> 
#include <iostream>

/* The gateway function */
void mexFunction( int nlhs, mxArray *plhs[],
                  int nrhs, const mxArray *prhs[])
{
   // Arguments (xpts,ypts,Cinv,measurements,ax,ay);
   //              0    1    2     3         4  5

    double ax = mxGetScalar(prhs[4]);
    double ay = mxGetScalar(prhs[5]);
   
    int nX = mxGetM(prhs[0]); // number of rows
    int nY = mxGetM(prhs[1]); // number of rows
    int nM = mxGetM(prhs[3]); // number of rows

    for ( int i = 0; i < nx; i++ ){
      for ( int j = 0; j < ny; i++ ){
      
      }
    }

    std::cout << "ax :" << ax << std::endl;
    std::cout << "ay :" << ay << std::endl;    
}
