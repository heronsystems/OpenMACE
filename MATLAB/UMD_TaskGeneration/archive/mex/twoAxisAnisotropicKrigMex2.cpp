#include <mex.h> 
#include <matrix.h> 
#include <cmath> 
#include <math.h> 
#include <iostream>
#include "blas.h"

/* The gateway function */
void mexFunction( int nlhs, mxArray *plhs[],
                  int nrhs, const mxArray *prhs[])
{
    // Arguments (xpts,ypts,Cinv,measurements,ax,ay);
    //              0    1    2     3         4  5

    // process inputs
    double * xpts = mxGetPr(prhs[0]);
    double * ypts = mxGetPr(prhs[1]);
    double * Cinv = mxGetPr(prhs[2]); // (nm+1) x (nm+1) matrix
    double * measurements = mxGetPr(prhs[3]);
    double ax = mxGetScalar(prhs[4]);
    double ay = mxGetScalar(prhs[5]);

    // aux vars
    int nx = mxGetM(prhs[0]); // number of rows
    int ny = mxGetM(prhs[1]); // number of rows
    int nm = mxGetM(prhs[3]); // number of rows

    // display 
    std::cout << "ax :" << ax << std::endl;
    std::cout << "ay :" << ay << std::endl; 
    std::cout << "nx :" << nx << std::endl;
    std::cout << "ny :" << ny << std::endl; 
    std::cout << "nm :" << nm << std::endl;


    // allocate
    mxArray * B = mxCreateDoubleMatrix(nm+1, 1, mxREAL);
    double *pB = mxGetPr(B);

    mxArray * w = mxCreateDoubleMatrix(nm+1, 1, mxREAL);
    double *pw = mxGetPr(w);

    mxArray * forecast = mxCreateDoubleMatrix(nx, ny, mxREAL);
    double *pForecast = mxGetPr(forecast);

    // compute
    double distKernel20, distKernel10;
    double delx, dely;
    for ( int i = 0; i < nx; i++ ){
      for ( int j = 0; j < ny; j++ ){
        for ( int k = 0; k < nm; k++ ){
          // A[i][j] = A[i + j*nrows].
          delx =  xpts[i] - measurements[k + 0*nm];
          dely =  ypts[j] - measurements[k + 1*nm];
          distKernel20 = sqrt((delx/ax)*(delx/ax)+ (dely/ay)*(dely/ay)); 
          distKernel10 = sqrt((delx/ay)*(delx/ay)+ (dely/ax)*(dely/ax)); 
          pB[k] = exp(-distKernel10) + exp(-distKernel20);
        }  
        pB[nm] = 1;    
        // w = Cinv*B
        for ( int ii = 0; ii < nm+1; ii++ ){
            pw[ii] = 0;
            for ( int jj = 0; jj < nm+1; jj++ ){
             // printf("C[%d][%d]=%3.3f\n",ii,jj,Cinv[ii + jj*(nm+1)]);
              pw[ii] += Cinv[ii + jj*(nm+1)]*pB[jj];         
            }
        }
        pForecast[j + i*nx] = 0;
        for ( int k = 0; k < nm; k++ ){
          pForecast[j + i*nx]  += pw[k]*(measurements[k + 2*nm] - pw[nm]);      
        }
        pForecast[j + i*nx]  += pw[nm];

      }
    }
    // return
    plhs[0]= forecast;
   
}
