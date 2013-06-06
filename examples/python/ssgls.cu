/*
 This Source Code Form is subject to the terms of the Mozilla Public
 License, v. 2.0. If a copy of the MPL was not distributed with this
 file, You can obtain one at http://mozilla.org/MPL/2.0/. 
 
 Compile using 
 nvcc --ptx -I ../../Patate/ -I /path/to/eigen-nvcc/ ssgls.cu
*/


#include <cuda.h>
#include <algorithm>

#include "Eigen/Core"
#include "grenaille.h"


//! [mypoint]
class MyPoint
{
  public:
  enum {Dim = 3};
  typedef float Scalar;
  typedef Eigen::Matrix<Scalar, Dim, 1> VectorType;
  typedef Eigen::Matrix<Scalar, 2, 1>   ScreenVectorType;

  MULTIARCH inline MyPoint(const VectorType &pos        = VectorType::Zero(), 
                           const VectorType& normal     = VectorType::Zero(),
                           const ScreenVectorType& spos = ScreenVectorType::Zero(), 
                           const Scalar dz = 0.f)
  : _pos(pos), _normal(normal), _spos(spos), _dz(dz){}

  MULTIARCH inline const VectorType& pos()	const { return _pos; }  
  MULTIARCH inline const VectorType& normal()	const { return _normal; }
  MULTIARCH inline const ScreenVectorType& spos() const { return _spos; } 
  MULTIARCH inline const float & dz()	const { return _dz; }  


  MULTIARCH inline VectorType& pos()	 { return _pos; }  
  MULTIARCH inline VectorType& normal()	 { return _normal; }
  MULTIARCH inline ScreenVectorType& spos() { return _spos; }  
  MULTIARCH inline float& dz()	 { return _dz; }  


  private:
    ScreenVectorType _spos;
    VectorType	_pos, _normal;
    float _dz; // depth threshold
};
//! [mypoint]

//! [w_def]
typedef MyPoint::Scalar Scalar;
typedef MyPoint::VectorType VectorType;
typedef MyPoint::ScreenVectorType ScreenVectorType;

class ProjectWeightFunc: public Grenaille::DistWeightFunc<MyPoint, Grenaille::SmoothWeightKernel<Scalar> >{
public:
  typedef MyPoint::Scalar Scalar;
  typedef MyPoint::VectorType VectorType;

  MULTIARCH inline ProjectWeightFunc(const Scalar& t = Scalar(1.), const float dz = 0.f)
  : Grenaille::DistWeightFunc<MyPoint, Grenaille::SmoothWeightKernel<Scalar> >(t), _dz(dz) { }
  
  MULTIARCH inline Scalar w(const VectorType& q, const MyPoint&  attributes) const
  {
    Scalar d  = attributes.spos().norm();
    const float dz = attributes.dz();
    if (d > _t || dz > _dz)
      return Scalar(0.);
    return _wk.f(d/_t);
  }
  private:
  float _dz;
};
//! [w_def]

//! [fit_def]
typedef Grenaille::Basket<MyPoint,ProjectWeightFunc,Grenaille::OrientedSphereFit, Grenaille::GLSParam> Gls;
//! [fit_def]


//! [data_acces]
__device__ int getId(const int x,
                       const int y,
                       const int width,
                       const int height,
                       const int component,
                       const int nbComponent)
{
  return (component) + nbComponent*(x + y * width);
}

__device__ VectorType getVector(const int x,
                                const int y,
                                const int width,
                                const int height,
                                const float * buffer)
{
  VectorType r;
  r << Scalar(buffer[getId(x,y,width,height,0,3)]),
       Scalar(buffer[getId(x,y,width,height,1,3)]),
       Scalar(buffer[getId(x,y,width,height,2,3)]);
  return r;
}


//! [data_acces]


//! [kernel]
extern "C" { 
__global__ void doGLS_kernel(  int* params, //[w, h, scale, nbQueries]
                               float* queries,
                               float *positions,
                               float *normals,
                               float* result)
{

  unsigned int ptid = blockIdx.x*blockDim.x + threadIdx.x;

  if (ptid < params[3])
  {
    const int &width     = params[0];
    const int &height    = params[1];
    const int &scale     = params[2];
    
    // cast float coordinates
    int x = queries[2*ptid];
    int y = queries[2*ptid + 1];  
  
    int dx, dy; // neighbor offset ids
    int nx, ny; // neighbor ids
    
    
    Gls gls;
    gls.setWeightFunc(ProjectWeightFunc(scale));
    gls.init( getVector(x,y,width,height,positions) );
                                                   
    if ( getVector(x,y,width,height,normals).squaredNorm() != 0.f ){         
         result[getId(x,y,width,height,0,1)] = 0.0;         
    }
    else{
      VectorType p, n;
    
    // collect neighborhood
      for(dy = -scale; dy != scale; dy++)
        for(dx = -scale; dx != scale; dx++){
          nx = x+dx;
          ny = y+dy;
          
          // Check image boundaries
          if (nx >= 0 && ny >= 0 && nx < width && ny < height){    
             VectorType query;

             n = getVector(nx,ny,width,height,normals);
                          
             // add nei only when the normal is properly defined
             // this condition could also be included in the Weight functor
             if ( n.squaredNorm() != 0.f ) {  
                p = getVector(nx,ny,width,height,positions); 
                
                n = 2.f * n - VectorType::Ones();
                      
                n.normalize();

                gls.addNeighbor(MyPoint(p,n,ScreenVectorType(nx,ny)));                                  
            }
          }
       }
    }

    // closed form minimization
    gls.finalize();
    result[getId(x,y,width,height,0,1)] = gls.kappa();
  }

}

}
//! [kernel]

