/*
 This Source Code Form is subject to the terms of the Mozilla Public
 License, v. 2.0. If a copy of the MPL was not distributed with this
 file, You can obtain one at http://mozilla.org/MPL/2.0/. 
*/


#include <cuda.h>
#include "_functorGLS.cu"


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
  : Grenaille::DistWeightFunc(t), _dz(dz) { }
  
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
typedef Basket<MyPoint,ProjecteWeightFunc,Grenaille::OrientedSphereFit, Grenaille::GLSParam> Gls;
//! [fit_def]


//! [data_acces]
__device__ int getId(const int x,
                        const int y,
                        const int width,
                        const int height,
                        const int dim)
{
  return (dim*width*height) + (x + y * width);
}
__device__ VectorType getVector(const int x,
                                const int y,
                                const int width,
                                const int height,
                                const unsigned char * buffer)
{
  VectorType r;
  r << buffer[getId(x,y,width,height,0)],
       buffer[getId(x,y,width,height,1)],
       buffer[getId(x,y,width,height,2)];
  return r;
}


//! [data_acces]


//! [kernel]
__global__ void doGLS_kernel(  const float* queries,
                               const int nbQueries,
                               const unsigned char *positions,
                               const unsigned char *normals,
                               const int *ids,
                               const int width,
                               const int height,
                               const int scale,
                               double* result)
{

  unsigned int ptid = blockIdx.x*blockDim.x + threadIdx.x;

  if (ptid < nbQueries)
  {
    // cast float coordinates
    int x = queries[2*ptid];
    int y = queries[2*ptid + 1];
  
  
    int dx, dy; // neighbor offset ids
    int nx, ny; // neighbor ids
    float dist2, w;   // squared neighbor distance to center and weight
    float s2 = float(scale*scale);
    
    int surfaceId = ids[getId(x,y,width,height,0)];
    
    Gls gls;
    gls.setWeightFunc(ProjectWeightFunc(tmax));
    gls.init( getVector(x,y,width,height,positions) );
                        
    int o = getId(x,y,width,height,0);   
                           
    if ( getVector(x,y,width,height,normals) == VectorType::Zero() ){         
         result[o] = 0.0;         
    }
    else{
      VectorType p, n;
    
    // collect neighborhood
      for(dy = -scale; dy != scale; dy++)
        for(dx = -scale; dx != scale; dx++){
          nx = x+dx;
          ny = y+dy;
          if (nx >= 0 && ny >= 0 && nx < width && ny < height){    
            MyPoint query;
            
            dist2 = dx*dx + dy*dy;
             // Check if we are in the circular screen-space neighborhood
             // and on the same surface
             if (dist2 < s2 /*&& surfaceId == ids[getId(x+dx,y+dy,width,height,0)]*/){
                n = getVector(nx,ny,width,height,normals);
                              
                              //todo: FINISH !!  
                if ( length(n) != 0.f ) {         
                  w = dist2 / s2 - 1.0;
                  w *= w;
                  
                  //get Neighbor properties
                  p = make_float3(positions[getId(nx,ny,width,height,0)],
                                  positions[getId(nx,ny,width,height,1)],
                                  positions[getId(nx,ny,width,height,2)]); 
                                  
                  gls.addNeighbor(p, normalize(2.0*n - 1.0), w);
                }
             }
          }
       }
    }

    // closed form minimization
    gls.finalize();
    result[o] = gls.kappa();
  }

}
//! [kernel]

