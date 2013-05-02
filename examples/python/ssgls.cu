

#include <cuda.h>
#include "_functorGLS.cu"


//__constant__ __device__ 
//float3 invalidNormal = make_float3(-1.0,-1.0,-1.0);

__device__ int convertTo1d(const int x,
                                 const int y,
                                 const int width,
                                 const int height,
                                 const int component,
                                 const int nbComponent)
{
  return (component) + nbComponent*(x + y * width);
}

//Warning: extern "C" needed when kernel is called by python, in order to avoid 
// name_mangling, that is incompatible with module.get_function ()
extern "C" { 
__global__ void doGLS_kernel(  
                               int* params, //[w, h, scale, nbQueries]
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
  
    FunctorGLS<float3, double> gls;
  
    int dx, dy; // neighbor offset ids
    int nx, ny; // neighbor ids
    float dist2;   // squared neighbor distance to center and weight
    float s2 = float(scale*scale);     
    
//    gls.init( make_float3( 2.0*positions[convertTo1d(x,y,width,height,0,3)]-1.0,
//                           2.0*positions[convertTo1d(x,y,width,height,1,3)]-1.0,
//                           2.0*positions[convertTo1d(x,y,width,height,2,3)]-1.0) );
    gls.init( make_float3( positions[convertTo1d(x,y,width,height,0,3)],
                           positions[convertTo1d(x,y,width,height,1,3)],
                           positions[convertTo1d(x,y,width,height,2,3)]) );

                               
    if ( normals[convertTo1d(x,y,width,height,0,3)] == 0 &&
         normals[convertTo1d(x,y,width,height,1,3)] == 0 &&
         normals[convertTo1d(x,y,width,height,2,3)] == 0){
         
         result[convertTo1d(x,y,width,height,0,1)] = 0.0;
         
         }
    else{


    for(dy = -scale; dy != scale; dy++)
      for(dx = -scale; dx != scale; dx++){
        nx = x+dx;
        ny = y+dy;
        if (nx >= 0 && ny >= 0 && nx < width && ny < height){      
          dist2 = dx*dx + dy*dy;
           // Check if we are in the circular screen-space neighborhood
           if (dist2 < s2){
               
              //get Neighbor properties
               float3 n = make_float3(normals[convertTo1d(nx,ny,width,height,0,3)],
                              normals[convertTo1d(nx,ny,width,height,1,3)],
                              normals[convertTo1d(nx,ny,width,height,2,3)]);
                              
              if ( length(n) != 0.f ) {  
              
               float w=dist2 / s2 - 1.0;
               w *= w;
                           
               float3 p = make_float3(positions[convertTo1d(nx,ny,width,height,0,3)],
                                      positions[convertTo1d(nx,ny,width,height,1,3)],
                                      positions[convertTo1d(nx,ny,width,height,2,3)]);                   
                gls.addNeighbor(p, normalize(2.0*n-1.0), w);
              }
           }
         }
      }
    }
    // closed form minimization
    gls.finalize();    
    result[convertTo1d(x,y,width,height,0,1)] = gls.kappa();
  }

}

}

