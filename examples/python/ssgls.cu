

#include <cuda.h>
#include "_functorGLS.cu"


//! [data_acces]
__device__ int getValue(const int x,
                        const int y,
                        const int width,
                        const int height,
                        const int dim)
{
  return (dim*width*height) + (x + y * width);
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
  
    FunctorGLS<float3, double> gls;
  
    int dx, dy; // neighbor offset ids
    int nx, ny; // neighbor ids
    float dist2, w;   // squared neighbor distance to center and weight
    float s2 = float(scale*scale);
    
    int surfaceId = ids[getValue(x,y,width,height,0)];        
    
    float3 p, n;
    gls.init( make_float3( positions[getValue(x,y,width,height,0)],
                           positions[getValue(x,y,width,height,1)],
                           positions[getValue(x,y,width,height,2)]) );
                        
    int o = getValue(x,y,width,height,0);   
                           
    if ( normals[getValue(x,y,width,height,0)] == 0 &&
         normals[getValue(x,y,width,height,1)] == 0 &&
         normals[getValue(x,y,width,height,2)] == 0){
         
         result[o] = 0.0;
         
         }
    else{
    
    // collect neighborhood
      for(dy = -scale; dy != scale; dy++)
        for(dx = -scale; dx != scale; dx++){
          nx = x+dx;
          ny = y+dy;
          if (nx >= 0 && ny >= 0 && nx < width && ny < height){      
            dist2 = dx*dx + dy*dy;
             // Check if we are in the circular screen-space neighborhood
             // and on the same surface
             if (dist2 < s2 /*&& surfaceId == ids[getValue(x+dx,y+dy,width,height,0)]*/){
                n = make_float3(normals[getValue(nx,ny,width,height,0)],
                                normals[getValue(nx,ny,width,height,1)],
                                normals[getValue(nx,ny,width,height,2)]);
                                
                if ( length(n) != 0.f ) {         
                  w = dist2 / s2 - 1.0;
                  w *= w;
                  
                  //get Neighbor properties
                  p = make_float3(positions[getValue(nx,ny,width,height,0)],
                                  positions[getValue(nx,ny,width,height,1)],
                                  positions[getValue(nx,ny,width,height,2)]); 
                                  
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

