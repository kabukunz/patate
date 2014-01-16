/*
This Source Code Form is subject to the terms of the Mozilla Public
License, v. 2.0. If a copy of the MPL was not distributed with this
file, You can obtain one at http://mozilla.org/MPL/2.0/. 
*/

/*!
\file examples/Grenaille/ssgls.cu
\brief Screen space GLS using c++/CUDA

\author: Nicolas Mellado, Gautier Ciaudo
*/

#include <stdio.h>
#include <stdlib.h>
#include <iostream>
#include <cmath>
#include <algorithm>
#include <FreeImagePlus.h>
#include <vector>

#include "Eigen/Core"
#include "Patate/grenaille.h"

#define CUDA_CHECK_RETURN(_value) {											\
    cudaError_t _m_cudaStat = _value;										\
    if (_m_cudaStat != cudaSuccess) {										\
        fprintf(stderr, "Error %s at line %d in file %s\n",					\
        cudaGetErrorString(_m_cudaStat), __LINE__, __FILE__);		        \
        exit(1);															\
    } }

//! [mypoint]
class MyPoint
{
public:
    enum {Dim = 3};
    typedef float Scalar;
    typedef Eigen::Matrix<Scalar, Dim, 1> VectorType;
    typedef Eigen::Matrix<Scalar, Dim, Dim> MatrixType;
    typedef Eigen::Matrix<Scalar, 2, 1>   ScreenVectorType;

    MULTIARCH inline MyPoint(   const VectorType& _pos        = VectorType::Zero(),
                                const VectorType& _normal     = VectorType::Zero(),
                                const ScreenVectorType& _spos = ScreenVectorType::Zero(),
                                const Scalar _dz = 0.f)
        : m_pos(_pos), m_normal(_normal), m_spos(_spos), m_dz(_dz){}

    MULTIARCH inline const VectorType& pos()	const { return m_pos; }
    MULTIARCH inline const VectorType& normal()	const { return m_normal; }
    MULTIARCH inline const ScreenVectorType& spos() const { return m_spos; }
    MULTIARCH inline const float & dz()	const { return m_dz; }


    MULTIARCH inline VectorType& pos()	 { return m_pos; }
    MULTIARCH inline VectorType& normal()	 { return m_normal; }
    MULTIARCH inline ScreenVectorType& spos() { return m_spos; }
    MULTIARCH inline float& dz()	 { return m_dz; }


private:
    ScreenVectorType m_spos;
    VectorType	m_pos, m_normal;
    float m_dz; // depth threshold
};
//! [mypoint]

typedef MyPoint::Scalar Scalar;
typedef MyPoint::VectorType VectorType;
typedef MyPoint::ScreenVectorType ScreenVectorType;

//! [w_def]
class ProjectWeightFunc: public Grenaille::DistWeightFunc<MyPoint, Grenaille::SmoothWeightKernel<Scalar> >
{
public:
    typedef MyPoint::Scalar Scalar;
    typedef MyPoint::VectorType VectorType;

    /*
    Default constructor (needed by Grenaille). Note that the screenspace
    evaluation position is specified as parameter
    */
    MULTIARCH inline ProjectWeightFunc( const Scalar& _t                = 1.f,
                                        const ScreenVectorType& _refPos = ScreenVectorType::Zero(),
                                        const Scalar& _dz               = 0.f)
        : Grenaille::DistWeightFunc<MyPoint, Grenaille::SmoothWeightKernel<Scalar> >(_t), m_refPos(_refPos), m_dz(_dz) {}

    MULTIARCH inline Scalar w(const VectorType& _q, const MyPoint&  _attributes) const
    {
        Scalar d  = (_attributes.spos()-m_refPos).norm();
        const Scalar dz = _attributes.dz();

        if (d > m_t || dz > m_dz)
            return Scalar(0.);

        return m_wk.f(d/m_t);
    }
private:
    ScreenVectorType m_refPos;
    float m_dz;
};
//! [w_def]

//! [fit_def]
typedef Grenaille::Basket<MyPoint,ProjectWeightFunc,Grenaille::OrientedSphereFit, Grenaille::GLSParam> Gls;
//! [fit_def]

//! [data_acces]
__device__ int getId(const int _x,
                     const int _y,
                     const int _width,
                     const int _height,
                     const int _component,
                     const int _nbComponent)
{
    return (_component) + _nbComponent*(_x + _y * _width);
}

__device__ VectorType getVector(const int _x,
                                const int _y,
                                const int _width,
                                const int _height,
                                const float* _buffer)
{
    VectorType r;
    r << Scalar(_buffer[getId(_x,_y,_width,_height,0,3)]),
        Scalar(_buffer[getId(_x,_y,_width,_height,1,3)]),
        Scalar(_buffer[getId(_x,_y,_width,_height,2,3)]);
    return r;
}


//! [data_acces]


//! [kernel]
__global__ void doGLS_kernel(int* _params, //[w, h, scale, _nbQueries]
                             float* _queries,
                             float* _positions,
                             float* _normals,
                             float* _result)
{

    unsigned int ptid = blockIdx.x*blockDim.x + threadIdx.x;

    if (ptid < _params[3])
    {
        const int &width     = _params[0];
        const int &height    = _params[1];
        const int &scale     = _params[2];

        // cast float coordinates
        int x = _queries[2*ptid];
        int y = _queries[2*ptid + 1];

        ScreenVectorType refPos;
        refPos << x, y;


        int dx, dy; // neighbor offset ids
        int nx, ny; // neighbor ids

        Gls gls;
        gls.setWeightFunc(ProjectWeightFunc(scale, refPos));
        gls.init( getVector(x,y,width,height,_positions) );

        if (getVector(x,y,width,height,_normals).squaredNorm() == 0.f )
        {
            _result[getId(x,y,width,height,0,1)] = -1.0;
        }
        else
        {
            //_result[getId(x,y,width,height,0,1)] = getVector(x,y,width,height,_normals)(0);
            VectorType p, n;

            // collect neighborhood
            VectorType one = VectorType::Zero();

            for(dy = -scale; dy != scale; dy++)
            {
                for(dx = -scale; dx != scale; dx++)
                {
                    nx = x+dx;
                    ny = y+dy;


                    // Check image boundaries
                    if (nx >= 0 && ny >= 0 && nx < width && ny < height)
                    {
                        n = getVector(nx,ny,width,height,_normals);

                        // add nei only when the _normal is properly defined
                        // need to use an explicit floating point comparison with pycuda
                        if (n.squaredNorm() != 0.f )
                        {

                            // RGB to XYZ remapping
                            n =  2.f * n - one;
                            n.normalize();

                            // GLS computation
                            gls.addNeighbor(MyPoint(getVector(nx,ny,width,height,_positions),
                                n,
                                ScreenVectorType(nx,ny)));
                        }
                    }
                }
            }
            // closed form minimization
            gls.finalize();
            _result[getId(x,y,width,height,0,1)] = gls.kappa();
        }
    }
}
//! [kernel]

/**
* \brief RGB basic color representation
*/
typedef struct
{
    double r,g,b;
}Color;

/**
* \brief Return Color corresponding to the _value param. Simulating a "seismic" like color map
*/
__host__ Color getColor(double _value, double _valueMin, double _valueMax)
{
    Color c = {1.0, 1.0, 1.0};
    double dv;

    // Unknown values in our kernel
    if(_value == -1.)
    {
        return c;
    }

    // Threshold
    if (_value < _valueMin)
    {
        _value = _valueMin;
    }

    if (_value > _valueMax)
    {
        _value = _valueMax;
    }

    // Interval
    dv = _valueMax - _valueMin;

    // Seismic color map like
    if(_value < (_valueMin + 0.5 * dv))
    {
        c.r = 2 * (_value - _valueMin) / dv;
        c.g = 2 * (_value - _valueMin) / dv;
        c.b = 1;
    }
    else
    {
        c.b = 2 - 2 * (_value - _valueMin) / dv;
        c.g = 2 - 2 * (_value - _valueMin) / dv;
        c.r = 1;
    }

    return c;
}

/**
* \brief Load input images with freeimageplus lib
*/
__host__ bool loadImages(fipImage& _positions, fipImage& _normals, const char* _positionsFilename, const char* _normalsFilename)
{
    if(!_positions.load(_positionsFilename))
    {
        fprintf(stderr, "Cannot load _positions\n");
        return 0;
    }

    if(!_normals.load(_normalsFilename))
    {
        fprintf(stderr, "Cannot load _normal map\n");
        return 0;
    }

    _positions.convertTo24Bits();
    _normals.convertTo24Bits();

    return 1;
}

/**
* \brief Init input datas to be used on host
*/
__host__ bool initInputDatas(const fipImage& _positions, const fipImage& _normals, float** _positionsInfos, float** _normalsInfos,
                             unsigned int& _width, unsigned int& _height)
{
    BYTE* positionsPixels = 0;
    positionsPixels = _positions.accessPixels();
    if(!positionsPixels)
    {
        fprintf(stderr, "Cannot get _positions datas\n");
        return 0;
    }

    BYTE* normalsPixels = 0;
    normalsPixels = _normals.accessPixels();
    if(!normalsPixels)
    {
        fprintf(stderr, "Cannot get normals datas\n");
        return 0;
    }

    _width = _positions.getWidth();
    _height = _positions.getHeight();

    (*_positionsInfos) = new float[_width*_height*3];
    (*_normalsInfos) = new float[_width*_height*3];
    if(!*_positionsInfos || !*_normalsInfos)
    {
        fprintf(stderr, "Cannot alloc memory in initInputDatas\n");
        return 0;
    }

    for(int i = 0; i < _width * _height; ++i)
    {
        (*_positionsInfos)[i * 3 + 0] = positionsPixels[i * 3 + 0] / 255.f * 2.f - 1.f;
        (*_positionsInfos)[i * 3 + 1] = positionsPixels[i * 3 + 1] / 255.f * 2.f - 1.f;
        (*_positionsInfos)[i * 3 + 2] = positionsPixels[i * 3 + 2] / 255.f * 2.f - 1.f;

        (*_normalsInfos)[i * 3 + 0] = normalsPixels[i * 3 + 0] / 255.f;
        (*_normalsInfos)[i * 3 + 1] = normalsPixels[i * 3 + 1] / 255.f;
        (*_normalsInfos)[i * 3 + 2] = normalsPixels[i * 3 + 2] / 255.f;
    }

    positionsPixels = 0;
    normalsPixels = 0;

    return 1;
}

bool initQueries(const unsigned int& _width, const unsigned int& _height, float** _queries, int& _nbQueries)
{
    _nbQueries = _width * _height;
    (*_queries) = new float[_width*_height*2];

    if(!*_queries)
    {
        fprintf(stderr, "Cannot alloc memory in initQueries\n");
        return 0;
    }

    for(int y = 0; y < _height; ++y)
    {
        for(int x = 0; x < _width; ++x)
        {
            (*_queries)[2 * (x + y * _width)] = x;
            (*_queries)[2 * (x + y * _width) + 1] = y;
        }
    }

    return 1;
}

/**
* \brief Save _results into png image
*/
__host__ bool saveResult(float* _results, const unsigned int& _width, const unsigned int& _height,
                         const char* _positionsFilename, const char* _resultFilename)
{
    float kappaMin = *std::min_element(_results, _results + _width*_height);
    float kappaMax = *std::max_element(_results, _results + _width*_height);
    std::cout << "Kappa min : " << kappaMin << std::endl;
    std::cout << "Kappa max : " << kappaMax << std::endl;

    fipImage result;
    if(!result.load(_positionsFilename))
    {
        fprintf(stderr, "Cannot load positions\n");
        return 0;
    }

    result.convertTo24Bits();

    BYTE* resultInfos = 0;
    resultInfos = result.accessPixels();
    if(!resultInfos)
    {
        fprintf(stderr, "Cannot get result datas\n");
        return 0;
    }

    for(int i = 0; i < _width * _height; ++i)
    {
        //check nan
        if(_results[i] != _results[i])
        {
            _results[i] = 0.f;
        }

        Color c = getColor(_results[i], -10., 10.);

        resultInfos[i * 3 + 0] = c.r * 255.;
        resultInfos[i * 3 + 1] = c.g * 255.;
        resultInfos[i * 3 + 2] = c.b * 255.;
        //resultInfos[i * 4 + 3] = 255.;
    }

    if(!result.save(_resultFilename, 0))
    {
        fprintf(stderr, "Cannot save image\n");
    }

    resultInfos = 0;
    result.clear();

    return 1;
}

int main()
{
    std::string positionsFilename = "./data/ssgls_sample_wc.png";
    std::string normalsFilename = "./data/ssgls_sample_normal.png";
    std::string resultFilename = "./data/ssgls_sample_results.png";

    fipImage positions, normals;

    if(!loadImages(positions, normals, positionsFilename.c_str(), normalsFilename.c_str()))
    {
        return 0;
    }

    float fScale = 10.f;
    unsigned int width = 0;
    unsigned int height = 0;
    float* positionsInfos = 0;
    float* normalsInfos = 0;

    if(!initInputDatas(positions, normals, &positionsInfos, &normalsInfos, width, height))
    {
        return 0;
    }

    std::cout << "Image size : " << width << "*" << height << std::endl;

    float *queries = 0;
    int nbQueries;
    if(!initQueries(width, height, &queries, nbQueries))
    {
        return 0;
    }

    std::cout << "Nb queries : " << nbQueries << std::endl;

    /*********** Init Output ************/
    float *results = new float[width*height];
    for(int i = 0; i < width * height; ++i)
    {
        results[i] = 0.f;
    }

    /************* Init device mem *************/
    size_t sizeResults = width * height * sizeof(float);
    size_t sizeImg = width * height * 3 * sizeof(float);
    int *params = new int[4];
    params[0] = width;
    params[1] = height;
    params[2] = (int)fScale;
    params[3] = nbQueries;

    float* positionsInfos_device;
    float* normalsInfos_device;
    float* results_device;
    float* queries_device;
    int* params_device;

    CUDA_CHECK_RETURN( cudaMalloc(&positionsInfos_device, sizeImg) );
    CUDA_CHECK_RETURN( cudaMemcpy(positionsInfos_device, positionsInfos, sizeImg, cudaMemcpyHostToDevice) );

    CUDA_CHECK_RETURN( cudaMalloc(&normalsInfos_device, sizeImg) );
    CUDA_CHECK_RETURN( cudaMemcpy(normalsInfos_device, normalsInfos, sizeImg, cudaMemcpyHostToDevice) );

    CUDA_CHECK_RETURN( cudaMalloc(&queries_device, sizeResults*2) );
    CUDA_CHECK_RETURN( cudaMemcpy(queries_device, queries, sizeResults*2, cudaMemcpyHostToDevice) );

    CUDA_CHECK_RETURN( cudaMalloc(&params_device, 4 * sizeof(int)) );
    CUDA_CHECK_RETURN( cudaMemcpy(params_device, params, 4 * sizeof(int), cudaMemcpyHostToDevice) );

    CUDA_CHECK_RETURN( cudaMalloc(&results_device, sizeResults) );
    CUDA_CHECK_RETURN( cudaMemcpy(results_device, results, sizeResults, cudaMemcpyHostToDevice) );

    /************* Memory conf *************/

    int numThreadsPerBlock = 128;
    int numBlocks = nbQueries / numThreadsPerBlock;
    if((nbQueries % numThreadsPerBlock) > 0)
    {
        numBlocks += 1;
    }

    dim3 dimGrid(numBlocks, 1);
    dim3 dimBlock(numThreadsPerBlock, 1, 1);

    /************* Kernel Call *************/

    std::cout << "ssCurvature running..." << std::endl;

    doGLS_kernel<<<dimGrid, dimBlock>>>(params_device, queries_device, positionsInfos_device, normalsInfos_device, results_device);

    CUDA_CHECK_RETURN(cudaThreadSynchronize());	// Wait for the GPU launched work to complete
    CUDA_CHECK_RETURN(cudaGetLastError());

    std::cout << "ssCurvature completed..." << std::endl;

    /************* Get Results *************/
    CUDA_CHECK_RETURN( cudaMemcpy(results, results_device, sizeResults, cudaMemcpyDeviceToHost) );

    std::cout << "Finalizing..." << std::endl;

    /********** Cuda Free ************/
    CUDA_CHECK_RETURN( cudaFree(positionsInfos_device) );
    CUDA_CHECK_RETURN( cudaFree(normalsInfos_device) );
    CUDA_CHECK_RETURN( cudaFree(results_device) );
    CUDA_CHECK_RETURN( cudaFree(queries_device) );
    CUDA_CHECK_RETURN( cudaFree(params_device) );

    /********** Saving _result ************/
    if(!saveResult(results, width, height, positionsFilename.c_str(), resultFilename.c_str()))
    {
        return 0;
    }

    /********** Free Memory *********/
    positions.clear();
    normals.clear();

    delete [] positionsInfos;
    delete [] normalsInfos;
    delete [] queries;
    delete [] results;
    delete [] params;

    CUDA_CHECK_RETURN(cudaDeviceReset());

    std::cout << "Finished !" << std::endl;

    return 0;
}
