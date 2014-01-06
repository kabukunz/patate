/*
This Source Code Form is subject to the terms of the Mozilla Public
License, v. 2.0. If a copy of the MPL was not distributed with this
file, You can obtain one at http://mozilla.org/MPL/2.0/. 
*/


/*!
\file examples/Grenaille/basic_cpu.h
\brief Basic use of Grenaille

\author: Nicolas Mellado, Gautier Ciaudo
*/
#include <cmath>
#include <algorithm>
#include <iostream>

#include "Patate/grenaille.h"
#include "Eigen/Eigen"

#include <vector>

using namespace std;
using namespace Grenaille;

#define DIMENSION 3

/*!
 * \brief Variant of the MyPointRef class allowing to work only with references.
 *
 * Using this approach, ones can use the patate library with already existing
 * data-structures and without any data-duplication.
 * 
 * In this example, we use this class to map an interlaced raw array containing
 * both point normals and coordinates.
 */
class MyPoint
{
public:
	enum {Dim = DIMENSION};
	typedef double Scalar;
	typedef Eigen::Matrix<Scalar, Dim, 1>   VectorType;
	typedef Eigen::Matrix<Scalar, Dim, Dim> MatrixType;

	MULTIARCH inline MyPoint(Scalar* interlacedArray, int pId)
		: _pos   (Eigen::Map< const VectorType >(interlacedArray + DIMENSION*2*pId  )), 
		  _normal(Eigen::Map< const VectorType >(interlacedArray + DIMENSION*2*pId+DIMENSION)) {
		  }

	MULTIARCH inline const Eigen::Map< const VectorType >& pos()    const { return _pos; }  
	MULTIARCH inline const Eigen::Map< const VectorType >& normal() const { return _normal; }

private:
  Eigen::Map< const VectorType > _pos, _normal;
};

typedef MyPoint::Scalar Scalar;
typedef MyPoint::VectorType VectorType;

// Define related structure
typedef DistWeightFunc<MyPoint,SmoothWeightKernel<Scalar> > WeightFunc; 
typedef Basket<MyPoint,WeightFunc,OrientedSphereFit,   GLSParam> Fit;


template<typename Fit>
void test_fit(Fit& fit, 
              Scalar* interlacedArray, 
              int n,
              const VectorType& p)
{
	Scalar tmax = 100.0;

	// Set a weighting function instance
	fit.setWeightFunc(WeightFunc(tmax));  

	// Set the evaluation position
	fit.init(p);

	// Iterate over samples and fit the primitive
	// A MyPoint instance is generated on the fly to bind the raw arrays to the
	// library representation. No copy is done at this step.
	for(int i = 0; i!= n; i++){
	  fit.addNeighbor(MyPoint(interlacedArray, i));
	}

	//finalize fitting
	fit.finalize();

	//Test if the fitting ended without errors
	if(fit.isReady())
	{
		cout << "Center: [" << fit.center().transpose() << "] ;  radius: " << fit.radius() << endl;

		cout << "Pratt normalization" 
			<< (fit.applyPrattNorm() ? " is now done." : " has already been applied.") << endl;

		// Play with fitting output
		cout << "Value of the scalar field at the initial point: " 
			<< p.transpose() 
			<< " is equal to " << fit.potential(p)
			<< endl;

		cout << "It's gradient at this place is equal to: "
			<< fit.primitiveGradient(p).transpose()
			<< endl;

		cout << "Approximation of the Hessian matrix: " << endl
			<< fit.primitiveHessian(p)
			<< endl;

		cout << "Fitted Sphere: " << endl
			<< "\t Tau  : "      << fit.tau()             << endl
			<< "\t Eta  : "      << fit.eta().transpose() << endl
			<< "\t Kappa: "      << fit.kappa()           << endl;

		cout << "The initial point " << p.transpose()              << endl
			<< "Is projected at   " << fit.project(p).transpose() << endl;
	}
}

// Build an interlaced array containing n position and normal vectors
Scalar* buildInterlacedArray(int n){
	Scalar* interlacedArray = new Scalar[2*DIMENSION*n];

	for(int k=0; k<n; ++k){

  	// For the simplicity of this example, we use Eigen Vectors to compute 
  	// both coordinates and normals, and then copy the raw values to an 
  	// interlaced array, discarding the Eigen representation.
	  Eigen::Matrix<Scalar, DIMENSION, 1> nvec = Eigen::Matrix<Scalar, DIMENSION, 1>::Random().normalized();
	  Eigen::Matrix<Scalar, DIMENSION, 1> pvec = nvec * Eigen::internal::random<Scalar>(0.9,1.1);
	  
	  // Grab coordinates and store them as raw buffer
	  memcpy(interlacedArray+2*DIMENSION*k,           pvec.data(), DIMENSION*sizeof(Scalar));
	  memcpy(interlacedArray+2*DIMENSION*k+DIMENSION, nvec.data(), DIMENSION*sizeof(Scalar));
	  
	}

  return interlacedArray;
}

int main()
{

	// Build arrays containing normals and positions, simulating data coming from
	// outside the library.
	int n = 1000;
	Scalar *interlacedArray = buildInterlacedArray(n);

	// set evaluation point and scale at the first coordinate
	VectorType p (interlacedArray);

  // Here we now perform the fit, starting from a raw interlaced buffer, without
  // any data duplication
	Fit fit;
	test_fit(fit, interlacedArray, n, p);
}