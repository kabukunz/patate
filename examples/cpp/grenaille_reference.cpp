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

/*!
 * \brief Variant of the MyPointRef class allowing to work only with references.
 *
 * Using this approach, ones can use the patate library with already existing
 * data-structures and without any data-duplication.
 * 
 * In this example, we use this class to bind raw arrays containing existing
 * point normals and coordinates.
 */
class MyPointRef
{
public:
	enum {Dim = 3};
	typedef double Scalar;
	typedef Eigen::Matrix<Scalar, Dim, 1>   VectorType;
	typedef Eigen::Matrix<Scalar, Dim, Dim> MatrixType;

	MULTIARCH inline MyPointRef(const VectorType &pos, const VectorType& normal)
		: _pos(pos), _normal(normal) {}

	MULTIARCH inline const VectorType& pos()    const { return _pos; }  
	MULTIARCH inline const VectorType& normal() const { return _normal; }

private:
	const VectorType &_pos, &_normal;
};

typedef MyPointRef::Scalar Scalar;
typedef MyPointRef::VectorType VectorType;

// Define related structure
typedef DistWeightFunc<MyPointRef,SmoothWeightKernel<Scalar> > WeightFunc; 
typedef Basket<MyPointRef,WeightFunc,OrientedSphereFit,   GLSParam> Fit;


template<typename Fit>
void test_fit(Fit& fit, 
              vector<VectorType>& positions, 
              vector<VectorType>& normals, 
              const VectorType& p)
{
  if(positions.size() != normals.size())
    return;

	Scalar tmax = 100.0;

	// Set a weighting function instance
	fit.setWeightFunc(WeightFunc(tmax));  

	// Set the evaluation position
	fit.init(p);

	// Iterate over samples and fit the primitive
	// A MyPointRef instance is generated on the fly to bind the raw arrays to the
	// library representation. No copy is done at this step, since we use constant
	// references.
	for(unsigned int i = 0; i!= positions.size(); i++){
	  fit.addNeighbor(MyPointRef(positions[i], normals[i]));
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

int main()
{

	// Build arrays containing normals and positions, simulating data coming from
	// outside the library.
	// 
	// For the simplicity of this example, we use the same datatype to represent
	// 3D vectors in these arrays and in MyPointRef. Note that you could use
	// different types such as Scalar*, since we rely on Eigen to define vectors.
	int n = 10000;
	vector<VectorType> positions(n);
	vector<VectorType> normals(n);

	for(int k=0; k<n; ++k){
	  VectorType n = VectorType::Random().normalized();
	  
	  normals[k]   = n;
	  positions[k] = n * Eigen::internal::random<Scalar>(0.9,1.1);
	}

	// set evaluation point and scale
	VectorType p = positions[0];

	Fit fit;
	test_fit(fit, positions, normals, p);
}
