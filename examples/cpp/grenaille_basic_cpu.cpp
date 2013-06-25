/*
 This Source Code Form is subject to the terms of the Mozilla Public
 License, v. 2.0. If a copy of the MPL was not distributed with this
 file, You can obtain one at http://mozilla.org/MPL/2.0/. 
*/


/*!
 \file examples/Grenaille/basic_cpu.h
 \brief very brief file description

 \author: Nicolas Mellado
 */
#include <cmath>
#include <algorithm>
#include <iostream>

#include "Patate/grenaille.h"
#include "Eigen/Eigen"

#include <vector>
using namespace std;
using namespace Grenaille;


// This class defines the input data format
class MyPoint{
public:
  enum {Dim = 3};
  typedef double Scalar;
  typedef Eigen::Matrix<Scalar, Dim, 1>   VectorType;
  typedef Eigen::Matrix<Scalar, Dim, Dim> MatrixType;

  MULTIARCH inline MyPoint(const VectorType &pos    = VectorType::Zero(), 
		 const VectorType& normal = VectorType::Zero())
    : _pos(pos), _normal(normal) {}
    
  MULTIARCH inline const VectorType& pos()    const { return _pos; }  
  MULTIARCH inline const VectorType& normal() const { return _normal; }

  MULTIARCH inline VectorType& pos()    { return _pos; }  
  MULTIARCH inline VectorType& normal() { return _normal; }

  static inline MyPoint Random() {
    VectorType n = VectorType::Random().normalized();
    VectorType p = n * Eigen::internal::random<Scalar>(0.9,1.1);
    return MyPoint (p, (n + VectorType::Random()*0.1).normalized());
  };

private:
  VectorType _pos, _normal;
};

typedef MyPoint::Scalar Scalar;
typedef MyPoint::VectorType VectorType;

// Define related structure
typedef DistWeightFunc<MyPoint,SmoothWeightKernel<Scalar> > WeightFunc; 
typedef Basket<MyPoint,WeightFunc,OrientedSphereFit, GLSParam, OrientedSphereSpaceDer, GLSDer/*, GLSGeomVar*/> Fit1;
typedef Basket<MyPoint,WeightFunc,UnorientedSphereFit, GLSParam> Fit2;


template<typename Fit>
void test_fit(Fit& fit, vector<MyPoint>& vecs, const VectorType& p)
{
  Scalar tmax = 100.0;
  
  fit.setWeightFunc(WeightFunc(tmax));
  fit.init(p);
  
  // Fit the primitive
  for(vector<MyPoint>::iterator it = vecs.begin(); it != vecs.end(); it++)
    fit.addNeighbor(*it);  
  fit.finalize();
  
  cout << "Center: [" << fit.center().transpose() << "] ;  radius: " << fit.radius() << endl;
  
  cout << "Pratt normalization" << (fit.applyPrattNorm() ? " is now done." : " has already been applied.") << endl;
  
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

//   Fit::VectorArray deta = fit.deta_normalized();

//   cout << "dkappa: " << deta
//        << endl;

  //cout << "geomVar: " << fit.geomVar() << endl;
}

int main() {
  // set evaluation point and scale
  VectorType p = VectorType::Random();
  
  // init input data
  int n = 10000;
  vector<MyPoint> vecs (n);
  
  p = vecs.at(0).pos();

  for(int k=0; k<n; ++k)
    vecs[k] = MyPoint::Random();
  
//   VectorType normal; normal << 0.0 , 0.0, 1.0;
  
  std::cout << "====================\nOrientedSphereFit:\n";
  Fit1 fit1;
  test_fit(fit1, vecs, p);
  
  std::cout << "\n\n====================\nUnorientedSphereFit:\n";
  Fit2 fit2;
  test_fit(fit2, vecs, p);
  
  
}
