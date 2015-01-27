#include <cstdlib>
#include <iostream>
#include <fstream>
#include <sstream>

#include <Patate/vitelotte.h>


typedef double Scalar;

typedef Eigen::Matrix<Scalar, 2, 1> Vector;
typedef Eigen::Matrix<Scalar, 3, 1> Vector3;
typedef Eigen::Matrix<Scalar, 9, 1> Vector9;

typedef Eigen::Matrix<Scalar, 2, 2> Matrix2;
typedef Eigen::Matrix<Scalar, 2, 3> Matrix2x3;
typedef Eigen::Matrix<Scalar, 9, 9> Matrix9;

typedef Vitelotte::_FVElement<Scalar> Elem;


class EvalFv
{
public:
    typedef Vector9 ReturnType;

    EvalFv(const Elem& elem_)
        : elem(elem_) {}

    ReturnType operator()(const Vector& p) const
    {
        return elem.eval(p);
    }

    const Elem& elem;
};


template < typename Fn >
class DiffAlongVector
{
public:
    typedef typename Fn::ReturnType ReturnType;

    DiffAlongVector(const Fn& fn_, const Vector& v_, Scalar delta_)
        : fn(fn_), v(v_), delta(delta_)
    {
        setV(v_);
    }

    ReturnType operator()(const Vector& p) const
    {
        return (fn(p + v) - fn(p - v)) / delta;
    }

    void setV(const Vector& v_)
    {
        v = v_.normalized() * (delta / 2);
    }

    const Fn& fn;
    Vector v;
    Scalar delta;
};


template < typename Fn >
class Reparam
{
public:
    typedef typename Fn::ReturnType ReturnType;

    Reparam(const Fn& fn_, const Vector& p_, const Vector& v_)
        : fn(fn_), p(p_), v(v_)
    {}

    ReturnType operator()(Scalar x) const
    {
        return fn(p + v*x);
    }

    const Fn& fn;
    Vector p;
    Vector v;
};


template < typename Fn >
class IntegrateQuad
{
public:
    typedef typename Fn::ReturnType ReturnType;

    IntegrateQuad(const Fn& fn_, Scalar lower_ = 0, Scalar upper_ = 1)
        : fn(fn_), lower(lower_), upper(upper_)
    {}

    ReturnType operator()() const
    {
        Scalar scale = (upper - lower) / 2;
        Scalar offset = (upper + lower) / 2;
        Scalar qp = std::sqrt(3. / 5.);
        Scalar qp1 = -scale * qp + offset;
        Scalar qp2 =  scale * qp + offset;

        return .5 * (
                   (5. / 9.) * fn(qp1) +
                   (8. / 9.) * fn(offset) +
                   (5. / 9.) * fn(qp2)
               );
    }

    const Fn& fn;
    Scalar lower;
    Scalar upper;
};

template < typename Fn >
class Diff2
{
public:
    typedef typename Fn::ReturnType ReturnType;

    Diff2(const Fn& fn_, const Vector& v0_, const Vector& v1_, Scalar delta_)
        : fn(fn_), v0(v0_), v1(v1_), delta(delta_)
    {
        setVectors(v0_, v1_);
    }

    ReturnType operator()(const Vector& p) const
    {
        return (  fn(p + v0 + v1)
                - fn(p + v0 - v1)
                - fn(p - v0 + v1)
                + fn(p - v0 - v1)) / (delta * delta);
    }

    void setVectors(const Vector& v0_, const Vector& v1_)
    {
        v0 = v0_.normalized() * (delta / 2);
        v1 = v1_.normalized() * (delta / 2);
    }

    const Fn& fn;
    Vector v0;
    Vector v1;
    Scalar delta;
};


Vector9 diffX2(const Elem& elem, Scalar delta, const Vector& p)
{
    Vector v = Vector(delta, 0);
    return (elem.eval(p + v) - 2 * elem.eval(p) + elem.eval(p - v)) / (delta * delta);
}

Vector9 diffY2(const Elem& elem, Scalar delta, const Vector& p)
{
    Vector v = Vector(0, delta);
    return (elem.eval(p + v) - 2 * elem.eval(p) + elem.eval(p - v)) / (delta * delta);
}

Vector9 diffXY(const Elem& elem, Scalar delta, const Vector& p)
{
    Vector vx = Vector(delta / 2, 0);
    Vector vy = Vector(0, delta / 2);
    return (elem.eval(p + vx + vy) - elem.eval(p + vx - vy)
            - elem.eval(p - vx + vy)+ elem.eval(p - vx - vy)) / (delta * delta);
}


int main(int argc, char** argv)
{
    srand(time(NULL));
    bool random = true;
    Scalar delta = 1.e-6;

    Eigen::IOFormat fmt(6, 0, " ", "\n", "  ", "", "", "");

    Elem elem(
        Vector(0, 0),
        Vector(1, 0),
        Vector(.5, std::sqrt(3.)/2.)
    );
    if(random)
    {
        elem = Elem(Vector::Random(), Vector::Random(), Vector::Random());
    }

    std::cout << "Points:\n";
    for(int i = 0; i < 3; ++i)
        std::cout << "  p" << i << ": " << elem.point(i).transpose() << "\n";
    std::cout << "2 * Area: " << elem.doubleArea() << "\n";
    std::cout << "Edge lengths:";
    for(int i = 0; i < 3; ++i)
        std::cout << " " << elem.edgeLength(i);
    std::cout << "\n";
    std::cout << "L:\n" << elem.dldn() << "\n";

    typedef DiffAlongVector<EvalFv> Diff;
    typedef Reparam<Diff> ReparamDiff;
    typedef IntegrateQuad<ReparamDiff> IntOverEdge;

    EvalFv fv(elem);
    Diff diff(fv, Vector(), delta);
    ReparamDiff reparamDiff(diff, Vector(), Vector());
    IntOverEdge integrate(reparamDiff);

    Matrix9 testBasis = Matrix9::Zero();
    for(int i = 0; i < 3; ++i)
    {
        const Vector& p1 = elem.point((i+1)%3);
        const Vector& p2 = elem.point((i+2)%3);

        testBasis.col(i) = elem.eval(elem.point(i));
        testBasis.col(i + 3) = elem.eval((p1 + p2) / 2);

        Vector n(p2 - p1);
        n = Vector(-n.y(), n.x());
        diff.setV(n);
        reparamDiff.p = p1;
        reparamDiff.v = p2 - p1;
        testBasis.col(i + 6) = integrate();
    }
    std::cout << "Test basis:\n" << testBasis.format(fmt) << "\n";

    typedef Eigen::Matrix<Scalar, 10, 1> TestMatrix;
    TestMatrix tm;
    for(int i = 0; i < 10; ++i)
        tm(i) = elem.eval(Vector::Random()).head<6>().sum();
    std::cout << "Random evals: " << tm.transpose() << "\n";

    Vector p(.5, .5);
    if(random)
        p = Vector::Random();

    {
        Matrix2 hessians[9];
        elem.hessian(p, hessians);
        Eigen::Matrix<Scalar, 9, 3> testHessian;
        for(int i = 0; i < 9; ++i)
        {
            testHessian(i, 0) = hessians[i](0, 0);
            testHessian(i, 1) = hessians[i](1, 1);
            testHessian(i, 2) = hessians[i](0, 1);
        }

        Eigen::Matrix<Scalar, 9, 3> testHessianNum;
        testHessianNum <<
            diffX2(elem, delta, p),
            diffY2(elem, delta, p),
            diffXY(elem, delta, p);
        std::cout << "Test hessian:\n" << (testHessian - testHessianNum).format(fmt) << "\n";
        std::cout << "Test hessian fact:\n" << (testHessian.array() / testHessianNum.array()).format(fmt) << "\n";
    }

//    {
//        Vector vx = Vector(delta / 2, 0);
//        Vector vy = Vector(0, delta / 2);

//        Vector3 bc = elem._LinearElement::eval(p);
//        Eigen::Matrix<Scalar, 3, 3> testFunc;
//        for(int i = 0; i < 3; ++i)
//        {
////            Matrix2 test = elem._bubbleHessian(bc);
//            Matrix2 test = elem._gradientSubExprHessian(i, bc);
//            testFunc(i, 0) = test(0, 0);
//            testFunc(i, 1) = test(1, 1);
//            testFunc(i, 2) = test(0, 1);

//            std::cout << "hessian " << i << ":\n" << test.format(fmt) << "\n";
//        }

////        typedef Eigen::Matrix<Scalar, 6, 1> Vector6;
////        typedef Eigen::Matrix<Scalar, 6, 3> Matrix6;
////        Matrix6 coeffs;
////        coeffs.setRandom();
////        std::cout << "coeffs:\n" << coeffs.format(fmt) << "\n";
////        for(int i = 0; i < 3; ++i)
////        {
////            testFunc(i, 0) = 2 * coeffs(3, i);
////            testFunc(i, 1) = 2 * coeffs(5, i);
////            testFunc(i, 2) =     coeffs(4, i);
////        }

//        Eigen::Matrix<Scalar, 3, 3> testFuncNum;
//        for(int i = 0; i < 3; ++i)
//        {
//            Matrix2 test;
////#define FUNC(_p) elem._bubble(elem._LinearElement::eval(_p))
//#define FUNC(_p) elem._gradientSubExpr(i, elem._LinearElement::eval(_p))
////#define FUNC(_p) (coeffs.col(i).dot((Vector6() << \
//        1, \
//        (_p).x(), \
//        (_p).y(), \
//        (_p).x()*(_p).x(), \
//        (_p).x()*(_p).y(), \
//        (_p).y()*(_p).y()).finished()))
//            test(0, 0) = (FUNC(p + vx) - 2 * FUNC(p) + FUNC(p - vx)) * 4 / (delta * delta);
//            test(1, 1) = (FUNC(p + vy) - 2 * FUNC(p) + FUNC(p - vy)) * 4 / (delta * delta);
//            test(0, 1) = (  FUNC(p + vx + vy)
//                          - FUNC(p + vx - vy)
//                          - FUNC(p - vx + vy)
//                          + FUNC(p - vx - vy)) / (delta * delta);
//            test(1, 0) = test(0, 1);

//            testFuncNum(i, 0) = test(0, 0);
//            testFuncNum(i, 1) = test(1, 1);
//            testFuncNum(i, 2) = test(0, 1);

//            std::cout << "hessian num" << i << ":\n" << test.format(fmt) << "\n";
//        }
//        std::cout << "Test func:\n" << (testFunc - testFuncNum).format(fmt) << "\n";
//        std::cout << "Test func fact:\n" << (testFunc.array() / testFuncNum.array()).format(fmt) << "\n";
//    }

    Eigen::MatrixXd evalPts;
    Eigen::MatrixXd evalBasis;
    unsigned tess = 32;
    unsigned nPts = (tess+1)*(tess+2) / 2;
    evalPts.resize(nPts, 2);
    evalBasis.resize(nPts, 10);
    Matrix2x3 pm;
    pm << elem.point(0), elem.point(1), elem.point(2);
    unsigned count = 0;
    for(unsigned i = 0; i <= tess; ++i)
    {
        for(unsigned j = 0; j <= i; ++j)
        {
            Scalar alpha = 1 - Scalar(i) / Scalar(tess);
            Scalar beta = Scalar(j) / Scalar(tess);
            Vector3 bc(alpha, 1 - alpha - beta, beta);

            Vector p = pm * bc;
            Vector9 e = elem.eval(p);
            evalPts.row(count) = p;
            evalBasis.row(count) << e.transpose(), e.head<6>().sum();
            ++count;
        }
    }

    for(unsigned bi = 0; bi < 10; ++bi)
    {
        std::ostringstream ns;
        ns << "basis" << bi << ".obj";

        std::ofstream out(ns.str().c_str());

        for(int pi = 0; pi < nPts; ++pi)
            out << "v " << evalPts.row(pi) << " " << evalBasis(pi, bi) << "\n";

        unsigned upper = 1; // obj start counting at 1
        unsigned lower = 2;
        for(unsigned i = 0; i < tess; ++i)
        {
            for(unsigned j = 0; j < i; ++j)
            {
                out << "f " << upper << " " << lower << " " << lower+1 << "\n";
                out << "f " << upper << " " << lower+1 << " " << upper+1 << "\n";
                upper += 1;
                lower += 1;
            }
            out << "f " << upper << " " << lower << " " << lower+1 << "\n";
            upper += 1;
            lower += 2;
        }
    }


    return 0;
}
