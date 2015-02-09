#include <cstdlib>
#include <iostream>
#include <fstream>
#include <sstream>

#include <Patate/vitelotte.h>


typedef double Scalar;

typedef Vitelotte::FVElement<Scalar> Elem;
typedef Vitelotte::FVElementFlat<Scalar> ElemFlat;

typedef Elem::Vector Vector;
typedef Elem::Vector3 Vector3;
typedef Elem::Vector9 Vector9;

typedef Elem::Matrix2 Matrix2;
typedef Elem::Matrix2x3 Matrix2x3;
typedef Eigen::Matrix<Scalar, 9, 2> Matrix9x2;
typedef Eigen::Matrix<Scalar, 9, 9> Matrix9;


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
    return (elem.eval(Vector(p + v)) - 2 * elem.eval(p) + elem.eval(Vector(p - v))) / (delta * delta);
}

Vector9 diffY2(const Elem& elem, Scalar delta, const Vector& p)
{
    Vector v = Vector(0, delta);
    return (elem.eval(Vector(p + v)) - 2 * elem.eval(p) + elem.eval(Vector(p - v))) / (delta * delta);
}

Vector9 diffXY(const Elem& elem, Scalar delta, const Vector& p)
{
    Vector vx = Vector(delta / 2, 0);
    Vector vy = Vector(0, delta / 2);
    return (elem.eval(Vector(p + vx + vy)) - elem.eval(Vector(p + vx - vy))
            - elem.eval(Vector(p - vx + vy))+ elem.eval(Vector(p - vx - vy))) / (delta * delta);
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
        testBasis.col(i + 3) = elem.eval(Vector((p1 + p2) / 2));

        Vector n(p2 - p1);
        n = Vector(-n.y(), n.x());
        diff.setV(n);
        reparamDiff.p = p1;
        reparamDiff.v = p2 - p1;
        testBasis.col(i + 6) = integrate();
    }
    std::cout << "Test basis:\n" << testBasis.format(fmt) << "\n";

    Vector dv1 = (elem.point(1) - elem.point(0)).normalized() * (delta/2);
    Vector dv2 = (elem.point(2) - elem.point(0)).normalized() * (delta/2);
    Matrix9x2 testDiffP0;
    testDiffP0 <<
        (elem.eval(Vector(elem.point(0) + dv2)) - elem.eval(Vector(elem.point(0) - dv2))) / delta,
        (elem.eval(Vector(elem.point(0) + dv1)) - elem.eval(Vector(elem.point(0) - dv1))) / delta;
    std::cout << "Test diff p0:\n" << testDiffP0.transpose() << "\n";

    typedef Eigen::Matrix<Scalar, 10, 1> TestMatrix;
    TestMatrix tm;
    for(int i = 0; i < 10; ++i)
        tm(i) = elem.eval(Vector(Vector::Random())).head<6>().sum();
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


    {
//        ElemFlat elem(Vector::Random(), Vector::Random(), Vector::Random());
        ElemFlat elem(Vector(0, 0), Vector(1, 0), Vector(.5, std::sqrt(3)/2));

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

            for(unsigned pi = 0; pi < nPts; ++pi)
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
    }

    return 0;
}
