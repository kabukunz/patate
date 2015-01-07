#ifndef _SINGULAR_ELEMENT_DECORATOR_H_
#define _SINGULAR_ELEMENT_DECORATOR_H_


#include <vector>

#include <Eigen/Core>
#include <Eigen/Sparse>


namespace Vitelotte
{


template < class _Element >
class SingularElementDecorator
{
public:
    typedef _Element Element;

    typedef typename Element::Scalar Scalar;
    typedef typename Element::Mesh Mesh;

    typedef typename Element::Vector Vector;
    typedef typename Element::Triplet Triplet;

    typedef typename Element::Status Status;

protected:
    typedef typename Mesh::Face Face;


public:
    inline explicit SingularElementDecorator(const Element& element=Element())
        : m_element(element) {}

    unsigned nCoefficients(const Mesh& mesh, Face element) const;

    template < typename InIt >
    void addCoefficients(InIt& it, const Mesh& mesh, Face element);

    inline Status status() const { return m_element.status(); }
    inline const std::string& errorString() const { return m_element.errorString(); }
    inline void resetStatus() { m_element.resetStatus(); }

    Element& element() { return m_element; }
    const Element& element() const { return m_element; }

private:
    Element m_element;
};


} // namespace Vitelotte

#include "singularElementDecorator.hpp"


#endif
