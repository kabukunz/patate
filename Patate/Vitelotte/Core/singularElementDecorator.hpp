#include "singularElementDecorator.h"


namespace Vitelotte
{


template < typename _Element >
unsigned
SingularElementDecorator<_Element>::nCoefficients(
        const Mesh& mesh, Face element) const
{
    return m_element.nCoefficients(mesh, element) *
            (mesh.isSingular(element)? 2: 1);
}

template < typename _Element >
template < typename InIt >
void
SingularElementDecorator<_Element>::addCoefficients(
        InIt& it, const Mesh& mesh, Face element)
{
    typedef typename Element::Mesh Mesh;

    InIt begin = it;
    m_element.addCoefficients(it, mesh, element);

    if(mesh.isSingular(element)) {
        InIt end = it;
        int from, to;
        typename Mesh::HalfedgeAroundFaceCirculator hit = mesh.halfedges(element);
        while(true)
        {
            if(mesh.isSingular(*hit))
            {
                from = mesh.vertexValueNode(*hit).idx();
                to = mesh.vertexFromValueNode(mesh.nextHalfedge(*hit)).idx();
                break;
            }
            ++hit;
        }

        for(InIt tit=begin; tit != end; ++tit)
        {
            int r = tit->row() == from? to: tit->row();
            int c = tit->col() == from? to: tit->col();

            *(it++) = Triplet(r, c, tit->value());
        }
    }
}


}
