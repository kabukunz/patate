#include "linearElementBuilder.h"


namespace Vitelotte
{


template < class _Mesh, typename _Scalar >
LinearElementBuilder<_Mesh, _Scalar>::LinearElementBuilder()
{
}

template < class _Mesh, typename _Scalar >
unsigned
LinearElementBuilder<_Mesh, _Scalar>::
    nCoefficients(const Mesh& mesh, Face element) const
{
    return 9;
}


template < class _Mesh, typename _Scalar >
template < typename InIt >
void
LinearElementBuilder<_Mesh, _Scalar>::
    addCoefficients(InIt& it, const Mesh& mesh, Face element) const
{
    assert(mesh.valence(element) == 3);

    Vector v[3];
    unsigned nodes[3];

    typename Mesh::HalfedgeAroundFaceCirculator hit = mesh.halfedges(element);
    --hit;
    for(int i = 0; i < 3; ++i)
    {
        v[i] = (mesh.position(mesh.toVertex(*hit)) -
                mesh.position(mesh.fromVertex(*hit))).template cast<Scalar>();
        ++hit;
        nodes[i] = mesh.vertexValueNode(*hit).idx();
    }

    Scalar inv4A = 1. / (2. * det2(v[0], v[1]));

    assert(inv4A > 0);

    for(int i = 0; i < 3; ++i)
    {
        for(int j = 0; j < 3; ++j)
        {
            *(it++) = Triplet(nodes[i], nodes[j], v[i].dot(v[j]) * inv4A);
        }
    }
}


}
