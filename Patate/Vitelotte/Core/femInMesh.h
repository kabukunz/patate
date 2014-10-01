#ifndef _FEM_INMESH_H_
#define _FEM_INMESH_H_

#include <cassert>
#include <map>
#include <list>
#include <fstream>
#include <float.h>
#include "femEdgeConstraint.h"
#include "femVertexConstraints.h"
#include "femNodeList.h"

class FemInMesh : public surface_mesh::Surface_mesh
{
public:
    enum Type
    {
        Quadratic,
        Fraeijs
    };

public:
    typedef std::pair<Vertex, Vertex> Constraint;
    typedef std::map<Constraint, EdgeConstraint> ConstraintMap;
    typedef std::map<Vertex, VertexConstraint> VertexConstraintMap;

    typedef ConstraintMap::const_iterator ConstConstraintMapIterator;

    typedef std::pair<Vertex, EdgeConstraint> NbConstraint;
    typedef std::list<NbConstraint> NbConstraintList;

public:

    inline FemInMesh(Type _facesType = Fraeijs);
    inline ~FemInMesh() {}

    inline void buildElements(bool _constrainTips, bool _flatBoundary);
    inline bool isSingular(const EdgeConstraint& _left, const EdgeConstraint& _right) const;

    NodeList& nodeList() { return m_nodeList; }
    const NodeList& nodeList() const { return m_nodeList; }

public:
    inline bool saveTriangulationIntoQvg(const std::string& _filename);
    inline bool loadConstraintMap(const std::string& _filename);

private:
    inline void fillNbConstraint(Vertex _vx, NbConstraintList& _nbConstraintList) const;
    inline ConstraintMap::iterator findConstraint(Vertex _v0, Vertex _v1);
    inline ConstraintMap::const_iterator findConstraint(Vertex _v0, Vertex _v1) const;

private:
    ConstraintMap m_constraintMap;
    VertexConstraintMap m_vertexConstraintMap;

    NodeList m_nodeList;

    size_t m_nbSingular;


    /************************************************************************/
    /*                           Vertex properties                          */
    /************************************************************************/

public:
    // returns true iff v0 is lexicographically smaller than v1, i.e. either if v0.x() < v1.x() or if v0.x() == v1.x() and v0.y() < v1.y(). 
    bool compVx(Vertex _v0, Vertex _v1) const
    {
        if(position(_v0).x < position(_v1).x)
        {
            return true;
        }
        else if((position(_v0).x == position(_v1).x) && (position(_v0).y < position(_v1).y))
        {
            return true;
        }

        return false;
    }

    bool isMarked(Vertex _v) const { return m_vMarked[_v]; }

private:
    Vertex_property<bool>   m_vMarked;


    /************************************************************************/
    /*                           Face properties                            */
    /************************************************************************/

public:
    bool isSingular(Face _f) const { return m_fSingular[_f]; }
    void setSingular(Face _f, bool _singular)
    {
        assert(!_singular || !m_fFlat[_f]);
        m_fSingular[_f] = _singular;
    }

    bool isFlat(Face _f) const { return m_fFlat[_f]; }
    void setFlat(Face _f, bool _flat)
    {
        assert(!m_fSingular[_f] || !_flat);
        m_fFlat[_f] = _flat;
    }

    Type type() const { return m_facesType; }
    void setType(Type _type) { m_facesType = _type; }

    unsigned sizeElement() const //old size()
    {
        switch(m_facesType)
        {
            case Quadratic: return 6;
            case Fraeijs: return 9;
        }
        return 0;
    }

    unsigned singularVertex(Face _f) const 
    {
        assert(m_fSingular[_f]);
        return m_fSingularVertex[_f];
    }

    void setSingularVertex(Face _f, unsigned _singular)
    {
        assert(m_fSingular[_f] && _singular < 3);
        m_fSingularVertex[_f] = _singular;
    }

    unsigned flatVertex(Face _f) const
    {
        assert(m_fFlat[_f]);
        return m_fSingularVertex[_f];
    }

    void setFlatVertex(Face _f, unsigned _singular)
    {
        assert(m_fFlat && _singular < 3);
        m_fSingularVertex[_f] = _singular;
    }

    inline Node node(Face _f, unsigned _index) const
    {
        assert(_index < sizeElement());
        return m_fNodes[_f].array[_index];
    }

    inline void setNode(Face _f, unsigned _index, Node _node)
    {
        assert(_index < sizeElement());
        m_fNodes[_f].array[_index] = _node;
    }

    inline Node singularNode(Face _f) const
    {
        assert(m_fSingular[_f]);
        return m_fNodes[_f].array[m_singularIndex];
    }

    inline void setSingularNode(Face _f, Node _node)
    {
        assert(m_fSingular[_f]);
        m_fNodes[_f].array[m_singularIndex] = _node;
    }
    
    inline surface_mesh::Surface_mesh::Vertex getCcwVertex(const Face& _f, const Vertex& _vx) const;
    inline surface_mesh::Surface_mesh::Vertex getCwVertex(const Face& _f, const Vertex& _vx) const;

    inline int getVertexIdxInFace(const Face& _f, const Vertex& _vx) const;

public:
    static const size_t m_nodePerElement = 10;
    typedef struct nodesArray { Node array[m_nodePerElement]; } nodesArray;

private:
    static const size_t m_singularIndex = m_nodePerElement - 1;

    Type m_facesType;
    Face_property<bool> m_fSingular;
    Face_property<bool> m_fFlat;
    Face_property<unsigned> m_fSingularVertex;
    Face_property<nodesArray> m_fNodes;
};

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////Implementation//////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

FemInMesh::FemInMesh(Type _facesType)
    : m_nbSingular(0)
{
    m_vMarked = add_vertex_property<bool>("v:marked", false);

    m_facesType = _facesType;
    m_fSingular = add_face_property<bool>("f:singular", false);
    m_fFlat = add_face_property<bool>("f:flat", false);
    m_fSingularVertex = add_face_property<unsigned>("f:singularVertex", 0);
    m_fNodes = add_face_property<nodesArray>("f:nodes");
}

//Get the ccw vertex of the face from vx
surface_mesh::Surface_mesh::Vertex FemInMesh::getCcwVertex(const Face& _f, const Vertex& _vx) const
{
    Surface_mesh::Vertex_around_face_circulator fvit= vertices(_f), fvend = fvit;

    do 
    {
        if(*fvit == _vx)
        {
            //ccw
            ++fvit;
            return *fvit;
        }

        ++fvit;
    }
    while(fvit != fvend);
}

//Get the cw vertex of the face from vx
surface_mesh::Surface_mesh::Vertex FemInMesh::getCwVertex(const Face& _f, const Vertex& _vx) const
{
    Surface_mesh::Vertex_around_face_circulator fvit= vertices(_f), fvend = fvit;
    do 
    {
        if(*fvit == _vx)
        {
            //cw
            --fvit;
            return *fvit;
        }

        ++fvit;
    }
    while(fvit != fvend);
}

int FemInMesh::getVertexIdxInFace(const Face& _f, const Vertex& _vx) const
{
    Surface_mesh::Vertex_around_face_circulator fvit= vertices(_f), fvend = fvit;

    int tmp = 0;
    
    do
    {
        if(*fvit == _vx)
        {
            return tmp;
        }

        ++tmp;
        ++fvit;
    }
    while(fvit != fvend);
    
    return -1;
}

void FemInMesh::buildElements(bool _constrainTips, bool _flatBoundary)
{
    typedef std::map<unsigned, Node> ContourMap;
    ContourMap contourMap;

    NbConstraintList nbConstraintList;

    //mark all constrained vertex as false
    for(ConstraintMap::iterator cons = m_constraintMap.begin(); cons != m_constraintMap.end(); ++cons) 
    {
        m_vMarked[cons->first.first] = false;
        m_vMarked[cons->first.second] = false;
    }


    for(ConstraintMap::iterator cons = m_constraintMap.begin(); cons != m_constraintMap.end(); ++cons)
    {
        for(int tip = 0; tip < 2; ++tip)
        {
            Vertex vx = tip ? cons->first.second : cons->first.first;
            if(isMarked(vx))
            {
                continue;
            }
            m_vMarked[vx] = true;

            fillNbConstraint(vx, nbConstraintList);
            assert(!nbConstraintList.empty());	// we should at least have cons

            NbConstraint prev = nbConstraintList.back();

            Surface_mesh::Face_around_vertex_circulator elem = faces(vx);

            while(getCcwVertex(*elem, vx) != prev.first)
            {
                --elem;
            }

            FemVector prevVec((position(prev.first) - position(vx)).normalize().x, (position(prev.first) - position(vx)).normalize().y);

            Node tmpNode;
            for(NbConstraintList::const_iterator it = nbConstraintList.begin(); it != nbConstraintList.end(); ++it)
            {
                assert(getCcwVertex(*elem, vx) == prev.first);

                bool singular = isSingular(prev.second, it->second);

                
                if(prev.second.contour(EdgeConstraint::Left))
                {
                    assert(!prev.second.diffuse(EdgeConstraint::SourceLeft));
                    ContourMap::iterator contIt = contourMap.find(prev.second.contour(EdgeConstraint::Left));
                    if(contIt == contourMap.end())
                    {
                        contIt = contourMap.insert(contIt, std::make_pair(prev.second.contour(EdgeConstraint::Left), m_nodeList.addUnknown()));
                    }
                    tmpNode = contIt->second;
                }
                else if(prev.second.tear() || !tmpNode.isValid())
                {
                    if(prev.second.diffuse(EdgeConstraint::Left))
                    {
                        tmpNode = m_nodeList.addConstraint(prev.second.color(EdgeConstraint::SourceLeft));
                    }
                    else
                    {
                        tmpNode = m_nodeList.addUnknown();
                    }
                }

                // Set tmpNode.
                assert(!node(*elem, getVertexIdxInFace(*elem, vx)).isValid());
                setNode(*elem, getVertexIdxInFace(*elem, vx), tmpNode);

                // Do we add constraint ?
                bool addConstraints = _constrainTips && prev.second.diffuse(EdgeConstraint::Left) && it->second.diffuse(EdgeConstraint::Right);
                FemVector vec((position(it->first) - position(vx)).normalize().x, (position(it->first) - position(vx)).normalize().y);
                FemScalar totalAngle=0;

                if(addConstraints)
                {
                    totalAngle = (prev.first == it->first) ? M_PI * 2. : angleCcw(prevVec, vec);
                }

                // Compute what is required to add constraints.
                while(getCwVertex(*elem, vx) != it->first)
                {
                    // Create a new tmpNode if singular and set it up at singular index.
                    if(singular)
                    {
                        assert(!isSingular(*elem));
                        ++m_nbSingular;
                        setSingular(*elem, true);
                        setSingularVertex(*elem, getVertexIdxInFace(*elem, vx));
                        if(!addConstraints)
                        {
                            tmpNode = m_nodeList.addUnknown();
                        }
                        else
                        {
                            FemVector v((position(getCwVertex(*elem, vx)) - position(vx)).normalize().x, (position(getCwVertex(*elem, vx)) - position(vx)).normalize().y);
                            tmpNode = m_nodeList.addConstraint(lerp(angleCcw(prevVec, v)/totalAngle, prev.second.color(EdgeConstraint::SourceLeft), it->second.color(EdgeConstraint::SourceRight)));
                        }
                        assert(!singularNode(*elem).isValid());
                        setSingularNode(*elem, tmpNode);
                    }

                    ++elem;

                    // Set up tmpNode on next face.
                    assert(!node(*elem, getVertexIdxInFace(*elem, vx)).isValid());
                    setNode(*elem, getVertexIdxInFace(*elem, vx), tmpNode);
                }

                // Final vertex.
                // If singular
                if(singular)
                {
                    assert(!isSingular(*elem));
                    ++m_nbSingular;
                    setSingular(*elem, true);
                    setSingularVertex(*elem, getVertexIdxInFace(*elem, vx));
              
                    // Special processing for contour.
                    if(it->second.contour(EdgeConstraint::Right))
                    {
                        ContourMap::iterator contIt = contourMap.find(it->second.contour(EdgeConstraint::Right));
                        if(contIt == contourMap.end())
                        {
                            contIt = contourMap.insert(contIt, std::make_pair(it->second.contour(EdgeConstraint::Right), m_nodeList.addUnknown()));
                        }
                        tmpNode = contIt->second;
                    }
                    // Special processing for last vertex if not tear.
                    else if(it == (--nbConstraintList.end()) && !it->second.tear())
                    {
                        Vertex vx2 = tip ? cons->first.first : cons->first.second;

                        Halfedge h = find_halfedge(vx, vx2);
                        Halfedge mirrorH = opposite_halfedge(h);

                        Face mirrorF = face(mirrorH); 

                        tmpNode = node(mirrorF, getVertexIdxInFace(mirrorF, vx));
                    }
                    // Standard singular processing
                    else
                    {
                        tmpNode = (it->second.diffuse(EdgeConstraint::Right)) ? m_nodeList.addConstraint(it->second.color(EdgeConstraint::SourceRight)) : m_nodeList.addUnknown();
                    }

                    assert(!singularNode(*elem).isValid());
                    setSingularNode(*elem, tmpNode);
                }
                // Not singular case
                else
                {
                    //					vgAssert(!elem->node(elem->index(vx)).isValid());
                    //					elem->setNode(elem->index(vx), node);
                }

                ++elem;
                prev = *it;
                prevVec = vec;
            }
        }
    }

    // process point constraints
    for(Surface_mesh::Vertex_iterator vx = vertices_begin(); vx != vertices_end(); ++vx)
    {
        Surface_mesh::Face_around_vertex_circulator elem = faces(*vx), elemEnd = elem;


        VertexConstraintMap::iterator cons = m_vertexConstraintMap.find(*vx);
        bool vxConstraint = cons != m_vertexConstraintMap.end();

        if(node(*elem, getVertexIdxInFace(*elem, *vx)).isValid())
        {
            assert(!vxConstraint);
            continue;
        }

        Node node = (vxConstraint && cons->second.diffuse()) ? m_nodeList.addConstraint(cons->second.color()) : m_nodeList.addUnknown();
       
        do
        {
            if(vxConstraint && cons->second.constraintGradient())
            {
                setFlat(*elem, true);
                setFlatVertex(*elem, getVertexIdxInFace(*elem, *vx));
            }
            setNode(*elem, getVertexIdxInFace(*elem, *vx), node);
            ++elem;
        } while(elem != elemEnd);
    }

    for(Surface_mesh::Face_iterator elem = faces_begin(); elem != faces_end(); ++elem)
    {
        Surface_mesh::Vertex_around_face_circulator fvit= vertices(*elem), fvend = fvit;
        unsigned vxIndex = 0;
        do
        {
            //Get cw and ccw vertex and reset iterator
            ++fvit;
            Vertex vx1 = *fvit;
            --fvit;
            --fvit;
            Vertex vx2 = *fvit;
            ++fvit;

            bool left = compVx(vx1, vx2);
            ConstraintMap::iterator consIt = findConstraint(vx1, vx2);
            bool found = (consIt != m_constraintMap.end());

            //Ccw direction so it is the halfedge of the face
            Halfedge h = find_halfedge(vx1, vx2);
            Halfedge mirrorH = opposite_halfedge(h);

            bool boundary = false;
            if(mirrorH.is_valid())
            {
                boundary = is_boundary(mirrorH);
            }

            Face mirrorF;
            unsigned oppositeMirrorVertex;
            if(!boundary)
            {
                mirrorF = face(mirrorH);
                oppositeMirrorVertex = getVertexIdxInFace(mirrorF, to_vertex(next_halfedge(mirrorH)));
            }


            EdgeConstraint cons;
            if(found)
            {
                cons = consIt->second;
                if(!left)
                    cons.flip();
            }

            int group = 1;
            if(m_facesType == FemInMesh::Quadratic || m_facesType == FemInMesh::Fraeijs)
            {
                if(cons.tear() || left || boundary)
                {
                    Node tmpNode;
                    if(boundary && _flatBoundary)
                    {
                        tmpNode = m_nodeList.addConstraint(FemColor::Zero());
                    }
                    else if(cons.diffuse(EdgeConstraint::Left))
                    {
                        tmpNode = m_nodeList.addConstraint((cons.color(EdgeConstraint::SourceLeft) + cons.color(EdgeConstraint::TargetLeft)) / 2.);
                    }
                    else if(cons.contour(EdgeConstraint::Left))
                    {
                        ContourMap::iterator contIt = contourMap.find(cons.contour(EdgeConstraint::Left));
                        if(contIt == contourMap.end())
                        {
                            contIt = contourMap.insert(contIt, std::make_pair(cons.contour(EdgeConstraint::Left), m_nodeList.addUnknown()));
                        }
                        tmpNode = contIt->second;
                    }
                    else
                    {
                        tmpNode = m_nodeList.addUnknown();
                    }

                    assert(tmpNode.isValid());
                    setNode(*elem, group * 3 + vxIndex, tmpNode);


                    if(!cons.tear() && !boundary)
                    {
                        assert(!node(mirrorF, group * 3 + oppositeMirrorVertex).isValid());
                        setNode(mirrorF, group * 3 + oppositeMirrorVertex, tmpNode);
                    }
                }
                
                ++group;
            }

            if(m_facesType == FemInMesh::Fraeijs)
            {
                if(cons.tearGradient() || left || boundary)
                {
                    Node tmpNode;

                    if(boundary && _flatBoundary)
                    {
                        tmpNode = m_nodeList.addConstraint(FemColor::Zero());
                    }
                    else if(cons.constraintGradient(EdgeConstraint::Left))
                    {
                        tmpNode = m_nodeList.addConstraint((cons.gradient(EdgeConstraint::SourceLeft) + cons.gradient(EdgeConstraint::TargetLeft)) / 2.
                        * (compVx(vx2, vx1) ? -1.: 1.)); //comp cw, ccw
                    }
                    else
                    {
                        tmpNode = m_nodeList.addUnknown();
                    }

                    assert(tmpNode.isValid());
                    setNode(*elem, group * 3 + vxIndex, tmpNode);

                    if(!cons.tearGradient() && !boundary)
                    {
                        assert(!node(mirrorF, group * 3 + oppositeMirrorVertex).isValid());
                        setNode(mirrorF, group * 3 + oppositeMirrorVertex, tmpNode);
                    }
                }

                ++group;
            }

            ++vxIndex;
            ++fvit;
        }
        while (fvit != fvend);
    }
}

FemInMesh::ConstraintMap::iterator FemInMesh::findConstraint(Vertex _v0, Vertex _v1)
{
    return m_constraintMap.find(compVx(_v0, _v1) ? Constraint(_v0, _v1) : Constraint(_v1, _v0));
}

FemInMesh::ConstraintMap::const_iterator FemInMesh::findConstraint(Vertex _v0, Vertex _v1) const
{
    return m_constraintMap.find(compVx(_v0, _v1) ? Constraint(_v0, _v1) : Constraint(_v1, _v0));
}

void FemInMesh::fillNbConstraint(Vertex _vx, NbConstraintList& _nbConstraintList) const
{
    _nbConstraintList.clear();

    Surface_mesh::Vertex_around_vertex_circulator vit = vertices(_vx), vend = vit;
    do
    {
        ConstraintMap::const_iterator found = findConstraint(_vx, *vit);
        if(found != m_constraintMap.end())
        {
            if(found->first.first == _vx)
            {
                _nbConstraintList.push_back(NbConstraint(found->first.second, found->second));
            }
            else
            {
                EdgeConstraint ec = found->second;
                ec.flip();
                _nbConstraintList.push_back(NbConstraint(found->first.first, ec));
            }
        }
    }
    while (++vit != vend);
}

bool FemInMesh::isSingular(const EdgeConstraint& _left, const EdgeConstraint& _right) const
{
    return _left.diffuse(EdgeConstraint::Left) != _right.diffuse(EdgeConstraint::Right) ||
            (_left.diffuse(EdgeConstraint::Left) && _left.color(EdgeConstraint::SourceLeft) != _right.color(EdgeConstraint::SourceRight));
}

bool FemInMesh::saveTriangulationIntoQvg(const std::string& _filename)
{
    typedef std::map<Vertex, size_t> VxMap;
    typedef std::map<Constraint, size_t> CurveMap;

    std::ofstream out(_filename.c_str());

    if(!out.good())
    {
        return false;
    }

    out << "qvg 1.0\n";
    out << n_vertices() << " "
        << m_nodeList.unknownSize() + m_nodeList.constraintSize();

    size_t curveCount = 0;
    /*for(ConstConstraintMapIterator cons = _triangulation->constraintBegin(); cons != _triangulation->constraintEnd(); ++cons)
    {
        int size = cons->second.curve().size();
        if(size == 3 || size == 4)
            ++curveCount;
    }*/
    out	<< " " << curveCount;

    out	<< " " << n_faces() << "\n";

    VxMap vxMap;
    CurveMap curveMap;

    int vxIndex = 0;
    for(Surface_mesh::Vertex_iterator vx = vertices_begin(); vx != vertices_end(); ++vx)
    {
        //todo : convert it to 3D
        out << "v " << position(*vx).x << " " << position(*vx).y << "\n";
        vxMap[*vx] = vxIndex++;
    }

    for(NodeList::ConstValueIterator node = m_nodeList.constraintsBegin(); node != m_nodeList.constraintsEnd(); ++node)
    {
        out << "n " << node->transpose() << "\n";
    }

    for(NodeList::ConstValueIterator node = m_nodeList.unknownsBegin(); node != m_nodeList.unknownsEnd(); ++node)
    {
        out << "n " << node->transpose() << "\n";
    }

    /*int curveIndex = 0;
    std::string curveName[] = { "", "", "", "cq", "cc"};
    for(FemTriangulation::ConstConstraintMapIterator cons=_triangulation->constraintBegin(); cons!=_triangulation->constraintEnd(); ++cons)
    {
        BezierSegment3 bs = cons->second.curve();
        if(bs.size() > 4 || curveName[bs.size()].empty())
            continue;
        out << curveName[bs.size()];
        for(int i=1; i<bs.size()-1; ++i)
            out << " " << bs.point(i).transpose();
        out << "\n";
        curveMap[cons->first] = curveIndex++;
    }*/

    for(Surface_mesh::Face_iterator elem = faces_begin(); elem != faces_end(); ++elem)
    {
        out << (isSingular(*elem) ? "fs" : "f");

        //should be 3 vertices in the face
        Surface_mesh::Vertex_around_face_circulator fvit= vertices(*elem), fvend = fvit;
        unsigned i = 0;
        do
        {
            out << " " << vxMap[*fvit];

            out << "/" << (node(*elem, i).isConstraint() ? node(*elem, i).index() : node(*elem, i).index() + m_nodeList.constraintSize());

            if(isSingular(*elem) && i == singularVertex(*elem))
            {
                out << "/" << (singularNode(*elem).isConstraint() ? singularNode(*elem).index() : singularNode(*elem).index() + m_nodeList.constraintSize());
            }
            ++i;
        }
        while (++fvit != fvend);


        for(unsigned i = 3; i < 6; ++i)
        {
            assert(!isSingular(*elem) || i != singularVertex(*elem));

            out << " " << (node(*elem, i).isConstraint() ? node(*elem, i).index() : node(*elem, i).index() + m_nodeList.constraintSize());

            /*Constraint cons((*elem)->vertex((i + 1) % 3), (*elem)->vertex((i + 2) % 3));

            if(compVx(cons.second, cons.first))
            {
                std::swap(cons.second, cons.first);
            }

            CurveMap::const_iterator it = curveMap.find(cons);
            if(it != curveMap.end())
            {
                out << "/" << it->second;
            }*/
        }

        out << "\n";
    }

    out.close();

    return true;
}

#define FEM_CHECK_PARSE_ERROR() \
    do\
{\
    in >> std::ws;\
    if(in.fail() || in.bad() || in.eof())\
{\
    return false;\
}\
} while(false)

bool FemInMesh::loadConstraintMap(const std::string& _filename)
{
    std::ifstream in(_filename.c_str());
    if(!in.good())
    {
        return false;
    }

    unsigned nbCons, v1, v2;

    in >> nbCons;

    FEM_CHECK_PARSE_ERROR();

    for(unsigned i = 0; i < nbCons; ++i)
    {
        std::string tmpStr;
        in >> tmpStr;

        if(tmpStr == "ec")
        {
            in >> v1 >> v2;

            EdgeConstraint e;

            bool tmpBool;
            float r, g, b, a;

            in >> tmpBool;
            e.setTear(tmpBool);

            in >> tmpBool;
            e.setTearGradient(tmpBool);

            for(unsigned j = 0; j < 2; ++j)
            {
                in >> tmpBool;
                e.setDiffuse(j, tmpBool);
            }

            for(unsigned j = 0; j < 4; ++j)
            {
                in >> r >> g >> b >> a;
                e.setColor(j, FemColor(r, g, b, a));
            }

            for(unsigned j = 0; j < 2; ++j)
            {
                in >> tmpBool;
                e.setContour(j, tmpBool);
            }

            for(unsigned j = 0; j < 2; ++j)
            {
                in >> tmpBool;
                e.setConstraintGradient(j, tmpBool);
            }

            for(unsigned j = 0; j < 4; ++j)
            {
                in >> r >> g >> b >> a;
                e.setGradient(j, FemColor(r, g, b, a));
            }

            //FEM_CHECK_PARSE_ERROR();

            m_constraintMap[Constraint(Vertex(v1), Vertex(v2))] = e;
        }
        else if(tmpStr == "vc")
        {
            in >> v1;

            bool bDiffuse;
            in >> bDiffuse;

            FemColor color;
            in >> color.x() >> color.y() >> color.z() >> color.w();

            bool bConstraintGradient;
            in >> bConstraintGradient;

            FemColor gradient;
            in >> gradient.x() >> gradient.y() >> gradient.z() >> gradient.w();

            VertexConstraint vertexConstraint;

            vertexConstraint.setDiffuse(bDiffuse);
            vertexConstraint.setColor(color);

            vertexConstraint.setConstraintGradient(bConstraintGradient);
            vertexConstraint.setGradient(gradient);

            m_vertexConstraintMap[Vertex(v1)] = vertexConstraint;
        }
    }

    in >> std::ws;
    if(!in.eof())
    {
        return false;
    }

    return true;
}
#endif
