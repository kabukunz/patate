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


namespace Vitelotte
{


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
    bool compVx(Vertex _v0, Vertex _v1) const;

    bool isMarked(Vertex _v) const { return m_vMarked[_v]; }

private:
    Vertex_property<bool>   m_vMarked;


    /************************************************************************/
    /*                           Face properties                            */
    /************************************************************************/

public:
    bool isSingular(Face _f) const { return m_fSingular[_f]; }
    void setSingular(Face _f, bool _singular);

    bool isFlat(Face _f) const { return m_fFlat[_f]; }
    void setFlat(Face _f, bool _flat);

    Type type() const { return m_facesType; }
    void setType(Type _type) { m_facesType = _type; }

    unsigned sizeElement() const;

    unsigned singularVertex(Face _f) const ;
    void setSingularVertex(Face _f, unsigned _singular);

    unsigned flatVertex(Face _f) const;
    void setFlatVertex(Face _f, unsigned _singular);

    inline Node node(Face _f, unsigned _index) const;
    inline void setNode(Face _f, unsigned _index, Node _node);

    inline Node singularNode(Face _f) const;
    inline void setSingularNode(Face _f, Node _node);
    
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


#include "femInMesh.hpp"

} // namespace Vitelotte

#endif
