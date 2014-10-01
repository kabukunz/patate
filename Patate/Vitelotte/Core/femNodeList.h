#ifndef _FEM_NODELIST_H_
#define _FEM_NODELIST_H_

#include <cassert>
#include "femNode.h"

class NodeList
{
public:
    typedef std::vector<FemColor, Eigen::aligned_allocator<FemColor> > ValueVector;

    typedef ValueVector::iterator ValueIterator;
    typedef ValueVector::const_iterator ConstValueIterator;

public:
    inline size_t constraintSize() const { return m_constraints.size(); }
    inline size_t unknownSize() const { return m_unknowns.size(); }

    inline ValueIterator constraintsBegin() { return m_constraints.begin(); }
    inline ConstValueIterator constraintsBegin() const { return m_constraints.begin(); }
    inline ValueIterator constraintsEnd() { return m_constraints.end(); }
    inline ConstValueIterator constraintsEnd() const { return m_constraints.end(); }

    inline ValueIterator unknownsBegin() { return m_unknowns.begin(); }
    inline ConstValueIterator unknownsBegin() const { return m_unknowns.begin(); }
    inline ValueIterator unknownsEnd() { return m_unknowns.end(); }
    inline ConstValueIterator unknownsEnd() const { return m_unknowns.end(); }

    inline Node addConstraint(const FemColor& _value)
    {
        m_constraints.push_back(_value);
        return Node(Node::ConstraintNode, m_constraints.size() - 1);
    }

    inline Node addUnknown(const FemColor& _value = FemColor::Zero())
    {
        m_unknowns.push_back(_value);
        return Node(Node::UnknownNode, m_unknowns.size() - 1);
    }

    inline FemColor getValue(Node _node) const
    {
        if(_node.isConstraint())
        {
            assert(_node.index() < m_constraints.size());
            return m_constraints[_node.index()];
        }

        assert(_node.index() < m_unknowns.size());
        return m_unknowns[_node.index()];
    }

    void setValue(Node _node, const FemColor& _value)
    {
        if(_node.isConstraint())
        {
            assert(_node.index() < m_constraints.size());
            m_constraints[_node.index()] = _value;
        }

        assert(_node.index() < m_unknowns.size());
        m_unknowns[_node.index()] = _value;
    }

private:
    ValueVector m_constraints;
    ValueVector m_unknowns;
};

#endif
