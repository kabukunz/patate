#ifndef _FEM_NODE_H_
#define _FEM_NODE_H_

#include <cassert>
#include "femUtils.h"


class Node
{
public:
    enum NodeType
    {
        UnknownNode,
        ConstraintNode,
        InvalidNode
    };

public:
    inline Node() : m_index(m_invalidIndex) {}

    inline Node(NodeType _type, int _index) :
    m_index((_type == ConstraintNode) ? -1 - _index : _index)
    {
        assert(_index != m_invalidIndex && _type != InvalidNode);
    }

    inline bool isValid() const { return m_index != m_invalidIndex; }
    inline bool isConstraint() const { return m_index < 0; }
    inline bool isUnknown() const { return !isConstraint(); }

    inline size_t index() const
    {
        assert(isValid());
        return isConstraint()? -1 - m_index : m_index;
    }

    bool operator<(const Node& _other) const
    {
        return m_index < _other.m_index;
    }

    bool operator==(const Node& _other) const
    {
        return m_index == _other.m_index;
    }

public:
    static const int m_invalidIndex = INT_MIN;

private:
    int m_index;
};

//const int Node::m_invalidIndex = std::numeric_limits<int>::min();

#endif