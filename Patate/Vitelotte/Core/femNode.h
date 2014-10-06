#ifndef _FEM_NODE_H_
#define _FEM_NODE_H_

#include <cassert>
#include "femUtils.h"


namespace Vitelotte
{


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
    inline Node(NodeType _type, int _index) ;

    inline bool isValid() const { return m_index != m_invalidIndex; }
    inline bool isConstraint() const { return m_index < 0; }
    inline bool isUnknown() const { return !isConstraint(); }

    inline size_t index() const;

    bool operator<(const Node& _other) const;
    bool operator==(const Node& _other) const;

public:
    static const int m_invalidIndex = INT_MIN;

private:
    int m_index;
};

//const int Node::m_invalidIndex = std::numeric_limits<int>::min();


#include "femNode.hpp"

} // namespace Vitelotte

#endif
