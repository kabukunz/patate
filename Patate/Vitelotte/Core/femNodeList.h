#ifndef _FEM_NODELIST_H_
#define _FEM_NODELIST_H_

#include <cassert>
#include "femNode.h"


namespace Vitelotte
{


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

    inline Node addConstraint(const FemColor& _value);
    inline Node addUnknown(const FemColor& _value = FemColor::Zero());

    inline FemColor getValue(Node _node) const;
    void setValue(Node _node, const FemColor& _value);

private:
    ValueVector m_constraints;
    ValueVector m_unknowns;
};


#include "femNodeList.hpp"

} // namespace Vitelotte

#endif
