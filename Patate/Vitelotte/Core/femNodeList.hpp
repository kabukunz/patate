
inline Node NodeList::addConstraint(const FemColor& _value)
{
    m_constraints.push_back(_value);
    return Node(Node::ConstraintNode, m_constraints.size() - 1);
}

inline Node NodeList::addUnknown(const FemColor& _value)
{
    m_unknowns.push_back(_value);
    return Node(Node::UnknownNode, m_unknowns.size() - 1);
}

inline FemColor NodeList::getValue(Node _node) const
{
    if(_node.isConstraint())
    {
        assert(_node.index() < m_constraints.size());
        return m_constraints[_node.index()];
    }

    assert(_node.index() < m_unknowns.size());
    return m_unknowns[_node.index()];
}

inline void NodeList::setValue(Node _node, const FemColor& _value)
{
    if(_node.isConstraint())
    {
        assert(_node.index() < m_constraints.size());
        m_constraints[_node.index()] = _value;
    }

    assert(_node.index() < m_unknowns.size());
    m_unknowns[_node.index()] = _value;
}
