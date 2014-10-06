
inline Node::Node(NodeType _type, int _index) :
    m_index((_type == ConstraintNode) ? -1 - _index : _index)
{
    assert(_index != m_invalidIndex && _type != InvalidNode);
}

inline size_t Node::index() const
{
    assert(isValid());
    return isConstraint()? -1 - m_index : m_index;
}

bool Node::operator<(const Node& _other) const
{
    return m_index < _other.m_index;
}

bool Node::operator==(const Node& _other) const
{
    return m_index == _other.m_index;
}
