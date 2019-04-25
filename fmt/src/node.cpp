#include <node.h>

Node::Node(config_t &config, SetType t, double c)
    : q(config), parent(nodeptr_t()), setType(t), cost(c)
{
}

Node::Node(config_t &config, nodeptr_t &p, SetType t, double c)
    : q(config), parent(p), setType(t), cost(c)
{
}

std::ostream &operator<<(std::ostream &os, const Node &node)
{
    os << "Point: ";
    for (size_t i = 0; i < node.q.size() - 1; ++i)
    {
        os << node.q[i] << ", ";
    }
    os << node.q.back() << " | Cost: " << node.cost
       << " | Type: " << node.setType << std::endl;
    return os;
}
