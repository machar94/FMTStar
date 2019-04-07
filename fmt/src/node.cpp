#include <node.h>

Node::Node(config_t &config, nodeptr_t &p, double c)
    : q(config), parent(p), cost(c)
{
}

std::ostream &operator<<(std::ostream &os, const Node &node)
{
    os << "Point: ";
    for (size_t i = 0; i < node.q.size() - 1; ++i)
    {
        os << node.q[i] << ", ";
    }
    os << node.q.back() << " | Cost: " << node.cost << std::endl;
    return os;
}
