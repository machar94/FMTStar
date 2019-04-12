#include <tree.h>

void Tree::Reserve(size_t N)
{
    nodes.reserve(N);
    size = N;
}

void Tree::AddNode(nodeptr_t &node)
{
    nodes.push_back(node);
}

void Tree::PrintNodes()
{
    std::cout << "Nodes in Tree:" << std::endl;
    for (size_t i = 0; i < nodes.size(); ++i)
    {
        std::cout << "Point " << i << ": ";
        for (size_t j = 0; j < nodes[i]->q.size() - 1; ++j)
        {
            std::cout << nodes[i]->q[j] << ", ";
        }
        std::cout << nodes[i]->q.back() << " | ";
        std::cout << "Cost: " << nodes[i]->cost << std::endl;
    }
}

void Tree::ClearTree()
{
    nodes.clear();
    Reserve(size);
}