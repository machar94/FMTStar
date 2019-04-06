#ifndef TREE_HPP
#define TREE_HPP

#include <memory>
#include <vector>

#include <openrave/openrave.h>

#include <node.h>

using nodes_t = std::vector<nodeptr_t>;

class Tree
{
    size_t size;
    nodes_t nodes;

public:
    Tree() : size(0) {};

    void Reserve(size_t N);

    void AddNode(nodeptr_t& node);

    void AddNode(config_t &q, nodeptr_t &parent);

    void PrintNodes();
};

#endif