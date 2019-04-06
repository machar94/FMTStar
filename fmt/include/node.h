#ifndef NODE_HPP
#define NODE_HPP

#include <openrave/openrave.h>

#include <vector>
#include <sstream>

using OpenRAVE::dReal;
using config_t = std::vector<dReal>;

class Node;
using nodeptr_t = std::shared_ptr<Node>;

class Node
{
public:
    config_t q;
    nodeptr_t parent;
    double cost;

public:
    Node(config_t &config, double c = 0.0) : q(config), cost(c) {};

    Node(config_t &config, nodeptr_t &parent, double c = 0.0);

    void SetParent(nodeptr_t& p)
    { 
        parent = p;
    }

    friend std::ostream& operator<<(std::ostream& os, const Node& node);
};

#endif