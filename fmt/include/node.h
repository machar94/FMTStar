#ifndef NODE_HPP
#define NODE_HPP

#include <openrave/openrave.h>

#include <vector>

using OpenRAVE::dReal;

typedef std::vector<dReal> config_t;

class Node
{
public:
    double cost;
    config_t q;
    std::shared_ptr<Node> parent;

public:
    Node(config_t config) : q(config), cost(0.0) {};

    void SetParent(std::shared_ptr<Node>& p)
    { 
        parent = p;
    }
};

#endif