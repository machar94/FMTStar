#ifndef NODE_HPP
#define NODE_HPP

#include <openrave/openrave.h>

#include <vector>
#include <sstream>

using OpenRAVE::dReal;
using config_t = std::vector<dReal>;

class Node;
using nodeptr_t = std::shared_ptr<Node>;

using nodes_t = std::vector<nodeptr_t>;

enum SetType
{
    OPEN,
    UNVISITED,
    CLOSED,
    INVALID
};

class Node
{
  public:
    config_t  q;
    nodeptr_t parent;
    SetType   setType;
    double    cost;

  public:
    Node(const config_t &config, SetType t = INVALID, double c = 0.0); 

    Node(const config_t &config, nodeptr_t &parent, SetType t = INVALID, double c = 0.0);

    void SetParent(nodeptr_t &p)
    {
        parent = p;
    }

    friend std::ostream &operator<<(std::ostream &os, const Node &node);
};

#endif