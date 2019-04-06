#ifndef FMT_HPP
#define FMT_HPP


#include <helper.h>
#include <node.h>

typedef std::shared_ptr<Node> nodeptr_t;
typedef std::vector<nodeptr_t> nodes_t;

class NodeComparator 
{ 
public: 
    int operator() (const nodeptr_t& p1, const nodeptr_t& p2) 
    { 
        return p1->cost > p2->cost; 
    } 
}; 

#endif