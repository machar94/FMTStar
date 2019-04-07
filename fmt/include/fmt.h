#ifndef FMT_HPP
#define FMT_HPP

#include <helper.h>
#include <tree.h>

class NodeComparator
{
  public:
    int operator()(const nodeptr_t &p1, const nodeptr_t &p2)
    {
        return p1->cost > p2->cost;
    }
};


#endif