#ifndef FMT_HPP
#define FMT_HPP

#include <helper.h>
#include <tree.h>

static const float red[4] = {1, 0, 0, 1};
static const float blue[4] = {0, 0, 1, 1};
static const float yellow[4] = {1, 1, 0.4, 1};
static const float babyblue[4] = {0, .8, 1, 1};

using path_t = std::vector<config_t>;

class NodeComparator
{
  public:
    int operator()(const nodeptr_t &p1, const nodeptr_t &p2)
    {
        return p1->cost > p2->cost;
    }
};


#endif