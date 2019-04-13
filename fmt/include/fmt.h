#ifndef FMT_HPP
#define FMT_HPP

#include <helper.h>
#include <tree.h>

static const float red[4] = {1, 0, 0, 1};
static const float blue[4] = {0, 0, 1, 1};
static const float yellow[4] = {1, 1, 0.4, 1};
static const float babyblue[4] = {0, .8, 1, 1};
static const float purple[4] = {0.6, 0, 0.6, 1};
static const float green[4] = {0, 0.8, 0, 1};
static const float orange[4] = {1, 0.4, 0, 1};

class Colors
{
  std::vector<const float*> colors;
  uint counter;
  public:
    Colors() : counter(0)
    {
      colors.push_back(yellow);
      colors.push_back(green);
      colors.push_back(red);
      colors.push_back(babyblue);
      colors.push_back(orange);
      colors.push_back(blue);
      colors.push_back(purple);
    }

    const float* GetColor()
    {
        const float* color = colors[counter];
        counter++;
        if (counter == colors.size())
        {
          counter = 0;
          std::cout << "Warning... ran out of colors. Will start reusing old colors again." << std::endl;
        }
        return color;
    }

};

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