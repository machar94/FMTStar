#include <cmath>

#include <helper.h>

template <class T>
inline void printVector(std::vector<T> &vec)
{
    for (auto &val : vec)
    {
        std::cout << val << " ";
    }
    std::cout << std::endl;
}

double CalcEuclidianDist(const nodeptr_t &n1, const nodeptr_t &n2)
{
    return CalcEuclidianDist(n1->q, n2->q);
}

double CalcEuclidianDist(const config_t &q1, const config_t &q2)
{
    double sum = 0.0;
    for (size_t i = 0; i < q1.size(); ++i)
    {
        sum += (q1[i] - q2[i]) * (q1[i] - q2[i]);
    }
    return sqrtf(sum);
}

void FindNearestNeighbors(
    nodes_t &set,
    nodes_t &neighbors,
    nodeptr_t &curr,
    double radius,
    std::unordered_map<nodeptr_t, nodes_t> &neighborTable)
{
    assert(neighbors.size() == 0);

    // Check first to see if the nearest neighbor calculation has already been
    // made
    if (neighborTable.count(curr) != 0)
    {
        neighbors = neighborTable[curr];
    }
    else
    {
        for (auto &node : set)
        {
            double dist = CalcEuclidianDist(curr->q, node->q);
            if (dist < radius && curr != node)
            {
                neighbors.push_back(node);
            }
        }
        neighborTable.emplace(curr, neighbors);
    }
}

std::vector<dReal> CalcDirVector(
    const config_t &v1, 
    const config_t &v2,
    double stepSize)
{
    std::vector<dReal> dir(v2.size(), 0.0);
    double dist = CalcEuclidianDist(v1, v2);

    for (size_t i = 0; i < v2.size(); ++i)
    {
        dir[i] = (v2[i] - v1[i]) / dist * stepSize;
    }
    return dir;
}

config_t operator+(const config_t &v1, const config_t &v2)
{
    config_t v3(v1.size(), 0.0);
    for (size_t i = 0; i < v1.size(); ++i)
    {
        v3[i] = v1[i] + v2[i];
    }
    return v3;
}
