#ifndef HELPER_HPP
#define HELPER_HPP

#include <iostream>
#include <vector>
#include <assert.h>
#include <unordered_map>

#include <tree.h>

template <class T>
inline void printVector(std::vector<T> &vec);

void findNearestNeighbors(
    nodes_t &set,
    nodes_t &neighbors,
    nodeptr_t &curr,
    double radius,
    std::unordered_map<nodeptr_t, nodes_t> &neighborTable);

double CalcEuclidianDist(const config_t &q1, const config_t &q2);

#endif