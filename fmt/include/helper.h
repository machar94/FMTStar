#ifndef HELPER_HPP
#define HELPER_HPP

#include <iostream>
#include <vector>

template<class T>
inline void printVector(std::vector<T> &vec)
{
    for (auto val : vec) 
    {
        std::cout << val << " ";
    }
    std::cout << std::endl;
}

#endif 