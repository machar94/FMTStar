#ifndef TRIGGER_HPP
#define TRIGGER_HPP

#include <string>
#include <utility>
#include <vector>
#include <sstream>
#include <memory>

#include <openrave/openrave.h>

using namespace OpenRAVE;

class Trigger;
using dynobj_t = std::pair<std::string, std::vector<dReal>>;
using triggers_t = std::vector<std::shared_ptr<Trigger>>;

class Trigger
{
public:
    double triggerPoint;
    std::vector<dynobj_t> dynobjs;

public:
    // Class is initialized with string holding paramters
    // <trigger point (double)> <KinBody1> <x-coord> <y-coord> <KinBody2> ...
    Trigger(std::istream &os);

};

#endif