#include <trigger.h>

Trigger::Trigger(std::istream &is)
{
    // Read in the trigger point (x coordinate)
    is >> triggerPoint;

    std::string bodyName;
    double x, y;
    while (is >> bodyName >> x >> y)
    {
        std::vector<dReal> position;
        position.push_back(x);
        position.push_back(y);
        dynobj_t obj(bodyName, position);
        dynobjs.push_back(obj);
    }
}