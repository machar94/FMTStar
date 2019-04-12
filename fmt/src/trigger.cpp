#include <trigger.h>

Trigger::Trigger(std::istream &is)
{
    // Read in the trigger point (x coordinate)
    is >> triggerPoint;

    std::string bodyName;
    double x, y, rot;
    while (is >> bodyName >> x >> y >> rot)
    {
        Transformation t; 
        t.position.push_back(x);
        t.position.push_back(y);
        t.rotation = rot;
        dynobj_t obj(bodyName, t);
        dynobjs.push_back(obj);
    }
}