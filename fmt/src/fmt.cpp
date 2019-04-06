#include <fmt.h>

using namespace OpenRAVE;

class FMT : public ModuleBase
{

public:
    FMT(EnvironmentBasePtr penv, std::istream &ss); 
    
    virtual ~FMT() {}

    bool init(std::ostream &sout, std::istream &sinput);
};

// Called to create a new plugin
InterfaceBasePtr CreateInterfaceValidated(InterfaceType type, const std::string &interfacename, std::istream &sinput, EnvironmentBasePtr penv)
{
    if (type == PT_Module && interfacename == "fmt")
    {
        return InterfaceBasePtr(new FMT(penv, sinput));
    }

    return InterfaceBasePtr();
}


// Called to query available plugins
void GetPluginAttributesValidated(PLUGININFO &info)
{
    info.interfacenames[PT_Module].push_back("FMT");
}

// Called before plugin is terminated
OPENRAVE_PLUGIN_API void DestroyPlugin() { }

FMT::FMT(EnvironmentBasePtr penv, std::istream &ss)
: ModuleBase(penv)
{
    RegisterCommand("init", boost::bind(&FMT::init, this, _1, _2),
                    "Initializes the Planner");
}

bool FMT::init(std::ostream &sout, std::istream &sinput)
{
    std::cout << "Initializing FMT* Planner...\n";
    printMessage();
    return true;
}
