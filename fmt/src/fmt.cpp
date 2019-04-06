#include <iostream>
#include <cmath>
#include <vector>
#include <algorithm>
#include <math.h>
#include <random>
#include <time.h>
#include <chrono>
#include <memory>
#include <queue>

#include <openrave/openrave.h>
#include <openrave-core.h>
#include <openrave/plugin.h>
#include <boost/bind.hpp>
#include <openrave/plannerparameters.h>
#include <openrave/planningutils.h>

#include <fmt.h>

using namespace OpenRAVE;

class FMT : public ModuleBase
{
    size_t N;              // Number of samples
    size_t dim;

    std::vector<dReal> startConfig;
    std::vector<dReal> goalConfig;
    std::vector<std::vector<dReal>> world;

    std::mt19937 gen;   // random number generator
    std::vector<std::uniform_real_distribution<dReal>> dists;

    nodes_t closed;
    nodes_t unvisited;
    std::priority_queue<nodeptr_t, nodes_t, NodeComparator> open;

    Tree tree;

public:
    FMT(EnvironmentBasePtr penv, std::istream &ss); 
    
    virtual ~FMT() {}

    bool Init(std::ostream &sout, std::istream &sinput);

    bool SetStart(std::ostream &sout, std::istream &sinput);

    bool SetGoal(std::ostream &sout, std::istream &sinput);

    bool DefineWorld(std::ostream &sout, std::istream &sinput);

    bool SetNumSamples(std::ostream &sout, std::istream &sinput);

    bool PrintClass(std::ostream &sout, std::istream &sinput);

    bool Run(std::ostream &sout, std::istream &sinput);

    void TestOpenSet();

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
, N(0.0)
, dim(0)
{
    RegisterCommand("Init", boost::bind(&FMT::Init, this, _1, _2),
                    "Initializes the Planner");
    RegisterCommand("SetStart", boost::bind(&FMT::SetStart, this, _1, _2),
                    "Sets the start configuration");
    RegisterCommand("SetGoal", boost::bind(&FMT::SetGoal, this, _1, _2),
                    "Sets the goal configuration");
    RegisterCommand("DefineWorld", boost::bind(&FMT::DefineWorld, this, _1, _2),
                    "Sets the lower and upper limits of the configuration space");
    RegisterCommand("SetNumSamples", boost::bind(&FMT::SetNumSamples, this, _1, _2),
                    "Sets the number of samples to be used by planner");
    RegisterCommand("PrintClass", boost::bind(&FMT::PrintClass, this, _1, _2),
                    "Prints the member variables of FMT");
    RegisterCommand("Run", boost::bind(&FMT::Run, this, _1, _2),
                    "Run FMT planner");
}

bool FMT::Init(std::ostream &sout, std::istream &sinput)
{
    std::cout << "Initializing FMT* Planner...\n";

    std::random_device rd;
    gen.seed(rd());
    
    return true;
}

bool FMT::SetStart(std::ostream &sout, std::istream &sinput)
{
    std::string val;
    while (sinput >> val)
    {
        goalConfig.push_back(atof(val.c_str()));
    }
    return true;
}

bool FMT::SetGoal(std::ostream &sout, std::istream &sinput)
{
    std::string val;
    while (sinput >> val)
    {
        startConfig.push_back(atof(val.c_str()));
    }
    return true;
}

bool FMT::DefineWorld(std::ostream &sout, std::istream &sinput)
{
    std::string llim, ulim;
    while (sinput >> llim >> ulim)
    {
        std::vector<dReal> limits;
        limits.push_back(atof(llim.c_str()));
        limits.push_back(atof(ulim.c_str()));
        world.push_back(limits);
    }

    dim = world[0].size();

    // Setup random number generators for sampling
    for (unsigned i = 0; i < world.size(); ++i)
    {
        std::uniform_real_distribution<dReal> dis(world[i][0], world[i][1]);
        dists.push_back(dis);
    }
    return true;
}

bool FMT::SetNumSamples(std::ostream &sout, std::istream &sinput)
{
    std::string numSamples;
    sinput >> numSamples;
    N = atof(numSamples.c_str());

    tree.Reserve(N);

    return true;
}

bool FMT::PrintClass(std::ostream &sout, std::istream &sinput)
{
    std::cout << "\n---- FMT Class Values ----\n";
    std::cout << "Number of Samples: " << N << std::endl;
    std::cout << "World Config Space: " << std::endl;
    for (auto & val: world)
    {
        std::cout << "\t";
        printVector(val);
    }
    std::cout << "Start: "; printVector(startConfig);
    std::cout << "Goal:  "; printVector(goalConfig);

    return true;
}

void FMT::TestOpenSet()
{
    const size_t numNodes = 5;
    
    // Add the start node to the tree
    nodeptr_t noParent;
    tree.AddNode(startConfig, noParent);

    for (size_t i = 0; i < numNodes; ++i)
    {
        config_t q;
        for (size_t j = 0; j < dim; ++j)
        {
            q.push_back(dists[j](gen));
        }
        nodeptr_t node = std::make_shared<Node>(q, noParent);
        tree.AddNode(node);
        
        node->cost = numNodes-i;
        open.push(node);
    }

    tree.PrintNodes();

    std::cout << "\nNodes in Open Set:\n";
    for (size_t i = 0; i < numNodes; ++i)
    {
        std::cout << *open.top();
        open.pop();
    }
}

bool FMT::Run(std::ostream &sout, std::istream &sinput)
{
    TestOpenSet();
    return true;
}