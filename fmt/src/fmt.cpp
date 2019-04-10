#include <random>
#include <time.h>
#include <chrono>
#include <queue>
#include <limits>

#include <openrave-core.h>
#include <openrave/plugin.h>
#include <boost/bind.hpp>
#include <boost/functional/hash.hpp>
#include <openrave/plannerparameters.h>
#include <openrave/planningutils.h>

#include <fmt.h>

using namespace OpenRAVE;

class FMT : public ModuleBase
{
    size_t N; // Number of samples
    size_t dim;
    double radius; // Radius for nearest neighbor search
    double stepSize;
    int seed;
    std::string planner;

    std::vector<dReal> startConfig;
    std::vector<dReal> goalConfig;
    std::vector<std::vector<dReal>> world;
    nodeptr_t goalNode;

    std::mt19937 gen; // random number generator
    std::vector<std::uniform_real_distribution<dReal>> dists;

    nodes_t closed;
    nodes_t unvisited;
    nodes_t total; // total = open + closed + unvisited

    // Handle to all of the points being plotted
    std::vector<GraphHandlePtr> ghandle;

    class OpenSet
    {
        std::priority_queue<nodeptr_t, nodes_t, NodeComparator> open;

      public:
        void push(nodeptr_t &node)
        {
            node->setType = OPEN;
            open.push(node);
        }

        const nodeptr_t &top() { return open.top(); }

        void pop() { open.pop(); }

        size_t size() const { return open.size(); }
    };

    OpenSet open;

    Tree tree;
    RobotBasePtr robot;

    std::unordered_map<nodeptr_t, nodes_t> neighborTable;

    nodeptr_t currNode;

  public:
    FMT(EnvironmentBasePtr penv, std::istream &ss);

    virtual ~FMT() {}

    bool Init(std::ostream &sout, std::istream &sinput);

    bool SetStart(std::ostream &sout, std::istream &sinput);

    bool SetGoal(std::ostream &sout, std::istream &sinput);

    bool DefineWorld(std::ostream &sout, std::istream &sinput);

    bool SetNumSamples(std::ostream &sout, std::istream &sinput);

    bool SetRadius(std::ostream &sout, std::istream &sinput);

    bool SetStepSize(std::ostream &sout, std::istream &sinput);

    bool SetSeed(std::ostream &sout, std::istream &sinput);

    bool SetPlanner(std::ostream &sout, std::istream &sinput);

    bool PrintClass(std::ostream &sout, std::istream &sinput);

    bool Run(std::ostream &sout, std::istream &sinput);

  private:
    // Initialize open, closed and unvisited sets
    // 1. Open set should have only start
    // 2. Closed set should be empty
    // 3. Unvisited should have N-1 configs (goal + N-2 samples)
    void SetupSets();

    // Addes N-1 samples (including goal config) to unvisited set
    void GenerateSamples();

    // Checks if a configuration is valid
    bool CheckCollision(const config_t &config) const;

    // Returns the closest node in open. This should always return a valid node
    // as the node used to generate curr was originally in the open set
    nodeptr_t FindClosestNodeInOpen(
        const nodes_t &nodes,
        const nodeptr_t &curr) const;

    // Checks to see if the path from n1 to n2 is collision free
    bool CollisionFree(nodeptr_t &n1, nodeptr_t &n2);

    bool FindPath();

    path_t BuildPath();

    void PlotPath(const float color[4], path_t &path);

    void ExecuteTrajectory(path_t &path);
    /* Testing Functionality */

    // Tests to see if min heap is working correctly
    void TestOpenSet();

    // Tests to see the size of each set (total, open, closed, unvisted)
    void TestSetSizes();
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
OPENRAVE_PLUGIN_API void DestroyPlugin() {}

FMT::FMT(EnvironmentBasePtr penv, std::istream &ss)
    : ModuleBase(penv), N(0.0), dim(0), stepSize(0.1), seed(-1), planner("naive")
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
    RegisterCommand("SetRadius", boost::bind(&FMT::SetRadius, this, _1, _2),
                    "Sets the radius used by nearest neighbors");
    RegisterCommand("SetStepSize", boost::bind(&FMT::SetStepSize, this, _1, _2),
                    "Sets the step size to be used in collision checking");
    RegisterCommand("SetSeed", boost::bind(&FMT::SetSeed, this, _1, _2),
                    "Sets the random seed to be used for the simulation");
    RegisterCommand("SetPlanner", boost::bind(&FMT::SetPlanner, this, _1, _2),
                    "Choose either of the following: naive, smart");
    RegisterCommand("PrintClass", boost::bind(&FMT::PrintClass, this, _1, _2),
                    "Prints the member variables of FMT");
    RegisterCommand("Run", boost::bind(&FMT::Run, this, _1, _2),
                    "Run FMT planner");
}

bool FMT::Init(std::ostream &sout, std::istream &sinput)
{
    std::cout << "Initializing FMT* Planner...\n";

    std::vector<RobotBasePtr> vrobots;
    GetEnv()->GetRobots(vrobots);
    robot = vrobots.at(0);

    if (seed == -1)
    {
        std::random_device rd;
        gen.seed(rd());
    }
    else
    {
        gen.seed(seed);
    }

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

bool FMT::SetRadius(std::ostream &sout, std::istream &sinput)
{
    std::string val;
    sinput >> val;
    radius = atof(val.c_str());

    return true;
}

bool FMT::SetStepSize(std::ostream &sout, std::istream &sinput)
{
    std::string val;
    sinput >> val;
    stepSize = atof(val.c_str());

    return true;
}

bool FMT::SetSeed(std::ostream &sout, std::istream &sinput)
{
    std::string val;
    sinput >> val;
    seed = atof(val.c_str());

    return true;
}

bool FMT::SetPlanner(std::ostream &sout, std::istream &sinput)
{
    std::string val;
    sinput >> val;

    if (val == "naive" || val == "smart")
    {
        planner = val;
    }
    else
    {
        std::cout << "Unrecognized planner.. using naive method" << std::endl;
        planner = "naive";
    }

    return true;
}

bool FMT::PrintClass(std::ostream &sout, std::istream &sinput)
{
    std::cout << "\n---- FMT Class Values ----\n";
    std::cout << "Number of Samples: " << N << std::endl;
    std::cout << "World Config Space: " << std::endl;
    for (auto &val : world)
    {
        std::cout << "\t";
        printVector(val);
    }
    std::cout << "Start: ";
    printVector(startConfig);
    std::cout << "Goal:  ";
    printVector(goalConfig);
    std::cout << "Radius: " << radius << std::endl;

    return true;
}

void FMT::TestOpenSet()
{
    const size_t numNodes = 5;

    nodeptr_t noParent;
    nodeptr_t node = std::make_shared<Node>(startConfig, noParent);

    // Add the start node to the tree
    tree.AddNode(node);

    for (size_t i = 0; i < numNodes; ++i)
    {
        config_t q;
        for (size_t j = 0; j < dim; ++j)
        {
            q.push_back(dists[j](gen));
        }
        node = std::make_shared<Node>(q, noParent);
        tree.AddNode(node);

        node->cost = numNodes - i;
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
    // Initialize and set the open, unvisited and closed sets
    SetupSets();

    bool isSolved = FindPath();

    path_t path = BuildPath();

    PlotPath(red, path);

    ExecuteTrajectory(path);

    return true;
}

void FMT::SetupSets()
{
    nodeptr_t nullParent;
    nodeptr_t start = std::make_shared<Node>(startConfig, nullParent);
    open.push(start);
    tree.AddNode(start);

    // Add valid configurations including goal to unvisited set
    GenerateSamples();

    // Total = open + unvisited + closed
    total = unvisited;
    total.push_back(start);
}

void FMT::GenerateSamples()
{
    // Generate N-2 random samples in configuration space and add to the
    // unvistied set. Afterwards add the goal configuration to the set.
    for (size_t i = 0; i < N - 2; ++i)
    {
        // Generate a valid configuration
        config_t config(dim, 0.0);
        do
        {
            for (size_t j = 0; j < dim; ++j)
            {
                config[j] = dists[j](gen);
            }

        } while (CheckCollision(config));

        // Create node and push it on to the unvisited nodes list
        nodeptr_t nodeptr = std::make_shared<Node>(config, UNVISITED);
        unvisited.push_back(nodeptr);
    }

    // Goal configuration needs to be in free space
    nodeptr_t nodeptr = std::make_shared<Node>(goalConfig, UNVISITED);
    unvisited.push_back(nodeptr);
}

bool FMT::CheckCollision(const config_t &config) const
{
    robot->SetActiveDOFValues(config);

    bool ret;
    ret = GetEnv()->CheckCollision(RobotBaseConstPtr(robot));
    ret = robot->CheckSelfCollision() || ret;
    return ret;
}

void FMT::TestSetSizes()
{
    std::cout << "\nPrinting out the size of the sets...\n";
    std::cout << "OpenSet  : " << open.size() << std::endl;
    std::cout << "ClosedSet: " << closed.size() << std::endl;
    std::cout << "Unvisited: " << unvisited.size() << std::endl;
    std::cout << "Total    : " << total.size() << std::endl;
}

nodeptr_t FMT::FindClosestNodeInOpen(
    const nodes_t &nodes,
    const nodeptr_t &curr) const
{
    double minCost = std::numeric_limits<double>::max();
    nodeptr_t closestNode;

    for (const auto &node : nodes)
    {
        if (node->setType != OPEN)
        {
            continue;
        }

        double cost = CalcEuclidianDist(node, curr);
        // Node represents y's cost to come in tree
        if ((cost + node->cost) < minCost)
        {
            minCost = cost + node->cost;
            closestNode = node;
        }
    }

    return closestNode;
}

bool FMT::CollisionFree(nodeptr_t &n1, nodeptr_t &n2)
{
    config_t q1 = n1->q, q2 = n2->q;
    auto dir = CalcDirVector(q1, q2, stepSize);

    while (true)
    {
        double dist = CalcEuclidianDist(q1, q2);
        if (dist < stepSize)
        {
            return true;
        }

        q1 = q1 + dir;
        if (CheckCollision(q1))
        {
            return false;
        }
    }
}

bool FMT::FindPath()
{
    // Set init to the current node
    currNode = open.top();

    while (currNode->q != goalConfig)
    {
        nodes_t open_new;

        // Other than start config, should always find neighbors through table
        nodes_t currNN;
        FindNearestNeighbors(total, currNN, currNode, radius, neighborTable);

        // Only perform wiring for nodes in unvisited set
        for (size_t i = 0; i < currNN.size(); ++i)
        {
            if (currNN[i]->setType != UNVISITED)
            {
                continue;
            }

            nodes_t xNN; // Neighbor x's nearest neighbors
            FindNearestNeighbors(total, xNN, currNN[i], radius, neighborTable);

            nodeptr_t yMin = FindClosestNodeInOpen(xNN, currNN[i]);

            if (CollisionFree(yMin, currNN[i]))
            {
                // Add node x to tree and add its parent is yMin
                currNN[i]->parent = yMin;
                tree.AddNode(currNN[i]);

                // Add node x to the open_new set
                // Wait to change to open
                open_new.push_back(currNN[i]);

                // Remove node x from unvisited and change to open
                auto it = std::remove(unvisited.begin(), unvisited.end(), currNN[i]);
                unvisited.erase(it, unvisited.end());

                // Update cost of node x (y->x) + cost(y)
                currNN[i]->cost = yMin->cost + CalcEuclidianDist(yMin, currNN[i]);
            }
        }

        // Update open list: (Vopen U Vopen_new) \ {z}
        open.pop();
        for (auto &node : open_new)
        {
            open.push(node);
        }

        if (open.size() == 0)
        {
            return false;
        }
        currNode = open.top();
    }

    goalNode = currNode;

    return true;
}

path_t FMT::BuildPath()
{
    path_t path;
    nodeptr_t currNode = goalNode;

    while (currNode.get())
    {
        path.push_back(currNode->q);
        currNode = currNode->parent;
    }

    return path;
}

void FMT::PlotPath(const float color[4], path_t &path)
{
    std::vector<float> p(3, 0.05);
    for (auto it = path.rbegin(); it != path.rend(); ++it)
    {
        p[0] = (*it)[0];
        p[1] = (*it)[1];
        ghandle.push_back(GetEnv()->plot3(&p[0], 1, 12, 5, color, 0));
    }
}

void FMT::ExecuteTrajectory(path_t &path)
{
    robot->SetActiveDOFValues(startConfig);

    TrajectoryBasePtr traj = RaveCreateTrajectory(GetEnv(), "");
    traj->Init(robot->GetActiveConfigurationSpecification());

    int i = 0;
    for (auto it = path.begin(); it != path.end(); ++it)
    {
        traj->Insert(i, *it);
        i++;
    }
    std::vector<double> maxVel(2, 1.0);
    std::vector<double> maxAcc(2, 5.0);
    OpenRAVE::planningutils::RetimeAffineTrajectory(traj, maxVel, maxAcc);
    robot->GetController()->SetPath(traj);
}