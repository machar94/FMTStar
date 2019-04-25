#include <random>
#include <time.h>
#include <chrono>
#include <queue>
#include <limits>
#include <thread>
#include <fstream>

#include <openrave-core.h>
#include <openrave/plugin.h>
#include <boost/bind.hpp>
#include <boost/functional/hash.hpp>
#include <openrave/plannerparameters.h>
#include <openrave/planningutils.h>

#include <fmt.h>
#include <trigger.h>

using namespace OpenRAVE;

class FMT : public ModuleBase
{
    size_t N; // Number of samples
    size_t dim;
    double radius; // Radius for nearest neighbor search
    double stepSize;
    double origPathLength;
    int seed;
    std::string planner;
    uint fwdCollisionCheck;

    std::vector<dReal> startConfig;
    std::vector<dReal> goalConfig;
    std::vector<std::vector<dReal>> world;
    nodeptr_t goalNode;
    nodeptr_t currNode;

    std::mt19937 gen; // random number generator
    std::vector<std::uniform_real_distribution<dReal>> dists;

    nodes_t total; // total = open + closed + unvisited

    // Handle to all of the points being plotted
    std::vector<GraphHandlePtr> ghandle;

    triggers_t triggers;

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

        void clearSet()
        {
            while (!open.empty())
            {
                open.pop();
            }
        }
    };

    RobotBasePtr robot;

    std::unordered_map<nodeptr_t, nodes_t> neighborTable;
    
    OpenSet open;
    Colors colors;

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

    bool CreateTrigger(std::ostream &sout, std::istream &sinput);

    bool SetFwdCollisionCheck(std::ostream &sout, std::istream &sinput);

    bool PrintClass(std::ostream &sout, std::istream &sinput);

    bool Run(std::ostream &sout, std::istream &sinput);

    bool RunWithReplan(std::ostream &sout, std::istream &sinput);

  private:
    // Initialize open, closed and unvisited sets
    // 1. Open set should have only start
    // 2. Closed set should be empty
    // 3. There should be N-1 configs unvisited (goal + N-2 samples)
    void SetupSets(config_t &startCfg, path_t &path, std::ostream &sout);

    // Addes N-1 samples (including goal config) to unvisited set
    void GenerateSamples(path_t &path, std::ostream &sout);

    // Checks if a configuration is valid
    bool CheckCollision(const config_t &config) const;

    // Returns the closest node in open. This should always return a valid node
    // as the node used to generate curr was originally in the open set
    nodeptr_t FindClosestNodeInOpen(
        const nodes_t &nodes,
        const nodeptr_t &curr) const;

    // Checks to see if the path from n1 to n2 is collision free
    bool CollisionFree(nodeptr_t &n1, nodeptr_t &n2);

    bool FindPath(const path_t & region);

    path_t BuildPath();

    void PlotPath(const float color[4], path_t &path);

    void PlotSet(const float color[4], const nodes_t &nodes, SetType type);

    void ExecuteTrajectory(path_t &path, config_t &startCfg);

    // Not actually multithreaded but simulates checking robot collision a
    // couple steps ahead and then executing the steps. Additionally checks
    // whether to change the environment based on the triggers
    //
    // Returns true when robot has reached the goal config, false if the
    // triggers require a replanning
    bool ExecuteMultiThreadTraj(path_t &path, config_t &startCfg);

    // Assumption: Triggers should be in order of when the would get tripped
    // Triggers get tripped from based on the x position of the robot. The
    // moment the x position of the robot in the world coordinate sytem crosses
    // the trigger point, the environment is updated based on the trigger class
    void CheckTriggers(const config_t &currPos);

    double PathLength(const path_t &path);

    void ClearCosts();

    uint GetSetSize(const nodes_t &nodes, SetType type);

    bool IsNodeInGoalRegion(const nodeptr_t & node, const path_t & region);

    void PrintClass_Internal();

    /* Testing Functionality */

    // Tests to see if min heap is working correctly
    void TestOpenSet();

    // Tests to see the size of each set (total, open, closed, unvisted)
    void TestSetSizes();

    // Tests whether the trigger was built properly
    void TestTriggers();
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
    : ModuleBase(penv), N(0.0), dim(0), stepSize(0.1), seed(-1), planner("naive"), colors(Colors())
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
    RegisterCommand("SetFwdCollisionCheck", boost::bind(&FMT::SetFwdCollisionCheck, this, _1, _2),
                    "When executing a trajectory, how many configuraitons forward"
                    " should the robot check to make sure it's positons are valid");
    RegisterCommand("CreateTrigger", boost::bind(&FMT::CreateTrigger, this, _1, _2),
                    "Creates a trigger to specify where to place tables upon "
                    "robot passing the trigger point passed in");
    RegisterCommand("PrintClass", boost::bind(&FMT::PrintClass, this, _1, _2),
                    "Prints the member variables of FMT");
    RegisterCommand("Run", boost::bind(&FMT::Run, this, _1, _2),
                    "Run FMT planner");
    RegisterCommand("RunWithReplan", boost::bind(&FMT::RunWithReplan, this, _1, _2),
                    "Run FMT planner with replanning algo and dynamic objects");
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
        startConfig.push_back(atof(val.c_str()));
    }
    return true;
}

bool FMT::SetGoal(std::ostream &sout, std::istream &sinput)
{
    std::string val;
    while (sinput >> val)
    {
        goalConfig.push_back(atof(val.c_str()));
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
    N = atoi(numSamples.c_str());
    total.reserve(N);

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
    seed = atoi(val.c_str());

    if (seed == -1)
    {
        std::random_device rd;
        gen.seed(rd());
    }
    else
    {
        std::cout << "Setting up seed..." << std::endl;
        gen.seed(seed);
    }

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

bool FMT::SetFwdCollisionCheck(std::ostream &sout, std::istream &sinput)
{
    std::string val;
    sinput >> val;
    fwdCollisionCheck = atoi(val.c_str());

    return true;
}

bool FMT::CreateTrigger(std::ostream &sout, std::istream &sinput)
{
    std::shared_ptr<Trigger> trigger = std::make_shared<Trigger>(sinput);
    triggers.push_back(trigger);

    return true;
}

bool FMT::PrintClass(std::ostream &sout, std::istream &sinput)
{
    PrintClass_Internal();

    return true;
}

void FMT::TestOpenSet()
{
    const size_t numNodes = 5;

    nodeptr_t noParent;
    nodeptr_t node = std::make_shared<Node>(startConfig, noParent);

    for (uint i = 0; i < numNodes; ++i)
    {
        config_t q;
        for (uint j = 0; j < dim; ++j)
        {
            q.push_back(dists[j](gen));
        }
        node = std::make_shared<Node>(q, noParent);

        node->cost = numNodes - i;
        open.push(node);
    }

    std::cout << "\nNodes in Open Set:\n";
    for (uint i = 0; i < numNodes; ++i)
    {
        std::cout << *open.top();
        open.pop();
    }
}

bool FMT::Run(std::ostream &sout, std::istream &sinput)
{
    GetEnv()->GetMutex().lock();
    // PrintClass_Internal();

    // TestTriggers();

    // Initialize and set the open, unvisited and closed sets
    path_t path(1, goalConfig);
    SetupSets(startConfig, path, sout); // replan set to false

    FindPath(path);

    path = BuildPath();
    std::cout << "OpenSet: " << open.size() << std::endl;
    PlotSet(colors.GetColor(), total, CLOSED);
    PlotSet(colors.GetColor(), total, UNVISITED);
    PlotPath(colors.GetColor(), path);
    TestSetSizes();

    ExecuteTrajectory(path, startConfig);
    GetEnv()->GetMutex().unlock();

    return true;
}

bool FMT::RunWithReplan(std::ostream &sout, std::istream &sinput)
{
    ghandle.clear();

    GetEnv()->GetMutex().lock();
    robot->SetActiveDOFValues(startConfig);
    config_t currConfig = startConfig;
    path_t path(1, goalConfig); // Goal region includes only goal config originally

    // TestTriggers();

    bool reachedGoal = false;
    while (reachedGoal == false)
    {
        auto begin = std::chrono::high_resolution_clock::now();
        SetupSets(currConfig, path, sout);
        printVector(currConfig);
        TestSetSizes();
        bool foundPath = FindPath(path);
        auto end = std::chrono::high_resolution_clock::now();
        auto dur = end - begin; 
        auto ms = std::chrono::duration_cast<std::chrono::milliseconds>(dur).count();        
        sout << ms << " ";

        if (!foundPath)
        {
            std::cout << "Unable to find a path!" << std::endl;
            break;
        }

        path = BuildPath();
        origPathLength = PathLength(path);
        PlotPath(colors.GetColor(), path);

        reachedGoal = ExecuteMultiThreadTraj(path, currConfig);
        if (reachedGoal)
        {
            break;
        }
    }
    GetEnv()->GetMutex().unlock();
    return true;
}

void FMT::SetupSets(config_t &startCfg, path_t &path, std::ostream &sout)
{
    total.clear();
    open.clearSet();
    // Add valid configurations including goal to unvisited set
    GenerateSamples(path, sout);

    // Add start to open and total sets 
    nodeptr_t start = std::make_shared<Node>(startCfg, OPEN);
    open.push(start);
        
    // Total = open + unvisited + closed
    total.push_back(start);
}

void FMT::GenerateSamples(path_t &path, std::ostream &sout)
{
    uint nodesAdded = 0;
    if (planner == "smart")
    {
        for (auto &config : path)
        {
            if (!CheckCollision(config))
            {
                nodeptr_t nodeptr = std::make_shared<Node>(config, UNVISITED);
                total.push_back(nodeptr);
                nodesAdded++;
            }
        }
        std::cout << "\n\nStarted off with " << total.size() << " nodes from previous path" << std::endl;
        sout << total.size() << " ";
    }

    
    // Generate N-2 random samples in configuration space and add to the
    // unvistied set. Afterwards add the goal configuration to the set.
    std::cout << "Sampling " << N - 2 - nodesAdded << " nodes randomly..." << std::endl;
    
    assert(N - 2 - nodesAdded > 0);
    for (uint i = 0; i < N - 2 - nodesAdded; ++i)
    {
        // Generate a valid configuration
        config_t config(dim, 0.0);
        do
        {
            for (uint j = 0; j < dim; ++j)
            {
                config[j] = dists[j](gen);
            }

        } while (CheckCollision(config));

        // Mark node as unvisited and push to total set
        nodeptr_t nodeptr = std::make_shared<Node>(config, UNVISITED);
        total.push_back(nodeptr);
    }

    // Check that goal configuration is in free space
    assert(CheckCollision(goalConfig) == false);
    nodeptr_t nodeptr = std::make_shared<Node>(goalConfig, UNVISITED);
    total.push_back(nodeptr);
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
    std::cout << "Closed   : " << GetSetSize(total, CLOSED) << std::endl;
    std::cout << "Unvisited: " << GetSetSize(total, UNVISITED) << std::endl;
    std::cout << "Total    : " << total.size() << std::endl;
}

uint FMT::GetSetSize(const nodes_t &nodes, SetType type)
{
    uint count = 0;
    for (const auto & node : nodes)
    {
        if (node->setType == type)
        {
            count++;
        }
    }
    return count;
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

bool FMT::FindPath(const path_t & region)
{
    // Set init to the current node
    currNode = open.top();

    while (IsNodeInGoalRegion(currNode, region) == false)
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
                // Update node x's parent to yMin
                currNN[i]->parent = yMin;

                // Add node x to the open_new set
                // Wait to change to open
                open_new.push_back(currNN[i]);

                // Update cost of node x (y->x) + cost(y)
                currNN[i]->cost = yMin->cost + CalcEuclidianDist(yMin, currNN[i]);
            }
        }

        // Update closed list: Vclosed U {Z}
        currNode->setType = CLOSED;

        // Update open list: (Vopen U Vopen_new) \ {z}
        open.pop();
        for (auto &node : open_new)
        {
            // Nodes go from unvisited set to open set
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

    // Don't add start location to path
    while (currNode->parent.get())
    {
        path.push_back(currNode->q);
        currNode = currNode->parent;
    }

    /*
    std::ofstream outFile("path.txt");
    if (outFile.is_open())
    {
        for (const auto &loc : path)
        {
            outFile << loc[0] << " " << loc[1] << std::endl;
        }
    }
    else
    {
        std::cout << "Something went wrong opening the path.txt file :( " << std::endl;
    }
    outFile.close();
    */
    return path;
}

void FMT::PlotPath(const float color[4], path_t &path)
{
    std::vector<float> p(3, 0.05);
    for (auto it = path.rbegin(); it != path.rend(); ++it)
    {
        p[0] = (*it)[0];
        p[1] = (*it)[1];
        ghandle.push_back(GetEnv()->plot3(&p[0], 1, 12, 8, color, 0));
    }
}

void FMT::ExecuteTrajectory(path_t &path, config_t &startCfg)
{
    robot->SetActiveDOFValues(startCfg);

    TrajectoryBasePtr traj = RaveCreateTrajectory(GetEnv(), "");
    traj->Init(robot->GetActiveConfigurationSpecification());

    int i = 0;
    for (auto it = path.rbegin(); it != path.rend(); ++it)
    {
        traj->Insert(i, *it);
        i++;
    }
    std::vector<double> maxVel(2, 1.0);
    std::vector<double> maxAcc(2, 5.0);
    OpenRAVE::planningutils::RetimeAffineTrajectory(traj, maxVel, maxAcc);
    robot->GetController()->SetPath(traj);

    GetEnv()->GetMutex().unlock();
    // std::cout << "Unlocked the environment" << std::endl;

    // std::cout << "Waiting for controller to finish" << std::endl;
    while (!robot->GetController()->IsDone())
    {
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
        // std::cout << "Not done yet.. sleeping" << std::endl;
    }
    // std::cout << "Controller is finished" << std::endl;
    GetEnv()->GetMutex().lock();
    // std::cout << "Gave up lock" << std::endl;
}

void FMT::PrintClass_Internal()
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
    std::cout << "Step size: " << stepSize << std::endl;
    std::cout << "Seed: " << seed << std::endl;
    std::cout << "Planner: " << planner << std::endl;
    std::cout << "# FWD Checks: " << fwdCollisionCheck << std::endl;
}

void FMT::PlotSet(const float color[4], const nodes_t &nodes, SetType type)
{
    std::vector<float> p(3, 0.05);
    for (const auto &node : nodes)
    {
        if (node->setType == type)
        {
            p[0] = node->q[0];
            p[1] = node->q[1];
            ghandle.push_back(GetEnv()->plot3(&p[0], 1, 12, 5, color, 0));
        }
    }
}

void FMT::TestTriggers()
{
    std::cout << "Printing out Triggers..." << std::endl;
    for (const auto &trigger : triggers)
    {
        std::cout << "Trigger Point: " << trigger->triggerPoint << std::endl;
        for (const auto &obj : trigger->dynobjs)
        {
            std::cout << "ObjName: " << obj.first
                      << " Rot: " << obj.second.rotation << " Loc: ";
            printVector(obj.second.position);
        }
    }
    std::cout << std::endl;
}

bool FMT::ExecuteMultiThreadTraj(path_t &path, config_t &startCfg)
{
    bool reachedGoal = false;

    while (!reachedGoal)
    {
        // Step 1. Check to see if the environment should update (Trigger)
        CheckTriggers(startCfg);

        // Step 2. Do a forward check to see if next couple of steps are free to move ahead
        bool robotCollided = false;
        uint lim = (fwdCollisionCheck < path.size()) ? fwdCollisionCheck : path.size();
        auto it = path.rbegin();
        for (uint i = 0; i < lim; ++i)
        {
            // std::cout << (*it)[0] << " " << (*it)[1] << std::endl;
            if (CheckCollision((*it)))
            {
                robotCollided = true;
                break;
            }
            it++;
        }
        // Need to replan
        if (robotCollided)
        {
            return reachedGoal;
        }

        // Step 3. Execute trajectory that was validated to be free
        // Send in small path in reverse to execute trajectory since it uses reverse iterators
        path_t smallPath(lim, config_t());
        for (int i = lim - 1; i >= 0; --i)
        {
            smallPath[i] = *path.rbegin();
            path.pop_back();
        }

        ExecuteTrajectory(smallPath, startCfg);
        startCfg = smallPath[0]; // Update to current pose after execute trajectory

        if (startCfg == goalConfig)
        {
            reachedGoal = true;
        }
    }

    return reachedGoal;
}

void FMT::CheckTriggers(const config_t &currPos)
{
    // All triggers have been hit, return immediately
    if (triggers.size() == 0)
    {
        return;
    }

    // currPos[0] <- x-coord
    if (currPos[0] > triggers.back()->triggerPoint)
    {
        // Update environment
        for (const auto &dynobj : triggers.back()->dynobjs)
        {
            auto x = dynobj.second.position[0];
            auto y = dynobj.second.position[1];
            RaveVector<dReal> axis(0, 0, 1);
            // std::cout << "rot: " << dynobj.second.rotation << std::endl;
            auto rot = geometry::quatFromAxisAngle(axis, dynobj.second.rotation * M_PI / 180);
            // std::cout << rot << std::endl;
            auto t = Transform(rot, {x, y, 0.74});
            GetEnv()->GetMutex().lock();
            GetEnv()->GetKinBody(dynobj.first)->SetTransform(t);
            GetEnv()->GetMutex().unlock();
        }

        triggers.pop_back();
    }
}

double FMT::PathLength(const path_t &path)
{
    double length = 0.0;
    for (unsigned i = 0; i < path.size()-1; ++i)
    {
        length += CalcEuclidianDist(path[i], path[i+1]);
    }
    return length;
}

void FMT::ClearCosts()
{
    for (auto & node : total)
    {
        node->cost = 0.0;
        nodeptr_t nullParent;
        node->parent = nullParent;
        node->setType = UNVISITED;
    }
}

bool FMT::IsNodeInGoalRegion(const nodeptr_t & node, const path_t & region)
{
    for (const auto & goal : region)
    {
        if (node->q == goal)
        {
            std::cout << "Joining previous path" << std::endl;
            return true;
        }
    }
    return false;
}