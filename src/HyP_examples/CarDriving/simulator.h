#ifndef CARWORLDSIMULATOR_H
#define CARWORLDSIMULATOR_H
#include <despot/planner.h>

#include <ros/ros.h>
#include <gazebo_msgs/ModelStates.h>
#include <gamma_simulator/AgentStates.h>

#include "coord.h"
#include "Path.h"
#include "state.h"
#include "WorldModel.h"
#include <despot/interface/pomdp.h>
#include <despot/core/pomdp_world.h>
using namespace despot;

class Simulator:public POMDPWorld {
public:
    typedef pair<float, Pedestrian> PedDistPair;

    Simulator(DSPOMDP* model, unsigned seed=0);
    ~Simulator();

    // int numPedInArea(PedStruct peds[ModelParams::N_PED_WORLD], int num_of_peds_world);

    // int numPedInCircle(PedStruct peds[ModelParams::N_PED_WORLD], int num_of_peds_world, double car_x, double car_y);
    // int run(int argc, char *argv[]);

    // PedStruct randomPed();
    // PedStruct randomFarPed(double car_x, double car_y);
    // PedStruct randomPedAtCircleEdge(double car_x, double car_y);

    // void generateFixedPed(PomdpState &s);

    //virtual DSPOMDP* InitializeModel(option::Option* options) ;
	//virtual void InitializeDefaultParameters();

	// void ImportPeds(std::string filename, PomdpStateWorld& world_state);
	// void ExportPeds(std::string filename, PomdpStateWorld& world_state);

	void PrintWorldState(PomdpStateWorld state, ostream& out = cout);

	void UpdateWorld();

    COORD start, goal;

    Path path;
    static WorldModel worldModel;

    WorldStateTracker* stateTracker;
	PomdpStateWorld world_state;
	PomdpStateWorld current_state;
	PomdpStateWorld next_state;
	int num_of_peds_world;

	Random* rand_;

    /**
     * Callback for model states from gazebo
     **/
    void modelStatesCB(const gazebo_msgs::ModelStatesConstPtr &msg);

    /**
     * Callback for agent velocity from gazebo
     **/
    void agentStatesCB(const gamma_simulator::AgentStatesConstPtr &msg);

    /**
     * ROS variables
     **/
    ros::Subscriber model_states_sub_;
    ros::Subscriber agent_states_sub_;
    ros::Publisher acc_pub_;
    ros::Publisher cmd_vel_pub_;
    ros::NodeHandlePtr nh_;
    std::shared_ptr<ros::AsyncSpinner> spinner_;

public:

	virtual bool Connect(){ return true;}

	virtual State* Initialize();

	virtual State* GetCurrentState() const;

	virtual bool ExecuteAction(ACT_TYPE action, OBS_TYPE& obs);

};

#endif
