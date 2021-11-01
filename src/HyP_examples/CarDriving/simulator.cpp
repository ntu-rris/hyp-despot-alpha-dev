#include "WorldModel.h"
#include"state.h"
#include"Path.h"
#include "despot/solver/despot.h"
#include "custom_particle_belief.h"
#include "simulator.h"
#include "ped_pomdp.h"

#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Accel.h>

#include <despot/GPUcore/CudaInclude.h>

#include <iostream>
#include <fstream>
using namespace std;

using namespace despot;
//#define LINE_CASE
#define CROSS_CASE
// #define STATIC

WorldModel Simulator::worldModel;


int n_sim = 1;

#ifdef LINE_CASE
	const double PED_X0 = 35;/// used to generate peds' locations, where x is in (PED_X0, PED_X1), and y is in (PED_Y0, PED_Y1)
	const double PED_Y0 = 35;
	const double PED_X1 = 42;
	const double PED_Y1 = 52;
	const int n_peds = 6; // should be smaller than ModelParams::N_PED_IN
#elif defined(CROSS_CASE)
	const double PED_X0 = 0;/// used to generate peds' locations, where x is in (PED_X0, PED_X1), and y is in (PED_Y0, PED_Y1)
	const double PED_Y0 = 0;
	const double PED_X1 = 20;
	const double PED_Y1 = 15;
	const int n_peds = 16;//6; // should be smaller than ModelParams::N_PED_IN
#endif


Simulator::Simulator(DSPOMDP* model, unsigned seed)/// set the path to be a straight line
: POMDPWorld(model, seed)
{
#ifdef LINE_CASE
	start.x=40;start.y=40;
	goal.x=40;goal.y=50;
#elif defined(CROSS_CASE)
	start.x=-10;start.y=-1.5;
	goal.x=6;goal.y=-1.5;
#endif

	path.push_back(start);

	//Push placeholder point for robot position and robot's next position
	path.push_back(COORD(-100, -100));
	path.push_back(COORD(-100, -100));
	path.push_back(goal);
	// path = p.interpolate();
	worldModel.setPath(path);
	num_of_peds_world=0;
	stateTracker=new WorldStateTracker(worldModel);
	rand_ = new Random(Globals::config.root_seed);
}

Simulator::~Simulator()
{
	nh_.reset();
}

// State* Simulator::Initialize(){

// 	// for tracking world state
// 	world_state.car.pos = 0;
// 	world_state.car.vel = 0;
// 	world_state.car.dist_travelled = 0;
// 	world_state.num = n_peds;

// 	if(FIX_SCENARIO==0)
// 	{
// 		if(DESPOT::Debug_mode || ((PedPomdp*)model_)->load_peds)
// 		{
// 			ImportPeds("Peds.txt", world_state);
// 			cout << "[FIX_SCENARIO] load peds from "<< "Peds.txt" << endl;
// 		}
// 		else
// 		{
// 			for(int i=0; i<n_peds; i++) {
// 				world_state.peds[i] = randomPed();
// 				world_state.peds[i].id = i;
// 			}
// 		}
// 	}
// 	else if(FIX_SCENARIO==1)
// 	{
// 		ImportPeds("Peds.txt", world_state);
// 		cout << "[FIX_SCENARIO] load peds from "<< "Peds.txt" << endl;
// 	}
// 	else if(FIX_SCENARIO==2)
// 	{
// 		//generate initial n_peds peds
// 		for(int i=0; i<n_peds; i++) {
// 			world_state.peds[i] = randomPed();
// 			world_state.peds[i].id = i;
// 		}
// 		ExportPeds("Peds.txt",world_state);
// 	}

// 	num_of_peds_world = n_peds;

// 	double total_reward_dis = 0, total_reward_nondis=0;
// 	int step = 0;
// 	cout<<"LASER_RANGE= "<<ModelParams::LASER_RANGE<<endl;

// 	/*stateTracker->updateCar(path[world_state.car.pos], world_state.car.dist_travelled);

// 	//update the peds in stateTracker
// 	for(int i=0; i<num_of_peds_world; i++) {
// 		Pedestrian p(world_state.peds[i].pos.x, world_state.peds[i].pos.y, world_state.peds[i].id);
// 		stateTracker->updatePed(p);
// 	}*/
// }

State* Simulator::Initialize(){
	ros::init(std::map<std::string, std::string>(), "despot_gazebo");
	nh_.reset(new ros::NodeHandle);
	spinner_.reset(new ros::AsyncSpinner(1));

	//Wait for gazebo to be ready
	ros::service::waitForService("/gazebo/get_model_state");

	//Subscribe to gazebo states topics
	model_states_sub_ = nh_->subscribe("/gazebo/model_states", 1, &Simulator::modelStatesCB, this);
	agent_states_sub_ = nh_->subscribe("/gamma_simulator/agent_states", 1, &Simulator::agentStatesCB, this);

	//Advertise cmd_vel publisher
	acc_pub_ = nh_->advertise<geometry_msgs::Accel>("accel", 1);
	cmd_vel_pub_ = nh_->advertise<geometry_msgs::Twist>("cmd_vel", 1);
	spinner_->start();

	return &current_state;
}

void Simulator::agentStatesCB(const gamma_simulator::AgentStatesConstPtr &msg)
{
	//TODO: Synchronise with modelStatesCB?
	//Traverse all agents and then find their corresponding model_id in received agent_states msg
	for(auto &ped : next_state.peds)
	{
		auto it = std::find(msg->id_map.begin(), msg->id_map.end(), ped.id);
		if(it != msg->id_map.end())
		{
			int index = std::distance(msg->id_map.begin(), it);
			ped.vel = sqrt(pow(msg->agent_states[index].twist.twist.linear.x, 2) + pow(msg->agent_states[index].twist.twist.linear.y, 2));

			//If velocity is positive x and greater than y
			if(msg->agent_states[index].twist.twist.linear.x > msg->agent_states[index].twist.twist.linear.y)
				ped.goal = 0;

			else 
				ped.goal = 1;
		}

	}

	//Only update current state after this agentStatesCB callback, as modelStatesCB runs at 1000Hz and will override valid vel/goal if it updates current_state
	current_state = std::move(next_state);
}

void Simulator::modelStatesCB(const gazebo_msgs::ModelStatesConstPtr &msg)
{
	//TODO: Synchronise with agentStatesCB?
	std::vector<PedStruct> new_peds;
	for(int i = 0; i < msg->name.size(); ++i)
	{
		//Is walking model, names are 'walkingxx'
		auto it = msg->name[i].find("walking");
		if(it != std::string::npos && msg->name[i].find("collision_model") == std::string::npos)
		{
			PedStruct ped;
			int model_id = std::stoi(msg->name[i].substr(it + std::string("walking").size(), msg->name[i].size()));

			//Update ped COORD
			ped.pos.x = msg->pose[i].position.x;
			ped.pos.y = msg->pose[i].position.y;

			//Update ped goal??
			#ifdef STATIC
			ped.goal = 0;
			ped.vel = 0;
			#endif

			//Update ped id
			ped.id = model_id;
			new_peds.push_back(std::move(ped));
		}

		else if(msg->name[i].find("mbot") != std::string::npos)
		{
			// //Set car position
			// double min = 2000000000;
			// int min_idx = 0;
			// for(int j = 0; j < path.size(); ++j)
			// {
			// 	auto w = path[j];
			// 	double dist = pow(msg->pose[i].position.x - w.x, 2) + pow(msg->pose[i].position.y - w.y, 2);
			// 	if(dist < min)
			// 	{
			// 		min = dist;
			// 		min_idx = j;
			// 	}
			// }
			next_state.car.pos = 1;
			path[1] = COORD(msg->pose[i].position.x, msg->pose[i].position.y);
			double heading = atan2(msg->pose[i].position.y, msg->pose[i].position.x);
			heading = 0;
			path[2] = COORD(msg->pose[i].position.x + cos(heading) * 0.3, msg->pose[i].position.y + sin(heading) * 0.3);
			worldModel.setPath(path);


			//Set car velocity
			next_state.car.vel = sqrt(pow(msg->twist[i].linear.x, 2) + pow(msg->twist[i].linear.y, 2));

			//Update car distance travelled

		}
	}

	next_state.peds = new_peds;
	next_state.num = new_peds.size();

	#ifdef STATIC
	current_state = next_state;
	#endif
}

State* Simulator::GetCurrentState() const{
	static PomdpStateWorld current_state_;
	current_state_ = current_state;

	// //cout << "[GetCurrentState] current num peds in simulator: " << num_of_peds_world << endl;

	// std::vector<PedDistPair> sorted_peds = stateTracker->getSortedPeds();
	// /*if (sorted_peds.size() ==0){
	// 	return NULL;
	// }*/

	// current_state.car.pos = world_state.car.pos;
	// current_state.car.vel = world_state.car.vel;
	// current_state.car.dist_travelled = world_state.car.dist_travelled;
	// current_state.num = /*num_of_peds_world*/sorted_peds.size();

	// if (sorted_peds.size() ==0){
	// 		current_state.num = world_state.num;

	// 		for(int i=0; i<current_state.num; i++) {
	// 			current_state.peds[i] = world_state.peds[i];
	// 			//current_state.peds_mode[i] = world_state.peds_mode[i];
	// 		}
	// }
	// else{
	// 	//update s.peds to the nearest n_peds peds
	// 	for(int i=0; i<current_state.num; i++) {
	// 		//cout << "[GetCurrentState] ped id:"<< sorted_peds[i].second.id << endl;
	// 		if(i<sorted_peds.size()){
	// 			current_state.peds[i] = world_state.peds[sorted_peds[i].second.id];
	// 			//current_state.peds_mode[i] = world_state.peds_mode[sorted_peds[i].second.id];
	// 		}
	// 	}
	// }
	return &current_state_;
}

void Simulator::UpdateWorld(){

	cout << "[Simulator::UpdateWorld] \n";

	// stateTracker->updateCar(path[current_state.car.pos], current_state.car.dist_travelled);
	stateTracker->updateCar(path[current_state.car.pos], 0);
	stateTracker->updateVel(current_state.car.vel);

	//update the peds in stateTracker
	for(int i=0; i<current_state.peds.size(); i++) {
		Pedestrian p(current_state.peds[i].pos.x, current_state.peds[i].pos.y, current_state.peds[i].id);
		stateTracker->updatePed(p);
	}

// #ifdef CROSS_CASE
// 	if(FIX_SCENARIO==0){
// 		int new_ped_count=0;
// 		while(new_ped_count<1 && numPedInCircle(world_state.peds, num_of_peds_world,path[world_state.car.pos].x, path[world_state.car.pos].y)<n_peds
// 			&& num_of_peds_world < ModelParams::N_PED_WORLD)
// 		{
// 			PedStruct new_ped= randomFarPed(path[world_state.car.pos].x, path[world_state.car.pos].y);
// 			new_ped.id = num_of_peds_world;
// 			world_state.peds[num_of_peds_world]=new_ped;

// 			num_of_peds_world++;
// 			world_state.num++;
// 			new_ped_count++;
// 			Pedestrian p(new_ped.pos.x, new_ped.pos.y, new_ped.id);
// 			stateTracker->updatePed(p); //add the new generated ped into stateTracker.ped_list

// 			cout << "[Simulator] Create new pedestrian "<< new_ped.id <<" in world simulator."<< endl;

// 		}

// 		cout << "[Simulator] Number of peds in laser range: "<< numPedInCircle(world_state.peds, num_of_peds_world,path[world_state.car.pos].x, path[world_state.car.pos].y)
// 				<< endl;
// 	}
// #endif

// 	if(DESPOT::Debug_mode)
// 		static_cast<PedPomdp*>(model_)->PrintWorldState(world_state);
}


bool Simulator::ExecuteAction(ACT_TYPE action, OBS_TYPE& obs){

	if(worldModel.isGlobalGoal(current_state.car)) {
		cout << "-------------------------------------------------------" << endl;
		cout << "goal_reached=1" << endl;
		geometry_msgs::Twist z;
		z.linear.x = 0;
		geometry_msgs::Accel a;
		a.linear.x = 0;
		cmd_vel_pub_.publish(z);
		acc_pub_.publish(a);
		return true;
	}

	int collision_peds_id=-1;
	if( current_state.car.vel > 0.001 && worldModel.inCollision(current_state,collision_peds_id) ) {
		cout << "-------------------------------------------------------" << endl;
		cout << "collision=1: " << collision_peds_id<<endl;
		geometry_msgs::Twist z;
		z.linear.x = 0;
		geometry_msgs::Accel a;
		a.linear.x = 0;
		cmd_vel_pub_.publish(z);
		acc_pub_.publish(a);
	}
#ifdef LINE_CASE
	else if(worldModel.inCollision(s,collision_peds_id)) {
	}
#elif defined(CROSS_CASE)
	else if(worldModel.inCollision(current_state,collision_peds_id)) {
	}
#endif

	if(FIX_SCENARIO==1){
		cout << "[FIX_SCENARIO] act= " << action << endl;
		action=0;
		cout << "[FIX_SCENARIO] rewrite act to " << action << endl;
	}

	// double reward;

	// bool terminate = static_cast<PedPomdp*>(model_)->Step(current_state,
	// 		rand_->NextDouble(),
	// 		action, reward, obs);

	obs=static_cast<PedPomdp*>(model_)->StateToIndex(GetCurrentState());

	geometry_msgs::Accel acc;
	acc.linear.x = 0;
	if(action == 1)
		acc.linear.x = 0.10;

	else if(action == 2)
		acc.linear.x = -0.10;

	acc_pub_.publish(acc);

	static double cur_vel = 0;
	geometry_msgs::Twist twist;
	cur_vel += acc.linear.x;
	if(cur_vel > 1.0)
		cur_vel = 1.0;

	else if(cur_vel < 0)
		cur_vel = 0;

	twist.linear.x = cur_vel;
	cmd_vel_pub_.publish(twist);

	//cout << "[Simulator] Current world state:" << endl;
	//static_cast<PedPomdp*>(model_)->PrintWorldState(world_state);

	// step_reward_=reward;

	// if(terminate) {
	// 	cout << "-------------------------------------------------------" << endl;
	// 	cout << "simulation terminate=1" << endl;

	// 	if(DESPOT::Debug_mode){
	// 		cout << "- final_state:\n";
	// 		static_cast<PedPomdp*>(model_)->PrintWorldState(static_cast<PomdpStateWorld&>(*GetCurrentState()), cout);
	// 	}
	// 	return true;

	// }
	return false;
}

// int Simulator::numPedInCircle(PedStruct peds[ModelParams::N_PED_WORLD], int num_of_peds_world, double car_x, double car_y)
// {
// 	int num_inside = 0;

// 	for (int i=0; i<num_of_peds_world; i++)
// 	{
// 		if((peds[i].pos.x - car_x)*(peds[i].pos.x - car_x) + (peds[i].pos.y - car_y)*(peds[i].pos.y - car_y) <= ModelParams::LASER_RANGE * ModelParams::LASER_RANGE) num_inside++;
// 	}

// 	return num_inside;
// }

// void Simulator::ImportPeds(std::string filename, PomdpStateWorld& world_state){
// 	ifstream fin;fin.open(filename, ios::in);
// 	assert(fin.is_open());
// 	if (fin.good())
// 	{
// 		int num_peds_infile=0;
// 		string str;
// 		getline(fin, str);
// 		istringstream ss(str);
// 		ss>>num_peds_infile;//throw headers
// 		cout<<"num_peds_infile"<<num_peds_infile<<endl;
// 		assert(num_peds_infile==n_peds);
// 		int i=0;
// 		while(getline(fin, str))
// 		{
// 			if(!str.empty() && i<n_peds)
// 			{
// 				istringstream ss(str);
// 				ss>> world_state.peds[i].id
// 				>> world_state.peds[i].goal
// 				>> world_state.peds[i].pos.x
// 				>> world_state.peds[i].pos.y
// 				>> world_state.peds[i].vel;
// 				i++;
// 			}
// 		}
// 		cout<<"peds imported"<<endl;
// 	}
// 	else
// 	{
// 		cout<<"Empty peds file!"<<endl;
// 		exit(-1);
// 	}
// }
// void Simulator::ExportPeds(std::string filename, PomdpStateWorld& world_state){
// 	std::ofstream fout;fout.open(filename, std::ios::trunc);
// 	assert(fout.is_open());

// 	fout<<n_peds<<endl;
// 	for(int i=0; i<n_peds; i++)
// 	{
// 		fout<<world_state.peds[i].id<<" "
// 				<<world_state.peds[i].goal<<" "
// 				<<world_state.peds[i].pos.x<<" "
// 				<<world_state.peds[i].pos.y<<" "
// 				<<world_state.peds[i].vel<<endl;
// 	}
// 	fout<<endl;
// }


// #ifdef LINE_CASE
// PedStruct Simulator::randomPed() {
// 	int n_goals = worldModel.goals.size();
// 	int goal = rand_->NextInt(n_goals);
// 	double x = rand_->NextDouble(PED_X0, PED_X1);
// 	double y = rand_->NextDouble(PED_Y0, PED_Y1);
// 	if(goal == n_goals-1) {
// 		// stop intention
// 		while(path.mindist(COORD(x, y)) < 1.0) {
// 			// dont spawn on the path
// 			x = rand_->NextDouble(PED_X0, PED_X1);
// 			y = rand_->NextDouble(PED_Y0, PED_Y1);
// 		}
// 	}
// 	int id = 0;
// 	return PedStruct(COORD(x, y), goal, id);
// }
// #elif defined(CROSS_CASE)

// PedStruct Simulator::randomPed() {
//    int goal;
//    double goal0_x_min = /*14*/12, goal0_x_max = 19/*21*/;
//    double goal0_y_min = 4.5, goal0_y_max = 11-0.5;

//    double goal1_x_min = 6.5, goal1_x_max = 13.5;
//    double goal1_y_min = /*-1*/-10, goal1_y_max = 4;



//    if(rand_->NextInt(100)>95) goal=worldModel.goals.size() - 1; //setting stop intention with 5% probability.
//    else goal = rand_->NextInt(worldModel.goals.size() - 1); //uniformly randomly select a goal from those that not is not stopping

//    double x;
//    double y;
//    double speed=ModelParams::PED_SPEED;
//    if(goal == 0){
// 	   x = rand_->NextDouble(goal0_x_min, goal0_x_max);
// 	   y = rand_->NextDouble(goal0_y_min, goal0_y_max);
//    }
//    else if(goal == 1){
// 	   x = rand_->NextDouble(goal1_x_min, goal1_x_max);
// 	   y = rand_->NextDouble(goal1_y_min, goal1_y_max);
//    }
//    else{// stop intention
// 	   speed=0;
// 	   if(rand_->NextInt(2)==0){
// 		   x = rand_->NextDouble(goal0_x_min, goal0_x_max);
// 		   y = rand_->NextDouble(goal0_y_min, goal0_y_max);
// 	   }
// 	   else{
// 		   x = rand_->NextDouble(goal1_x_min, goal1_x_max);
// 		   y = rand_->NextDouble(goal1_y_min, goal1_y_max);
// 	   }
// 	   while(path.mindist(COORD(x, y)) < 1.0) {
// 		   // dont spawn on the path
// 		   if(rand_->NextInt(2)==0){
// 			   x = rand_->NextDouble(goal0_x_min, goal0_x_max);
// 			   y = rand_->NextDouble(goal0_y_min, goal0_y_max);
// 		   }
// 		   else{
// 			   x = rand_->NextDouble(goal1_x_min, goal1_x_max);
// 			   y = rand_->NextDouble(goal1_y_min, goal1_y_max);
// 		   }
// 	   }
//    }
//    int id = 0;
//    return PedStruct(COORD(x, y), goal, id, speed);
// }

// PedStruct Simulator::randomFarPed(double car_x, double car_y) { //generate pedestrians that are not close to the car
//     int goal;
//     double goal0_x_min = /*14*/28, goal0_x_max = /*21*/31;
//     double goal0_y_min = /*4.5*/2, goal0_y_max = /*11-0.5*/12;

//     double goal1_x_min = 6.5, goal1_x_max = 13.5;
//     double goal1_y_min = /*-1*/-4, goal1_y_max = /*4*/1;

//     double x;
//     double y;

//     cout << __FUNCTION__ << " rand seed = " << rand_->seed() << endl;

//     if(rand_->NextInt(2)==0){
//         goal = 0;
//         x = rand_->NextDouble(goal0_x_min, goal0_x_max);
//         y = rand_->NextDouble(goal0_y_min, goal0_y_max);
//     }
//     else{
//         goal = 1;
//         x = rand_->NextDouble(goal1_x_min, goal1_x_max);
//         y = rand_->NextDouble(goal1_y_min, goal1_y_max);
//     }

//     while(COORD::EuclideanDistance(COORD(car_x, car_y), COORD(x, y)) < 2.0) {
//         if(rand_->NextInt(2)==0){
//             goal = 0;
//             x = rand_->NextDouble(goal0_x_min, goal0_x_max);
//             y = rand_->NextDouble(goal0_y_min, goal0_y_max);
//         }
//         else{
//             goal = 1;
//             x = rand_->NextDouble(goal1_x_min, goal1_x_max);
//             y = rand_->NextDouble(goal1_y_min, goal1_y_max);
//         }
//     }

//     int id = 0;
//     return PedStruct(COORD(x, y), goal, id);
// }
// #endif



