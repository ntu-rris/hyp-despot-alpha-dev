#include<limits>
#include<cmath>
#include<cstdlib>
#include<numeric>
#include"WorldModel.h"

#include <despot/GPUcore/thread_globals.h>
#include <despot/core/globals.h>

#include"math_utils.h"
#include"coord.h"

#include <despot/solver/despot.h>

using namespace std;



WorldModel::WorldModel(): freq(ModelParams::control_freq),
    in_front_angle_cos(cos(ModelParams::IN_FRONT_ANGLE_DEG / 180.0 * M_PI)) {
    goal_file_name_ = "null";

    InitPedGoals();

	if (DESPOT::Debug_mode)
		ModelParams::NOISE_GOAL_ANGLE = 0.000001;
}

void WorldModel::InitPedGoals(){

     if(goal_file_name_ == "null"){
        std::cout<<"Using default goals"<<std::endl;

       /* goals = { // indian cross 2 larger map
            COORD(3.5, 20.0),
            COORD(-3.5, 20.0), 
            COORD(3.5, -20.0), 
            COORD(-3.5, -20.0),
            COORD(20.0  , 3.5),
            COORD( 20.0 , -3.5), 
            COORD(-20.0 , 3.5), 
            COORD( -20.0, -3.5),
            COORD(-1, -1) // stop
        };*/

        goals = {
		//    COORD(/*-2*/-20, 7.5),
		//    COORD(10, /*17*/27),
		//    COORD(-1,-1)
        COORD(10, 0),
        COORD(0, 10)
		};
    }
    else{
        goals.resize(0);

        std::ifstream file;
        file.open(goal_file_name_, std::ifstream::in);

        if(file.fail()){
            std::cout<<"open goal file failed !!!!!!"<<std::endl;
            return;
        }

        std::string line;
        int goal_num = 0;
        while (std::getline(file, line))
        {
            std::istringstream iss(line);
            
            double x;
            double y;
            while (iss >> x >>y){
                std::cout << x <<" "<< y<<std::endl;
                goals.push_back(COORD(x, y));
            }

            goal_num++;
            if(goal_num > 99) break;
        }

        file.close();
    }

}

void WorldModel::InitRVO(){
    if(!Globals::config.use_multi_thread_ )
        //NumThreads=Globals::config.NUM_THREADS;
        Globals::config.NUM_THREADS=1;

    int NumThreads=Globals::config.NUM_THREADS;

    ped_sim_.resize(NumThreads);
    for(int tid=0; tid<NumThreads;tid++)
    {
        ped_sim_[tid] = new RVO::RVOSimulator();
        
        // Specify global time step of the simulation.
        ped_sim_[tid]->setTimeStep(0.33f);    

        ped_sim_[tid]->setAgentDefaults(5.0f, 5, 1.5f, 1.5f, 0.25f, 2.5f); 
    }
    
}

WorldModel::~WorldModel(){
    ped_sim_.clear();
    ped_sim_.resize(0);
}

bool WorldModel::isLocalGoal(const PomdpState& state) {
    return state.car.dist_travelled > ModelParams::GOAL_TRAVELLED-1e-4 || state.car.pos >= path.size()-1;
}

bool WorldModel::isLocalGoal(const PomdpStateWorld& state) {
    return state.car.dist_travelled > ModelParams::GOAL_TRAVELLED || state.car.pos >= path.size()-1;
}

bool WorldModel::isGlobalGoal(const CarStruct& car) {
    double d = COORD::EuclideanDistance(path[car.pos], path[path.size()-1]);
    return (d<ModelParams::GOAL_TOLERANCE);
}

int WorldModel::defaultPolicy(const std::vector<State*>& particles)  {
	const PomdpState *state=static_cast<const PomdpState*>(particles[0]);
    double mindist = numeric_limits<double>::infinity();
    auto& carpos = path[state->car.pos];
    double carvel = state->car.vel;
    // Closest pedestrian in front
    for (int i=0; i<state->num; i++) {
		auto& p = state->peds[i];
		if(!inFront(p.pos, state->car.pos)) continue;

        double d = COORD::EuclideanDistance(carpos, p.pos);
        if (d >= 0 && d < mindist) 
			mindist = d;
    }

    if(DoPrintCPU &&state->scenario_id ==0) printf("mindist, carvel= %f %f\n",mindist,carvel);

    // TODO set as a param
    if (mindist < /*2*/3.5) {
		return (carvel <= 0.01) ? 0 : 2;
    }

    if (mindist < /*4*/5) {
		if (carvel > 1.0+1e-4) return 2;
		else if (carvel < 0.5-1e-4) return 1;
		else return 0;
    }
    return carvel >= ModelParams::VEL_MAX-1e-4 ? 0 : 1;
}
enum {
	CLOSE_STATIC,
	CLOSE_MOVING,
	MEDIUM_FAST,
	MEDIUM_MED,
	MEDIUM_SLOW,
	FAR_MAX,
	FAR_NOMAX,
};


bool WorldModel::inFront(COORD ped_pos, int car) const {
    if(ModelParams::IN_FRONT_ANGLE_DEG >= 180.0) {
        // inFront check is disabled
        return true;
    }
	const COORD& car_pos = path[car];
	const COORD& forward_pos = path[path.forward(car, 1.0)];
	double d0 = COORD::EuclideanDistance(car_pos, ped_pos);
	//if(d0<=0) return true;
	if(d0 <= /*0.7*/3.5) return true;
	double d1 = COORD::EuclideanDistance(car_pos, forward_pos);
	if(d1<=0) return false;
	double dot = DotProduct(forward_pos.x - car_pos.x, forward_pos.y - car_pos.y,
			ped_pos.x - car_pos.x, ped_pos.y - car_pos.y);
	double cosa = dot / (d0 * d1);
	assert(cosa <= 1.0 + 1E-8 && cosa >= -1.0 - 1E-8);
    return cosa > in_front_angle_cos;
	//double angle = acos(cosa);
}

/**
 * H: center of the head of the car
 * N: a point right in front of the car
 * M: an arbitrary point
 *
 * Check whether M is in the safety zone
 */
bool inCollision(double Mx, double My, double Hx, double Hy, double Nx, double Ny);
bool inRealCollision(double Mx, double My, double Hx, double Hy, double Nx, double Ny);
bool inCollision(double ped_x, double ped_y, double car_x, double car_y);//for scooter collision check;

bool WorldModel::inCollision(const PomdpState& state) {
    const int car = state.car.pos;
	const COORD& car_pos = path[car];
	const COORD& forward_pos = path[path.forward(car, 1.0)];

    for(int i=0; i<state.num; i++) {
        const COORD& pedpos = state.peds[i].pos;
        if(::inCollision(pedpos.x, pedpos.y, car_pos.x, car_pos.y, forward_pos.x, forward_pos.y)) {
            return true;
        }
    }
    return false;
}

bool WorldModel::inRealCollision(const PomdpState& state) {
    const int car = state.car.pos;
    const COORD& car_pos = path[car];
    const COORD& forward_pos = path[path.forward(car, 1.0)];

    for(int i=0; i<state.num; i++) {
        const COORD& pedpos = state.peds[i].pos;
        if(::inRealCollision(pedpos.x, pedpos.y, car_pos.x, car_pos.y, forward_pos.x, forward_pos.y)) {
            return true;
        }
        /*if(::inCollision(pedpos.x, pedpos.y, car_pos.x, car_pos.y)) {
            return true;
        }*/
    }
    return false;
}

bool WorldModel::inCollision(const PomdpStateWorld& state) {
    const int car = state.car.pos;
    const COORD& car_pos = path[car];
    const COORD& forward_pos = path[path.forward(car, 1.0)];

    for(int i=0; i<state.num; i++) {
        const COORD& pedpos = state.peds[i].pos;
        if(::inCollision(pedpos.x, pedpos.y, car_pos.x, car_pos.y, forward_pos.x, forward_pos.y)) {
            return true;
        }
    }
    return false;
}

bool WorldModel::inCollision(const PomdpState& state, int &id) {
	id=-1;
    const int car = state.car.pos;
	const COORD& car_pos = path[car];
	const COORD& forward_pos = path[path.forward(car, 1.0)];

    for(int i=0; i<state.num; i++) {
        const COORD& pedpos = state.peds[i].pos;
        if(::inCollision(pedpos.x, pedpos.y, car_pos.x, car_pos.y, forward_pos.x, forward_pos.y)) {
        	id=state.peds[i].id;
            return true;
        }
    }
    return false;
}

bool WorldModel::inCollision(const PomdpStateWorld& state, int &id) {
    id=-1;
    const int car = state.car.pos;
    const COORD& car_pos = path[car];
    const COORD& forward_pos = path[path.forward(car, 1.0)];

    for(int i=0; i<state.num; i++) {
        const COORD& pedpos = state.peds[i].pos;
        if(::inCollision(pedpos.x, pedpos.y, car_pos.x, car_pos.y, forward_pos.x, forward_pos.y)) {
            id=state.peds[i].id;

            logd << "[WorldModel::inCollision] car_pos: ("<< car_pos.x <<","<< car_pos.y<<"), forward_pos: ("
            		<< forward_pos.x <<","<< forward_pos.y<<"), ped_pos: ("
            		<< pedpos.x <<","<< pedpos.y<<")\n";
            return true;
        }
    }
    return false;
}


void WorldModel::getClosestPed(const PomdpState& state, 
		int& closest_front_ped,
		double& closest_front_dist,
		int& closest_side_ped,
		double& closest_side_dist) {
	closest_front_ped = -1;
	closest_front_dist = numeric_limits<double>::infinity();
	closest_side_ped = -1;
	closest_side_dist = numeric_limits<double>::infinity();
    const auto& carpos = path[state.car.pos];

	// Find the closest pedestrian in front
    for(int i=0; i<state.num; i++) {
		const auto& p = state.peds[i];
		bool front = inFront(p.pos, state.car.pos);
        double d = COORD::EuclideanDistance(carpos, p.pos);
        if (front) {
			if (d < closest_front_dist) {
				closest_front_dist = d;
				closest_front_ped = i;
			}
		} else {
			if (d < closest_side_dist) {
				closest_side_dist = d;
				closest_side_ped = i;
			}
		}
    }
}

bool WorldModel::isMovingAway(const PomdpState& state, int ped) {
    const auto& carpos = path[state.car.pos];
	const auto& nextcarpos = path[path.forward(state.car.pos, 1.0)];

	const auto& pedpos = state.peds[ped].pos;
	const auto& goalpos = goals[state.peds[ped].goal];

	if (goalpos.x == -1 && goalpos.y == -1)
		return false;

	return DotProduct(goalpos.x - pedpos.x, goalpos.y - pedpos.y,
			nextcarpos.x - carpos.x, nextcarpos.y - carpos.y) > 0;
}

///get the min distance between car and the peds in its front
double WorldModel::getMinCarPedDist(const PomdpState& state) {
    double mindist = numeric_limits<double>::infinity();
    const auto& carpos = path[state.car.pos];

	// Find the closest pedestrian in front
    for(int i=0; i<state.num; i++) {
		const auto& p = state.peds[i];
		if(!inFront(p.pos, state.car.pos)) continue;
        double d = COORD::EuclideanDistance(carpos, p.pos);
        if (d >= 0 && d < mindist) mindist = d;
    }

	return mindist;
}

///get the min distance between car and the peds
double WorldModel::getMinCarPedDistAllDirs(const PomdpState& state) {
    double mindist = numeric_limits<double>::infinity();
    const auto& carpos = path[state.car.pos];

	// Find the closest pedestrian in front
    for(int i=0; i<state.num; i++) {
		const auto& p = state.peds[i];
        double d = COORD::EuclideanDistance(carpos, p.pos);
        if (d >= 0 && d < mindist) mindist = d;
    }

	return mindist;
}

int WorldModel::minStepToGoal(const PomdpState& state) {
    double d = ModelParams::GOAL_TRAVELLED - state.car.dist_travelled;
    if (d < 0) d = 0;
    return int(ceil(d / (ModelParams::VEL_MAX/freq)));
}

void WorldModel::PedStep(PedStruct &ped, Random& random) {
    const COORD& goal = goals[ped.goal];
	if (goal.x == -1 && goal.y == -1) {  //stop intention
		return;
	}

	MyVector goal_vec(goal.x - ped.pos.x, goal.y - ped.pos.y);
    double a = goal_vec.GetAngle();
	double noise = random.NextGaussian() * ModelParams::NOISE_GOAL_ANGLE;
    a += noise;

	//TODO noisy speed
    MyVector move(a, ped.vel/freq, 0);
    ped.pos.x += move.dw;
    ped.pos.y += move.dh;
    return;
}

void WorldModel::PedStep(PedStruct &ped, double& random) {

    const COORD& goal = goals[ped.goal];
	if (goal.x == -1 && goal.y == -1) {  //stop intention
		return;
	}

	MyVector goal_vec(goal.x - ped.pos.x, goal.y - ped.pos.y);
    double a = goal_vec.GetAngle();

	//double noise = random.NextGaussian() * ModelParams::NOISE_GOAL_ANGLE;
	double noise = sqrt(-2 * log(random));
	if(FIX_SCENARIO!=1 && !CPUDoPrint){
		random=QuickRandom::RandGeneration(random);
	}

	noise *= cos(2 * M_PI * random)* ModelParams::NOISE_GOAL_ANGLE;
    a += noise;

	//TODO noisy speed
    MyVector move(a, ped.vel/freq, 0);
    ped.pos.x += move.dw;
    ped.pos.y += move.dh;

    return;
}

double gaussian_prob(double x, double stddev) {
    double a = 1.0 / stddev / sqrt(2 * M_PI);
    double b = - x * x / 2.0 / (stddev * stddev);
    return a * exp(b);
}

double WorldModel::ISPedStep(CarStruct &car, PedStruct &ped, Random& random) {
//gaussian + distance
    double max_is_angle = 7.0*M_PI/64.0;
    COORD carpos = path[car.pos];
    if(COORD::EuclideanDistance(ped.pos, carpos)>3.5){
        const COORD& goal = goals[ped.goal];
        if (goal.x == -1 && goal.y == -1) {  //stop intention
            return 1;
        }

        MyVector goal_vec(goal.x - ped.pos.x, goal.y - ped.pos.y);
        double a = goal_vec.GetAngle();
        double noise = random.NextGaussian() * ModelParams::NOISE_GOAL_ANGLE;
        a += noise;

        //TODO noisy speed
        MyVector move(a, ped.vel/freq, 0);
        ped.pos.x += move.dw;
        ped.pos.y += move.dh;
        return 1;
    } else{
        double weight = 1.0;
        const COORD& goal = goals[ped.goal];
        if (goal.x == -1 && goal.y == -1) {  //stop intention
            return weight;
        }

        MyVector goal_vec(goal.x - ped.pos.x, goal.y - ped.pos.y);
        double goal_angle = goal_vec.GetAngle();

        //compute the angle to robot
        MyVector rob_vec(path[car.pos].x - ped.pos.x, path[car.pos].y - ped.pos.y);
        double rob_angle = rob_vec.GetAngle();

        double final_mean; //final mean angle

        if(abs(goal_angle-rob_angle) <= M_PI){
            if(goal_angle > rob_angle) final_mean = goal_angle - min(max_is_angle, goal_angle-rob_angle);
            else  final_mean = goal_angle + min(max_is_angle, rob_angle-goal_angle);
        }
        else{
            if(goal_angle > rob_angle) final_mean = goal_angle + min(max_is_angle, rob_angle+2*M_PI-goal_angle);
            else  final_mean = goal_angle - min(max_is_angle, goal_angle+2*M_PI-rob_angle);
        }

        if(final_mean>M_PI) final_mean -= M_PI;
        else if(final_mean<-M_PI) final_mean += M_PI;

        //random.NextGaussian() returns a random number sampled from N(0,1)
        double noise = random.NextGaussian() * ModelParams::NOISE_GOAL_ANGLE; //change to the number sampled from N(0, ModelParams::NOISE_GOAL_ANGLE)
        double final_angle = final_mean + noise; //change to the number sampled from N(rob_angle, ModelParams::NOISE_GOAL_ANGLE)

        //TODO noisy speed
        MyVector move(final_angle, ped.vel/freq, 0);
        ped.pos.x += move.dw;
        ped.pos.y += move.dh;

        weight = gaussian_prob((final_angle - goal_angle) / ModelParams::NOISE_GOAL_ANGLE, 1) /
                 gaussian_prob((final_angle - final_mean) / ModelParams::NOISE_GOAL_ANGLE, 1) ;
        return weight;
    }

}

void WorldModel::PedStepDeterministic(PedStruct& ped, int step) {
    const COORD& goal = goals[ped.goal];
	if (goal.x == -1 && goal.y == -1) {  //stop intention
		return;
	}

	MyVector goal_vec(goal.x - ped.pos.x, goal.y - ped.pos.y);
    goal_vec.AdjustLength(step * ped.vel / freq);
    ped.pos.x += goal_vec.dw;
    ped.pos.y += goal_vec.dh;
}

double WorldModel::pedMoveProb(COORD prev, COORD curr, int goal_id) {
	const double K = 0.001;
    const COORD& goal = goals[goal_id];
	double move_dist = Norm(curr.x-prev.x, curr.y-prev.y),
		   goal_dist = Norm(goal.x-prev.x, goal.y-prev.y);
	double sensor_noise = 0.1;
    if(ModelParams::is_simulation) sensor_noise = 0.02;

    bool debug=false;
	// CHECK: beneficial to add back noise?
	if(debug) cout<<"goal id "<<goal_id<<endl;
	if (goal.x == -1 && goal.y == -1) {  //stop intention 
		return (move_dist < sensor_noise) ? 0.4 : 0;
	} else {
		if (move_dist < sensor_noise) return 0;

		double cosa = DotProduct(curr.x-prev.x, curr.y-prev.y, goal.x-prev.x, goal.y-prev.y) / (move_dist * goal_dist);
		if(cosa >1) cosa = 1;
		else if(cosa < -1) cosa = -1;
		double angle = acos(cosa);
		return gaussian_prob(angle, ModelParams::NOISE_GOAL_ANGLE) + K;
	}
}
void WorldModel::FixGPUVel(CarStruct &car)
{
	float tmp=car.vel/(ModelParams::AccSpeed/freq);
	car.vel=((int)(tmp+0.5))*(ModelParams::AccSpeed/freq);
}
void WorldModel::RobStep(CarStruct &car, Random& random) {
    double dist = car.vel / freq;
    int nxt = path.forward(car.pos, dist);
    car.pos = nxt;
    car.dist_travelled += dist;
}

void WorldModel::RobStep(CarStruct &car, double& random) {
    double dist = car.vel / freq;
    int nxt = path.forward(car.pos, dist);
    car.pos = nxt;
    car.dist_travelled += dist;
}

void WorldModel::RobStep(CarStruct &car, double& random, double acc) {
    double end_vel = car.vel + acc / freq;
    end_vel = max(min(end_vel, ModelParams::VEL_MAX), 0.0);
    double dist = (car.vel + end_vel)/2.0 / freq;
    int nxt = path.forward(car.pos, dist);
    car.pos = nxt;
    car.dist_travelled += dist;

}

void WorldModel::RobStep(CarStruct &car, Random& random, double acc) {
    double end_vel = car.vel + acc / freq;
    end_vel = max(min(end_vel, ModelParams::VEL_MAX), 0.0);
    double dist = (car.vel + end_vel)/2.0 / freq;

    int nxt = path.forward(car.pos, dist);
    car.pos = nxt;
    car.dist_travelled += dist;
}
void WorldModel::RobVelStep(CarStruct &car, double acc, Random& random) {
    const double N = ModelParams::NOISE_ROBVEL;
    if (N>0) {
        double prob = random.NextDouble();
        if (prob > N) {
            car.vel += acc / freq;
        }
    } else {
        car.vel += acc / freq;
    }

	car.vel = max(min(car.vel, ModelParams::VEL_MAX), 0.0);

	return;
}

void WorldModel::RobVelStep(CarStruct &car, double acc, double& random) {
    const double N = ModelParams::NOISE_ROBVEL;
    if (N>0) {
    	if(FIX_SCENARIO!=1 && !CPUDoPrint)
    		random=QuickRandom::RandGeneration(random);
        double prob = random;
        if (prob > N) {
            car.vel += acc / freq;
        }
    } else {
        car.vel += acc / freq;
    }

	car.vel = max(min(car.vel, ModelParams::VEL_MAX), 0.0);

	return;
}
double WorldModel::ISRobVelStep(CarStruct &car, double acc, Random& random) {
    const double N = 4 * ModelParams::NOISE_ROBVEL;
    double weight = 1;
    if (N>0) {
        double prob = random.NextDouble();
        if (prob > N) {
            car.vel += acc / freq;
            weight = (1.0 - ModelParams::NOISE_ROBVEL)/(1.0 -N);
        }
        else weight = ModelParams::NOISE_ROBVEL / N;
    } else {
        car.vel += acc / freq;
    }

    car.vel = max(min(car.vel, ModelParams::VEL_MAX), 0.0);

    return weight;
}

void WorldModel::setPath(Path path) {
    this->path = path;

}

void WorldModel::updatePedBelief(PedBelief& b, const PedStruct& curr_ped) {
    const double ALPHA = 0.8;
	const double SMOOTHING=ModelParams::BELIEF_SMOOTHING;

    bool debug=false;

    for(int i=0; i<goals.size(); i++) {
		double prob = pedMoveProb(b.pos, curr_ped.pos, i);
		if(debug) cout << "likelihood " << i << ": " << prob << endl;
        b.prob_goals[i] *=  prob;

		// Important: Keep the belief noisy to avoid aggressive policies
		b.prob_goals[i] += SMOOTHING / goals.size(); // CHECK: decrease or increase noise
	}
	if(debug) {
        for(double w: b.prob_goals) {
            cout << w << " ";
        }
        cout << endl;
    }

    // normalize
    double total_weight = accumulate(b.prob_goals.begin(), b.prob_goals.end(), double(0.0));
	if(debug) cout << "[updatePedBelief] total_weight = " << total_weight << endl;
    for(double& w : b.prob_goals) {
        w /= total_weight;
    }


    double moved_dist = COORD::EuclideanDistance(b.pos, curr_ped.pos);
    b.vel = ALPHA * b.vel + (1-ALPHA) * moved_dist * ModelParams::control_freq;
	b.pos = curr_ped.pos;
}

PedBelief WorldModel::initPedBelief(const PedStruct& ped) {
    PedBelief b = {ped.id, ped.pos, ModelParams::PED_SPEED, vector<double>(goals.size(), 1.0/goals.size())};
    return b;
}

double timestamp() {
    static double starttime=get_time_second();
    return get_time_second()-starttime;
}

void WorldStateTracker::cleanPed() {
    vector<Pedestrian> ped_list_new;
    for(int i=0;i<ped_list.size();i++)
    {
        bool insert=true;
        double w1,h1;
        w1=ped_list[i].w;
        h1=ped_list[i].h;
        for(const auto& p: ped_list_new) {
            double w2,h2;
            w2=p.w;
            h2=p.h;
            if (abs(w1-w2)<=0.1&&abs(h1-h2)<=0.1) {
                insert=false;
                break;
            }
        }
        if (timestamp() - ped_list[i].last_update > 0.2) insert=false;
        if (insert)
            ped_list_new.push_back(ped_list[i]);
    }
    ped_list=ped_list_new;
}

void WorldStateTracker::updatePed(const Pedestrian& ped, bool doPrint){
    int i=0;
    for(;i<ped_list.size();i++) {
        if (ped_list[i].id==ped.id) {
            //found the corresponding ped,update the pose
            ped_list[i].w=ped.w;
            ped_list[i].h=ped.h;
            ped_list[i].last_update = timestamp();
            if (doPrint) cout <<"[updatePed] existing ped updated" << ped.id << endl;
            break;
        }
        if (abs(ped_list[i].w-ped.w)<=0.1 && abs(ped_list[i].h-ped.h)<=0.1)   //overlap
        {
        	if (doPrint) cout <<"[updatePed] overlapping ped skipped" << ped.id << endl;
            return;
        }
    }
    if (i==ped_list.size()) {
        //not found, new ped
        ped_list.push_back(ped);
        ped_list.back().last_update = timestamp();
        if (doPrint) cout <<"[updatePed] new ped added" << ped.id << endl;
    }
}

void WorldStateTracker::removePeds() {
	ped_list.resize(0);
}

void WorldStateTracker::updateCar(const COORD& car, const double dist) {
    carpos=car;
    car_dist_trav=dist;
}

bool WorldStateTracker::emergency() {
    //TODO improve emergency stop to prevent the false detection of leg
    double mindist = numeric_limits<double>::infinity();
    for(auto& ped : ped_list) {
		COORD p(ped.w, ped.h);
        double d = COORD::EuclideanDistance(carpos, p);
        if (d < mindist) mindist = d;
    }
	cout << "emergency mindist = " << mindist << endl;
	return (mindist < 0.5);
}

void WorldStateTracker::updateVel(double vel) {
	carvel = vel;
}

vector<WorldStateTracker::PedDistPair> WorldStateTracker::getSortedPeds(bool doPrint) {
    // sort peds
    vector<PedDistPair> sorted_peds;

    if(doPrint) cout << "[getSortedPeds] state tracker ped_list size " << ped_list.size() << endl;
    for(const auto& p: ped_list) {
        COORD cp(p.w, p.h);
        float dist = COORD::EuclideanDistance(cp, carpos);
        sorted_peds.push_back(PedDistPair(dist, p));

		if(doPrint) cout << "[getSortedPeds] ped id:"<< p.id << endl;

    }
    sort(sorted_peds.begin(), sorted_peds.end(),
            [](const PedDistPair& a, const PedDistPair& b) -> bool {
                return a.first < b.first;
            });


    return sorted_peds;
}

PomdpState WorldStateTracker::getPomdpState() {
    auto sorted_peds = getSortedPeds();

    // construct PomdpState
    PomdpState pomdpState;
    pomdpState.car.pos = model.path.nearest(carpos);
    pomdpState.car.vel = carvel;
	pomdpState.car.dist_travelled = car_dist_trav;
    pomdpState.num = sorted_peds.size();

	if (pomdpState.num > ModelParams::N_PED_IN) {
		pomdpState.num = ModelParams::N_PED_IN;
	}

    for(int i=0;i<pomdpState.num;i++) {
        const auto& ped = sorted_peds[i].second;
        pomdpState.peds[i].pos.x=ped.w;
        pomdpState.peds[i].pos.y=ped.h;
		pomdpState.peds[i].id = ped.id;
		pomdpState.peds[i].goal = -1;
    }
	return pomdpState;
}

void WorldBeliefTracker::update() {
    // update car
    car.pos = model.path.nearest(stateTracker.carpos);
    car.vel = stateTracker.carvel;
	car.dist_travelled = /*0*/stateTracker.car_dist_trav;

    auto sorted_peds = stateTracker.getSortedPeds();
    map<int, PedStruct> newpeds;
    for(const auto& dp: sorted_peds) {
        auto& p = dp.second;
        PedStruct ped(COORD(p.w, p.h), -1, p.id);
        newpeds[p.id] = ped;
    }

    // remove disappeared peds
    vector<int> peds_to_remove;
    for(const auto& p: peds) {
        if (newpeds.find(p.first) == newpeds.end()) {
            peds_to_remove.push_back(p.first);
            logi << "["<<__FUNCTION__<< "]" << " removing ped "<< p.first << endl;
        }
    }
    for(const auto& i: peds_to_remove) {
        peds.erase(i);
    }

    // update existing peds
    for(auto& kv : peds) {
        model.updatePedBelief(kv.second, newpeds[kv.first]);
    }

    // add new peds
    for(const auto& kv: newpeds) {
		auto& p = kv.second;
        if (peds.find(p.id) == peds.end()) {
            peds[p.id] = model.initPedBelief(p);
        }
    }

	sorted_beliefs.clear();
	for(const auto& dp: sorted_peds) {
		auto& p = dp.second;
		sorted_beliefs.push_back(peds[p.id]);
	}


    return;
}

int PedBelief::sample_goal() const {
    double r = /*double(rand()) / RAND_MAX*/ Random::RANDOM.NextDouble();
    int i = 0;
    r -= prob_goals[i];
    while(r > 0) {
        i++;
        r -= prob_goals[i];
    }
    return i;
}

int PedBelief::maxlikely_goal() const {
    double ml = 0;
    int mi = prob_goals.size()-1; // stop intention
    for(int i=0; i<prob_goals.size(); i++) {
        if (prob_goals[i] > ml && prob_goals[i] > 0.5) {
            ml = prob_goals[i];
            mi = i;
        }
    }
    return mi;
}

void WorldBeliefTracker::printBelief() const {
    //return;
	int num = 0;
    for(int i=0; i < sorted_beliefs.size() && i < ModelParams::N_PED_IN; i++) {
		auto& p = sorted_beliefs[i];
		if (COORD::EuclideanDistance(p.pos, model.path[car.pos]) < 10) {
            cout << "ped belief " << p.id << ": ";
            for (int g = 0; g < p.prob_goals.size(); g ++)
                cout << " " << p.prob_goals[g];
            cout << endl;
		}
    }
}

PomdpState WorldBeliefTracker::sample() {
    PomdpState s;
    s.car = car;

	s.num = 0;
    for(int i=0; i < sorted_beliefs.size() && i < ModelParams::N_PED_IN; i++) {
		auto& p = sorted_beliefs[i];
		if (COORD::EuclideanDistance(p.pos, model.path[car.pos]) < 10) {
			s.peds[s.num].pos = p.pos;
			s.peds[s.num].goal = p.sample_goal();
			s.peds[s.num].id = p.id;
            s.peds[s.num].vel = p.vel;
			s.num ++;
		}
    }

    return s;
}

vector<PomdpState> WorldBeliefTracker::sample(int num) {

	if(DESPOT::Debug_mode)
		Random::RANDOM.seed(0);

    vector<PomdpState> particles;
    for(int i=0; i<num; i++) {
        particles.push_back(sample());
    }

    cout << "Num peds for planning: " << particles[0].num << endl;

    return particles;
}

vector<PedStruct> WorldBeliefTracker::predictPeds() {
    vector<PedStruct> prediction;

    for(const auto& p: sorted_beliefs) {
        double dist = COORD::EuclideanDistance(p.pos, model.path[car.pos]);
        int step = (p.vel + car.vel>1e-5)?int(dist / (p.vel + car.vel) * ModelParams::control_freq):100000;

        for(int j=0; j<1; j++) {
            int goal = p.maxlikely_goal();
            PedStruct ped0(p.pos, goal, p.id);
			ped0.vel = p.vel;

            for(int i=0; i<4; i++) {
                PedStruct ped = ped0;
                model.PedStepDeterministic(ped, step + i);
                prediction.push_back(ped);
            }
        }
    }
    return prediction;
}

void WorldBeliefTracker::PrintState(const State& s, ostream& out) const {
	const PomdpState & state=static_cast<const PomdpState&> (s);
    COORD& carpos = model.path[state.car.pos];

	out << "Rob Pos: " << carpos.x<< " " <<carpos.y << endl;
	out << "Rob travelled: " << state.car.dist_travelled << endl;
	for(int i = 0; i < state.num; i ++) {
		out << "Ped Pos: " << state.peds[i].pos.x << " " << state.peds[i].pos.y << endl;
		out << "Goal: " << state.peds[i].goal << endl;
		out << "id: " << state.peds[i].id << endl;
	}
	out << "Vel: " << state.car.vel << endl;
	out<<  "num  " << state.num << endl;
	double min_dist = COORD::EuclideanDistance(carpos, state.peds[0].pos);
	out << "MinDist: " << min_dist << endl;
}

void WorldModel::RVO2PedStep(PedStruct peds[], Random& random, int num_ped){
    int threadID=GetThreadID();
    ped_sim_[threadID]->clearAllAgents();

    for(int i=0; i<num_ped; i++){
        ped_sim_[threadID]->addAgent(RVO::Vector2(peds[i].pos.x, peds[i].pos.y));
    }

    // Set the preferred velocity for each agent.
    for (size_t i = 0; i < ped_sim_[threadID]->getNumAgents(); ++i) {
        int goal_id = peds[i].goal;
        if (goal_id >= goals.size()-1) { /// stop intention
            ped_sim_[threadID]->setAgentPrefVelocity(i, RVO::Vector2(0.0f, 0.0f));
        } else{
            RVO::Vector2 goal(goals[goal_id].x, goals[goal_id].y);
            if ( absSq(goal - ped_sim_[threadID]->getAgentPosition(i)) < ped_sim_[threadID]->getAgentRadius(i) * ped_sim_[threadID]->getAgentRadius(i) ) {
                // Agent is within one radius of its goal, set preferred velocity to zero
                ped_sim_[threadID]->setAgentPrefVelocity(i, RVO::Vector2(0.0f, 0.0f));
            } else {
                // Agent is far away from its goal, set preferred velocity as unit vector towards agent's goal.
                ped_sim_[threadID]->setAgentPrefVelocity(i, normalize(goal - ped_sim_[threadID]->getAgentPosition(i)));
            }
        }
        
    }

    ped_sim_[threadID]->doStep();

    for(int i=0; i<num_ped; i++){
        peds[i].pos.x=ped_sim_[threadID]->getAgentPosition(i).x() + random.NextGaussian() * ModelParams::NOISE_PED_POS / freq;
        peds[i].pos.y=ped_sim_[threadID]->getAgentPosition(i).y() + random.NextGaussian() * ModelParams::NOISE_PED_POS / freq;
    }
}

void WorldModel::RVO2PedStep(PedStruct peds[], Random& random, int num_ped, CarStruct car){
    int threadID=GetThreadID();
    ped_sim_[threadID]->clearAllAgents();

    //adding pedestrians
    for(int i=0; i<num_ped; i++){
        ped_sim_[threadID]->addAgent(RVO::Vector2(peds[i].pos.x, peds[i].pos.y));
    }

    // adding car as a "special" pedestrian
    double car_x, car_y, car_yaw;
    car_x = path[car.pos].x;
    car_y = path[car.pos].y;
    car_yaw = path.getYaw(car.pos);

    /// for audi r8
    ped_sim_[threadID]->addAgent(RVO::Vector2(car_x, car_y), 4.0f, 2, 1.0f, 2.0f, 1.15f, 3.0f, RVO::Vector2(), "vehicle");
    ped_sim_[threadID]->setAgentPrefVelocity(num_ped, RVO::Vector2(car.vel * cos(car_yaw), car.vel * sin(car_yaw)));

    ped_sim_[threadID]->addAgent(RVO::Vector2(car_x + 0.56 * 2.33 * cos(car_yaw), car_y + 1.4* sin(car_yaw)), 4.0f, 2, 1.0f, 2.0f, 1.15f, 3.0f, RVO::Vector2(), "vehicle");
    ped_sim_[threadID]->setAgentPrefVelocity(num_ped+1, RVO::Vector2(car.vel * cos(car_yaw), car.vel * sin(car_yaw)));

    ped_sim_[threadID]->addAgent(RVO::Vector2(car_x + 0.56*3.66 * cos(car_yaw), car_y + 2.8* sin(car_yaw)), 4.0f, 2, 1.0f, 2.0f, 1.15f, 3.0f, RVO::Vector2(), "vehicle");
    ped_sim_[threadID]->setAgentPrefVelocity(num_ped+2, RVO::Vector2(car.vel * cos(car_yaw), car.vel * sin(car_yaw))); 

    ped_sim_[threadID]->addAgent(RVO::Vector2(car_x + 0.56*5 * cos(car_yaw), car_y + 2.8* sin(car_yaw)), 4.0f, 2, 1.0f, 2.0f, 1.6f, 3.0f, RVO::Vector2(), "vehicle");
    ped_sim_[threadID]->setAgentPrefVelocity(num_ped+3, RVO::Vector2(car.vel * cos(car_yaw), car.vel * sin(car_yaw))); 

    // Set the preferred velocity for each agent.
    for (size_t i = 0; i < num_ped; ++i) {
        int goal_id = peds[i].goal;
        if (goal_id >= goals.size()-1) { /// stop intention
            ped_sim_[threadID]->setAgentPrefVelocity(i, RVO::Vector2(0.0f, 0.0f));
        } else{
            RVO::Vector2 goal(goals[goal_id].x, goals[goal_id].y);
            if ( absSq(goal - ped_sim_[threadID]->getAgentPosition(i)) < ped_sim_[threadID]->getAgentRadius(i) * ped_sim_[threadID]->getAgentRadius(i) ) {
                // Agent is within one radius of its goal, set preferred velocity to zero
                ped_sim_[threadID]->setAgentPrefVelocity(i, RVO::Vector2(0.0f, 0.0f));
            } else {
                // Agent is far away from its goal, set preferred velocity as unit vector towards agent's goal.
                ped_sim_[threadID]->setAgentPrefVelocity(i, normalize(goal - ped_sim_[threadID]->getAgentPosition(i))*1.2);
            }
        }
        
    }

    ped_sim_[threadID]->doStep();

    for(int i=0; i<num_ped; i++){
        peds[i].pos.x=ped_sim_[threadID]->getAgentPosition(i).x() + random.NextGaussian() * (ped_sim_[threadID]->getAgentPosition(i).x() - peds[i].pos.x)/5.0; //random.NextGaussian() * ModelParams::NOISE_PED_POS / freq;
        peds[i].pos.y=ped_sim_[threadID]->getAgentPosition(i).y() + random.NextGaussian() * (ped_sim_[threadID]->getAgentPosition(i).y() - peds[i].pos.y)/5.0;//random.NextGaussian() * ModelParams::NOISE_PED_POS / freq;
      
    }

}


double GenerateGaussian(double rNum)
{
    if(FIX_SCENARIO!=1 && !CPUDoPrint)
        rNum=QuickRandom::RandGeneration(rNum);
    double result = sqrt(-2 * log(rNum));
    if(FIX_SCENARIO!=1 && !CPUDoPrint)
        rNum=QuickRandom::RandGeneration(rNum);

    result*= cos(2 * M_PI * rNum);
    return result;
}


void WorldModel::RVO2PedStep(PedStruct peds[], double& random, int num_ped){

    int threadID=GetThreadID();
    ped_sim_[threadID]->clearAllAgents();

    for(int i=0; i<num_ped; i++){
        ped_sim_[threadID]->addAgent(RVO::Vector2(peds[i].pos.x, peds[i].pos.y));
    }

    // Set the preferred velocity for each agent.
    for (size_t i = 0; i < ped_sim_[threadID]->getNumAgents(); ++i) {
        int goal_id = peds[i].goal;
        if (goal_id >= goals.size()-1) { /// stop intention
            ped_sim_[threadID]->setAgentPrefVelocity(i, RVO::Vector2(0.0f, 0.0f));
        } else{
            RVO::Vector2 goal(goals[goal_id].x, goals[goal_id].y);
            if ( absSq(goal - ped_sim_[threadID]->getAgentPosition(i)) < ped_sim_[threadID]->getAgentRadius(i) * ped_sim_[threadID]->getAgentRadius(i) ) {
                // Agent is within one radius of its goal, set preferred velocity to zero
                ped_sim_[threadID]->setAgentPrefVelocity(i, RVO::Vector2(0.0f, 0.0f));
            } else {
                // Agent is far away from its goal, set preferred velocity as unit vector towards agent's goal.
                ped_sim_[threadID]->setAgentPrefVelocity(i, normalize(goal - ped_sim_[threadID]->getAgentPosition(i)));
            }
        }
        
    }

    ped_sim_[threadID]->doStep();

    for(int i=0; i<num_ped; i++){
        double rNum=GenerateGaussian(random);
        peds[i].pos.x=ped_sim_[threadID]->getAgentPosition(i).x() + rNum * ModelParams::NOISE_PED_POS / freq;
        rNum=GenerateGaussian(rNum);
        peds[i].pos.y=ped_sim_[threadID]->getAgentPosition(i).y() + rNum * ModelParams::NOISE_PED_POS / freq;
    }
}

void WorldModel::RVO2PedStep(PedStruct peds[], double& random, int num_ped, CarStruct car){
    int threadID=GetThreadID();
    ped_sim_[threadID]->clearAllAgents();

    //adding pedestrians
    for(int i=0; i<num_ped; i++){
        ped_sim_[threadID]->addAgent(RVO::Vector2(peds[i].pos.x, peds[i].pos.y));
    }

    // adding car as a "special" pedestrian
    double car_x, car_y, car_yaw;
    car_x = path[car.pos].x;
    car_y = path[car.pos].y;
    car_yaw = path.getYaw(car.pos);
//    ped_sim_[threadID]->addAgent(RVO::Vector2(car_x, car_y), 3.0f, 2, 1.0f, 2.0f, 1.0f, 3.0f, RVO::Vector2(), "vehicle");
    ped_sim_[threadID]->addAgent(RVO::Vector2(car_x + 0.42 * cos(car_yaw), car_y + 0.42 * sin(car_yaw)), 3.0f, 2, 1.0f, 2.0f, 1.15f, 3.0f, RVO::Vector2(), "vehicle");

    ped_sim_[threadID]->setAgentPrefVelocity(num_ped, RVO::Vector2(car.vel * cos(car_yaw), car.vel * sin(car_yaw)));

    // Set the preferred velocity for each agent.
    for (size_t i = 0; i < num_ped; ++i) {
        int goal_id = peds[i].goal;
        if (goal_id >= goals.size()-1) { /// stop intention
            ped_sim_[threadID]->setAgentPrefVelocity(i, RVO::Vector2(0.0f, 0.0f));
        } else{
            RVO::Vector2 goal(goals[goal_id].x, goals[goal_id].y);
            if ( absSq(goal - ped_sim_[threadID]->getAgentPosition(i)) < ped_sim_[threadID]->getAgentRadius(i) * ped_sim_[threadID]->getAgentRadius(i) ) {
                // Agent is within one radius of its goal, set preferred velocity to zero
                ped_sim_[threadID]->setAgentPrefVelocity(i, RVO::Vector2(0.0f, 0.0f));
            } else {
                // Agent is far away from its goal, set preferred velocity as unit vector towards agent's goal.
                ped_sim_[threadID]->setAgentPrefVelocity(i, normalize(goal - ped_sim_[threadID]->getAgentPosition(i))*1.2);
            }
        }
        
    }

    ped_sim_[threadID]->doStep();

    for(int i=0; i<num_ped; i++){
        double rNum=GenerateGaussian(random);
        peds[i].pos.x=ped_sim_[threadID]->getAgentPosition(i).x() + rNum * (ped_sim_[threadID]->getAgentPosition(i).x() - peds[i].pos.x)/5.0; //random.NextGaussian() * ModelParams::NOISE_PED_POS / freq;
        rNum=GenerateGaussian(rNum);
        peds[i].pos.y=ped_sim_[threadID]->getAgentPosition(i).y() + rNum * (ped_sim_[threadID]->getAgentPosition(i).y() - peds[i].pos.y)/5.0;//random.NextGaussian() * ModelParams::NOISE_PED_POS / freq;
      
    }

}



