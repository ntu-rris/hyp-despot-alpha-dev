using namespace std;

#include <despot/GPUinterface/GPUpomdp.h>
namespace despot {


// Model-related GPU functions 

DEVICE bool (*DvcModelStep_)(Dvc_State&, float, ACT_TYPE, float&, OBS_TYPE&)=NULL;
DEVICE bool (*DvcModelStepIntObs_)(Dvc_State&, float, ACT_TYPE, float&, int*)=NULL;
DEVICE float (*DvcModelObsProb_)(OBS_TYPE&, Dvc_State&, ACT_TYPE)=NULL;
DEVICE float (*DvcModelObsProbIntObs_)(int*, Dvc_State&, ACT_TYPE)=NULL;
DEVICE Dvc_ValuedAction (*DvcModelGetBestAction_)()=NULL;
DEVICE int (*DvcModelNumActions_)() = NULL;
DEVICE float (*DvcModelGetMaxReward_)()=NULL;
DEVICE float (*DvcModelGetCarVel_)(Dvc_State* , int)=NULL; //Specific to car driving
DEVICE void (*DvcModelPrintDvcState_)(Dvc_State*)=NULL;//Useful for printing state inside the solver



// Memory management-related functions
DEVICE void (*DvcModelCopyNoAlloc_)(Dvc_State*, const Dvc_State*, int pos,
		bool offset_des)=NULL;
DEVICE void (*DvcModelCopyToShared_)(Dvc_State*, const Dvc_State*, int pos,
		bool offset_des)=NULL;
DEVICE Dvc_State* (*DvcModelGet_)(Dvc_State* , int )=NULL;


// Lowerbound-related GPU functions



// Upperbound-related GPU functions

/*Unused function pointers*/
//DEVICE Dvc_State* (*DvcModelAlloc_)(int num)=NULL;
//DEVICE Dvc_State* (*DvcModelCopy_)(const Dvc_State*, int pos)=NULL;
//DEVICE void (*DvcModelFree_)(Dvc_State*)=NULL;



/*DvcModelStep_ = NULL;
DvcModelStepIntObs_ = NULL;

DvcModelGetBestAction_ = NULL;
DvcModelGetMaxReward_ = NULL;
Dvc_DSPOMDP::DvcModelNumAction_ = NULL;

DvcModelCopyNoAlloc_ = NULL;
DvcModelCopyToShared_ = NULL;
DvcModelGet_ = NULL;*/


} // namespace despot
