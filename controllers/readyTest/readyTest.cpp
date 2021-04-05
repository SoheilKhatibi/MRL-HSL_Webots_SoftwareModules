#include <webots/Robot.hpp>
#include <webots/Supervisor.hpp>
#include <webots/Gyro.hpp>
#include <webots/Motor.hpp>
#include <webots/Keyboard.hpp>

#include "luatables.h"
#include "luaOPKinematics.h"

#include <math.h>
#include <cmath>
#include <boost/foreach.hpp>
#include <boost/interprocess/shared_memory_object.hpp>
#include <boost/interprocess/managed_shared_memory.hpp>
#include <boost/interprocess/mapped_region.hpp>
#include <boost/interprocess/sync/interprocess_semaphore.hpp>

#define NMOTORS 20

using namespace webots;

static const char *jointNames[NMOTORS] = {"Neck", "Head",
                                          "ShoulderL", "ArmUpperL", "ArmLowerL",
                                          "PelvYL", "PelvL", "LegUpperL", "LegLowerL", "AnkleL", "FootL", 
                                          "PelvYR", "PelvR", "LegUpperR", "LegLowerR", "AnkleR", "FootR",
                                          "ShoulderR", "ArmUpperR", "ArmLowerR",
                                         };

// static const char *motorNames[NMOTORS] = {
//   "ShoulderR" /*ID1 */, "ShoulderL" /*ID2 */, "ArmUpperR" /*ID3 */, "ArmUpperL" /*ID4 */, "ArmLowerR" /*ID5 */,
//   "ArmLowerL" /*ID6 */, "PelvYR" /*ID7 */,    "PelvYL" /*ID8 */,    "PelvR" /*ID9 */,     "PelvL" /*ID10*/,
//   "LegUpperR" /*ID11*/, "LegUpperL" /*ID12*/, "LegLowerR" /*ID13*/, "LegLowerL" /*ID14*/, "AnkleR" /*ID15*/,
//   "AnkleL" /*ID16*/,    "FootR" /*ID17*/,     "FootL" /*ID18*/,     "Neck" /*ID19*/,      "Head" /*ID20*/
// };

Supervisor *robot;
webots::Gyro *gyro;
webots::Motor *mMotors[NMOTORS];
Keyboard *keyboard;

Node *Ashkan;
Field *AshkanTranslation;
Field *AshkanRotation;

std::vector<int> moveDir(NMOTORS, 0);
std::vector<int> jointReverse;

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// -- Walk Parameters
// -- Stance and velocity limit values
std::vector<double> stanceLimitX(2, 0);
std::vector<double> stanceLimitY(2, 0);
std::vector<double> stanceLimitA(2, 0);
std::vector<double> velLimitX(2, 0);
std::vector<double> velLimitY(2, 0);
std::vector<double> velLimitA(2, 0);
std::vector<double> velDelta(3, 0);
double vaFactor;

double velXHigh;
double velDeltaXHigh;

// --Toe/heel overlap checking values
std::vector<double> footSizeX(2, 0);
double stanceLimitMarginY;
double stanceLimitY2;

// --OP default stance width: 0.0375*2 = 0.075
// --Heel overlap At radian 0.15 at each foot = 0.05*sin(0.15)*2=0.015
// --Heel overlap At radian 0.30 at each foot = 0.05*sin(0.15)*2=0.030

// --Stance parameters
double bodyHeight;
double bodyTilt;
double footX;
double footY;
double supportX;
double supportY;
std::vector<double> qLArm0(3, 0);
std::vector<double> qRArm0(3, 0);
// qLArmKick0 = Config.walk.qLArmKick
// qRArmKick0 = Config.walk.qRArmKick

// --Hardness parameters
double hardnessSupport;
double hardnessSwing;

// hardnessArm0 = Config.walk.hardnessArm or 0.2
// hardnessArm = Config.walk.hardnessArm or 0.2

// --Gait parameters
double tStep0;
double tStep;
double tZmp;
double stepHeight0;
double stepHeight;
double ph1Single, ph2Single;
double ph1Zmp, ph2Zmp;

// --Compensation parameters
double hipRollCompensation;
std::vector<double> ankleMod(2, 0);
// spreadComp = Config.walk.spreadComp or 0
double turnCompThreshold;
double turnComp;

// --Gyro stabilization parameters
std::vector<double> ankleImuParamX(4, 0);
std::vector<double> ankleImuParamY(4, 0);
std::vector<double> kneeImuParamX(4, 0);
std::vector<double> hipImuParamY(4, 0);
std::vector<double> armImuParamX(4, 0);
std::vector<double> armImuParamY(4, 0);

// --Support bias parameters to reduce backlash-based instability
double velFastForward;
double velFastTurn;
double supportFront;
double supportFront2;
double supportBack;
double supportSideX;
double supportSideY;
double supportTurn;

double frontComp;
double AccelComp;

// --Initial body swing
double supportModYInitial;

// --WalkKick parameters
// walkKickDef = Config.walk.walkKickDef
double walkKickPh;
double toeTipCompensation;

// use_alternative_trajectory = Config.walk.use_alternative_trajectory or 0

// ----------------------------------------------------------
// -- Walk state variables
// ----------------------------------------------------------

std::vector<double> uTorso(3, 0);
std::vector<double> uLeft(3, 0);
std::vector<double> uRight(3, 0);

std::vector<double> uTorso1(3, 0);
std::vector<double> uTorso2(3, 0);
std::vector<double> uLeft1(3, 0);
std::vector<double> uLeft2(3, 0);
std::vector<double> uRight1(3, 0);
std::vector<double> uRight2(3, 0);
std::vector<double> uSupport(3, 0);

std::vector<double> pLLeg;
std::vector<double> pRLeg;
std::vector<double> pTorso;

std::vector<double> velCurrent(3, 0);
std::vector<double> velCommand(3, 0);
std::vector<double> velDiff(3, 0);

// --ZMP exponential coefficients:
double aXP, aXN, aYP, aYN;

// --Gyro stabilization variables
std::vector<double> ankleShift(2, 0);
double kneeShift;
std::vector<double> hipShift(2, 0);
std::vector<double> armShift(2, 0);

bool active;
bool started;
int iStep0;
int iStep;
double t0;
// double t;
double tLastStep;

double ph0;
double ph;

int stopRequest;
// canWalkKick = 1 --Can we do walkkick with this walk code?
int walkKickRequest;
// walkKick = walkKickDef['FrontLeft']
int current_step_type;

int initial_step;

int upper_body_overridden;
int motion_playing;

std::vector<double> qLArmOR0(3, 0);
std::vector<double> qRArmOR0(3, 0);

// bodyRot0 = {0, bodyTilt, 0}

std::vector<double> qLArmOR(3, 0);
std::vector<double> qRArmOR(3, 0);

std::vector<double> bodyRot(3, 0);

// qLArmOR1 = {0, 0, 0}
// qRArmOR1 = {0, 0, 0}
// bodyRot1 = {0, 0, 0}

double phSingle;

// --Current arm pose
// qLArm = math.pi / 180 * vector.new({90, 40, -160})
// qRArm = math.pi / 180 * vector.new({90, -40, -160})

// --Standard offset
std::vector<double> uLRFootOffset(3, 0);

// --Walking/Stepping transition variables
std::vector<double> uLeftI(3, 0);
std::vector<double> uRightI(3, 0);
std::vector<double> uTorsoI(3, 0);
int supportI;
bool start_from_step;

// comdot = {0, 0}
int stepkick_ready;
int stepKickRequest;
int has_ball;
int supportLeg;
std::vector<double> supportMod(2, 0);
double shiftFactor;
double m1X;
double m2X;
double m1Y;
double m2Y;
std::vector<double> uRight15(3, 0);
std::vector<double> uLeft15(3, 0);
std::vector<double> uTorsoActual(3, 0);

// ----------------------------------------------------------
// -- End initialization
// ----------------------------------------------------------
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

//////////////////// Model ////////////////////
std::vector<double> robotPose(3, 0);
//////////////////// Model ////////////////////

//////////////////// Game ////////////////////
int gamePhase;
std::vector<double> homePose(3, 0);
double maxStep;
std::vector<double> thClose(2, 0);
//////////////////// Game ////////////////////

void set_actuator_command(std::vector<double> a, int index) {
    for (int i = 0; i < int(a.size()); i++) {
        // actuator.command[index+i-1] = moveDir[index+i-1]*(a[i]+jointBias[index+i-1]);
        mMotors[index + i]->setPosition(moveDir[index + i]*a[i]);
    }
}

void set_lleg_command(std::vector<double> val) {
    // std::cout<<"777777777777 "<<val.size()<<std::endl;
    // for (int i = 0; i < int(val.size()); i++) {
    //     std::cout<<"777777777777 "<<val[i]<<std::endl;
    // }
    // Command();
    set_actuator_command(val, 5);
}

double getTime(){
    // struct timeval t;
    // gettimeofday(&t, NULL);
    // return t.tv_sec + 1E-6*t.tv_usec;

    return robot->getTime();
}

double mod_angle(double a) {
    if (a == NULL) {
        return NULL;
    }
    // Reduce angle to [-pi, pi)
    a = remainder(a, 2 * M_PI);
    if (a >= M_PI) {
        a = a - 2 * M_PI;
    }
    return a;
}

std::vector<double> pose_global(std::vector<double> pRelative, std::vector<double> pose) {
    double ca = cos(pose[2]);
    double sa = sin(pose[2]);
    std::vector<double> GlobalPose(3, 0);
    GlobalPose[0] = pose[0] + ca*pRelative[0] - sa*pRelative[1];
    GlobalPose[1] = pose[1] + sa*pRelative[0] + ca*pRelative[1];
    GlobalPose[2] = pose[2] + pRelative[2];
    return GlobalPose;
}

std::vector<double> pose_relative(std::vector<double> pGlobal, std::vector<double> pose) {
    double ca = cos(pose[2]);
    double sa = sin(pose[2]);
    double px = pGlobal[0] - pose[0];
    double py = pGlobal[1] - pose[1];
    double pa = pGlobal[2] - pose[2];
    std::vector<double> LocalPose(3, 0);
    LocalPose[0] = ca * px + sa * py;
    LocalPose[1] = -sa * px + ca * py;
    LocalPose[2] = mod_angle(pa);
    return LocalPose;
}

void stance_reset() { // standup/sitdown/falldown handling
    // if (start_from_step) {
    //     uLeft = uLeftI;
    //     uRight = uRightI;
    //     uTorso = uTorsoI;
    //     if (supportI == 0) { // start with left support
    //         iStep0 = -1;
    //         iStep = 0;
    //     } else {
    //         iStep0 = 0; // start with right support
    //         iStep = 1;
    //     }
    //     initial_step = 1 // start walking asap
    // } else if (Motion.sm:get_previous_state()._NAME == 'PunchKick') {
    //     std::cout<<"Kick Resetted"<<std::endl;
    //     if (mcm.get_kick_type() == 'kickForwardLeft') {
    //         iStep0 = -1;
    //         iStep = 0;
    //     } else {
    //         iStep0 = 0;
    //         iStep = 1;
    //     }
    // } else {
        std::cout<<"Stance Resetted"<<std::endl;
        uLeft = pose_global({-supportX, footY, 0}, uTorso);
        uRight = pose_global({-supportX, -footY, 0}, uTorso);
        iStep0 = -1;
        iStep = 0;
    // }

    uLeft1 = uLeft;
    uLeft2 = uLeft;

    uRight1 = uRight;
    uRight2 = uRight;
    
    uTorso1 = uTorso;
    uTorso2 = uTorso;

    uSupport = uTorso;

    tLastStep = getTime();
    
    walkKickRequest = 0;
    current_step_type = 0;
    motion_playing = 0;
    upper_body_overridden = 0;
    uLRFootOffset = {0, footY, 0};
    start_from_step = false;
}

void entryWalk() {
    std::cout<<"Motion: Walk entry"<<std::endl;
    // SJ: now we always assume that we start walking with feet together
    // Because joint readings are not always available with darwins
    stance_reset();

    walkKickRequest = 0;
    stepkick_ready = false;
    stepKickRequest = 0;
    velCurrent = {0, 0, 0};
    velCommand = {0, 0, 0};
}

std::vector<double> MultiplyVectorByScalar(std::vector<double> v, double k){
	std::vector<double> ans(v.size(), 0);
    for(int i=0; i<int(v.size()); ++i)
		ans[i] = v[i] * k;
    return ans;
}

void set_velocity(std::vector<double> v) {
    // Filter the commanded speed
    v[0] = std::min(std::max(v[0], velLimitX[0]), velLimitX[1]);
    v[1] = std::min(std::max(v[1], velLimitY[0]), velLimitY[1]);
    v[2] = std::min(std::max(v[2], velLimitA[0]), velLimitA[1]);

    // Slow down when turning
    double vFactor = 1 - abs(v[2]) / vaFactor;

    double stepMag = sqrt(pow(v[0], 2) + pow(v[1], 2));
    double magFactor = std::min(velLimitX[1] * vFactor, stepMag) / (stepMag + 0.000001);

    velCommand[0] = v[0] * magFactor;
    velCommand[1] = v[1] * magFactor;
    velCommand[2] = v[2];

    velCommand[0] = std::min(std::max(velCommand[0], velLimitX[0]), velLimitX[1]);
    velCommand[1] = std::min(std::max(velCommand[1], velLimitY[0]), velLimitY[1]);
    velCommand[2] = std::min(std::max(velCommand[2], velLimitA[0]), velLimitA[1]);
}

void update_velocity() {
    if (velCurrent[0] > velXHigh) {
        // Slower accelleration at high speed
        velDiff[0] = std::min(std::max(velCommand[0] - velCurrent[0], -velDelta[0]), velDeltaXHigh);
    } else {
        velDiff[0] = std::min(std::max(velCommand[0] - velCurrent[0], -velDelta[0]), velDelta[0]);
    }
    velDiff[1] = std::min(std::max(velCommand[1] - velCurrent[1], -velDelta[1]), velDelta[1]);
    velDiff[2] = std::min(std::max(velCommand[2] - velCurrent[2], -velDelta[2]), velDelta[2]);

    velCurrent[0] = velCurrent[0] + velDiff[0];
    velCurrent[1] = velCurrent[1] + velDiff[1];
    velCurrent[2] = velCurrent[2] + velDiff[2];

    if (initial_step > 0) {
        velCurrent = {0, 0, 0};
        initial_step = initial_step - 1;
    }
}

void startWalk() {
    stopRequest = 0;
    if (!active) {
        active = true;
        started = false;
        iStep0 = -1;
        t0 = getTime();
        tLastStep = getTime();
        initial_step = 2;
    }
}

void stopWalk() {
    // Always stops with feet together (which helps transition)
    stopRequest = std::max(1, stopRequest);
}

std::vector<double> se2_interpolate(double t, std::vector<double> u1, std::vector<double> u2) {
    // helps smooth out the motions using a weighted average
    std::vector<double> out(3, 0);
    out[0] = u1[0] + t * (u2[0] - u1[0]);
    out[1] = u1[1] + t * (u2[1] - u1[1]);
    out[2] = u1[2] + t * mod_angle(u2[2] - u1[2]);
    return out;
}

double procFunc(double a, double deadband, double maxvalue) {
    // Piecewise linear function for IMU feedback
    double b;
    if (a > 0) {
        b = std::min(std::max(0.0, abs(a) - deadband), maxvalue);
    } else {
        b = -std::min(std::max(0.0, abs(a) - deadband), maxvalue);
    }
    return b;
}

std::vector<double> step_right_destination(std::vector<double> vel, std::vector<double> uLeft, std::vector<double> uRight) {
    std::vector<double> u0 = se2_interpolate(0.5, uLeft, uRight);
    // std::cout<<"u0:"<<u0[0]<<" "<<u0[1]<<" "<<u0[2]<<std::endl;
    // Determine nominal midpoint position 1.5 steps in future
    std::vector<double> u1 = pose_global(vel, u0);
    // std::cout<<"u1:"<<u1[0]<<" "<<u1[1]<<" "<<u1[2]<<std::endl;
    std::vector<double> u2 = pose_global(MultiplyVectorByScalar(vel, 0.5), u1);
    // std::cout<<"u2:"<<u2[0]<<" "<<u2[1]<<" "<<u2[2]<<std::endl;
    std::vector<double> uRightPredict = pose_global(MultiplyVectorByScalar(uLRFootOffset, -1), u2);
    // std::cout<<"uRightPredict:"<<uRightPredict[0]<<" "<<uRightPredict[1]<<" "<<uRightPredict[2]<<std::endl;
    // std::cout<<"uLeft:"<<uLeft[0]<<" "<<uLeft[1]<<" "<<uLeft[2]<<std::endl;
    std::vector<double> uRightLeft = pose_relative(uRightPredict, uLeft);
    // std::cout<<"uRightLeft:"<<uRightLeft[0]<<" "<<uRightLeft[1]<<" "<<uRightLeft[2]<<std::endl;
    // Do not pidgeon toe, cross feet:

    // Check toe and heel overlap
    double toeOverlap = footSizeX[0] * uRightLeft[2];
    double heelOverlap = footSizeX[1] * uRightLeft[2];
    double limitY = std::max(stanceLimitY[0], stanceLimitY2 + std::max(toeOverlap, heelOverlap));

    // print("Toeoverlap Heeloverlap",toeOverlap,heelOverlap,limitY)

    uRightLeft[0] = std::min(std::max(uRightLeft[0], stanceLimitX[0]), stanceLimitX[1]);
    uRightLeft[1] = std::min(std::max(uRightLeft[1], -stanceLimitY[1]), -limitY);
    uRightLeft[2] = std::min(std::max(uRightLeft[2], -stanceLimitA[1]), -stanceLimitA[0]);
    // std::cout<<"uRightLeft:"<<uRightLeft[0]<<" "<<uRightLeft[1]<<" "<<uRightLeft[2]<<std::endl;
    // std::cout<<"uLeft:"<<uLeft[0]<<" "<<uLeft[1]<<" "<<uLeft[2]<<std::endl;
    return pose_global(uRightLeft, uLeft);
}

std::vector<double> step_left_destination(std::vector<double> vel, std::vector<double> uLeft, std::vector<double> uRight) {
    std::vector<double> u0 = se2_interpolate(.5, uLeft, uRight);
    // Determine nominal midpoint position 1.5 steps in future
    std::vector<double> u1 = pose_global(vel, u0);
    std::vector<double> u2 = pose_global(MultiplyVectorByScalar(vel, 0.5), u1);
    std::vector<double> uLeftPredict = pose_global(uLRFootOffset, u2);
    std::vector<double> uLeftRight = pose_relative(uLeftPredict, uRight);
    // Do not pidgeon toe, cross feet:

    // Check toe and heel overlap
    double toeOverlap = -footSizeX[0] * uLeftRight[2];
    double heelOverlap = -footSizeX[1] * uLeftRight[2];
    double limitY = std::max(stanceLimitY[0], stanceLimitY2 + std::max(toeOverlap, heelOverlap));

    // print("Toeoverlap Heeloverlap",toeOverlap,heelOverlap,limitY)

    uLeftRight[0] = std::min(std::max(uLeftRight[0], stanceLimitX[0]), stanceLimitX[1]);
    uLeftRight[1] = std::min(std::max(uLeftRight[1], limitY), stanceLimitY[1]);
    uLeftRight[2] = std::min(std::max(uLeftRight[2], stanceLimitA[0]), stanceLimitA[1]);
    return pose_global(uLeftRight, uRight);
}

std::vector<double> step_torso(std::vector<double> uLeft, std::vector<double> uRight, double shiftFactor) {
    std::vector<double> u0 = se2_interpolate(0.5, uLeft, uRight);
    std::vector<double> uLeftSupport = pose_global({supportX, supportY, 0}, uLeft);
    std::vector<double> uRightSupport = pose_global({supportX, -supportY, 0}, uRight);
    return se2_interpolate(shiftFactor, uLeftSupport, uRightSupport);
}

void zmp_solve(double zs, double z1, double z2, double x1, double x2, double &aP, double &aN) {
    double T1 = tStep * ph1Zmp;
    double T2 = tStep * ph2Zmp;
    double m1 = (zs - z1) / T1;
    double m2 = -(zs - z2) / (tStep - T2);

    double c1 = x1 - z1 + tZmp * m1 * sinh(-T1 / tZmp);
    double c2 = x2 - z2 + tZmp * m2 * sinh((tStep - T2) / tZmp);
    double expTStep = exp(tStep / tZmp);
    aP = (c2 - c1 / expTStep) / (expTStep - 1.0 / expTStep);
    aN = (c1 * expTStep - c2) / (expTStep - 1.0 / expTStep);
}

void foot_phase(double ph, double &xf, double &zf) {
    // Computes relative x,z motion of foot during single support phase
    // phSingle = 0: x=0, z=0, phSingle = 1: x=1,z=0
    phSingle = std::min(std::max(ph-ph1Single, 0.0)/(ph2Single-ph1Single), 1.0);
    double phSingleSkew = pow(phSingle, 0.8) - 0.17*phSingle*(1-phSingle);
    xf = .5*(1-cos(M_PI*phSingleSkew));
    zf = .5*(1-cos(2*M_PI*phSingleSkew));
    // std::cout<<"xf, zf: "<<xf<<"  "<<zf<<std::endl;
}

std::vector<double> zmp_com(double ph) {
    // std::cout<<"PH::  "<<ph<<std::endl;
    std::vector<double> com = {0, 0, 0};
    // std::cout<<"tZmp: "<<tZmp<<std::endl;
    double expT = exp(tStep*ph/tZmp);
    // std::cout<<"expT: "<<expT<<std::endl;
    // std::cout<<"uSupport: "<<uSupport[0]<<" "<<uSupport[1]<<" "<<uSupport[2]<<std::endl;
    com[0] = uSupport[0] + aXP*expT + aXN/expT;
    // std::cout<<uSupport[1]<<"  "<<aYP * expT<<" "<<aYN / expT<<std::endl;
    com[1] = uSupport[1] + aYP*expT + aYN/expT;
    // if (ph < ph1Zmp) {
    //     com[0] = com[0] + m1X*tStep*(ph-ph1Zmp)-tZmp*m1X*sinh(tStep*(ph-ph1Zmp)/tZmp);
    //     com[1] = com[1] + m1Y*tStep*(ph-ph1Zmp)-tZmp*m1Y*sinh(tStep*(ph-ph1Zmp)/tZmp);
    // } else if (ph > ph2Zmp) {
    //     com[0] = com[0] + m2X*tStep*(ph-ph2Zmp)-tZmp*m2X*sinh(tStep*(ph-ph2Zmp)/tZmp);
    //     com[1] = com[1] + m2Y*tStep*(ph-ph2Zmp)-tZmp*m2Y*sinh(tStep*(ph-ph2Zmp)/tZmp);
    // }
    // com[3] = .5*(uLeft[3] + uRight[3]);
    // Linear speed turning
    com[2] = ph * (uLeft2[2]+uRight2[2])/2 + (1-ph)* (uLeft1[2]+uRight1[2])/2;
    // std::cout<<"------------"<<uTorso[0]<<" "<<uTorso[1]<<" "<<uTorso[2]<<std::endl;
    return com;
}

void motion_legs(std::vector<double> qLegs, bool gyro_off = false) {
    // double phComp = std::min({1.0, phSingle / 0.1, (1.0 - phSingle) / 0.1});

    // // Ankle stabilization using gyro feedback
    // const double *imuGyr = gyro->getValues();

    // double gyro_roll0 = imuGyr[0];
    // double gyro_pitch0 = imuGyr[1];
    // if (gyro_off) {
    //     gyro_roll0 = 0;
    //     gyro_pitch0 = 0;
    // }

    // // get effective gyro angle considering body angle offset
    // double yawAngle;
    // if (!active) { // double support
    //     yawAngle = (uLeft[2] + uRight[2]) / 2 - uTorsoActual[2];
    // } else if (supportLeg == 0) { // Left support
    //     yawAngle = uLeft[2] - uTorsoActual[2];
    // } else if (supportLeg == 1) {
    //     yawAngle = uRight[2] - uTorsoActual[2];
    // }

    // double gyro_roll = gyro_roll0 * cos(yawAngle) + -gyro_pitch0 * sin(yawAngle);
    // double gyro_pitch = gyro_pitch0 * cos(yawAngle) - gyro_roll0 * sin(yawAngle);

    // double armShiftX = procFunc(gyro_pitch * armImuParamY[1], armImuParamY[2], armImuParamY[3]);
    // double armShiftY = procFunc(gyro_roll * armImuParamY[1], armImuParamY[2], armImuParamY[3]);

    // double ankleShiftX = procFunc(gyro_pitch * ankleImuParamX[1], ankleImuParamX[2], ankleImuParamX[3]);
    // double ankleShiftY = procFunc(gyro_roll * ankleImuParamY[1], ankleImuParamY[2], ankleImuParamY[3]);
    // double kneeShiftX = procFunc(gyro_pitch * kneeImuParamX[1], kneeImuParamX[2], kneeImuParamX[3]);
    // double hipShiftY = procFunc(gyro_roll * hipImuParamY[1], hipImuParamY[2], hipImuParamY[3]);

    // ankleShift[0] = ankleShift[0] + ankleImuParamX[0] * (ankleShiftX - ankleShift[0]);
    // ankleShift[1] = ankleShift[1] + ankleImuParamY[0] * (ankleShiftY - ankleShift[1]);
    // kneeShift = kneeShift + kneeImuParamX[0] * (kneeShiftX - kneeShift);
    // hipShift[1] = hipShift[1] + hipImuParamY[0] * (hipShiftY - hipShift[1]);
    // armShift[0] = armShift[0] + armImuParamX[0] * (armShiftX - armShift[0]);
    // armShift[1] = armShift[1] + armImuParamY[0] * (armShiftY - armShift[1]);

    // // TODO: Toe/heel lifting
    // if (!active) { // Double support, standing still
    //     // qLegs[12] = qLegs[12] + ankleShift[2];                                         // Ankle roll stabilization
    //     // qLegs[2] = qLegs[2] + hipShift[2];                                             // Hip roll stabilization
    //     qLegs[3] = qLegs[3] + kneeShift;                                                  // Knee pitch stabilization
    //     qLegs[4] = qLegs[4] + ankleShift[0];                                              // Ankle pitch stabilization
    //     // qLegs[6] = qLegs[6] + ankleShift[2];                                           // Ankle roll stabilization

    //     // qLegs[8] = qLegs[8]  + hipShift[2];                                            // Hip roll stabilization
    //     qLegs[9] = qLegs[9] + kneeShift;                                                // Knee pitch stabilization
    //     qLegs[10] = qLegs[10] + ankleShift[0];                                            // Ankle pitch stabilization
    // } else if (supportLeg == 0) {   // Left support
    //     qLegs[1] = qLegs[1] + hipShift[1];                                                // Hip roll stabilization
    //     qLegs[3] = qLegs[3] + kneeShift;                                                  // Knee pitch stabilization
    //     qLegs[4] = qLegs[4] + ankleShift[0];                                              // Ankle pitch stabilization
    //     qLegs[5] = qLegs[5] + ankleShift[1];                                              // Ankle roll stabilization

    //     qLegs[10] = qLegs[10] + toeTipCompensation * phComp;                              // Lifting toetip
    //     qLegs[1] = qLegs[1] + hipRollCompensation * phComp;                               // Hip roll compensation
    // } else {
    //     qLegs[7] = qLegs[7] + hipShift[1];                                                // Hip roll stabilization
    //     qLegs[9] = qLegs[9] + kneeShift;                                                // Knee pitch stabilization
    //     qLegs[10] = qLegs[10] + ankleShift[0];                                            // Ankle pitch stabilization
    //     qLegs[11] = qLegs[11] + ankleShift[1];                                            // Ankle roll stabilization

    //     qLegs[4] = qLegs[4] + toeTipCompensation * phComp;                                // Lifting toetip
    //     qLegs[7] = qLegs[7] - hipRollCompensation * phComp;                               // Hip roll compensation
    // }
    set_lleg_command(qLegs);
}

void motion_arms() {
    // if has_ball > 0 then
    //     return
    // end

    // local qLArmActual = {}
    // local qRArmActual = {}

    // qLArmActual[1], qLArmActual[2] = qLArm0[1] + armShift[1], qLArm0[2] + armShift[2]
    // qRArmActual[1], qRArmActual[2] = qRArm0[1] + armShift[1], qRArm0[2] + armShift[2]

    // if upper_body_overridden > 0 or motion_playing > 0 then
    //     qLArmActual[1], qLArmActual[2], qLArmActual[3] = qLArmOR[1], qLArmOR[2], qLArmOR[3]
    //     qRArmActual[1], qRArmActual[2], qRArmActual[3] = qRArmOR[1], qRArmOR[2], qRArmOR[3]
    // end

    // --Check leg hitting
    // RotLeftA = util.mod_angle(uLeft[3] - uTorso[3])
    // RotRightA = util.mod_angle(uTorso[3] - uRight[3])

    // LLegTorso = util.pose_relative(uLeft, uTorso)
    // RLegTorso = util.pose_relative(uRight, uTorso)

    // qLArmActual[2] =
    //     math.max(
    //     5 * math.pi / 180 + math.max(0, RotLeftA) / 2 + math.max(0, LLegTorso[2] - 0.04) / 0.02 * 6 * math.pi / 180,
    //     qLArmActual[2]
    // )
    // qRArmActual[2] =
    //     math.min(
    //     -5 * math.pi / 180 - math.max(0, RotRightA) / 2 - math.max(0, -RLegTorso[2] - 0.04) / 0.02 * 6 * math.pi / 180,
    //     qRArmActual[2]
    // )
    // if upper_body_overridden > 0 or motion_playing > 0 then
    // else
    //     qLArmActual[3] = qLArm0[3]
    //     qRArmActual[3] = qRArm0[3]
    // end
    // Body.set_larm_command(qLArmActual)
    // Body.set_rarm_command(qRArmActual)
}

void update_still() {
    uTorso = step_torso(uLeft, uRight, 0.5);

    // Arm movement compensation
    double armPosCompX, armPosCompY;
    if (upper_body_overridden > 0 || motion_playing > 0) {
        // mass shift to X
        double elbowX =
            -sin(qLArmOR[0] - M_PI / 2 + bodyRot[0]) * cos(qLArmOR[0]) -
            sin(qRArmOR[0] - M_PI / 2 + bodyRot[0]) * cos(qRArmOR[0]);
        // mass shift to Y
        double elbowY = sin(qLArmOR[1]) + sin(qRArmOR[1]);
        armPosCompX = elbowX * -0.007;
        armPosCompY = elbowY * -0.007;
        pTorso[3] = bodyRot[0];
        pTorso[4] = bodyRot[1];
        pTorso[5] = bodyRot[2];
    } else {
        armPosCompX = 0;
        armPosCompY = 0;
        pTorso[3] = 0;
        pTorso[4] = bodyTilt;
        pTorso[5] = 0;
    }

    uTorsoActual = pose_global({-footX + armPosCompX, armPosCompY, 0}, uTorso);

    pTorso[5] = pTorso[5] + uTorsoActual[2];
    pTorso[0] = uTorsoActual[0];
    pTorso[1] = uTorsoActual[1];

    pLLeg[0] = uLeft[0];
    pLLeg[1] = uLeft[1];
    pLLeg[5] = uLeft[2];
    
    pRLeg[0] = uRight[0];
    pRLeg[1] = uRight[1];
    pRLeg[5] = uRight[2];

    std::vector<double> qLegs = inverse_legs(pLLeg, pRLeg, pTorso, supportLeg);
    // mcm.set_motion_supportLeg(2)
    // mcm.set_walk_pTorso(pTorso);
    motion_legs(qLegs, true);
    // motion_arms()
}

void UpdateKeyboard(){
    int key = keyboard->getKey();
    if (key>0){
        // std::cout<<key<<std::endl;
        if (key == 'I'){
            // std::cout<<"I"<<std::endl;
            std::vector<double> v(3, 0);
            v[0] = velCommand[0] + 0.02;
            v[1] = velCommand[1];
            v[2] = velCommand[2];
            set_velocity(v);
            std::cout<<"Command velocity: "<<velCommand[0]<<" "<<velCommand[1]<<" "<<velCommand[2]<<std::endl;
        } else if (key == 'J'){
            // std::cout<<"J"<<std::endl;
            std::vector<double> v(3, 0);
            v[0] = velCommand[0];
            v[1] = velCommand[1];
            v[2] = velCommand[2] + 0.1;
            set_velocity(v);
            std::cout<<"Command velocity: "<<velCommand[0]<<" "<<velCommand[1]<<" "<<velCommand[2]<<std::endl;
        }else if (key == 'K'){
            // std::cout<<"K"<<std::endl;
            std::vector<double> v(3, 0);
            v[0] = 0;
            v[1] = 0;
            v[2] = 0;
            set_velocity(v);
            std::cout<<"Command velocity: "<<velCommand[0]<<" "<<velCommand[1]<<" "<<velCommand[2]<<std::endl;
        }else if (key == 'L'){
            // std::cout<<"L"<<std::endl;
            std::vector<double> v(3, 0);
            v[0] = velCommand[0];
            v[1] = velCommand[1];
            v[2] = velCommand[2] - 0.1;
            set_velocity(v);
            std::cout<<"Command velocity: "<<velCommand[0]<<" "<<velCommand[1]<<" "<<velCommand[2]<<std::endl;
        }else if (key == ','){
            // std::cout<<","<<std::endl;
            std::vector<double> v(3, 0);
            v[0] = velCommand[0] - 0.02;
            v[1] = velCommand[1];
            v[2] = velCommand[2];
            set_velocity(v);
            std::cout<<"Command velocity: "<<velCommand[0]<<" "<<velCommand[1]<<" "<<velCommand[2]<<std::endl;
        }else if (key == 'H'){
            // std::cout<<"H"<<std::endl;
            std::vector<double> v(3, 0);
            v[0] = velCommand[0];
            v[1] = velCommand[1] + 0.02;
            v[2] = velCommand[2];
            set_velocity(v);
            std::cout<<"Command velocity: "<<velCommand[0]<<" "<<velCommand[1]<<" "<<velCommand[2]<<std::endl;
        }else if (key == ';'){
            // std::cout<<";"<<std::endl;
            std::vector<double> v(3, 0);
            v[0] = velCommand[0];
            v[1] = velCommand[1] - 0.02;
            v[2] = velCommand[2];
            set_velocity(v);
            std::cout<<"Command velocity: "<<velCommand[0]<<" "<<velCommand[1]<<" "<<velCommand[2]<<std::endl;
        } else if (key == '8'){
            // std::cout<<"8"<<std::endl;
            stopWalk();
        } else if (key == '9'){
            // std::cout<<"9"<<std::endl;
            startWalk();
        }
    }
    // std::cout<<key<<std::endl;
}

void entryGame() {
    maxStep = 0.06;
    gamePhase = 1;
    thClose = {0.03, 3 * M_PI / 180};
}

std::vector<double> toEuler(double x,double y,double z,double angle) {
	double heading, attitude, bank;
    double s=sin(angle);
	double c=cos(angle);
	double t=1-c;
	//  if axis is not already normalised then uncomment this
	// double magnitude = Math.sqrt(x*x + y*y + z*z);
	// if (magnitude==0) throw error;
	// x /= magnitude;
	// y /= magnitude;
	// z /= magnitude;
	// if ((x*y*t + z*s) > 0.998) { // north pole singularity detected
	// 	heading = 2*atan2(x*sin(angle/2),cos(angle/2));
	// 	attitude = M_PI/2;
	// 	bank = 0;
	// 	return;
	// }
	// if ((x*y*t + z*s) < -0.998) { // south pole singularity detected
	// 	heading = -2*atan2(x*sin(angle/2),cos(angle/2));
	// 	attitude = -M_PI/2;
	// 	bank = 0;
	// 	return;
	// }
	heading = atan2(y * s- x * z * t , 1 - (y*y+ z*z ) * t);
	attitude = asin(x * y * t + z * s) ;
	bank = atan2(x * s - y * z * t , 1 - (x*x + z*z) * t);
    return {heading, attitude, bank};
}

void updateModel() {
    const double *rot = AshkanRotation->getSFRotation();
    const double *tra = AshkanTranslation->getSFVec3f();
    // std::cout<<"tra: "<<tra[0]<<" "<<tra[1]<<" "<<tra[2]<<std::endl;
    // std::cout<<"rot: "<<rot[0]<<" "<<rot[1]<<" "<<rot[2]<<" "<<rot[3]<<std::endl;
    std::vector<double> out = toEuler(rot[0], rot[1], rot[2], rot[3]);
    // std::cout<<"out: "<<out[0]*180/M_PI<<" "<<out[1]*180/M_PI<<" "<<out[2]*180/M_PI<<std::endl;
    robotPose[0] = tra[0];
    robotPose[1] = -tra[2];
    robotPose[2] = mod_angle(out[0] - M_PI/2);
    // std::cout<<"angles: "<<out[0]*180/M_PI<<" "<<(out[0] - M_PI/2)*180/M_PI<<" "<<mod_angle(out[0] - M_PI/2)*180/M_PI<<std::endl;
    // std::cout<<"Pose: "<<robotPose[0]<<" "<<robotPose[1]<<" "<<robotPose[2]*180/M_PI<<std::endl;
}

void updateGame() {
    if (gamePhase == 0) {
        return;
    }
    std::vector<double> homeRelative = pose_relative(homePose, robotPose);
    double rhome = sqrt(pow(homeRelative[0], 2) + pow(homeRelative[1], 2));

    std::vector<double> v(3, 0);

    if (gamePhase == 1) { // Approach phase
        v[0] = maxStep * homeRelative[0] / rhome;
        v[1] = maxStep * homeRelative[1] / rhome;
        if ( abs(atan2(homeRelative[1], homeRelative[0])) > 20 * M_PI/180 ) {
            v[0] = 0;
            v[1] = 0;
        }
        v[2] = 0.2 * atan2(homeRelative[1], homeRelative[0]);
        if (rhome < thClose[0]) {
            gamePhase = 2;
        }
    } else if (gamePhase == 2) { // Turning phase, face center
        v[0] = maxStep * homeRelative[0] / rhome;
        v[1] = maxStep * homeRelative[1] / rhome;
        v[2] = 0.2 * homeRelative[2];
    }

    set_velocity(v);

    if ((gamePhase != 3) && (rhome < thClose[0]) && (abs(homeRelative[2]) < thClose[1])) {
        stopWalk();
        gamePhase = 3;
    }

    // To prevent robot keep walking after falling down
    if (gamePhase == 3) {
        stopWalk();
    } else {
        startWalk();
    }

    if ((gamePhase == 3) && (rhome > 0.3)) {
        gamePhase = 1;
    }
}

void updateWalk() {
    // advanceMotion();
    double t = getTime();
    if (!active) {
        // mcm.set_walk_isMoving(0); // not walking
        update_still();
        return;
    }

    if (!started) {
        started = true;
        tLastStep = getTime();
    }
    ph0 = ph;

    ph = (t - tLastStep) / tStep;
    // std::cout<<t<<" "<<tLastStep<<" "<<tStep<<std::endl;
    // std::cout<<ph<<"  "<<floor(ph)<<std::endl;
    // std::cout<<(ph > 1)<<std::endl;
    if (ph > 1) {
        iStep = iStep + 1;
        ph = ph - floor(ph);
        tLastStep = tLastStep + tStep;
    }
    // std::cout<<"phase: "<<ph<<std::endl;

    // Stop when stopping sequence is done
    if ((iStep > iStep0) && (stopRequest == 2)) {
        stopRequest = 0;
        active = false;
        // return 'stop';
    }

    if (iStep > iStep0) {
        update_velocity();
        iStep0 = iStep;
        supportLeg = iStep % 2; // 0 for left support, 1 for right support
        uLeft1 = uLeft2;
        uRight1 = uRight2;
        uTorso1 = uTorso2;

        supportMod = {0, 0}; // Support Point modulation for walkkick
        shiftFactor = 0.5; // How much should we shift final Torso pose?

        // check_walkkick();
        // check_stepkick();

        // if stepkick_ready then
        //     largestep.init_switch(uLeft, uRight, uTorso, comdot)
        //     return 'step'
        // end

        // if walkKickRequest == 0 and stepKickRequest == 0 then
        if (stopRequest == 1) {
            stopRequest = 2;
            velCurrent = {0, 0, 0};
            velCommand = {0, 0, 0};
            if (supportLeg == 0) { // Left support
                uRight2 = pose_global(MultiplyVectorByScalar(uLRFootOffset, -2), uLeft1);
            } else { // Right support
                uLeft2 = pose_global(MultiplyVectorByScalar(uLRFootOffset, 2), uRight1);
            }
        } else { // Normal walk, advance steps
            tStep = tStep0;
            if (supportLeg == 0) { // Left support
                uRight2 = step_right_destination(velCurrent, uLeft1, uRight1);
            } else { // Right support
                uLeft2 = step_left_destination(velCurrent, uLeft1, uRight1);
            }
            // Velocity-based support point modulation
            toeTipCompensation = 0;
            if (velDiff[0] > 0) { // Accelerating to front
                supportMod[0] = supportFront2;
            } else if (velCurrent[0] > velFastForward) {
                supportMod[0] = supportFront;
                toeTipCompensation = ankleMod[0];
            } else if (velCurrent[0] < 0) {
                supportMod[0] = supportBack;
            } else if (abs(velCurrent[2]) > velFastTurn) {
                supportMod[0] = supportTurn;
            } else {
                if (velCurrent[1] > 0.015) {
                    supportMod[0] = supportSideX;
                    supportMod[1] = supportSideY;
                } else if (velCurrent[1] < -0.015) {
                    supportMod[0] = supportSideX;
                    supportMod[1] = -supportSideY;
                }
            }
        }
        //     end
        // end

        uTorso2 = step_torso(uLeft2, uRight2, shiftFactor);

        // Adjustable initial step body swing
        if (initial_step > 0) {
            if (supportLeg == 0) {
                supportMod[1] = supportModYInitial;
            } else {
                supportMod[1] = -supportModYInitial;
            }
        }

        // Apply velocity-based support point modulation for uSupport
        if (supportLeg == 0) {
            std::vector<double> uLeftTorso = pose_relative(uLeft1, uTorso1);
            // std::cout<<"uLeft1: "<<uLeft1[0]<<" "<<uLeft1[1]<<" "<<uLeft1[2]<<std::endl;
            // std::cout<<"uTorso1: "<<uTorso1[0]<<" "<<uTorso1[1]<<" "<<uTorso1[2]<<std::endl;
            // std::cout<<"uLeftTorso: "<<uLeftTorso[0]<<" "<<uLeftTorso[1]<<" "<<uLeftTorso[2]<<std::endl;
            // std::cout<<"uTorso: "<<uTorso[0]<<" "<<uTorso[1]<<" "<<uTorso[2]<<std::endl;
            std::vector<double> supportModtmp = {supportMod[0], supportMod[1], 0};
            // std::cout<<"supportModtmp: "<<supportModtmp[0]<<" "<<supportModtmp[1]<<" "<<supportModtmp[2]<<std::endl;
            // std::cout<<"uTorso: "<<uTorso[0]<<" "<<uTorso[1]<<" "<<uTorso[2]<<std::endl;
            std::vector<double> uTorsoModded = pose_global(supportModtmp, uTorso);
            // std::cout<<"uTorsoModded: "<<uTorsoModded[0]<<" "<<uTorsoModded[1]<<" "<<uTorsoModded[2]<<std::endl;
            std::vector<double> uLeftModded = pose_global(uLeftTorso, uTorsoModded);
            // std::cout<<"uLeftModded: "<<uLeftModded[0]<<" "<<uLeftModded[1]<<" "<<uLeftModded[2]<<std::endl;
            uSupport = pose_global({supportX, supportY, 0}, uLeftModded);
            // Body.set_lleg_hardness(hardnessSupport);
            // Body.set_rleg_hardness(hardnessSwing);
        } else {
            std::vector<double> uRightTorso = pose_relative(uRight1, uTorso1);
            // std::cout<<"uRight1: "<<uRight1[0]<<" "<<uRight1[1]<<" "<<uRight1[2]<<std::endl;
            // std::cout<<"uTorso1: "<<uTorso1[0]<<" "<<uTorso1[1]<<" "<<uTorso1[2]<<std::endl;
            // std::cout<<"uRightTorso: "<<uRightTorso[0]<<" "<<uRightTorso[1]<<" "<<uRightTorso[2]<<std::endl;
            // std::cout<<"uTorso: "<<uTorso[0]<<" "<<uTorso[1]<<" "<<uTorso[2]<<std::endl;
            // std::cout<<"uRightTorso: "<<uRightTorso[0]<<" "<<uRightTorso[1]<<" "<<uRightTorso[2]<<std::endl;
            std::vector<double> uTorsoModded = pose_global({supportMod[0], supportMod[1], 0}, uTorso);
            // std::cout<<"uTorsoModded: "<<uTorsoModded[0]<<" "<<uTorsoModded[1]<<" "<<uTorsoModded[2]<<std::endl;
            std::vector<double> uRightModded = pose_global(uRightTorso, uTorsoModded);
            // std::cout<<"uRightModded: "<<uRightModded[0]<<" "<<uRightModded[1]<<" "<<uRightModded[2]<<std::endl;
            uSupport = pose_global({supportX, -supportY, 0}, uRightModded);
            // Body.set_lleg_hardness(hardnessSwing);
            // Body.set_rleg_hardness(hardnessSupport);
        }
        // std::cout<<"-------------------------------------------------------------------------------------------------------"<<std::endl;

        // Compute ZMP coefficients
        m1X = (uSupport[0] - uTorso1[0]) / (tStep * ph1Zmp);
        m2X = (uTorso2[0] - uSupport[0]) / (tStep * (1 - ph2Zmp));
        m1Y = (uSupport[1] - uTorso1[1]) / (tStep * ph1Zmp);
        m2Y = (uTorso2[1] - uSupport[1]) / (tStep * (1 - ph2Zmp));
        zmp_solve(uSupport[0], uTorso1[0], uTorso2[0], uTorso1[0], uTorso2[0], aXP, aXN);
        zmp_solve(uSupport[1], uTorso1[1], uTorso2[1], uTorso1[1], uTorso2[1], aYP, aYN);

        // Compute COM speed at the boundary

        double dx0 = (aXP - aXN) / tZmp + m1X * (1 - cosh(ph1Zmp * tStep / tZmp));
        double dy0 = (aYP - aYN) / tZmp + m1Y * (1 - cosh(ph1Zmp * tStep / tZmp));

        double dx1 =
            (aXP * exp(tStep / tZmp) - aXN * exp(-tStep / tZmp)) / tZmp +
            m2X * (1 - cosh((1 - ph2Zmp) * tStep / tZmp));
        double dy1 =
            (aYP * exp(tStep / tZmp) - aYN * exp(-tStep / tZmp)) / tZmp +
            m2Y * (1 - cosh((1 - ph2Zmp) * tStep / tZmp));

        // print("xdot0:",dx0,dy0);
        // print("xdot1:",dx1,dy1);
        std::vector<double> comdot = {dx1, dy1}; // Final COM velocity
    } // End new step

    double xFoot, zFoot;
    foot_phase(ph, xFoot, zFoot);
    if (initial_step > 0) {
        zFoot = 0;
    } // Don't lift foot at initial step

    pLLeg[2] = 0;
    pRLeg[2] = 0;
    // std::cout<<"xFoot:"<<xFoot<<std::endl;
    // std::cout<<"uRight1:"<<uRight1[0]<<" "<<uRight1[1]<<" "<<uRight1[2]<<std::endl;
    // std::cout<<"uRight2:"<<uRight2[0]<<" "<<uRight2[2]<<" "<<uRight2[2]<<std::endl;
    if (supportLeg == 0) { // Left support
        if (current_step_type > 1) { // walkkick
            if (xFoot < walkKickPh) {
                uRight = se2_interpolate(xFoot * 2, uRight1, uRight15);
            } else {
                uRight = se2_interpolate(xFoot * 2 - 1, uRight15, uRight2);
            }
        } else {
            uRight = se2_interpolate(xFoot, uRight1, uRight2);
        }
        pRLeg[2] = stepHeight * zFoot;
    } else { // Right support
        if (current_step_type > 1) { // walkkick
            if (xFoot < walkKickPh) {
                uLeft = se2_interpolate(xFoot * 2, uLeft1, uLeft15);
            } else {
                uLeft = se2_interpolate(xFoot * 2 - 1, uLeft15, uLeft2);
            }
        } else {
            uLeft = se2_interpolate(xFoot, uLeft1, uLeft2);
        }
        pLLeg[2] = stepHeight * zFoot;
    }
    // std::vector<double> uTorsoOld = uTorso;
    // std::cout<<"abcd: "<<pRLeg[2]<<" "<<pLLeg[2]<<std::endl;
    uTorso = zmp_com(ph);
    // std::cout<<"uTorso: "<<uTorso[0]<<" "<<uTorso[1]<<" "<<uTorso[2]<<std::endl;

    // Turning
    double turnCompX = 0;
    if (abs(velCurrent[2]) > turnCompThreshold and velCurrent[0] > -0.01) {
        turnCompX = turnComp;
    }

    // Walking front
    double frontCompX = 0;
    if (velCurrent[0] > 0.04) {
        frontCompX = frontComp;
    }
    if (velDiff[0] > 0.02) {
        frontCompX = frontCompX + AccelComp;
    }

    // Arm movement compensation
    double armPosCompX, armPosCompY;
    if (upper_body_overridden > 0 or motion_playing > 0) {
        // mass shift to X
        double elbowX = -sin(qLArmOR[1] - M_PI / 2 + bodyRot[1]) * cos(qLArmOR[2]) - sin(qRArmOR[1] - M_PI / 2 + bodyRot[1]) * cos(qRArmOR[2]);
        // mass shift to Y
        double elbowY = sin(qLArmOR[2]) + sin(qRArmOR[2]);
        armPosCompX = elbowX * -0.009;
        armPosCompY = elbowY * -0.009;

        pTorso[3] = bodyRot[0];
        pTorso[4] = bodyRot[1];
        pTorso[5] = bodyRot[2];
    } else {
        armPosCompX = 0;
        armPosCompY = 0;
        pTorso[3] = 0;
        pTorso[4] = bodyTilt;
        pTorso[5] = 0;
    }

    if (has_ball > 0) {
        turnCompX = turnCompX - 0.01;
    }
    // std::cout<<"333333333333333333333333333333333333333"<<std::endl;
    // std::cout<<"33333333333333333333333333333333333333333"<<std::endl;
    uTorsoActual = pose_global({-footX + frontCompX + turnCompX + armPosCompX, armPosCompY, 0.0}, uTorso);
    // std::cout<<"444444444444444444444444444444444444444444"<<std::endl;
    // std::cout<<"tmp: "<<tmp[0]<<" "<<tmp[1]<<" "<<tmp[2]<<std::endl;
    // std::cout<<"uTorso: "<<uTorso[0]<<" "<<uTorso[1]<<" "<<uTorso[2]<<std::endl;
    // std::cout<<"uTorsoActual: "<<uTorsoActual[0]<<" "<<uTorsoActual[1]<<" "<<uTorsoActual[2]<<std::endl;
    pTorso[0] = uTorsoActual[0];
    pTorso[1] = uTorsoActual[1];
    pTorso[5] = pTorso[5] + uTorsoActual[2];
    
    pLLeg[0] = uLeft[0];
    pLLeg[1] = uLeft[1];
    pLLeg[5] = uLeft[2];

    pRLeg[0] = uRight[0];
    pRLeg[1] = uRight[1];
    pRLeg[5] = uRight[2];

    // pLLeg = {0.0, 0.05, 0.0, 0.0, 0.0, 0.0};
    // pRLeg = {0.0, -0.05, 0.0, 0.0, 0.0, 0.0};
    // pTorso = {-0.035, 0.0, 0.41696, 0.0, 0.349066, 0.0};
    // std::cout<<"pLLeg: "<<pLLeg[0]<<" "<<pLLeg[1]<<" "<<pLLeg[2]<<" "<<pLLeg[3]<<" "<<pLLeg[4]<<" "<<pLLeg[5]<<" "<<std::endl;
    // std::cout<<"pRLeg: "<<pRLeg[0]<<" "<<pRLeg[1]<<" "<<pRLeg[2]<<" "<<pRLeg[3]<<" "<<pRLeg[4]<<" "<<pRLeg[5]<<" "<<std::endl;
    // std::cout<<"pTorso: "<<pTorso[0]<<" "<<pTorso[1]<<" "<<pTorso[2]<<" "<<pTorso[3]<<" "<<pTorso[4]<<" "<<pTorso[5]<<" "<<std::endl;
    // std::cout<<pLLeg[2]<<std::endl;
    // std::cout<<pRLeg[2]<<std::endl;
    // std::cout<<supportLeg<<"     "<<pTorso[1]<<std::endl;
    // std::cout<<"---------------------------------------------------------------------------------------"<<std::endl;
    std::vector<double> qLegs = inverse_legs(pLLeg, pRLeg, pTorso, supportLeg);
    // std::cout<<qLegs[0]<<" "<<qLegs[1]<<" "<<qLegs[2]<<" "<<qLegs[3]<<" "<<qLegs[4]<<" "<<qLegs[5]<<" "<<qLegs[6]<<" "<<qLegs[7]<<" "<<qLegs[8]<<" "<<qLegs[9]<<" "<<qLegs[10]<<" "<<qLegs[11]<<" "<<std::endl;
    // std::cout<<"-----------------------------------------------------------------------------------------------------------------------------------------"<<std::endl;
    motion_legs(qLegs);
    // // motion_arms();
    // // end motion_body

}

int main(int argc, char **argv) {
    
    
    /////////////////////////////////////////////-----Initialization-----/////////////////////////////////////////////
    LuaTable input (LuaTable::fromFile("Config_WebotsOP_Walk.lua"));
    stanceLimitX[0] = input["stanceLimitX"][1].getDefault<double>(false);
    stanceLimitX[1] = input["stanceLimitX"][2].getDefault<double>(false);

    stanceLimitY[0] = input["stanceLimitY"][1].getDefault<double>(false);
    stanceLimitY[1] = input["stanceLimitY"][2].getDefault<double>(false);
    
    stanceLimitA[0] = input["stanceLimitA"][1].getDefault<double>(false);
    stanceLimitA[1] = input["stanceLimitA"][2].getDefault<double>(false);
    
    velLimitX[0] = input["velLimitX"][1].getDefault<double>(false);
    velLimitX[1] = input["velLimitX"][2].getDefault<double>(false);
    
    velLimitY[0] = input["velLimitY"][1].getDefault<double>(false);
    velLimitY[1] = input["velLimitY"][2].getDefault<double>(false);
    
    velLimitA[0] = input["velLimitA"][1].getDefault<double>(false);
    velLimitA[1] = input["velLimitA"][2].getDefault<double>(false);

    velDelta[0] = input["velDelta"][1].getDefault<double>(false);
    velDelta[1] = input["velDelta"][2].getDefault<double>(false);
    velDelta[2] = input["velDelta"][3].getDefault<double>(false);

    vaFactor = input["vaFactor"].getDefault<double>(false);
    
    velXHigh = input["velXHigh"].getDefault<double>(false);
    velDeltaXHigh = input["velDeltaXHigh"].getDefault<double>(false);
    
    footSizeX[0] = input["footSizeX"][1].getDefault<double>(false);
    footSizeX[1] = input["footSizeX"][2].getDefault<double>(false);

    stanceLimitMarginY = input["stanceLimitMarginY"].getDefault<double>(false);
    stanceLimitY2 = 2 * input["footY"].getDefault<double>(false) - stanceLimitMarginY;
    
    bodyHeight = input["bodyHeight"].getDefault<double>(false);
    bodyTilt = input["bodyTilt"].getDefault<double>(false);
    footX = input["footX"].getDefault<double>(false);
    footY = input["footY"].getDefault<double>(false);
    supportX = input["supportX"].getDefault<double>(false);
    supportY = input["supportY"].getDefault<double>(false);
    
    qLArm0[0] = input["qLArm"][1].getDefault<double>(false);
    qLArm0[1] = input["qLArm"][2].getDefault<double>(false);
    qLArm0[2] = input["qLArm"][3].getDefault<double>(false);
    
    qRArm0[0] = input["qRArm"][1].getDefault<double>(false);
    qRArm0[1] = input["qRArm"][2].getDefault<double>(false);
    qRArm0[2] = input["qRArm"][3].getDefault<double>(false);

    hardnessSupport = input["hardnessSupport"].getDefault<double>(false);
    hardnessSwing = input["hardnessSwing"].getDefault<double>(false);

    tStep0 = input["tStep"].getDefault<double>(false);
    tStep = input["tStep"].getDefault<double>(false);
    tZmp = input["tZmp"].getDefault<double>(false);
    stepHeight0 = input["stepHeight0"].getDefault<double>(false);
    stepHeight = input["stepHeight"].getDefault<double>(false);

    ph1Single = input["phSingle"][1].getDefault<double>(false);
    ph2Single = input["phSingle"][2].getDefault<double>(false);
    
    hipRollCompensation = input["hipRollCompensation"].getDefault<double>(false);

    ankleMod[0] = input["ankleMod"][1].getDefault<double>(false);
    ankleMod[1] = input["ankleMod"][2].getDefault<double>(false);

    velFastForward = input["velFastForward"].getDefault<double>(false);
    velFastTurn = input["velFastTurn"].getDefault<double>(false);
    supportFront = input["supportFront"].getDefault<double>(false);
    supportFront2 = input["supportFront2"].getDefault<double>(false);
    supportBack = input["supportBack"].getDefault<double>(false);
    supportSideX = input["supportSideX"].getDefault<double>(false);
    supportSideY = input["supportSideY"].getDefault<double>(false);
    supportTurn = input["supportTurn"].getDefault<double>(false);
    frontComp = input["frontComp"].getDefault<double>(false);
    AccelComp = input["AccelComp"].getDefault<double>(false);
    supportModYInitial = input["supportModYInitial"].getDefault<double>(false);
    walkKickPh = input["walkKickPh"].getDefault<double>(false);
    turnCompThreshold = input["turnCompThreshold"].getDefault<double>(false);
    turnComp = input["turnComp"].getDefault<double>(false);
    
    ankleImuParamX[0] = input["ankleImuParamX"][1].getDefault<double>(false);
    ankleImuParamX[1] = input["ankleImuParamX"][2].getDefault<double>(false);
    ankleImuParamX[2] = input["ankleImuParamX"][3].getDefault<double>(false);
    ankleImuParamX[3] = input["ankleImuParamX"][4].getDefault<double>(false);
    
    ankleImuParamY[0] = input["ankleImuParamY"][1].getDefault<double>(false);
    ankleImuParamY[1] = input["ankleImuParamY"][2].getDefault<double>(false);
    ankleImuParamY[2] = input["ankleImuParamY"][3].getDefault<double>(false);
    ankleImuParamY[3] = input["ankleImuParamY"][4].getDefault<double>(false);
    
    kneeImuParamX[0] = input["kneeImuParamX"][1].getDefault<double>(false);
    kneeImuParamX[1] = input["kneeImuParamX"][2].getDefault<double>(false);
    kneeImuParamX[2] = input["kneeImuParamX"][3].getDefault<double>(false);
    kneeImuParamX[3] = input["kneeImuParamX"][4].getDefault<double>(false);
    
    hipImuParamY[0] = input["hipImuParamY"][1].getDefault<double>(false);
    hipImuParamY[1] = input["hipImuParamY"][2].getDefault<double>(false);
    hipImuParamY[2] = input["hipImuParamY"][3].getDefault<double>(false);
    hipImuParamY[3] = input["hipImuParamY"][4].getDefault<double>(false);
    
    armImuParamX[0] = input["armImuParamX"][1].getDefault<double>(false);
    armImuParamX[1] = input["armImuParamX"][2].getDefault<double>(false);
    armImuParamX[2] = input["armImuParamX"][3].getDefault<double>(false);
    armImuParamX[3] = input["armImuParamX"][4].getDefault<double>(false);
    
    armImuParamY[0] = input["armImuParamY"][1].getDefault<double>(false);
    armImuParamY[1] = input["armImuParamY"][2].getDefault<double>(false);
    armImuParamY[2] = input["armImuParamY"][3].getDefault<double>(false);
    armImuParamY[3] = input["armImuParamY"][4].getDefault<double>(false);

    homePose[0] = input["homePose"][1].getDefault<double>(false);
    homePose[1] = input["homePose"][2].getDefault<double>(false);
    homePose[2] = input["homePose"][3].getDefault<double>(false);


    uTorso = {supportX, 0, 0};
    uLeft = {0, footY, 0};
    uRight = {0, -footY, 0};

    pLLeg = {0, footY, 0, 0, 0, 0};
    pRLeg = {0, -footY, 0, 0, 0, 0};
    pTorso = {supportX, 0, bodyHeight, 0, bodyTilt, 0};

    velCurrent = {0, 0, 0};
    velCommand = {0, 0, 0};
    velDiff = {0, 0, 0};

    aXP = 0;
    aXN = 0;
    aYP = 0;
    aYN = 0;

    kneeShift = 0;

    active = false;
    started = false;

    iStep0 = -1;
    iStep = 0;
    t0 = getTime();

    ph1Zmp = ph1Single;
    ph2Zmp = ph2Single;

    toeTipCompensation = 0;

    ph0 = 0;
    ph = 0;

    stopRequest = 2;
    walkKickRequest = 0;
    current_step_type = 0;
    initial_step = 2;
    upper_body_overridden = 0;
    motion_playing = 0;

    qLArmOR0[0] = qLArm0[0];
    qLArmOR0[1] = qLArm0[1];
    qLArmOR0[2] = qLArm0[2];
    
    qRArmOR0[0] = qRArm0[0];
    qRArmOR0[1] = qRArm0[1];
    qRArmOR0[2] = qRArm0[2];

    qLArmOR[0] = qLArm0[0];
    qLArmOR[1] = qLArm0[1];
    qLArmOR[2] = qLArm0[2];

    qRArmOR[0] = qRArm0[0];
    qRArmOR[1] = qRArm0[1];
    qRArmOR[2] = qRArm0[2];

    bodyRot = {0, bodyTilt, 0};

    phSingle = 0;

    uLRFootOffset = {0, footY + supportY, 0};

    uLeftI = {0, 0, 0};
    uRightI = {0, 0, 0};
    uTorsoI = {0, 0, 0};
    supportI = 0;
    start_from_step = false;
    stepkick_ready = false;
    has_ball = 0;

    for (int i = 0; i < NMOTORS; i++) {
        moveDir[i] = 1;
    }

    jointReverse = {
        0, // Head: 1,2
        // LArm: 3,4,5
        6,7,8, // LLeg: 6,7,8,9,10,11,
        15, // RLeg: 12,13,14,15,16,17
        17,19 // RArm: 18,19,20
    };

    for (int i = 0; i < int(jointReverse.size()); i++) {
        moveDir[jointReverse[i]] = -1;
    }
    /////////////////////////////////////////////-----Initialization-----/////////////////////////////////////////////


    robot = new Supervisor();
    int timeStep = (int)robot->getBasicTimeStep();
    
    gyro = robot->getGyro("Gyro");
    gyro->enable(timeStep);
    keyboard = robot->getKeyboard();
    keyboard->enable(timeStep);

    for (int i = 0; i < NMOTORS; i++) {
        mMotors[i] = robot->getMotor(jointNames[i]);
        // std::cout<<mMotors[i]->getMinPosition()<<std::endl;
    }

    // entryModel();
    entryGame();
    entryWalk();
    
    Ashkan = robot->getFromDef("Ashkan");
    AshkanTranslation = Ashkan->getField("translation");
    AshkanRotation = Ashkan->getField("rotation");
    // const double tra[3] = {0, 1, 0};
    // const double rot[4] = {1, 0, 0, 0};
    while (robot->step(timeStep) != -1) {
        // Ashkan->resetPhysics();
        // robot->simulationResetPhysics();
        // AshkanTranslation->setSFVec3f(tra);
        // AshkanRotation->setSFRotation(rot);
        // Ashkan->resetPhysics();
        // robot->simulationResetPhysics();
        UpdateKeyboard();

        updateModel();
        updateGame();
        updateWalk();
    };

    delete robot;
    return 0;
}


