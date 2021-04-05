#include "luaOPKinematics.h"
#include "OPKinematics.h"

std::vector<double> inverse_legs(std::vector<double> pLLeg, std::vector<double> pRLeg, std::vector<double> pTorso, int supportLeg) {
  std::vector<double> qLegs;
  int leg = 0;
  qLegs = darwinop_kinematics_inverse_legs(&pLLeg[0], 
				      &pRLeg[0],
				      &pTorso[0], leg);
  return qLegs;
}