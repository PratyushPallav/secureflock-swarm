#include "secureflock_loop_functions.h"
#include "../../controllers/footbot_secureflock/footbot_secureflock.h"
#include <argos3/core/simulator/simulator.h>
#include <argos3/core/simulator/space/space.h>
#include <argos3/core/utility/logging/argos_log.h>
#include <argos3/core/utility/math/rng.h>
#include <argos3/core/utility/math/range.h>
#include <argos3/plugins/robots/foot-bot/simulator/footbot_entity.h>
#include <algorithm>
#include <vector>
#include <cmath>
#include <unistd.h>

CSecureFlockLoopFunctions::CSecureFlockLoopFunctions() :
   m_fInformedFraction(0.15), m_fMaliciousFraction(0.0), m_cFalse(-4.0, 0.0) {
   m_vecSurvivors.push_back(CVector2( 4.0,  0.0));
   m_vecSurvivors.push_back(CVector2( 0.0,  4.0));
   m_vecSurvivors.push_back(CVector2( 0.0, -4.0));
}

void CSecureFlockLoopFunctions::OpenLog() {
   if(m_cOut.is_open()) m_cOut.close();
   m_cOut.open("secureflock_log.csv", std::ios::out | std::ios::trunc);
   m_cOut << "tick";
   for(size_t k = 0; k < m_vecSurvivors.size(); ++k) m_cOut << ",cov_s" << k;
   m_cOut << ",cohesion,phantom,n_naive,naive_at_surv,naive_at_phantom,naive_est_err" << std::endl;
   char acCwd[1024];
   if(getcwd(acCwd, sizeof(acCwd)) != NULL)
      LOGERR << "[secureflock] CSV log: " << acCwd << "/secureflock_log.csv" << std::endl;
}

void CSecureFlockLoopFunctions::Init(TConfigurationNode& t_node) {
   TConfigurationNode& tP = GetNode(t_node, "secureflock");
   GetNodeAttributeOrDefault(tP, "informed_fraction",  m_fInformedFraction,  m_fInformedFraction);
   GetNodeAttributeOrDefault(tP, "malicious_fraction", m_fMaliciousFraction, m_fMaliciousFraction);
   Real fFx = m_cFalse.GetX(), fFy = m_cFalse.GetY();
   GetNodeAttributeOrDefault(tP, "false_x", fFx, fFx);
   GetNodeAttributeOrDefault(tP, "false_y", fFy, fFy);
   m_cFalse.Set(fFx, fFy);
   UInt32 unNumSurv = (UInt32)m_vecSurvivors.size();
   GetNodeAttributeOrDefault(tP, "num_survivors", unNumSurv, unNumSurv);
   if(unNumSurv >= 1 && unNumSurv < m_vecSurvivors.size()) m_vecSurvivors.resize(unNumSurv);
   Real fGroundFrac = -1.0;
   GetNodeAttributeOrDefault(tP, "ground_fraction", fGroundFrac, fGroundFrac);

   CSpace::TMapPerType& tFB = GetSpace().GetEntitiesByType("foot-bot");
   std::vector<CFootBotSecureFlock*> vecC;
   for(CSpace::TMapPerType::iterator it = tFB.begin(); it != tFB.end(); ++it) {
      CFootBotEntity* pcFB = any_cast<CFootBotEntity*>(it->second);
      vecC.push_back(&dynamic_cast<CFootBotSecureFlock&>(pcFB->GetControllableEntity().GetController()));
   }
   UInt32 unTotal = vecC.size();

   CRandom::CRNG* pcRNG = CRandom::CreateRNG("argos");
   for(UInt32 i = unTotal; i > 1; --i) {
      UInt32 j = pcRNG->Uniform(CRange<UInt32>(0, i));
      std::swap(vecC[i-1], vecC[j]);
   }

   UInt32 unInformed  = (UInt32)std::round(m_fInformedFraction  * unTotal);
   UInt32 unMalicious = (UInt32)std::round(m_fMaliciousFraction * unTotal);
   LOGERR << "[secureflock] " << unTotal << " robots, "
          << unInformed << " informed across " << m_vecSurvivors.size()
          << " survivor(s), " << unMalicious << " malicious" << std::endl;

   for(UInt32 i = 0; i < unTotal; ++i) {
      if(i < unInformed) {
         vecC[i]->SetRole(CFootBotSecureFlock::INFORMED);
         vecC[i]->SetTrueGoal(m_vecSurvivors[i % m_vecSurvivors.size()]);
      } else if(i < unInformed + unMalicious) {
         vecC[i]->SetRole(CFootBotSecureFlock::MALICIOUS);
         vecC[i]->SetFalseGoal(m_cFalse);
      } else {
         vecC[i]->SetRole(CFootBotSecureFlock::NAIVE);
      }
   }

   /* partial sensing: only a fraction of the naive robots can perceive the beacon;
      the rest are blind and must rely on shared radio reports */
   if(fGroundFrac >= 0.0) {
      std::vector<CFootBotSecureFlock*> vecNaiveC;
      for(UInt32 i = 0; i < unTotal; ++i)
         if(vecC[i]->GetRole() == CFootBotSecureFlock::NAIVE) vecNaiveC.push_back(vecC[i]);
      UInt32 nG = (UInt32)std::round(fGroundFrac * vecNaiveC.size());
      for(size_t k = 0; k < vecNaiveC.size(); ++k) vecNaiveC[k]->SetGround(k < nG);
      LOGERR << "[secureflock] ground_fraction=" << fGroundFrac << " -> " << nG
             << "/" << vecNaiveC.size() << " naive can sense the beacon" << std::endl;
   }
   OpenLog();
}

void CSecureFlockLoopFunctions::Reset() { OpenLog(); }

CColor CSecureFlockLoopFunctions::GetFloorColor(const CVector2& c_position_on_plane) {
   for(size_t k = 0; k < m_vecSurvivors.size(); ++k)
      if((c_position_on_plane - m_vecSurvivors[k]).Length() < 0.6) return CColor::GREEN;
   if((c_position_on_plane - m_cFalse).Length() < 0.6) return CColor::RED;
   return CColor::WHITE;
}

void CSecureFlockLoopFunctions::PostStep() {
   CSpace::TMapPerType& tFB = GetSpace().GetEntitiesByType("foot-bot");
   std::vector<CVector2> vecHonest, vecNaive;
   Real fEstErrSum = 0.0;
   CVector2 cAll; UInt32 nAll = 0;
   for(CSpace::TMapPerType::iterator it = tFB.begin(); it != tFB.end(); ++it) {
      CFootBotEntity* pcFB = any_cast<CFootBotEntity*>(it->second);
      const CVector3& p = pcFB->GetEmbodiedEntity().GetOriginAnchor().Position;
      CVector2 cPos(p.GetX(), p.GetY());
      cAll += cPos; ++nAll;
      CFootBotSecureFlock& c =
         dynamic_cast<CFootBotSecureFlock&>(pcFB->GetControllableEntity().GetController());
      CFootBotSecureFlock::ERole eRole = c.GetRole();
      if(eRole != CFootBotSecureFlock::MALICIOUS) vecHonest.push_back(cPos);
      if(eRole == CFootBotSecureFlock::NAIVE) {
         vecNaive.push_back(cPos);
         CVector2 cEst = c.GetGoalEstimate();
         Real fE = 1e9;
         for(size_t k = 0; k < m_vecSurvivors.size(); ++k) {
            Real d = (cEst - m_vecSurvivors[k]).Length();
            if(d < fE) fE = d;
         }
         fEstErrSum += fE;
      }
   }
   if(nAll == 0) return;
   cAll /= (Real)nAll;
   Real fCoh = 0.0;
   for(CSpace::TMapPerType::iterator it = tFB.begin(); it != tFB.end(); ++it) {
      CFootBotEntity* pcFB = any_cast<CFootBotEntity*>(it->second);
      const CVector3& p = pcFB->GetEmbodiedEntity().GetOriginAnchor().Position;
      fCoh += (CVector2(p.GetX(), p.GetY()) - cAll).Length();
   }
   fCoh /= (Real)nAll;

   Real fPh = 1e9;
   for(size_t i = 0; i < vecHonest.size(); ++i) {
      Real d = (vecHonest[i] - m_cFalse).Length();
      if(d < fPh) fPh = d;
   }

   const Real R = 1.0;
   UInt32 nNaive = vecNaive.size(), nAtSurv = 0, nAtPh = 0;
   for(size_t i = 0; i < vecNaive.size(); ++i) {
      bool bAtSurv = false;
      for(size_t k = 0; k < m_vecSurvivors.size(); ++k)
         if((vecNaive[i] - m_vecSurvivors[k]).Length() < R) { bAtSurv = true; break; }
      if(bAtSurv) ++nAtSurv;
      if((vecNaive[i] - m_cFalse).Length() < R) ++nAtPh;
   }
   Real fEstErr = (nNaive > 0) ? fEstErrSum / (Real)nNaive : 0.0;

   m_cOut << GetSpace().GetSimulationClock();
   for(size_t k = 0; k < m_vecSurvivors.size(); ++k) {
      Real fMin = 1e9;
      for(size_t i = 0; i < vecHonest.size(); ++i) {
         Real d = (vecHonest[i] - m_vecSurvivors[k]).Length();
         if(d < fMin) fMin = d;
      }
      m_cOut << "," << fMin;
   }
   m_cOut << "," << fCoh << "," << fPh << ","
          << nNaive << "," << nAtSurv << "," << nAtPh << "," << fEstErr << std::endl;
}

void CSecureFlockLoopFunctions::Destroy() { if(m_cOut.is_open()) m_cOut.close(); }

REGISTER_LOOP_FUNCTIONS(CSecureFlockLoopFunctions, "secureflock_loop_functions")
