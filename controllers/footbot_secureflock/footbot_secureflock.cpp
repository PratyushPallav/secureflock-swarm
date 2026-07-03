#include "footbot_secureflock.h"
#include <argos3/core/utility/configuration/argos_configuration.h>
#include <argos3/core/utility/math/vector3.h>
#include <argos3/core/utility/datatypes/byte_array.h>
#include <algorithm>
#include <vector>
#include <cmath>
#include <cstdlib>
#include <string>

void CFootBotSecureFlock::SWheelTurningParams::Init(TConfigurationNode& t_node) {
   try {
      TurningMechanism = NO_TURN;
      CDegrees cAngle;
      GetNodeAttribute(t_node, "hard_turn_angle_threshold", cAngle); HardTurnOnAngleThreshold = ToRadians(cAngle);
      GetNodeAttribute(t_node, "soft_turn_angle_threshold", cAngle); SoftTurnOnAngleThreshold = ToRadians(cAngle);
      GetNodeAttribute(t_node, "no_turn_angle_threshold",   cAngle); NoTurnAngleThreshold     = ToRadians(cAngle);
      GetNodeAttribute(t_node, "max_speed", MaxSpeed);
   } catch(CARGoSException& ex) { THROW_ARGOSEXCEPTION_NESTED("wheel_turning params", ex); }
}
void CFootBotSecureFlock::SFlockingInteractionParams::Init(TConfigurationNode& t_node) {
   try {
      GetNodeAttribute(t_node, "target_distance", TargetDistance);
      GetNodeAttribute(t_node, "gain", Gain);
      GetNodeAttribute(t_node, "exponent", Exponent);
   } catch(CARGoSException& ex) { THROW_ARGOSEXCEPTION_NESTED("flocking params", ex); }
}
Real CFootBotSecureFlock::SFlockingInteractionParams::GeneralizedLennardJones(Real f_distance) {
   Real fNormDistExp = ::pow(TargetDistance / f_distance, Exponent);
   return -Gain / f_distance * (fNormDistExp * fNormDistExp - fNormDistExp);
}

static Real WMSR1D(std::vector<Real> vals, Real own, UInt32 f) {
   std::sort(vals.begin(), vals.end());
   int n = (int)vals.size();
   std::vector<bool> rm(n, false);
   int c = 0;
   for(int i = n - 1; i >= 0 && c < (int)f; --i) if(vals[i] > own) { rm[i] = true; ++c; }
   c = 0;
   for(int i = 0; i < n && c < (int)f; ++i)      if(vals[i] < own) { rm[i] = true; ++c; }
   Real sum = own; int cnt = 1;
   for(int i = 0; i < n; ++i) if(!rm[i]) { sum += vals[i]; ++cnt; }
   return sum / (Real)cnt;
}

CFootBotSecureFlock::CFootBotSecureFlock() :
   m_pcWheels(NULL), m_pcLEDs(NULL), m_pcCamera(NULL),
   m_pcRABA(NULL), m_pcRABS(NULL), m_pcPos(NULL), m_pcLight(NULL), m_pcRNG(NULL),
   m_eRole(NAIVE), m_unId(0), m_bHasEstimate(false),
   m_bDefense(true), m_fSigma(3.0), m_fFloor(0.3), m_fAlpha(0.1),
   m_bGround(false), m_fLightDetect(0.01), m_fAgree(0.0), m_fLastLight(0.0),
   m_fCohesion(1.0), m_strMethod("trust"), m_unF(1), m_strAttacker("static"),
   m_fPacketLoss(0.0), m_fPosNoise(0.0) {}

void CFootBotSecureFlock::Init(TConfigurationNode& t_node) {
   m_pcWheels = GetActuator<CCI_DifferentialSteeringActuator>("differential_steering");
   m_pcLEDs   = GetActuator<CCI_LEDsActuator>("leds");
   m_pcRABA   = GetActuator<CCI_RangeAndBearingActuator>("range_and_bearing");
   m_pcCamera = GetSensor<CCI_ColoredBlobOmnidirectionalCameraSensor>("colored_blob_omnidirectional_camera");
   m_pcRABS   = GetSensor<CCI_RangeAndBearingSensor>("range_and_bearing");
   m_pcPos    = GetSensor<CCI_PositioningSensor>("positioning");
   m_pcLight  = GetSensor<CCI_FootBotLightSensor>("footbot_light");
   m_pcRNG    = CRandom::CreateRNG("argos");
   try {
      m_sWheelTurningParams.Init(GetNode(t_node, "wheel_turning"));
      m_sFlockingParams.Init(GetNode(t_node, "flocking"));
   } catch(CARGoSException& ex) { THROW_ARGOSEXCEPTION_NESTED("controller params", ex); }
   try {
      TConfigurationNode& tT = GetNode(t_node, "trust");
      GetNodeAttributeOrDefault(tT, "defense",      m_bDefense,     m_bDefense);
      GetNodeAttributeOrDefault(tT, "tolerance",    m_fSigma,       m_fSigma);
      GetNodeAttributeOrDefault(tT, "floor",        m_fFloor,       m_fFloor);
      GetNodeAttributeOrDefault(tT, "alpha",        m_fAlpha,       m_fAlpha);
      GetNodeAttributeOrDefault(tT, "ground",       m_bGround,      m_bGround);
      GetNodeAttributeOrDefault(tT, "light_detect", m_fLightDetect, m_fLightDetect);
      GetNodeAttributeOrDefault(tT, "agree",        m_fAgree,       m_fAgree);
      GetNodeAttributeOrDefault(tT, "cohesion",     m_fCohesion,    m_fCohesion);
      GetNodeAttributeOrDefault(tT, "method",       m_strMethod,    m_strMethod);
      GetNodeAttributeOrDefault(tT, "f",            m_unF,          m_unF);
      GetNodeAttributeOrDefault(tT, "attacker",     m_strAttacker,  m_strAttacker);
      GetNodeAttributeOrDefault(tT, "packet_loss",  m_fPacketLoss,  m_fPacketLoss);
      GetNodeAttributeOrDefault(tT, "pos_noise",    m_fPosNoise,    m_fPosNoise);
   } catch(CARGoSException&) { }
   const std::string& strId = GetId();
   size_t p = strId.find_first_of("0123456789");
   m_unId = (p == std::string::npos) ? 0 : (UInt32)atoi(strId.substr(p).c_str());
   Reset();
}

void CFootBotSecureFlock::Reset() {
   m_pcCamera->Enable();
   m_pcLEDs->SetSingleColor(12, CColor::RED);
   m_bHasEstimate = false;
   m_cGoalEstimate = CVector2();
   m_mapTrust.clear();
   m_fLastLight = 0.0;
}

void CFootBotSecureFlock::ControlStep() {
   CVector2 cLightVec; Real fMaxL = 0.0; CRadians cPeakAngle;
   {
      const CCI_FootBotLightSensor::TReadings& tL = m_pcLight->GetReadings();
      for(size_t i = 0; i < tL.size(); ++i) {
         cLightVec += CVector2(tL[i].Value, tL[i].Angle);
         if(tL[i].Value > fMaxL) { fMaxL = tL[i].Value; cPeakAngle = tL[i].Angle; }
      }
   }
   m_fLastLight = fMaxL;

   if(m_eRole == INFORMED) {
      m_cGoalEstimate = m_cTrueGoal; m_bHasEstimate = true;
   } else if(m_eRole == MALICIOUS) {
      m_cGoalEstimate = m_cFalseGoal; m_bHasEstimate = true;
   } else { /* NAIVE */
      const CCI_RangeAndBearingSensor::TReadings& tMsgs = m_pcRABS->GetReadings();
      std::vector<UInt32> vecId;
      std::vector<CVector2> vecEst;
      for(size_t i = 0; i < tMsgs.size(); ++i) {
         if(m_fPacketLoss > 0.0 && m_pcRNG->Bernoulli(m_fPacketLoss)) continue;
         CByteArray cData = tMsgs[i].Data;
         UInt8 unSender, unFlag; Real fX, fY;
         cData >> unSender; cData >> unFlag; cData >> fX; cData >> fY;
         if(unFlag == 1) { vecId.push_back((UInt32)unSender); vecEst.push_back(CVector2(fX, fY)); }
      }

      const CCI_PositioningSensor::SReading& sP = m_pcPos->GetReading();
      CVector2 cMy(sP.Position.GetX(), sP.Position.GetY());
      if(m_fPosNoise > 0.0) cMy += CVector2(m_pcRNG->Gaussian(m_fPosNoise), m_pcRNG->Gaussian(m_fPosNoise));
      CRadians cYaw, cT1, cT2; sP.Orientation.ToEulerAngles(cYaw, cT1, cT2);

      bool bBeacon = false; CRadians cBeaconG;
      if(m_bGround && cLightVec.Length() > m_fLightDetect) {
         cBeaconG = (cYaw + cPeakAngle).SignedNormalize();
         bBeacon = true;
      }

      std::vector<UInt32> fId; std::vector<CVector2> fEst;
      for(size_t i = 0; i < vecEst.size(); ++i) {
         bool bKeep = true;
         if(bBeacon) {
            CRadians cToEst = (vecEst[i] - cMy).Angle();
            if(Cos(cToEst - cBeaconG) < m_fAgree) bKeep = false;
         }
         if(bKeep) { fId.push_back(vecId[i]); fEst.push_back(vecEst[i]); }
      }

      if(!fEst.empty()) {
         if(!m_bDefense) {
            CVector2 cSum;
            for(size_t i = 0; i < fEst.size(); ++i) cSum += fEst[i];
            m_cGoalEstimate = cSum / (Real)fEst.size(); m_bHasEstimate = true;
         } else if(m_strMethod == "wmsr") {
            if(!m_bHasEstimate) {
               CVector2 cS; for(size_t i = 0; i < fEst.size(); ++i) cS += fEst[i];
               m_cGoalEstimate = cS / (Real)fEst.size(); m_bHasEstimate = true;
            }
            std::vector<Real> vxs, vys;
            for(size_t i = 0; i < fEst.size(); ++i) { vxs.push_back(fEst[i].GetX()); vys.push_back(fEst[i].GetY()); }
            Real nx = WMSR1D(vxs, m_cGoalEstimate.GetX(), m_unF);
            Real ny = WMSR1D(vys, m_cGoalEstimate.GetY(), m_unF);
            m_cGoalEstimate.Set(nx, ny);
         } else {
            std::vector<Real> vx, vy;
            for(size_t i = 0; i < fEst.size(); ++i) { vx.push_back(fEst[i].GetX()); vy.push_back(fEst[i].GetY()); }
            std::sort(vx.begin(), vx.end()); std::sort(vy.begin(), vy.end());
            size_t m = vx.size() / 2;
            Real medX = (vx.size() % 2) ? vx[m] : 0.5 * (vx[m-1] + vx[m]);
            Real medY = (vy.size() % 2) ? vy[m] : 0.5 * (vy[m-1] + vy[m]);
            CVector2 cMedian(medX, medY);
            CVector2 cWeighted; Real fWsum = 0.0;
            for(size_t i = 0; i < fEst.size(); ++i) {
               Real fDev   = (fEst[i] - cMedian).Length();
               Real fScore = ::exp(-(fDev * fDev) / (2.0 * m_fSigma * m_fSigma));
               std::map<UInt32, Real>::iterator it = m_mapTrust.find(fId[i]);
               Real fPrev  = (it == m_mapTrust.end()) ? 0.5 : it->second;
               Real fTrust = (1.0 - m_fAlpha) * fPrev + m_fAlpha * fScore;
               m_mapTrust[fId[i]] = fTrust;
               if(fTrust >= m_fFloor) { cWeighted += fEst[i] * fTrust; fWsum += fTrust; }
            }
            if(fWsum > 0.0) { m_cGoalEstimate = cWeighted / fWsum; m_bHasEstimate = true; }
         }
      }
   }

   CByteArray cBuf;
   cBuf << (UInt8)m_unId;
   cBuf << (UInt8)(m_bHasEstimate ? 1 : 0);
   cBuf << m_cGoalEstimate.GetX();
   cBuf << m_cGoalEstimate.GetY();
   while(cBuf.Size() < 50) cBuf << (UInt8)0;
   m_pcRABA->SetData(cBuf);

   CVector2 cGoal;
   if(m_eRole == MALICIOUS && m_strAttacker == "follow") cGoal = CVector2();
   else cGoal = m_bHasEstimate ? GoalVector() : CVector2();
   SetWheelSpeedsFromVector(cGoal + m_fCohesion * FlockingVector());
}

CVector2 CFootBotSecureFlock::GoalVector() {
   const CCI_PositioningSensor::SReading& sPos = m_pcPos->GetReading();
   CVector2 cPos(sPos.Position.GetX(), sPos.Position.GetY());
   if(m_fPosNoise > 0.0) cPos += CVector2(m_pcRNG->Gaussian(m_fPosNoise), m_pcRNG->Gaussian(m_fPosNoise));
   CRadians cZ, cY, cX; sPos.Orientation.ToEulerAngles(cZ, cY, cX);
   CVector2 cToGoal = m_cGoalEstimate - cPos;
   cToGoal.Rotate(-cZ);
   if(cToGoal.Length() > 0.0f) { cToGoal.Normalize(); cToGoal *= 1.0f * m_sWheelTurningParams.MaxSpeed; }
   return cToGoal;
}

CVector2 CFootBotSecureFlock::FlockingVector() {
   const CCI_ColoredBlobOmnidirectionalCameraSensor::SReadings& s = m_pcCamera->GetReadings();
   if(s.BlobList.empty()) return CVector2();
   CVector2 cAccum; size_t unSeen = 0;
   for(size_t i = 0; i < s.BlobList.size(); ++i) {
      if(s.BlobList[i]->Color == CColor::RED &&
         s.BlobList[i]->Distance < m_sFlockingParams.TargetDistance * 1.80f) {
         cAccum += CVector2(m_sFlockingParams.GeneralizedLennardJones(s.BlobList[i]->Distance),
                            s.BlobList[i]->Angle);
         ++unSeen;
      }
   }
   if(unSeen == 0) return CVector2();
   cAccum /= unSeen;
   if(cAccum.Length() > m_sWheelTurningParams.MaxSpeed) { cAccum.Normalize(); cAccum *= m_sWheelTurningParams.MaxSpeed; }
   return cAccum;
}

void CFootBotSecureFlock::SetWheelSpeedsFromVector(const CVector2& c_heading) {
   CRadians cHeadingAngle = c_heading.Angle().SignedNormalize();
   Real fHeadingLength = c_heading.Length();
   Real fBaseAngularWheelSpeed = Min<Real>(fHeadingLength, m_sWheelTurningParams.MaxSpeed);
   auto& P = m_sWheelTurningParams;
   if(P.TurningMechanism == SWheelTurningParams::HARD_TURN) {
      if(Abs(cHeadingAngle) <= P.SoftTurnOnAngleThreshold) P.TurningMechanism = SWheelTurningParams::SOFT_TURN;
   }
   if(P.TurningMechanism == SWheelTurningParams::SOFT_TURN) {
      if(Abs(cHeadingAngle) > P.HardTurnOnAngleThreshold) P.TurningMechanism = SWheelTurningParams::HARD_TURN;
      else if(Abs(cHeadingAngle) <= P.NoTurnAngleThreshold) P.TurningMechanism = SWheelTurningParams::NO_TURN;
   }
   if(P.TurningMechanism == SWheelTurningParams::NO_TURN) {
      if(Abs(cHeadingAngle) > P.HardTurnOnAngleThreshold) P.TurningMechanism = SWheelTurningParams::HARD_TURN;
      else if(Abs(cHeadingAngle) > P.NoTurnAngleThreshold) P.TurningMechanism = SWheelTurningParams::SOFT_TURN;
   }
   Real fSpeed1, fSpeed2;
   switch(P.TurningMechanism) {
      case SWheelTurningParams::NO_TURN:   fSpeed1 = fBaseAngularWheelSpeed; fSpeed2 = fBaseAngularWheelSpeed; break;
      case SWheelTurningParams::SOFT_TURN: {
         Real f = (P.HardTurnOnAngleThreshold - Abs(cHeadingAngle)) / P.HardTurnOnAngleThreshold;
         fSpeed1 = fBaseAngularWheelSpeed - fBaseAngularWheelSpeed * (1.0 - f);
         fSpeed2 = fBaseAngularWheelSpeed + fBaseAngularWheelSpeed * (1.0 - f); break; }
      case SWheelTurningParams::HARD_TURN: fSpeed1 = -P.MaxSpeed; fSpeed2 = P.MaxSpeed; break;
   }
   Real fLeft, fRight;
   if(cHeadingAngle > CRadians::ZERO) { fLeft = fSpeed1; fRight = fSpeed2; }
   else                               { fLeft = fSpeed2; fRight = fSpeed1; }
   m_pcWheels->SetLinearVelocity(fLeft, fRight);
}

REGISTER_CONTROLLER(CFootBotSecureFlock, "footbot_secureflock_controller")
