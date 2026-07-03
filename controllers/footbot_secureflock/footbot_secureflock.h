#ifndef FOOTBOT_SECUREFLOCK_H
#define FOOTBOT_SECUREFLOCK_H

#include <argos3/core/control_interface/ci_controller.h>
#include <argos3/plugins/robots/generic/control_interface/ci_differential_steering_actuator.h>
#include <argos3/plugins/robots/generic/control_interface/ci_leds_actuator.h>
#include <argos3/plugins/robots/generic/control_interface/ci_colored_blob_omnidirectional_camera_sensor.h>
#include <argos3/plugins/robots/generic/control_interface/ci_range_and_bearing_actuator.h>
#include <argos3/plugins/robots/generic/control_interface/ci_range_and_bearing_sensor.h>
#include <argos3/plugins/robots/generic/control_interface/ci_positioning_sensor.h>
#include <argos3/plugins/robots/foot-bot/control_interface/ci_footbot_light_sensor.h>
#include <argos3/core/utility/math/vector2.h>
#include <argos3/core/utility/math/rng.h>
#include <map>
#include <string>

using namespace argos;

class CFootBotSecureFlock : public CCI_Controller {
public:
   enum ERole { INFORMED, NAIVE, MALICIOUS };

   struct SWheelTurningParams {
      enum ETurningMechanism { NO_TURN = 0, SOFT_TURN, HARD_TURN } TurningMechanism;
      CRadians HardTurnOnAngleThreshold, SoftTurnOnAngleThreshold, NoTurnAngleThreshold;
      Real MaxSpeed;
      void Init(TConfigurationNode& t_node);
   };
   struct SFlockingInteractionParams {
      Real TargetDistance, Gain, Exponent;
      Real GeneralizedLennardJones(Real f_distance);
      void Init(TConfigurationNode& t_node);
   };

   CFootBotSecureFlock();
   virtual ~CFootBotSecureFlock() {}
   virtual void Init(TConfigurationNode& t_node);
   virtual void ControlStep();
   virtual void Reset();
   virtual void Destroy() {}

   void SetRole(ERole e_role) { m_eRole = e_role; }
   void SetTrueGoal(const CVector2& c_goal) { m_cTrueGoal = c_goal; }
   void SetFalseGoal(const CVector2& c_goal) { m_cFalseGoal = c_goal; }
   void SetGround(bool b) { m_bGround = b; }
   ERole GetRole() const { return m_eRole; }
   UInt32 GetNumericId() const { return m_unId; }
   const CVector2& GetGoalEstimate() const { return m_cGoalEstimate; }
   Real GetMaxLight() const { return m_fLastLight; }

private:
   CVector2 FlockingVector();
   CVector2 GoalVector();
   void SetWheelSpeedsFromVector(const CVector2& c_heading);

   CCI_DifferentialSteeringActuator* m_pcWheels;
   CCI_LEDsActuator* m_pcLEDs;
   CCI_ColoredBlobOmnidirectionalCameraSensor* m_pcCamera;
   CCI_RangeAndBearingActuator* m_pcRABA;
   CCI_RangeAndBearingSensor* m_pcRABS;
   CCI_PositioningSensor* m_pcPos;
   CCI_FootBotLightSensor* m_pcLight;
   CRandom::CRNG* m_pcRNG;

   SWheelTurningParams m_sWheelTurningParams;
   SFlockingInteractionParams m_sFlockingParams;

   ERole m_eRole;
   UInt32 m_unId;
   CVector2 m_cTrueGoal;
   CVector2 m_cFalseGoal;
   CVector2 m_cGoalEstimate;
   bool m_bHasEstimate;
   std::map<UInt32, Real> m_mapTrust;

   bool m_bDefense;
   Real m_fSigma, m_fFloor, m_fAlpha;

   bool m_bGround;
   Real m_fLightDetect, m_fAgree;
   Real m_fLastLight;
   Real m_fCohesion;

   std::string m_strMethod;
   UInt32 m_unF;
   std::string m_strAttacker;

   Real m_fPacketLoss;   // prob. of dropping each received broadcast
   Real m_fPosNoise;     // std dev (m) of Gaussian localisation noise
};
#endif
