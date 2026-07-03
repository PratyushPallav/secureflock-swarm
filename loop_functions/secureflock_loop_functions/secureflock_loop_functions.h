#ifndef SECUREFLOCK_LOOP_FUNCTIONS_H
#define SECUREFLOCK_LOOP_FUNCTIONS_H
#include <argos3/core/simulator/loop_functions.h>
#include <argos3/core/utility/math/vector2.h>
#include <argos3/core/utility/datatypes/color.h>
#include <fstream>
#include <vector>
using namespace argos;

class CSecureFlockLoopFunctions : public CLoopFunctions {
public:
   CSecureFlockLoopFunctions();
   virtual ~CSecureFlockLoopFunctions() {}
   virtual void Init(TConfigurationNode& t_node);
   virtual void Reset();
   virtual void PostStep();
   virtual void Destroy();
   virtual CColor GetFloorColor(const CVector2& c_position_on_plane);
private:
   void OpenLog();
   Real m_fInformedFraction;
   Real m_fMaliciousFraction;
   std::vector<CVector2> m_vecSurvivors;
   CVector2 m_cFalse;
   std::ofstream m_cOut;
};
#endif
