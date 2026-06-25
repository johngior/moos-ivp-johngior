/************************************************************/
/*    NAME: Ioannis Giordamlis                              */
/*    ORGN: MIT                                             */
/*    FILE: BHV_Pulse.h                                     */
/*    DATE:                                                 */
/************************************************************/

#ifndef BHV_PULSE_HEADER
#define BHV_PULSE_HEADER

#include <string>
#include "IvPBehavior.h"

class BHV_Pulse : public IvPBehavior {
public:
  BHV_Pulse(IvPDomain);
  ~BHV_Pulse() {};

  bool          setParam(std::string, std::string);
  void          onSetParamComplete();
  void          onCompleteState();
  void          onIdleState();
  void          onHelmStart();
  void          postConfigStatus();
  void          onRunToIdleState();
  void          onIdleToRunState();
  IvPFunction* onRunState();

protected: // Local utility functions
  bool          updateInfoIn();
  void          postPulse();

protected: // Configuration parameters
  double        m_range;
  double        m_pulse_duration;
  double        m_delay;

protected: // State variables
  double        m_osx;
  double        m_osy;
  double        m_curr_time;

  int           m_wpt_index;
  bool          m_wpt_index_initialized;

  double        m_wpt_index_change_time;
  bool          m_pulse_pending;
};

#ifdef WIN32
  #define IVP_EXPORT_FUNCTION __declspec(dllexport)
#else
  #define IVP_EXPORT_FUNCTION
#endif

extern "C" {
  IVP_EXPORT_FUNCTION IvPBehavior* createBehavior(std::string name,
                                                  IvPDomain domain)
  {
    return new BHV_Pulse(domain);
  }
}

#endif