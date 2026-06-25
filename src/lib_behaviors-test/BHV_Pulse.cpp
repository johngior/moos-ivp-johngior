/************************************************************/
/*    NAME: Ioannis Giordamlis                              */
/*    ORGN: MIT                                             */
/*    FILE: BHV_Pulse.cpp                                   */
/*    DATE:                                                 */
/************************************************************/

#include "BHV_Pulse.h"
#include "MBUtils.h"
#include "BuildUtils.h"
#include "XYRangePulse.h"

using namespace std;

//---------------------------------------------------------------
// Constructor

BHV_Pulse::BHV_Pulse(IvPDomain domain) :
  IvPBehavior(domain)
{
  IvPBehavior::setParam("name", "pulse");

  m_domain = subDomain(m_domain, "course,speed");

  // Default pulse settings.
  // In section 4.5 we will allow the user to set these in alpha.bhv.
  m_range          = 20;
  m_pulse_duration = 4;
  m_delay          = 5;

  // State variables
  m_osx = 0;
  m_osy = 0;
  m_curr_time = 0;

  m_wpt_index = -1;
  m_wpt_index_initialized = false;

  m_wpt_index_change_time = -1;
  m_pulse_pending = false;

  // Variables needed from the InfoBuffer
  addInfoVars("NAV_X, NAV_Y, WPT_INDEX");
}

//---------------------------------------------------------------
// Procedure: setParam()

bool BHV_Pulse::setParam(string param, string val)
{
  param = tolower(param);

  // Section 4.4 does not require user parameters yet.
  // pulse_range and pulse_duration will be handled in 4.5.

  return(false);
}

//---------------------------------------------------------------
// Procedure: updateInfoIn()
// Purpose: Read NAV_X, NAV_Y, WPT_INDEX, and current time.

bool BHV_Pulse::updateInfoIn()
{
  bool ok_x;
  bool ok_y;
  bool ok_wpt;

  m_osx = getBufferDoubleVal("NAV_X", ok_x);
  m_osy = getBufferDoubleVal("NAV_Y", ok_y);

  double wpt_index_double = getBufferDoubleVal("WPT_INDEX", ok_wpt);

  m_curr_time = getBufferCurrTime();

  if(!ok_x || !ok_y) {
    postWMessage("Missing NAV_X or NAV_Y in InfoBuffer.");
    return(false);
  }

  if(!ok_wpt) {
    postWMessage("Missing WPT_INDEX in InfoBuffer.");
    return(false);
  }

  int new_wpt_index = (int)(wpt_index_double);

  // First reading: initialize but do not create a pulse yet.
  if(!m_wpt_index_initialized) {
    m_wpt_index = new_wpt_index;
    m_wpt_index_initialized = true;
    return(true);
  }

  // If the waypoint index changes, mark the time.
  // The pulse will be posted 5 seconds later.
  if(new_wpt_index != m_wpt_index) {
    m_wpt_index = new_wpt_index;
    m_wpt_index_change_time = m_curr_time;
    m_pulse_pending = true;
  }

  return(true);
}

//---------------------------------------------------------------
// Procedure: postPulse()
// Purpose: Create and post the VIEW_RANGE_PULSE message.

void BHV_Pulse::postPulse()
{
  XYRangePulse pulse;

  pulse.set_x(m_osx);
  pulse.set_y(m_osy);
  pulse.set_label("bhv_pulse");

  pulse.set_rad(m_range);
  pulse.set_time(m_curr_time);
  pulse.set_duration(m_pulse_duration);

  pulse.set_color("edge", "yellow");
  pulse.set_color("fill", "yellow");

  string spec = pulse.get_spec();

  postMessage("VIEW_RANGE_PULSE", spec);
}

//---------------------------------------------------------------
// Procedure: onSetParamComplete()

void BHV_Pulse::onSetParamComplete()
{
}

//---------------------------------------------------------------
// Procedure: onCompleteState()

void BHV_Pulse::onCompleteState()
{
}

//---------------------------------------------------------------
// Procedure: onIdleState()

void BHV_Pulse::onIdleState()
{
}

//---------------------------------------------------------------
// Procedure: onHelmStart()

void BHV_Pulse::onHelmStart()
{
}

//---------------------------------------------------------------
// Procedure: postConfigStatus()

void BHV_Pulse::postConfigStatus()
{
}

//---------------------------------------------------------------
// Procedure: onRunToIdleState()

void BHV_Pulse::onRunToIdleState()
{
}

//---------------------------------------------------------------
// Procedure: onIdleToRunState()

void BHV_Pulse::onIdleToRunState()
{
}

//---------------------------------------------------------------
// Procedure: onRunState()

IvPFunction* BHV_Pulse::onRunState()
{
  bool ok = updateInfoIn();

  if(!ok)
    return(0);

  // If a waypoint change was detected, wait 5 seconds,
  // then post one range pulse.
  if(m_pulse_pending) {
    double elapsed_time = m_curr_time - m_wpt_index_change_time;

    if(elapsed_time >= m_delay) {
      postPulse();
      m_pulse_pending = false;
    }
  }

  // This behavior only posts a visual artifact.
  // It does not create an IvP objective function.
  return(0);
}