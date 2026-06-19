/************************************************************/
/*    NAME: Alexios Vavvas                                  */
/*    ORGN: MIT, Cambridge MA                               */
/*    FILE: PointAssign.cpp                                 */
/*    DATE: 2026/06/17                                      */
/************************************************************/

#include <iterator>
#include "MBUtils.h"
#include "ACTable.h"
#include "XYPoint.h"
#include "PointAssign.h"

using namespace std;

//---------------------------------------------------------
// Constructor()

PointAssign::PointAssign()
{
  m_assign_by_region = false;

  // Region given in lab spec, corners x in [-25, 200]
  m_region_xmin = -25;
  m_region_xmax = 200;

  m_total_received   = 0;
  m_invalid_received = 0;
  m_first_received   = false;
  m_last_received    = false;
  m_assign_counter   = 0;
  m_timer_unpaused   = false;
}

//---------------------------------------------------------
// Destructor

PointAssign::~PointAssign()
{
}

//---------------------------------------------------------
// Procedure: OnNewMail()

bool PointAssign::OnNewMail(MOOSMSG_LIST &NewMail)
{
  AppCastingMOOSApp::OnNewMail(NewMail);

  MOOSMSG_LIST::iterator p;
  for(p=NewMail.begin(); p!=NewMail.end(); p++) {
    CMOOSMsg &msg = *p;
    string key    = msg.GetKey();
    string sval   = msg.GetString();

    if(key == "VISIT_POINT")
      handleVisitPoint(sval);

    else if(key != "APPCAST_REQ") // handled by AppCastingMOOSApp
      reportRunWarning("Unhandled Mail: " + key);
  }

  return(true);
}

//---------------------------------------------------------
// Procedure: handleVisitPoint()
//   Handles one incoming VISIT_POINT posting. Three forms:
//     "firstpoint"           -> open the set, relay to all vehicles
//     "lastpoint"            -> close the set, relay to all vehicles
//     "x=.., y=.., id=.."    -> a point, assign to one vehicle

void PointAssign::handleVisitPoint(const string& sval)
{
  if(sval == "firstpoint") {
    m_first_received = true;
    for(unsigned int i=0; i<m_vnames.size(); i++)
      Notify("VISIT_POINT_" + toupper(m_vnames[i]), "firstpoint");
    return;
  }

  if(sval == "lastpoint") {
    m_last_received = true;
    for(unsigned int i=0; i<m_vnames.size(); i++)
      Notify("VISIT_POINT_" + toupper(m_vnames[i]), "lastpoint");
    return;
  }

  // Otherwise expect "x=.., y=.., id=.."
  string xstr, ystr, idstr;
  bool ok = true;
  ok = ok && tokParse(sval, "x",  ',', '=', xstr);
  ok = ok && tokParse(sval, "y",  ',', '=', ystr);
  ok = ok && tokParse(sval, "id", ',', '=', idstr);

  if(!ok || !isNumber(xstr) || !isNumber(ystr) || !isNumber(idstr)) {
    m_invalid_received++;
    reportRunWarning("Invalid VISIT_POINT: " + sval);
    return;
  }

  double x  = atof(xstr.c_str());
  double y  = atof(ystr.c_str());
  unsigned int id = (unsigned int)atoi(idstr.c_str());

  m_total_received++;
  assignPoint(sval, x, y, id);
}

//---------------------------------------------------------
// Procedure: assignPoint()
//   Picks a vehicle, relays the point under VISIT_POINT_<VNAME>,
//   and posts a viewable point colored per-vehicle.

void PointAssign::assignPoint(const string& sval, double x, double y,
                              unsigned int id)
{
  if(m_vnames.empty()) {
    reportRunWarning("No vname configured - cannot assign points");
    return;
  }

  unsigned int idx = vehicleForPoint(x);
  m_vcount[idx]++;

  Notify("VISIT_POINT_" + toupper(m_vnames[idx]), sval);

  postViewPoint(x, y, uintToString(id), m_colors[idx]);
}

//---------------------------------------------------------
// Procedure: vehicleForPoint()
//   assign_by_region=false: alternating, round-robin by arrival order.
//   assign_by_region=true : split x-range into N equal bands (west->east).

unsigned int PointAssign::vehicleForPoint(double x)
{
  unsigned int n = m_vnames.size();

  if(!m_assign_by_region) {
    unsigned int idx = m_assign_counter % n;
    m_assign_counter++;
    return(idx);
  }

  double span = m_region_xmax - m_region_xmin;
  if(span <= 0)
    return(0);

  int idx = (int)((x - m_region_xmin) / span * n);
  if(idx < 0)
    idx = 0;
  if(idx >= (int)n)
    idx = n - 1;
  return((unsigned int)idx);
}

//---------------------------------------------------------
// Procedure: postViewPoint()

void PointAssign::postViewPoint(double x, double y, string label, string color)
{
  XYPoint point(x, y);
  point.set_label(label);
  point.set_color("vertex", color);
  point.set_param("vertex_size", "4");

  string spec = point.get_spec();
  Notify("VIEW_POINT", spec);
}

//---------------------------------------------------------
// Procedure: OnConnectToServer()

bool PointAssign::OnConnectToServer()
{
   registerVariables();
   return(true);
}

//---------------------------------------------------------
// Procedure: Iterate()

bool PointAssign::Iterate()
{
  AppCastingMOOSApp::Iterate();

  // Handshake: now that we are connected & registered for VISIT_POINT,
  // release the (paused) timer script so we don't miss the burst (3.6.1)
  if(!m_timer_unpaused) {
    Notify("UTS_PAUSE", "false");
    m_timer_unpaused = true;
  }

  AppCastingMOOSApp::PostReport();
  return(true);
}

//---------------------------------------------------------
// Procedure: OnStartUp()

bool PointAssign::OnStartUp()
{
  AppCastingMOOSApp::OnStartUp();

  // Palette cycled through as vehicles are added
  vector<string> palette;
  palette.push_back("yellow");
  palette.push_back("dodger_blue");
  palette.push_back("green");
  palette.push_back("red");

  STRING_LIST sParams;
  m_MissionReader.EnableVerbatimQuoting(false);
  if(!m_MissionReader.GetConfiguration(GetAppName(), sParams))
    reportConfigWarning("No config block found for " + GetAppName());

  STRING_LIST::iterator p;
  for(p=sParams.begin(); p!=sParams.end(); p++) {
    string orig  = *p;
    string line  = *p;
    string param = tolower(biteStringX(line, '='));
    string value = line;

    bool handled = false;
    if(param == "vname") {
      m_vnames.push_back(value);
      m_colors.push_back(palette[m_vnames.size() % palette.size()]);
      m_vcount.push_back(0);
      handled = true;
    }
    else if(param == "assign_by_region") {
      handled = setBooleanOnString(m_assign_by_region, value);
    }

    if(!handled)
      reportUnhandledConfigWarning(orig);
  }

  if(m_vnames.empty())
    reportConfigWarning("No vname configured - no vehicles to assign to");

  registerVariables();
  return(true);
}

//---------------------------------------------------------
// Procedure: registerVariables()

void PointAssign::registerVariables()
{
  AppCastingMOOSApp::RegisterVariables();
  Register("VISIT_POINT", 0);
}

//------------------------------------------------------------
// Procedure: buildReport()

bool PointAssign::buildReport()
{
  string mode = m_assign_by_region ? "by-region (E/W)" : "alternating";

  m_msgs << "Assignment Mode:         " << mode             << endl;
  m_msgs << "Total Points Received:   " << m_total_received  << endl;
  m_msgs << "Invalid Points Received: " << m_invalid_received<< endl;
  m_msgs << "First Point Received:    " << boolToString(m_first_received) << endl;
  m_msgs << "Last Point Received:     " << boolToString(m_last_received)  << endl;
  m_msgs << endl;

  ACTable actab(3);
  actab << "Vehicle | Color | Assigned";
  actab.addHeaderLines();
  for(unsigned int i=0; i<m_vnames.size(); i++)
    actab << m_vnames[i] << m_colors[i] << uintToString(m_vcount[i]);
  m_msgs << actab.getFormattedString();

  return(true);
}
