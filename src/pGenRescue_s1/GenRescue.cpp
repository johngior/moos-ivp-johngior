/************************************************************/
/*    NAME: Mike Benjamin                                   */
/*    ORGN: MIT                                             */
/*    FILE: GenRescue.cpp                                   */
/*    DATE: April 18th, 2022                                */
/************************************************************/

#include <iterator>
#include "GenRescue.h"
#include "MBUtils.h"
#include "ColorParse.h"
#include "XYPoint.h"
#include "XYSegList.h"
#include "GeomUtils.h"
#include "PathUtils.h"
#include "XYFormatUtilsPoly.h"
#include "XYFieldGenerator.h"
#include <set>
#include <limits>
#include <cmath>
#include <algorithm>
#include "NodeRecord.h"
#include "NodeRecordUtils.h"
#include "GenRescue.h"


using namespace std;

//---------------------------------------------------------
// Constructor()
static double pointDistance(double x1, double y1,
                            double x2, double y2)
{
  double dx = x2 - x1;
  double dy = y2 - y1;
  return(sqrt((dx * dx) + (dy * dy)));
}


static vector<Swimmer> buildLookAheadRoute(
  vector<Swimmer> remaining,
  double start_x,
  double start_y)
{
  vector<Swimmer> route;

  double curr_x = start_x;
  double curr_y = start_y;

  while(!remaining.empty()) {
    unsigned int best_ix = 0;
    double best_score = numeric_limits<double>::max();

    for(unsigned int i=0; i<remaining.size(); i++) {
      double leg_one = pointDistance(curr_x, curr_y,
                                     remaining[i].x, remaining[i].y);

      double leg_two = 0;

      if(remaining.size() > 1) {
        leg_two = numeric_limits<double>::max();

        for(unsigned int j=0; j<remaining.size(); j++) {
          if(i == j)
            continue;

          double candidate_leg =
            pointDistance(remaining[i].x, remaining[i].y,
                          remaining[j].x, remaining[j].y);

          if(candidate_leg < leg_two)
            leg_two = candidate_leg;
        }
      }

      // Two-vertex look-ahead:
      // prefer a target that is near us and near another swimmer.
      double score = leg_one + leg_two;

      if(score < best_score) {
        best_score = score;
        best_ix = i;
      }
    }

    route.push_back(remaining[best_ix]);

    curr_x = remaining[best_ix].x;
    curr_y = remaining[best_ix].y;

    remaining.erase(remaining.begin() + best_ix);
  }

  return(route);
}

GenRescue::GenRescue()
{
  m_nav_x = 0;
  m_nav_y = 0;
  m_nav_x_set = false;
  m_nav_y_set = false;
  m_plan_pending = false;
  
  m_enemy_x = 0;
  m_enemy_y = 0;
  m_enemy_report_set = false;
  m_enemy_name = "";
}

//---------------------------------------------------------
// Procedure: OnNewMail()

bool GenRescue::OnNewMail(MOOSMSG_LIST &NewMail)
{
  AppCastingMOOSApp::OnNewMail(NewMail);

  MOOSMSG_LIST::iterator p;
  for(p=NewMail.begin(); p!=NewMail.end(); p++) {
    CMOOSMsg &msg = *p;
    string key  = msg.GetKey();
    string sval = msg.GetString();

    bool handled = true;
    if(key == "SWIMMER_ALERT") 
      handled = handleMailNewSwimmer(sval);
    else if(key == "FOUND_SWIMMER") 
      handled = handleMailFoundSwimmer(sval);
    else if(key == "NODE_REPORT")
      handled = handleMailNodeReport(sval);
    else if(key == "NAV_X") {
      m_nav_x = msg.GetDouble();
      m_nav_x_set = true;
    }
    else if(key == "NAV_Y") {
      m_nav_y = msg.GetDouble();
      m_nav_y_set = true;
    }

    else if(key != "APPCAST_REQ") // handle by AppCastingMOOSApp
      handled = false;
    
    if(!handled)
      reportRunWarning("Unhandled Mail: " + key +"=" + sval);
    
  }
  return(true);
}
 
//---------------------------------------------------------
// Procedure: OnConnectToServer()

bool GenRescue::OnConnectToServer()
{
  RegisterVariables();
  return(true);
}

//---------------------------------------------------------
// Procedure: Iterate()

bool GenRescue::Iterate()
{
  AppCastingMOOSApp::Iterate();
  
  //if(m_plan_pending)
  if(m_plan_pending && m_nav_x_set && m_nav_y_set) {
    postShortestPath();
    m_plan_pending = false;
  }

  AppCastingMOOSApp::PostReport();
  return(true);
}

//---------------------------------------------------------
// Procedure: OnStartUp()

bool GenRescue::OnStartUp()
{
  AppCastingMOOSApp::OnStartUp(); 

  STRING_LIST sParams;
  m_MissionReader.GetConfiguration(GetAppName(), sParams);
  
  STRING_LIST::iterator p;
  for(p=sParams.begin(); p!=sParams.end(); p++) {
    string sLine  = *p;
    string param  = tolower(biteStringX(sLine, '='));
    string value  = sLine;
    if(param == "vname")
      m_vname = value;
  }
  
  RegisterVariables();	
  return(true);
}

//---------------------------------------------------------
// Procedure: RegisterVariables()

void GenRescue::RegisterVariables()
{
  AppCastingMOOSApp::RegisterVariables();
  
  Register("SWIMMER_ALERT", 0);
  Register("FOUND_SWIMMER", 0);
  Register("NODE_REPORT", 0);
  Register("NAV_X", 0);
  Register("NAV_Y", 0);
  
}


//---------------------------------------------------------
// Procedure: handleMailNewSwimmer()

bool GenRescue::handleMailNewSwimmer(string str)
{
  string id = tokStringParse(str, "id", ',', '=');
  double x  = tokDoubleParse(str, "x", ',', '=');
  double y  = tokDoubleParse(str, "y", ',', '=');

  if(id == "") {
    reportRunWarning("SWIMMER_ALERT missing id: " + str);
    return(false);
  }

  // Ignore repeated alerts for a swimmer we already know.
  if(m_swimmers.count(id) > 0)
    return(true);

  Swimmer swimmer;
  swimmer.id    = id;
  swimmer.x     = x;
  swimmer.y     = y;
  swimmer.found = false;

  m_swimmers[id] = swimmer;
  m_plan_pending = true;

  reportEvent("New swimmer received: id=" + id +
              ", x=" + doubleToString(x, 1) +
              ", y=" + doubleToString(y, 1));

  return(true);
}


//---------------------------------------------------------
// Procedure: handleMailFoundSwimmer()

bool GenRescue::handleMailFoundSwimmer(string str)
{
  string id = tokStringParse(str, "id", ',', '=');

  if(id == "") {
    reportRunWarning("FOUND_SWIMMER missing id: " + str);
    return(false);
  }

  auto it = m_swimmers.find(id);

  // It is possible to hear that a swimmer was found before
  // receiving that swimmer's alert. Nothing useful to remove yet.
  if(it == m_swimmers.end()) {
    reportEvent("FOUND_SWIMMER received for unknown id=" + id);
    return(true);
  }

  // Ignore duplicate FOUND_SWIMMER messages.
  if(it->second.found)
    return(true);

  it->second.found = true;
  m_plan_pending = true;

  reportEvent("Swimmer removed from route: id=" + id);
  return(true);
}

//---------------------------------------------------------
// Procedure: handleMailNodeReport()
bool GenRescue::handleMailNodeReport(string str)
{
  NodeRecord record = string2NodeRecord(str);

  string vname = tolower(record.getName());
  if(vname == "")
    return(false);

  // Ignore own node reports.
  if(vname == tolower(m_vname))
    return(true);

  m_enemy_name = vname;
  m_enemy_x = record.getX();
  m_enemy_y = record.getY();
  m_enemy_report_set = true;

  return(true);
}


//---------------------------------------------------------
// Procedure: postShortestPath()
void GenRescue::postShortestPath()
{
  if(!m_nav_x_set || !m_nav_y_set)
    return;

  vector<Swimmer> candidates;

  for(const auto& entry : m_swimmers) {
    const Swimmer& swimmer = entry.second;

    if(!swimmer.found)
      candidates.push_back(swimmer);
  }

  // Enemy-aware concession:
  // Remove up to two swimmers that the opponent is clearly closer to.
  if(m_enemy_report_set && candidates.size() > 2) {
    vector<pair<double, unsigned int> > enemy_ranges;

    for(unsigned int i=0; i<candidates.size(); i++) {
      double enemy_dist =
        pointDistance(m_enemy_x, m_enemy_y,
                      candidates[i].x, candidates[i].y);

      enemy_ranges.push_back(make_pair(enemy_dist, i));
    }

    sort(enemy_ranges.begin(), enemy_ranges.end());

    set<string> conceded_ids;
    unsigned int max_concede = 2;

    for(unsigned int k=0;
        (k<enemy_ranges.size()) && (k<max_concede);
        k++) {

      unsigned int ix = enemy_ranges[k].second;

      double enemy_dist = enemy_ranges[k].first;
      double my_dist =
        pointDistance(m_nav_x, m_nav_y,
                      candidates[ix].x, candidates[ix].y);

      // Concede only when the opponent has a meaningful advantage.
      // 8m is deliberately conservative for equal-speed vehicles.
      if(enemy_dist + 8.0 < my_dist)
        conceded_ids.insert(candidates[ix].id);
    }

    vector<Swimmer> filtered;

    for(unsigned int i=0; i<candidates.size(); i++) {
      if(conceded_ids.count(candidates[i].id) == 0)
        filtered.push_back(candidates[i]);
    }

    // Never concede every swimmer.
    if(!filtered.empty())
      candidates = filtered;
  }

  if(candidates.empty()) {
    postNullPath();
    return;
  }

  vector<Swimmer> route =
    buildLookAheadRoute(candidates, m_nav_x, m_nav_y);

  m_path = XYSegList();
  m_path.set_label("rescue_route");

  for(unsigned int i=0; i<route.size(); i++)
    m_path.add_vertex(route[i].x, route[i].y);

  Notify("VIEW_SEGLIST", m_path.get_spec());

  string update_str = "points = " + m_path.get_spec_pts();

  Notify("SURVEY_UPDATE", update_str);
  reportEvent("Lookahead route posted: " + update_str);
}

//---------------------------------------------------------
// Procedure: postNullPath()
//   Purpose: If a found swimmer represents the last swimmer
//            to be found, then post a survey update essentially
// 

void GenRescue::postNullPath()
{
#if 0
  if(!m_nav_x_set || !m_nav_y_set)
    return;
  if(m_map_pts.size() != 0)
    return;
  
  XYSegList segl;
  segl.add_vertex(m_nav_x, m_nav_y);
  
  // Seglist needs a name, refer when drawging and erasing
  segl.set_label("one");
  Notify("VIEW_SEGLIST", segl.get_spec());

  string update_var = "SURVEY_UPDATE";
  string update_str = "points = " + segl.get_spec_pts();

  Notify(update_var, update_str);
  reportEvent("SURVEY_UPDATE=" + update_str);
#endif
}


//---------------------------------------------------------
// Procedure: buildReport()

bool GenRescue::buildReport()
{
  m_msgs << "Known swimmers: " << m_swimmers.size() << endl;

  for(const auto& entry : m_swimmers) {
    const Swimmer& swimmer = entry.second;

    m_msgs << "  id=" << swimmer.id
           << "  x=" << doubleToString(swimmer.x, 1)
           << "  y=" << doubleToString(swimmer.y, 1)
           << "  found=" << boolToString(swimmer.found)
           << endl;
  }

  unsigned int active_count = 0;

  for(const auto& entry : m_swimmers) {
    if(!entry.second.found)
      active_count++;
  }

  m_msgs << endl;
  m_msgs << "Active swimmers: " << active_count << endl;
  m_msgs << "Plan pending:    " << boolToString(m_plan_pending) << endl;

  
  return(true);
}
