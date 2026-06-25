/*****************************************************************/
/*    NAME: M.Benjamin                                           */
/*    ORGN: Dept of Mechanical Eng / CSAIL, MIT Cambridge MA     */
/*    FILE: BHV_Scout.cpp                                        */
/*    DATE: April 30th 2022                                      */
/*****************************************************************/

#include <cstdlib>
#include <math.h>
#include "BHV_Scout.h"
#include "MBUtils.h"
#include "AngleUtils.h"
#include "BuildUtils.h"
#include "GeomUtils.h"
#include "ZAIC_PEAK.h"
#include "OF_Coupler.h"
#include "XYFormatUtilsPoly.h"

using namespace std;

//-----------------------------------------------------------
// Constructor()

BHV_Scout::BHV_Scout(IvPDomain gdomain) : 
  IvPBehavior(gdomain)
{
  IvPBehavior::setParam("name", "scout");
 
  // Default values for behavior state variables
  m_osx  = 0;
  m_osy  = 0;
  m_osh  = 0;

  // All distances are in meters, all speed in meters per second
  // Default values for configuration parameters 
  m_desired_speed  = 1; 
  m_capture_radius = 10;
  m_anti_cluster_radius = 20.0;
  m_edge_buffer    = 25.0; // Distance inward from the main boundary (your red area)
  //m_momentum_weight = 1.5; // Default: subtracts 1.5 meters from the "score" for every 1 degree of turn

  m_pt_set = false;
  m_path_index = 0; // ADD THIS

  m_anti_cluster_radius = 20.0; // Distance to maintain from known clusters
  
  addInfoVars("NAV_X, NAV_Y");
  addInfoVars("RESCUE_REGION");
  addInfoVars("SCOUTED_SWIMMER");
}

bool BHV_Scout::setParam(string param, string val) 
{
  param = tolower(param);
  bool handled = true;
  
  if(param == "capture_radius")
    handled = setPosDoubleOnString(m_capture_radius, val);
  else if(param == "desired_speed")
    handled = setPosDoubleOnString(m_desired_speed, val);
  else if(param == "edge_buffer") // ADD THIS
    handled = setPosDoubleOnString(m_edge_buffer, val);
  else if(param == "tmate")
    handled = setNonWhiteVarOnString(m_tmate, val);
  else
    handled = false;

  srand(time(NULL));
  return(handled);
}

//-----------------------------------------------------------
// Procedure: onEveryState()

//-----------------------------------------------------------
// Procedure: onEveryState()

void BHV_Scout::onEveryState(string str) 
{
  if(getBufferVarUpdated("SCOUTED_SWIMMER")) {
    string report = getBufferStringVal("SCOUTED_SWIMMER");
    if(report != "") {
      // 1. Store the swimmer's location for our anti-clustering logic
      handleNewSwimmer(report);
      
      // 2. Alert the teammate
      if(m_tmate != "") {
        postOffboardMessage(m_tmate, "SWIMMER_ALERT", report);
      } else {
        postWMessage("Mandatory Teammate name is null");
      }
    }
  }
}

//-----------------------------------------------------------
// Procedure: handleNewSwimmer()
// Parses standard string formats to extract x and y
void BHV_Scout::handleNewSwimmer(string report) 
{
  double x = 0, y = 0;
  bool x_set = false, y_set = false;
  
  vector<string> svector = parseString(report, ',');
  for(unsigned int i=0; i<svector.size(); i++) {
    string param = tolower(biteStringX(svector[i], '='));
    string value = svector[i];
    if(param == "x") {
      x = atof(value.c_str());
      x_set = true;
    }
    else if(param == "y") {
      y = atof(value.c_str());
      y_set = true;
    }
  }
  
  if(x_set && y_set) {
    m_known_swimmers.push_back(XYPoint(x, y));
  }
}

//-----------------------------------------------------------
// Procedure: onRunState()

IvPFunction *BHV_Scout::onRunState() 
{
  // Part 1: Get vehicle position from InfoBuffer and post a 
  // warning if problem is encountered
  bool ok1, ok2;
  m_osx = getBufferDoubleVal("NAV_X", ok1);
  m_osy = getBufferDoubleVal("NAV_Y", ok2);
  bool ok3;
  m_osh = getBufferDoubleVal("NAV_HEADING", ok3); // GET HEADING
  if(!ok1 || !ok2 || !ok3) {
    postWMessage("No ownship X/Y/HDG info in info_buffer.");
    return(0);
  }
  
  // Part 2: Determine if the vehicle has reached the destination 
  // point and if so, declare completion.
  updateScoutPoint();
  double dist = hypot((m_ptx-m_osx), (m_pty-m_osy));
  //postEventMessage("Dist=" + doubleToStringX(dist,1));
  if(dist <= m_capture_radius) {
    m_pt_set = false;
    postViewPoint(false);
    return(0);
  }

  // Part 3: Post the waypoint as a string for consumption by 
  // a viewer application.
  postViewPoint(true);

  // Part 4: Build the IvP function 
  IvPFunction *ipf = buildFunction();
  if(ipf == 0) 
    postWMessage("Problem Creating the IvP Function");
  
  return(ipf);
}

//-----------------------------------------------------------
// Procedure: updateScoutPoint()

void BHV_Scout::updateScoutPoint()
{
  if(m_pt_set)
    return;

  string region_str = getBufferStringVal("RESCUE_REGION");
  if(region_str == "") {
    postWMessage("Unknown RESCUE_REGION");
    return;
  }
  
  m_rescue_region = string2Poly(region_str);
  if(!m_rescue_region.is_convex()) {
    postWMessage("Badly formed RESCUE_REGION");
    return;
  }
  
  int num_candidates = 500; // High number to guarantee it finds a point inside the red zone
  bool found_valid = false;

  for(int i = 0; i < num_candidates; i++) {
    double ptx = 0;
    double pty = 0;
    bool ok = randPointInPoly(m_rescue_region, ptx, pty);
    
    if(ok) {
      // Check distance to the outer polygon walls
      double min_dist_to_edge = -1;
      unsigned int num_vertices = m_rescue_region.size(); 
      
      for(unsigned int v = 0; v < num_vertices; v++) {
        double x1 = m_rescue_region.get_vx(v);
        double y1 = m_rescue_region.get_vy(v);
        double x2 = m_rescue_region.get_vx((v + 1) % num_vertices);
        double y2 = m_rescue_region.get_vy((v + 1) % num_vertices);
        
        double dist = distPointToSeg(ptx, pty, x1, y1, x2, y2);
        if(min_dist_to_edge < 0 || dist < min_dist_to_edge) {
          min_dist_to_edge = dist;
        }
      }
      
      // Is this random point safely inside your red area?
      if(min_dist_to_edge >= m_edge_buffer) {
        m_ptx = ptx;
        m_pty = pty;
        found_valid = true;
        break; // We found a good point, stop looking!
      }
    }
  }

  if(!found_valid) {
    postWMessage("Unable to generate scout point inside the red area");
    return;
  }
    
  m_pt_set = true;
  string msg = "New pt: " + doubleToStringX(m_ptx) + "," + doubleToStringX(m_pty);
  postEventMessage(msg);
}

//-----------------------------------------------------------
// Procedure: postViewPoint()

void BHV_Scout::postViewPoint(bool viewable) 
{

  XYPoint pt(m_ptx, m_pty);
  pt.set_vertex_size(5);
  pt.set_vertex_color("orange");
  pt.set_label(m_us_name + "'s next waypoint");
  
  string point_spec;
  if(viewable)
    point_spec = pt.get_spec("active=true");
  else
    point_spec = pt.get_spec("active=false");
  postMessage("VIEW_POINT", point_spec);
}


//-----------------------------------------------------------
// Procedure: buildFunction()

IvPFunction *BHV_Scout::buildFunction() 
{
  if(!m_pt_set)
    return(0);
  
  ZAIC_PEAK spd_zaic(m_domain, "speed");
  spd_zaic.setSummit(m_desired_speed);
  spd_zaic.setPeakWidth(0.5);
  spd_zaic.setBaseWidth(1.0);
  spd_zaic.setSummitDelta(0.8);  
  if(spd_zaic.stateOK() == false) {
    string warnings = "Speed ZAIC problems " + spd_zaic.getWarnings();
    postWMessage(warnings);
    return(0);
  }
  
  double rel_ang_to_wpt = relAng(m_osx, m_osy, m_ptx, m_pty);
  ZAIC_PEAK crs_zaic(m_domain, "course");
  crs_zaic.setSummit(rel_ang_to_wpt);
  crs_zaic.setPeakWidth(0);
  crs_zaic.setBaseWidth(180.0);
  crs_zaic.setSummitDelta(0);  
  crs_zaic.setValueWrap(true);
  if(crs_zaic.stateOK() == false) {
    string warnings = "Course ZAIC problems " + crs_zaic.getWarnings();
    postWMessage(warnings);
    return(0);
  }

  IvPFunction *spd_ipf = spd_zaic.extractIvPFunction();
  IvPFunction *crs_ipf = crs_zaic.extractIvPFunction();

  OF_Coupler coupler;
  IvPFunction *ivp_function = coupler.couple(crs_ipf, spd_ipf, 50, 50);

  return(ivp_function);
}

//-----------------------------------------------------------
// Procedure: onIdleState()

void BHV_Scout::onIdleState() 
{
  m_curr_time = getBufferCurrTime();
}
