/************************************************************/
/*    NAME: johngior                                              */
/*    ORGN: MIT, Cambridge MA                               */
/*    FILE: GenPath.cpp                                        */
/*    DATE: December 29th, 1963                             */
/************************************************************/

#include <iterator>
#include "MBUtils.h"
#include "ACTable.h"
#include "GenPath.h"
#include "MBUtils.h"
#include "XYPoint.h"
#include "XYSegList.h"
#include <cmath>

#include "MOOS/libMOOS/Thirdparty/AppCasting/AppCastingMOOSApp.h"


using namespace std;

//---------------------------------------------------------
// Constructor()

GenPath::GenPath()
{
  // 1. Initialize original state variables
  m_nav_x = 0.0;
  m_nav_y = 0.0;
  m_points_ready = false;

  // 2. Initialize AppCasting tracking variables
  m_visit_radius      = 5.0;   // Default visit radius (e.g., 5 meters)
  m_tot_pts_received  = 0;
  m_invalid_pts       = 0;
  m_pts_visited       = 0;
  m_first_pt_received = false;
  m_last_pt_received  = false;
  m_nav_received      = false;
}

//---------------------------------------------------------
// Destructor

GenPath::~GenPath()
{
}

//---------------------------------------------------------
// Procedure: OnNewMail()

bool GenPath::OnNewMail(MOOSMSG_LIST &NewMail)
{
  AppCastingMOOSApp::OnNewMail(NewMail);

  MOOSMSG_LIST::iterator p;
  for(p=NewMail.begin(); p!=NewMail.end(); p++) {
    CMOOSMsg &msg = *p;
    std::string key    = msg.GetKey();
    std::string sval = msg.GetString();
    double dval      = msg.GetDouble();

    // 1. Track Vehicle Position
    if (key == "NAV_X") {
      m_nav_x = dval;
    } 
    else if (key == "NAV_Y") {
      m_nav_y = dval;
    }

    // 2. Collect the Visit Points
    else if (key == "VISIT_POINT") {
      if (sval == "firstpoint") {
        m_visit_points.clear();      // Clear old points to start fresh
      }
      else if (sval == "lastpoint") {
        m_points_ready = true;       // All points received, ready to calculate path
      }
      else {
        m_visit_points.push_back(sval); // Store the coordinate string
      }
    }

#if 0 // Keep these around just for template
    string comm  = msg.GetCommunity();
    double dval  = msg.GetDouble();
    string sval  = msg.GetString(); 
    string msrc  = msg.GetSource();
    double mtime = msg.GetTime();
    bool   mdbl  = msg.IsDouble();
    bool   mstr  = msg.IsString();
#endif

     else if(key == "FOO") 
       cout << "great!";

     else if(key != "APPCAST_REQ") // handled by AppCastingMOOSApp
       reportRunWarning("Unhandled Mail: " + key);
   }
	
   return(true);
}

//---------------------------------------------------------
// Procedure: OnConnectToServer()

bool GenPath::OnConnectToServer()
{
   registerVariables();
   return(true);
}

//---------------------------------------------------------
// Procedure: Iterate()
//            happens AppTick times per second

bool GenPath::Iterate()
{
  AppCastingMOOSApp::Iterate();

  // Only calculate the path when the full burst of points has been received
  if (m_points_ready) {
    
    // 1. Parse the string points into a temporary vector of XYPoints
    std::vector<XYPoint> unvisited_points;
    for (int i = 0; i < m_visit_points.size(); i++) {
      std::string x_str = tokStringParse(m_visit_points[i], "x", ',', '=');
      std::string y_str = tokStringParse(m_visit_points[i], "y", ',', '=');
      
      double x = atof(x_str.c_str());
      double y = atof(y_str.c_str());
      unvisited_points.push_back(XYPoint(x, y));
    }

    XYSegList my_seglist;

    // 2. Greedy Shortest Path Algorithm
    // Start from the vehicle's current position
    double current_x = m_nav_x;
    double current_y = m_nav_y;

    while (!unvisited_points.empty()) {
      int closest_idx = 0;
      double min_dist = -1;

      // Find the closest point in the unvisited list
      for (int i = 0; i < unvisited_points.size(); i++) {
        double px = unvisited_points[i].get_vx();
        double py = unvisited_points[i].get_vy();
        
        // Calculate Euclidean distance
        double dist = std::hypot(px - current_x, py - current_y);

        if (min_dist == -1 || dist < min_dist) {
          min_dist = dist;
          closest_idx = i;
        }
      }

      // 3. Add the closest point to your segment list
      double next_x = unvisited_points[closest_idx].get_vx();
      double next_y = unvisited_points[closest_idx].get_vy();
      my_seglist.add_vertex(next_x, next_y);

      // Update current position to the newly visited point
      current_x = next_x;
      current_y = next_y;

      // Remove the visited point from the unvisited list
      unvisited_points.erase(unvisited_points.begin() + closest_idx);
    }

    // 4. Publish the newly generated path to the MOOSDB
    std::string update_str = "points = ";
    update_str += my_seglist.get_spec();

    // Replace "WPT_UPDATE" with the actual updates parameter 
    // configured in your Waypoint behavior in the .bhv file
    Notify("TRANSIT_UPDATES", update_str);

    // 5. Reset the flag so we don't recalculate this continuously
    m_points_ready = false;
  }
  
  // Optional: Post an appcast report here if you want to see data in pMarineViewer
  AppCastingMOOSApp::PostReport();
  return(true);
}

//---------------------------------------------------------
// Procedure: OnStartUp()
//            happens before connection is open

bool GenPath::OnStartUp()
{
  AppCastingMOOSApp::OnStartUp();

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
    if(param == "foo") {
      handled = true;
    }
    else if(param == "bar") {
      handled = true;
    }

    if(!handled)
      reportUnhandledConfigWarning(orig);

  }
  
  registerVariables();	
  return(true);
}

//---------------------------------------------------------
// Procedure: registerVariables()

void GenPath::registerVariables()
{
  AppCastingMOOSApp::RegisterVariables();
  // Register("FOOBAR", 0);
  Register("VISIT_POINT", 0);
  Register("NAV_X", 0);
  Register("NAV_Y", 0);
}


//------------------------------------------------------------
// Procedure: buildReport()

/*bool GenPath::buildReport() 
{
  m_msgs << "============================================" << endl;
  m_msgs << "File:                                       " << endl;
  m_msgs << "============================================" << endl;

  ACTable actab(4);
  actab << "Alpha | Bravo | Charlie | Delta";
  actab.addHeaderLines();
  actab << "one" << "two" << "three" << "four";
  m_msgs << actab.getFormattedString();

  return(true);
}*/

bool GenPath::buildReport() 
{
  m_msgs << "============================================" << endl;
  m_msgs << "Visit Radius:            " << m_visit_radius << endl;
  m_msgs << "Total Points Received:   " << m_tot_pts_received << endl;
  m_msgs << "Invalid Points Received: " << m_invalid_pts << endl;
  m_msgs << "First Point Received:    " << (m_first_pt_received ? "true" : "false") << endl;
  m_msgs << "Last Point Received:     " << (m_last_pt_received ? "true" : "false") << endl;
  m_msgs << "NAV_X/Y Received:        " << (m_nav_received ? "true" : "false") << endl;
  m_msgs << endl;
  m_msgs << "Tour Status" << endl;
  m_msgs << "------------------------" << endl;
  m_msgs << "  Points Visited:        " << m_pts_visited << endl;
  m_msgs << "  Points Unvisited:      " << m_visit_points.size() << endl; // From your vector

  return(true);
}




