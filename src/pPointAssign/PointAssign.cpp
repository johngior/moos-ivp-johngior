/************************************************************/
/*    NAME: johngior                                              */
/*    ORGN: MIT, Cambridge MA                               */
/*    FILE: PointAssign.cpp                                        */
/*    DATE: December 29th, 1963                             */
/************************************************************/

#include <iterator>
#include "MBUtils.h"
#include "ACTable.h"
#include "PointAssign.h"
#include "XYPoint.h"

using namespace std;

//---------------------------------------------------------
// Constructor()

PointAssign::PointAssign()
{
  // ... any existing initialization code ...

  // Initialize AppCasting tracking variables
  m_total_pts_received = 0;
  m_pts_sent_to_v1 = 0;
  m_pts_sent_to_v2 = 0;
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
    std::string sval = msg.GetString();

    if (key == "VISIT_POINT"){
      if (sval == "first point"){
        m_visit_points.clear();      // Start of a new batch
      }
      else if (sval == "lastpoint"){
        m_points_ready = true;       // End of the batch, ready to distribute
      }
      else{
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

bool PointAssign::OnConnectToServer()
{
   registerVariables();
   return(true);
}

//---------------------------------------------------------
// Procedure: Iterate()
//            happens AppTick times per second

bool PointAssign::Iterate()
{
  AppCastingMOOSApp::Iterate();
  // Do your thing here!
  if (m_points_ready && m_vnames.size() > 0) {
    // 1. Post "firstpoint" to all known vehicles
    for (int i = 0; i < m_vnames.size(); i++) {
      std::string var_name = "VISIT_POINT_" + toupper(m_vnames[i]);
      Notify(var_name, "firstpoint");
    }

    // 2. Distribute the points
    int v_index = 0; // Used for alternating assignment

    for (int i = 0; i < m_visit_points.size(); i++) {
      std::string point_str = m_visit_points[i];
      std::string target_vehicle = "";

      if (m_assign_by_region) {
        // Parse the X coordinate from the string (e.g., "x=8, y=9, id=1")
        std::string x_str = tokStringParse(point_str, "x", ',', '=');
        double x_pos = atof(x_str.c_str());

        // West half (-25 to 87.5) goes to first vehicle, East half goes to second
        if (x_pos < 87.5) {
          target_vehicle = m_vnames[0];
        } else {
          target_vehicle = m_vnames[1];
        }
      }
      else {
        // Alternating assignment
        target_vehicle = m_vnames[v_index];
        v_index = (v_index + 1) % m_vnames.size();
      }

      // Route the point to the targeted vehicle
      std::string var_name = "VISIT_POINT_" + toupper(target_vehicle);
      Notify(var_name, point_str);

      // Optional: Post VIEW_POINT here so you can see it in pMarineViewer
      // postViewPoint(x_pos, y_pos, id_str, color);

      // 1. Parse the X, Y, and ID from the string (e.g., "x=8, y=9, id=1")
      string x_str  = tokStringParse(point_str, "x", ',', '=');
      string y_str  = tokStringParse(point_str, "y", ',', '=');
      string id_str = tokStringParse(point_str, "id", ',', '=');

      double x_pos = atof(x_str.c_str());
      double y_pos = atof(y_str.c_str());

      // 2. Set the color based on which vehicle got the point
      string color = "yellow";
      if (target_vehicle == "gilda"){
        color = "red";
      }

      // 3. Publish to pMarineViewer
      postViewPoint(x_pos, y_pos, id_str, color);
    }

    // 3. Post "lastpoint" to all known vehicles
    for (int i = 0; i < m_vnames.size(); i++) {
      std::string var_name = "VISIT_POINT_" + toupper(m_vnames[i]);
      Notify(var_name, "lastpoint");
    }

    // 4. Reset the flag so we don't distribute the same points again
    m_points_ready = false;
  }
  AppCastingMOOSApp::PostReport();
  return(true);
}

//---------------------------------------------------------
// Procedure: OnStartUp()
//            happens before connection is open

bool PointAssign::OnStartUp()
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
    if(param == "vname") {
      m_vnames.push_back(value);
      handled = true;
    }
    else if(param == "assign_by_region") {
      m_assign_by_region = (tolower(value) == "true");
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

void PointAssign::registerVariables()
{
  AppCastingMOOSApp::RegisterVariables();
  // Register("FOOBAR", 0);
  Register("VISIT_POINT", 0);
}


//------------------------------------------------------------
// Procedure: buildReport()

/*bool PointAssign::buildReport() 
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

bool PointAssign::buildReport() 
{
  m_msgs << "============================================" << endl;
  m_msgs << "pPointAssign Status Report                  " << endl;
  m_msgs << "============================================" << endl;
  m_msgs << "Total Points Received from Script: " << m_total_pts_received << endl;
  m_msgs << endl;
  m_msgs << "Distribution Status" << endl;
  m_msgs << "------------------------" << endl;
  m_msgs << "  Points assigned to Vehicle 1: " << m_pts_sent_to_v1 << endl;
  m_msgs << "  Points assigned to Vehicle 2: " << m_pts_sent_to_v2 << endl;

  return(true);
}

void PointAssign::postViewPoint(double x, double y, string label, string color) {
  XYPoint point(x, y);
  point.set_label(label); 
  point.set_color("vertex", color); 
  point.set_param("vertex_size", "4");

  string spec = point.get_spec(); // gets the string representation of a point
  Notify("VIEW_POINT", spec);
}




