/************************************************************/
/*    NAME: johngior                                              */
/*    ORGN: MIT, Cambridge MA                               */
/*    FILE: GenPath.h                                          */
/*    DATE: December 29th, 1963                             */
/************************************************************/

#ifndef GenPath_HEADER
#define GenPath_HEADER
#include <vector>
#include <string>

#include "MOOS/libMOOS/Thirdparty/AppCasting/AppCastingMOOSApp.h"

class GenPath : public AppCastingMOOSApp
{
 public:
   GenPath();
   ~GenPath();

 protected: // Standard MOOSApp functions to overload  
   bool OnNewMail(MOOSMSG_LIST &NewMail);
   bool Iterate();
   bool OnConnectToServer();
   bool OnStartUp();

 protected: // Standard AppCastingMOOSApp function to overload 
   bool buildReport();

 protected:
   void registerVariables();

 private: // Configuration variables

 private: // State variables
 // State variables for ownship position
 double m_nav_x;
 double m_nav_y;

 // State variables for visit points
 std::vector<std::string> m_visit_points;
 bool m_points_ready;
 // Tracking variables for AppCasting
 double m_visit_radius;
 int    m_tot_pts_received;
 int    m_invalid_pts;
 bool   m_first_pt_received;
 bool   m_last_pt_received;
 bool   m_nav_received;
 int    m_pts_visited;
};

#endif 
