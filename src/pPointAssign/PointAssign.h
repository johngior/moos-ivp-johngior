/************************************************************/
/*    NAME: johngior                                              */
/*    ORGN: MIT, Cambridge MA                               */
/*    FILE: PointAssign.h                                          */
/*    DATE: December 29th, 1963                             */
/************************************************************/

#ifndef PointAssign_HEADER
#define PointAssign_HEADER

#include "MOOS/libMOOS/Thirdparty/AppCasting/AppCastingMOOSApp.h"
#include <vector>
#include <string>

class PointAssign : public AppCastingMOOSApp
{
 public:
   PointAssign();
   ~PointAssign();

 protected: // Standard MOOSApp functions to overload  
   bool OnNewMail(MOOSMSG_LIST &NewMail);
   bool Iterate();
   bool OnConnectToServer();
   bool OnStartUp();

 protected: // Standard AppCastingMOOSApp function to overload 
   bool buildReport();

 protected:
   void registerVariables();

   // ADD YOUR HELPER FUNCTION DECLARATION HERE:
   void postViewPoint(double x, double y, std::string label, std::string color);

 private: // Configuration variables
 bool m_assign_by_region;
 std::vector<std::string> m_vnames;

 private: // State variables
 std::vector<std::string> m_visit_points;
 bool m_points_ready;
 int m_total_pts_received;
 int m_pts_sent_to_v1;
 int m_pts_sent_to_v2;
//bool m_assign_by_region;
//std::vector<std::string> m_vnames;
};

#endif 
