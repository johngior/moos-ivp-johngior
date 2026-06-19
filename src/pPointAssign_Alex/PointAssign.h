/************************************************************/
/*    NAME: Alexios Vavvas                                  */
/*    ORGN: MIT, Cambridge MA                               */
/*    FILE: PointAssign.h                                   */
/*    DATE: 2026/06/17                                      */
/************************************************************/

#ifndef PointAssign_HEADER
#define PointAssign_HEADER

#include <string>
#include <vector>
#include "MOOS/libMOOS/Thirdparty/AppCasting/AppCastingMOOSApp.h"

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
   void handleVisitPoint(const std::string& sval);
   void assignPoint(const std::string& sval, double x, double y, unsigned int id);
   void postViewPoint(double x, double y, std::string label, std::string color);
   unsigned int vehicleForPoint(double x);   // region/alternating selection

 private: // Configuration variables
   std::vector<std::string> m_vnames;        // vehicle names, in order added
   std::vector<std::string> m_colors;        // one color per vehicle
   bool   m_assign_by_region;                // true: east/west, false: alternating

   double m_region_xmin;
   double m_region_xmax;

 private: // State variables
   unsigned int m_total_received;            // valid points received
   unsigned int m_invalid_received;
   bool   m_first_received;
   bool   m_last_received;
   unsigned int m_assign_counter;            // for alternating assignment
   std::vector<unsigned int> m_vcount;       // points assigned per vehicle
   bool   m_timer_unpaused;                  // handshake: unpaused timer yet?
};

#endif
