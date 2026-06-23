/************************************************************/
/*    NAME: Mike Benjamin                                   */
/*    ORGN: MIT                                             */
/*    FILE: GenRescue.h                                     */
/*    DATE: April 18th, 2022                                */
/************************************************************/

#ifndef P_GEN_RESCUE_HEADER
#define P_GEN_RESCUE_HEADER

#include <vector>
#include <string>
#include <map>
#include "MOOS/libMOOS/Thirdparty/AppCasting/AppCastingMOOSApp.h"
#include "XYPoint.h"
#include "XYPolygon.h"


struct Swimmer
{
  std::string id;
  double x;
  double y;
  bool found;
};

class GenRescue : public AppCastingMOOSApp
{
 public:
   GenRescue();
   ~GenRescue() {};

 protected:
  bool OnNewMail(MOOSMSG_LIST &NewMail);
  bool Iterate();
  bool OnConnectToServer();
  bool OnStartUp();
  bool buildReport();
  void RegisterVariables();
  
 protected:
  bool handleMailNewSwimmer(std::string);
  bool handleMailFoundSwimmer(std::string);
  bool handleMailNodeReport(std::string);
  bool handleMailRescueRegion(std::string);
  void postShortestPath();
  void postNullPath();

 private: // Config variables
  std::string m_vname;
  
 private: // State variables
  std::map<std::string, Swimmer> m_swimmers;
  XYSegList  m_path;
  
  double     m_nav_x;
  double     m_nav_y;
  bool       m_nav_x_set;
  bool       m_nav_y_set;
  
  double     m_enemy_x;
  double     m_enemy_y;
  bool       m_enemy_report_set;
  std::string m_enemy_name;
  
  bool       m_plan_pending;
};

#endif 




