/*****************************************************************/
/*    NAME: Michael Benjamin                                     */
/*    ORGN: Dept of Mechanical Engineering, MIT, Cambridge MA    */
/*    FILE: ContactMgrV20.h                                      */
/*    DATE: Feb 24th 2010                                        */
/*                                                               */
/* This file is part of MOOS-IvP                                 */
/*                                                               */
/* MOOS-IvP is free software: you can redistribute it and/or     */
/* modify it under the terms of the GNU General Public License   */
/* as published by the Free Software Foundation, either version  */
/* 3 of the License, or (at your option) any later version.      */
/*                                                               */
/* MOOS-IvP is distributed in the hope that it will be useful,   */
/* but WITHOUT ANY WARRANTY; without even the implied warranty   */
/* of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See  */
/* the GNU General Public License for more details.              */
/*                                                               */
/* You should have received a copy of the GNU General Public     */
/* License along with MOOS-IvP.  If not, see                     */
/* <http://www.gnu.org/licenses/>.                               */
/*****************************************************************/

#ifndef CONTACT_MANAGER_V20_HEADER
#define CONTACT_MANAGER_V20_HEADER

#include <vector>
#include "MOOS/libMOOS/Thirdparty/AppCasting/AppCastingMOOSApp.h"
#include "ContactMgrCore.h"

class ContactMgrV20 : public AppCastingMOOSApp, public ContactMgrCore
{
 public:
  ContactMgrV20();
  virtual ~ContactMgrV20() {}

  bool OnNewMail(MOOSMSG_LIST &NewMail);
  bool Iterate();
  bool OnConnectToServer();
  bool OnStartUp();

 public:
  bool buildReport();

 protected:
  void registerVariables();

  bool handleConfigMaxRetHist(std::string);
  bool handleConfigRecapInterval(std::string);

  std::string handleConfigDeprecations(std::string);

  void handleMailReportRequest(std::string, std::string);
  void handleMailAlertRequest(std::string, std::string);

  void postSummaries();
  void postRadii(bool=true);
  void postRangeReports();
  void pruneRangeReports();
  void postEarlyWarningRadii();

 protected: // ContactMgrCore output goes to the MOOSDB and appcast
  void cmgrPost(const std::string& var, const std::string& sval);
  void cmgrPost(const std::string& var, double dval);
  void cmgrEvent(const std::string& event);
  void cmgrConfigWarning(const std::string& warning);

 protected: // Configuration parameters
  bool         m_display_radii;
  std::string  m_alert_rng_color;

  bool         m_post_closest_range;
  bool         m_post_closest_relbng;
  bool         m_post_all_ranges;
  double       m_contacts_recap_interval;
  double       m_range_report_timeout;
  unsigned int m_range_report_maxsize;

  double       m_closest_contact_rng_one;
  double       m_closest_contact_rng_two;

  unsigned int m_max_retired_hist;

  bool         m_early_warning_radii; // true if rendering ranges
  std::string  m_ewarn_radii_color;   // default is yellow

 protected: // State variables
  double m_contacts_recap_posted;

  // Main Record #1 Optional req range reports all keyed on varname
  std::map<std::string, double>      m_map_rep_range;
  std::map<std::string, double>      m_map_rep_reqtime;
  std::map<std::string, std::string> m_map_rep_group;
  std::map<std::string, std::string> m_map_rep_vtype;
  std::map<std::string, std::string> m_map_rep_contacts;
  std::map<std::string, bool>        m_map_rep_refresh;

  std::string m_closest_name;

  // memory of previous status postings: A posting to the MOOS var is
  // only made when a change is detected between curr and prev.
  std::string m_prev_contacts_list;
  std::string m_prev_contacts_retired;
  std::string m_prev_contacts_alerted;
  std::string m_prev_contacts_recap;
  std::string m_prev_contact_closest;
  unsigned int m_prev_contacts_count;

  double  m_prev_closest_range;
  double  m_prev_closest_relbng;  // relbng of closest contact
  double  m_prev_closest_contact_val;

  unsigned int m_alert_requests_received;
};

#endif
