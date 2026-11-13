/*****************************************************************/
/*    NAME: Michael Benjamin                                     */
/*    ORGN: Dept of Mechanical Engineering, MIT, Cambridge MA    */
/*    FILE: ContactMgrV20.cpp                                    */
/*    DATE: Feb 24th 2010                                        */
/*    DATE: Jan 2025 Early warnings / Disable bhvs added         */
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

#include <unistd.h>
#include <cmath>
#include "ContactMgrV20.h"
#include "MBUtils.h"
#include "AngleUtils.h"
#include "ColorParse.h"
#include "XYCircle.h"
#include "ACTable.h"

using namespace std;

//---------------------------------------------------------
// Constructor()

ContactMgrV20::ContactMgrV20()
{
  // Configuration Variables
  m_alert_rng_color     = "gray65";

  m_display_radii = false;

  m_post_closest_range  = false;
  m_post_closest_relbng = false;
  m_post_all_ranges     = false;
  m_contacts_recap_interval = 1;

  m_max_retired_hist = 5;

  m_range_report_timeout = 10;
  m_range_report_maxsize = 20;
  
  m_early_warning_radii = false;
  m_ewarn_radii_color   = "green";
  
  // State Variables
  m_contacts_recap_posted = 0;
  m_prev_contacts_count   = 0;
  m_prev_closest_range    = 0;
  m_prev_closest_contact_val = 0;

  m_alert_requests_received = 0;
}

//---------------------------------------------------------
// Procedure: OnNewMail()

bool ContactMgrV20::OnNewMail(MOOSMSG_LIST &NewMail)
{
  AppCastingMOOSApp::OnNewMail(NewMail);

  MOOSMSG_LIST::iterator p;
  for(p=NewMail.begin(); p!=NewMail.end(); p++) {
    CMOOSMsg &msg = *p;

    string key  = msg.GetKey();
    double dval = msg.GetDouble();
    string sval = msg.GetString(); 
    string src  = msg.GetSource(); 
    string aux  = msg.GetSourceAux(); 
    string whynot;
    
    if(aux != "")
      src += ":" + aux;

    bool handled = true;
    if(key == "NAV_X")
      m_osx = dval;
    else if(key == "NAV_Y")
      m_osy = dval;
    else if(key == "NAV_HEADING")
      m_osh = dval;
    else if(key == "NAV_SPEED") 
      m_osv = dval;
    else if(key == "NODE_REPORT") 
      handled = handleMailNodeReport(sval, whynot);
    else if(key == "BCM_REPORT_REQUEST") 
      handleMailReportRequest(sval, src);
    else if(key == "BCM_DISPLAY_RADII")
      handled = setBooleanOnString(m_display_radii, sval);
    else if(key == "BCM_ALERT_REQUEST")
      handleMailAlertRequest(sval, src);
    else if(key == "IVPHELM_STATE")
      handleMailHelmState(sval);
    else if((m_disable_var != "") && (key == m_disable_var))
      handleMailModEnableContact(sval, "disable");
    else if((m_enable_var != "") && (key == m_enable_var))
      handleMailModEnableContact(sval, "enable");
    else if((m_expunge_var != "") && (key == m_expunge_var))
      handleMailModEnableContact(sval, "expunge");
    else
      handled = false;

    if(!handled) 
      reportRunWarning("Bad Mail: " + key + ":" + whynot);
  }
  return(true);
}

//---------------------------------------------------------
// Procedure: OnConnectToServer()

bool ContactMgrV20::OnConnectToServer()
{
  registerVariables();  
  return(true);
}


//---------------------------------------------------------
// Procedure: Iterate()

bool ContactMgrV20::Iterate()
{
  AppCastingMOOSApp::Iterate();
  setCurrTimeUTC(m_curr_time);
  m_ledger.setCurrTimeUTC(m_curr_time);
  m_ledger.extrapolate();

  updateRanges();

  checkForEarlyWarnings();
  checkForCeaseWarnings();
  postEarlyWarningRadii();
  
  if(!m_hold_alerts_for_helm || m_helm_in_drive_noted) 
    checkForAlerts();

  postRadii();

  checkForNewRetiredContacts();
  postRangeReports();
  postSummaries();

  AppCastingMOOSApp::PostReport();
  return(true);
}

//---------------------------------------------------------
// Procedure: OnStartUp()

bool ContactMgrV20::OnStartUp()
{
  AppCastingMOOSApp::OnStartUp();

  // PID published to support uMemWatch or similar oversight
  Notify("PCONTACTMGRV20_PID", getpid());

   // Look for latitude, longitude initial datum
  double lat_origin, lon_origin;
  bool   ok1 = m_MissionReader.GetValue("LatOrigin", lat_origin);
  bool   ok2 = m_MissionReader.GetValue("LongOrigin", lon_origin);
  if(!ok1 || !ok2)
    reportConfigWarning("Lat or Lon Origin not set in *.moos file.");

  bool ok_init = m_ledger.setGeodesy(lat_origin, lon_origin);
  if(!ok_init)
    reportConfigWarning("Geodesy failed to initialize");
  
  setOwnship(m_host_community);
  if(m_ownship == "") 
    reportUnhandledConfigWarning("ownship name unknown, host_community unset");
  
  // Part 1: Set the basic configuration parameters.
  STRING_LIST sParams;
  m_MissionReader.EnableVerbatimQuoting(false);
  m_MissionReader.GetConfiguration(GetAppName(), sParams);
  
  STRING_LIST::reverse_iterator p;
  for(p=sParams.rbegin(); p!=sParams.rend(); p++) {
    string orig  = *p;
    string line  = stripBlankEnds(*p);
    string param = tolower(biteStringX(line, '='));
    string value = line;

    param = handleConfigDeprecations(param);
    
    // Alerts, filters, warnings, flags etc. are ContactMgrCore's
    bool handled = setParam(param, value);
    if(handled)
      continue;

    if(param == "range_report_timeout")
      handled = setNonNegDoubleOnString(m_range_report_timeout, value);
    else if(param == "range_report_maxsize")
      handled = setUIntOnString(m_range_report_maxsize, value);
    else if(param == "display_radii")
      handled = setBooleanOnString(m_display_radii, value);
    else if(param == "display_radii_id")
      handled = true; // deprecated
    else if(param == "post_closest_range") 
      handled = setBooleanOnString(m_post_closest_range, value);
    else if(param == "post_closest_relbng") 
      handled = setBooleanOnString(m_post_closest_relbng, value);
    else if(param == "post_all_ranges") 
      handled = setBooleanOnString(m_post_all_ranges, value);
    else if(param == "max_retired_history")
      handled = handleConfigMaxRetHist(value);
    else if(param == "contact_local_coords") 
      handled = true;  // deprecated, now always using contact ledger
	//handled = handleConfigCoords(value);
    else if(param == "recap_interval")
      handled = handleConfigRecapInterval(value);
    else if(param == "alert_range_color")
      handled = setColorOnString(m_alert_rng_color, value);
    else if(param == "cpa_range_color") 
      handled = true;
    //handled = setColorOnString(m_alert_rng_cpa_color, value);
    else if(param == "early_warning_radii")
      handled = setBooleanOnString(m_early_warning_radii, value);
    else if(param == "ewarn_radii_color")
      handled = setColorOnString(m_ewarn_radii_color, value);

    if(!handled)
      reportUnhandledConfigWarning(orig);
  }

  // Part 2: If alerts were configured, ensure they are complete/valid
  map<string, CMAlert>::iterator q;
  for(q=m_map_alerts.begin(); q!=m_map_alerts.end(); q++) {
    string  alert_id = q->first;
    CMAlert alert = q->second;
    if(!alert.valid())
      reportConfigWarning("Invalid alert config:" + alert_id);
  }

  registerVariables();
  return(true);
}

//------------------------------------------------------------
// Procedure: registerVariables()

void ContactMgrV20::registerVariables()
{
  AppCastingMOOSApp::RegisterVariables();

  Register("NODE_REPORT", 0);
  Register("BCM_DISPLAY_RADII", 0);
  Register("BCM_ALERT_REQUEST", 0);
  Register("BCM_REPORT_REQUEST", 0);
  Register("NAV_X", 0);
  Register("NAV_Y", 0);
  Register("NAV_SPEED", 0);
  Register("NAV_HEADING", 0);
  Register("IVPHELM_STATE", 0);

  if(m_disable_var != "")
    Register(m_disable_var, 0);
  if(m_enable_var != "")
    Register(m_enable_var, 0);
  if(m_expunge_var != "")
    Register(m_expunge_var, 0);
}

//---------------------------------------------------------
// Procedure: handleMailReportRequest()
//   Example: BCM_REPORT_REQUEST = var=BCM_CONTACTS_85, range=85,
//                                 reason=teaming
//      Note: The reason field is allowed but not processed.
//            It is allowed to support apps such as alogmtask that
//            want to grab report variables related to task teaming

void ContactMgrV20::handleMailReportRequest(string str, string src)
{
  bool ok = true;
  
  string moos_var;
  double range = -1;
  string group;
  string vtype;
  string reason;
  bool   refresh = false;
  vector<string> svector = parseString(str, ',');
  for(unsigned int i=0; i<svector.size(); i++) {
    string param = tolower(biteStringX(svector[i], '='));
    string value = svector[i];
    if((param == "var") && !strContainsWhite(value) && (value != ""))
      moos_var = value;
    else if((param == "range") && isNumber(value))
      range = atof(value.c_str());
    else if(param == "group")
      group = value;
    else if(param == "type")
      vtype = value;
    else if(param == "refresh")
      refresh = (tolower(value) == "true");
    else if(param == "reason")
      reason = value;
    else
      ok = false;
  }

  if(!ok || (moos_var == "") || (range < 0)) {
    string msg = "Failed REPORT_REQUEST: " + str;
    reportEvent(msg);
    reportRunWarning(msg);
  }    
  else {
    string msg = "New BCM_REPORT_REQUEST (src=" + src + "): " + str;
    reportEvent(msg);
    // If this is a new request, make new map entry for all fields
    if(!m_map_rep_range.count(moos_var) ||
       m_map_rep_range[moos_var] != range) {
      m_map_rep_range[moos_var] = range;
      m_map_rep_group[moos_var] = group;
      m_map_rep_vtype[moos_var] = vtype;
      m_map_rep_contacts[moos_var] = "";
      m_map_rep_refresh[moos_var] = refresh;
    }
    // If this is a repeat request (same moos_var and range) then
    // just update the current time.
    m_map_rep_reqtime[moos_var] = m_curr_time;
    m_map_rep_refresh[moos_var] = refresh;
  }
}

//---------------------------------------------------------
// Procedure: handleMailAlertRequest()
//    Format: BCM_ALERT_REQUEST = "id=avdcol_,
//            on_flag=CONTACT_INFO=name=$[VNAME] # contact=$[VNAME], 
//            alert_range=80, cpa_range=84
                  
void ContactMgrV20::handleMailAlertRequest(string value, string src)
{
  m_alert_requests_received++;

  string msg = "New BCM_ALERT_REQUEST (src=" + src + "): " + value;
  reportEvent(msg);

  bool ok = handleConfigAlert(value, src);
  if(!ok)
    reportRunWarning("Unhandled Alert Request: " + value);   
}

//--------------------------------------------------------------------
// Procedure: handleConfigDeprecations()

string ContactMgrV20::handleConfigDeprecations(string param)
{
  param = tolower(param);  

  string new_param = param;

  if(param == "alert_range")
    reportConfigWarning(param + " is deprecated");
  else if(param == "alert_cpa_range")
    reportConfigWarning(param + " is deprecated");
  else if(param == "default_alert_range_color")
    new_param = "alert_range_color";
  else if(param == "default_cpa_range_color")
    new_param = "cpa_range_color";
  else if(param == "contacts_recap_interval")
    new_param = "recap_interval";
  
  if(param != new_param)
    reportConfigWarning(param + " is deprecated. Use instead: " + new_param);

  return(new_param);
}


//---------------------------------------------------------
// Procedure: handleConfigMaxRetHist()
//      Note: Enforces the unsigned int to be in range [1,50]

bool ContactMgrV20::handleConfigMaxRetHist(string str)
{
  bool ok = setUIntOnString(m_max_retired_hist, str);
  if(!ok)
    return(false);

  if((m_max_retired_hist > 50) || (m_max_retired_hist < 1)) {
    string msg = "max_retired_hist should be in range [1,50]";
    reportConfigWarning(msg);
  }
  
  if(m_max_retired_hist > 50)
    m_max_retired_hist = 50;
  if(m_max_retired_hist < 1)
    m_max_retired_hist = 1;

  return(true);
}

//---------------------------------------------------------
// Procedure: handleConfigRecapInterval()

bool ContactMgrV20::handleConfigRecapInterval(string str)
{
  if(tolower(str) == "off") {
    m_contacts_recap_interval = -1;
    return(true);
  }

  return(setPosDoubleOnString(m_contacts_recap_interval, str));
}

//---------------------------------------------------------
// Procedure: pruneRangeReports()

void ContactMgrV20::pruneRangeReports()
{
  // Part 1: Identify the oldest report
  string oldest_varname;
  double oldest_age = 0;
  
  map<string, double>::iterator p;
  for(p=m_map_rep_reqtime.begin(); p!=m_map_rep_reqtime.end(); p++) {
    string varname = p->first;
    double reqtime = p->second;
    double age = m_curr_time - reqtime;

    if(age < m_range_report_timeout)
      continue;
    
    if((oldest_varname == "") || (age > oldest_age)) {
      oldest_varname = varname;
      oldest_age = age;
    }
  }

  if(oldest_varname == "")
    return;
  
  // Part 2: Remove from memory the oldest report
  m_map_rep_range.erase(oldest_varname);
  m_map_rep_reqtime.erase(oldest_varname);
  m_map_rep_group.erase(oldest_varname);
  m_map_rep_vtype.erase(oldest_varname);
  m_map_rep_contacts.erase(oldest_varname);
  m_map_rep_refresh.erase(oldest_varname);
}


//---------------------------------------------------------
// Procedure: postRangeReports()

void ContactMgrV20::postRangeReports()
{
  // Part 1: Ensure requested range reports don't grow unbounded
  if(m_map_rep_range.size() > m_range_report_maxsize)
    pruneRangeReports();
  
  // Part 2: For each report (varname), figure out which contacts
  //         satisfy the range threshold for that report
  //         Also check contact group name if report specifies
  //         Also check contact vehicle type if report specifies
  map<string, double>::iterator p;
  for(p=m_map_rep_range.begin(); p!=m_map_rep_range.end(); p++) {
    string varname = p->first;
    double rthresh = p->second;

    // Part 2A: Get the list of contacts for this report
    string contacts;   

    vector<string> vnames = m_ledger.getVNames();
    for(unsigned int i=0; i<vnames.size(); i++) {
      string vname = vnames[i];

      // Part 2AA: If report specifies group, check contact for match
      bool group_match = true;
      if(m_map_rep_group[varname] != "") { 
	if(tolower(m_map_rep_group[varname]) !=
	   tolower(m_ledger.getGroup(vname)))
	  group_match = false;
      }

      // Part 2AB: If report specifies vtype, check contact for match
      bool vtype_match = true;
      if(m_map_rep_vtype[varname] != "") {
	if(tolower(m_map_rep_vtype[varname]) !=
	   tolower(m_ledger.getType(vname)))
	  vtype_match = false;
      }

      // Part 2AC: Check if the range is satisfied
      bool range_sat = false;
      double now_range = m_map_node_ranges_extrap[vname];
      //double now_range = q->second;
      if(now_range < rthresh) 
	range_sat = true;

      if(group_match && vtype_match && range_sat) {
	if(contacts != "")
	  contacts += ",";
	contacts += vname;
      }
    }
    // Part 2B: If refresh requested or the report is different, then post it!
    if(m_map_rep_refresh[varname] || (contacts != m_map_rep_contacts[varname])) {
      Notify(varname, contacts);
      m_map_rep_contacts[varname] = contacts;
      m_map_rep_refresh[varname]  = false;
    }
  }
}

//---------------------------------------------------------
// Procedure: postSummaries()

void ContactMgrV20::postSummaries()
{
  // ==========================================================
  // Part 1: Handle the contacts_retired list
  // ==========================================================
  string contacts_retired;
  list<string>::iterator q;
  for(q=m_contacts_retired.begin(); q!=m_contacts_retired.end(); q++) {
    string this_contact = *q;
    if(contacts_retired != "")
      contacts_retired += ",";
    contacts_retired += this_contact;
  }

  if(m_prev_contacts_retired != contacts_retired) {
    Notify("CONTACTS_RETIRED", contacts_retired);
    m_prev_contacts_retired = contacts_retired;
  }

  // Limit the size of the list so this doesn't also grow unbounded.
  // Prune after posting so all newly retired get >=1  posting
  while(m_contacts_retired.size() > m_max_retired_hist)
    m_contacts_retired.pop_back();

  // ==========================================================
  // Part 2: Everything else
  // ==========================================================
  string contacts_list;
  string contacts_alerted;
  string contacts_recap;

  string closest_contact;
  double closest_range = 0;
  double closest_relbng = 0;

  list<double> ranges;

  vector<string> vnames = m_ledger.getVNames();
  for(unsigned int i=0; i<vnames.size(); i++) {
    string     contact_name = vnames[i];
    NodeRecord node_record  = m_ledger.getRecord(contact_name);

    if(contacts_list != "")
      contacts_list += ",";
    contacts_list += contact_name;

    double age   = m_curr_time - node_record.getTimeStamp();
    double range = m_map_node_ranges_actual[contact_name];

    ranges.push_front(range);
    
    // Update who is the closest contact and it's range
    if((closest_contact == "") || (range < closest_range)) {
      closest_contact = contact_name;
      closest_range   = range;

      double cnx = m_ledger.getX(contact_name);
      double cny = m_ledger.getY(contact_name);
      closest_relbng  = relBearing(m_osx, m_osy, m_osh, cnx, cny);
    }
	
    if(contacts_recap != "")
      contacts_recap += " # ";
    contacts_recap += "vname=" + contact_name;
    contacts_recap += ",range=" + doubleToString(range, 2);
    contacts_recap += ",age=" + doubleToString(age, 2);
  }
  
  if((closest_contact != "") && m_post_closest_range) {
    // Round to integer and only post when changed, to reduce postings
    long int closest_range_int = closest_range;
    closest_range = (double)(closest_range_int);
    if(closest_range != m_prev_closest_range) {
      Notify("CONTACT_CLOSEST_RANGE", closest_range);
      m_prev_closest_range = closest_range;
    }
  }

  // Added mikerb Oct 15, 2023
  if((closest_contact != "") && m_post_closest_relbng) {
    // Round to integer and only post when changed, to reduce postings
    long int closest_relbng_int = closest_relbng;
    closest_relbng = (double)(closest_relbng_int);
    if(closest_relbng != m_prev_closest_relbng) {
      Notify("CONTACT_CLOSEST_RELBNG", closest_relbng_int);
      m_prev_closest_relbng = closest_relbng;
    }
  }
  // End Added mikerb Oct 15, 2023

  if((m_prev_contact_closest != closest_contact) &&
     (closest_contact != "")){
    Notify("CONTACT_CLOSEST", closest_contact);
    Notify("CONTACT_CLOSEST_TIME", m_curr_time);
    m_prev_contact_closest = closest_contact;
  }

  
  unsigned int contacts_count = m_par.getAlertedGroupCount(true);
  if(contacts_count != m_prev_contacts_count) {
    Notify("CONTACTS_COUNT", (double)(contacts_count));
    m_prev_contacts_count = contacts_count;
  }

  if(m_prev_contacts_list != contacts_list) {
    Notify("CONTACTS_LIST", contacts_list);
    m_prev_contacts_list = contacts_list;
  }

  contacts_alerted = m_par.getAlertedGroup(true);
  if(m_prev_contacts_alerted != contacts_alerted) {
    Notify("CONTACTS_ALERTED", contacts_alerted);
    m_prev_contacts_alerted = contacts_alerted;
  }

  double time_since_last_recap = m_curr_time - m_contacts_recap_posted;

  // recaps may be configured "off" and the interval would be -1.
  if((m_contacts_recap_interval > 0) &&
     (time_since_last_recap > m_contacts_recap_interval)) {
    m_contacts_recap_posted = m_curr_time;
    Notify("CONTACTS_RECAP", contacts_recap);
    m_prev_contacts_recap = contacts_recap;
  }

  //============================================================
  // Build the CONTACT_RANGES posting

  if(m_post_all_ranges) {
    string ranges_report;
    ranges.sort();
    list<double>::iterator p2;
    for(p2=ranges.begin(); p2!=ranges.end(); p2++) {
      if(ranges_report != "")
	ranges_report += ",";
      double range = *p2;
      ranges_report += doubleToStringX(range,1);
    }
    Notify("CONTACT_RANGES", ranges_report);
  }

}


//---------------------------------------------------------
// Procedure: postEarlyWarningRadii()

void ContactMgrV20::postEarlyWarningRadii()
{
  // Sanity check - if no early warning or drawing enabled 
  if(!m_early_warning_radii || (m_early_warning_time <= 0))
    return;

  vector<string> vnames = m_ledger.getVNames();
  for(unsigned int i=0; i<vnames.size(); i++) {
    string contact = vnames[i];

    if(m_map_range_will_warn.count(contact) == 0)
      continue;
    
    double rng_ewarn = m_map_range_will_warn[contact];
    if(rng_ewarn <= 0)
      continue;
    
    XYCircle circle(m_osx, m_osy, rng_ewarn);
    circle.set_label("xewarn_" + contact);
    circle.set_label_color("gray50");
    circle.set_color("edge", m_ewarn_radii_color);
    circle.set_vertex_size(0);
    circle.set_edge_size(1);
    circle.set_active(true);
    circle.set_duration(3);
    circle.set_time(m_curr_time);
    string str = circle.get_spec();
    Notify("VIEW_CIRCLE", str);
  }
}

//---------------------------------------------------------
// Procedure: postRadii()

void ContactMgrV20::postRadii(bool active)
{
  // Sanity check 1: Check if all radii displaying is disabled
  if(!m_display_radii)
    return;
  // Sanity check 2: Check that there is at lease one alert
  if(m_map_alerts.size() == 0)
    return;

  double max_alert_range = getMaxAlertRange();

  if(!active || (max_alert_range <= 0) || 
     (m_alert_rng_color == "invisible") ||
     (m_alert_rng_color == "empty") ||
     (m_alert_rng_color == ""))
    active = false;
  
  XYCircle circle(m_osx, m_osy, max_alert_range);
  circle.set_label("alert_" + m_ownship);
  circle.set_color("edge", m_alert_rng_color);
  circle.set_vertex_size(0);
  circle.set_edge_size(1);
  circle.set_active(active);
  circle.set_duration(3);
  circle.set_time(m_curr_time);
  string s1 = circle.get_spec();
  Notify("VIEW_CIRCLE", s1);
}

//---------------------------------------------------------
// Procedure: buildReport()
//      Note: A virtual function of the AppCastingMOOSApp superclass, 
//            conditionally invoked if either a terminal or appcast 
//            report is needed.
//
// Alert Configurations (2):
// ---------------------
// Alert ID = avd
//   VARNAME   = CONTACT_INFO
//   PATTERN   = name=$[VNAME] # contact=$[VNAME]
//   RANGE     = 1000
//   CPA_RANGE = 1200
//
// Alert ID = trail
//   VARNAME   = TRAIL_INFO
//   PATTERN   = name=trail_$[VNAME] # contact=$[VNAME]
//   RANGE     = 600
//   CPA_RANGE = 750
//
// Alert Status Summary
// ----------------------
//        List: henry
//     Alerted: 
//     Retired: 
//       Recap: vname=henry,range=105.85,age=1.75
//  
// Contact Status Summary:
// ---------------------- 
//   Contact   Range    Alerts   Alerts  Alerts    Alerts
//                      Total    Active  Resolved  Active
//   -------   -----    -------  ------  --------  ------
//   gilda     188.2    4        3       1         (a,b)
//   henry     19.0     0        0       0         
//   ike       65.9     23       12      11        (a)
// 
//
// Events (Last 5):
// ---------------------
// 203.1  CONTACT_INFO = name=gilda # contact=gilda
// 143.2  CONTACT_INFO = name=gilda # contact=gilda
//  43.2  CONTACT_INFO = name=gilda # contact=gilda

bool ContactMgrV20::buildReport()
{
  //=================================================================
  // Part 1: Header Content
  //=================================================================
  string alert_count = uintToString(m_map_alerts.size());
  string bcm_req_received = uintToString(m_alert_requests_received);
  string max_age = doubleToStringX(m_contact_max_age,2);
  string reject_range = "off";

  string os_group = m_os_group;
  if(os_group == "")
    os_group = "not specified";
  string os_type = m_os_type;
  if(os_type == "")
    os_type = "not specified";

  if(m_reject_range > 0)
    reject_range = doubleToStringX(m_reject_range,2);
  m_msgs << "Ownship Group:      " << os_group << endl;
  m_msgs << "Ownship Type:       " << os_type << endl;
  m_msgs << "Contact Max Age:    " << max_age << endl;
  m_msgs << "Reject Range:       " << reject_range << endl;
  m_msgs << "Early Warning Time: " << m_early_warning_time << endl;

  if(m_early_warning_time > 0) {
    string ew_radii = boolToString(m_early_warning_radii);
    m_msgs << "EarlyWarn Radii:    " << ew_radii << endl;
    m_msgs << "EarlyWarn EWFlags:  " << m_cease_warning_flags.size() << endl;
    m_msgs << "EarlyWarn CWFlags:  " << m_early_warning_flags.size() << endl;
    m_msgs << "AlertsDisabled:     " << stringSetToString(m_alerts_disabled) << endl;
    m_msgs << "max_alert_range:    " << doubleToStringX(getMaxAlertRange(),2) << endl;
  }
  
  if(m_disable_var != "")
    m_msgs << "Disable Var:        " << m_disable_var << endl;
  if(m_enable_var != "")
    m_msgs << "Enable Var:         " << m_enable_var << endl;
  m_msgs << "BCM_ALERT_REQUESTs: " << bcm_req_received << endl;
  m_msgs << "DisplayRadii:       " << boolToString(m_display_radii) << endl;
  m_msgs << endl; 

  //=================================================================
  // Part 2: Alert Configurations
  //=================================================================
  m_msgs << "Alert Configurations (" << alert_count << "):"  << endl;
  m_msgs << "---------------------" << endl;
  map<string, CMAlert>::iterator p;
  unsigned int acounter = 1;
  for(p=m_map_alerts.begin(); p!=m_map_alerts.end(); p++) {
    string alert_id = p->first;
    m_msgs << uintToString(acounter) << " ID: " << alert_id << endl;
    vector<string> alert_summary = p->second.getSummary();
    for(unsigned int i=0; i<alert_summary.size(); i++) {
      m_msgs << "  " << alert_summary[i] << endl;;
    }
    acounter++;
  }
  m_msgs << endl;

  //=================================================================
  // Part 3: Alert Status Summary
  //=================================================================
  m_msgs << "Alert Status Summary: " << endl;
  m_msgs << "----------------------" << endl;
  m_msgs << "       List: " << m_prev_contacts_list         << endl;
  m_msgs << "    Alerted: " << m_prev_contacts_alerted      << endl;
  m_msgs << "    Retired: " << m_prev_contacts_retired      << endl;
  m_msgs << "      Recap: " << m_prev_contacts_recap        << endl;
  m_msgs << endl;

  //=================================================================
  // Part 4: Contact Status Summary
  //=================================================================
  m_msgs << "Contact Status Summary:" << endl;
  m_msgs << "-----------------------" << endl;

  ACTable actab(4,4);
  actab.setColumnJustify(1, "right");
  actab << "Contact | Range  | Alerts  | Alerts ";
  actab << "        | Actual | Total   | Active ";
  actab.addHeaderLines();

  unsigned contacts_reported = 0;

  vector<string> vnames = m_ledger.getVNames();
  for(unsigned int i=0; i<vnames.size(); i++) {
    string  contact = vnames[i];

    contacts_reported++;
    if(contacts_reported < 8) {
      string range = doubleToString(m_map_node_ranges_actual[contact], 1);
      string alerts_total  = uintToString(m_par.getAlertsTotal(contact));
      string alerts_active = uintToString(m_par.getAlertsActive(contact));
      actab << contact << range << alerts_total << alerts_active;
    }
    else
      break;
  }
  m_msgs << actab.getFormattedString();

  //=================================================================
  // Part 5: Custom Contact Reports
  //=================================================================
  m_msgs << endl << endl;
  m_msgs << "Custom Contact Reports:" << endl;
  m_msgs << "-----------------------" << endl;

  ACTable actab2(5,5);
  actab2.setColumnJustify(1, "right");
  actab2 << "VarName | Range  | Group  | VType | Contacts   ";
  actab2.addHeaderLines();

  for(map<string,double>::iterator p=m_map_rep_range.begin();
      p!=m_map_rep_range.end(); p++) {
    string varname = p->first;
    string range = doubleToStringX(p->second,2);
    string group = m_map_rep_group[varname];
    string vtype = m_map_rep_vtype[varname];
    vector<string> svector = parseString(m_map_rep_contacts[varname], ',');
    string contacts = uintToString(svector.size());   
    actab2 << varname << range << group << vtype << contacts;
  }
  m_msgs << actab2.getFormattedString();
  
  return(true);
}

//---------------------------------------------------------
// Procedure: cmgrPost()
//      Note: ContactMgrCore's postings, posted right away

void ContactMgrV20::cmgrPost(const string& var, const string& sval)
{
  Notify(var, sval);
}

void ContactMgrV20::cmgrPost(const string& var, double dval)
{
  Notify(var, dval);
}

//---------------------------------------------------------
// Procedure: cmgrEvent()

void ContactMgrV20::cmgrEvent(const string& event)
{
  reportEvent(event);
}

//---------------------------------------------------------
// Procedure: cmgrConfigWarning()

void ContactMgrV20::cmgrConfigWarning(const string& warning)
{
  reportConfigWarning(warning);
}
