/*****************************************************************/
/*    NAME: Michael Benjamin                                     */
/*    ORGN: Dept of Mechanical Engineering, MIT, Cambridge MA    */
/*    FILE: ContactMgrCore.cpp                                   */
/*    DATE: Feb 24th 2010                                        */
/*    DATE: Nov 2026 Separated from ContactMgrV20                */
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

#include <cmath>
#include "ContactMgrCore.h"
#include "LinearExtrapolator.h"
#include "MBUtils.h"
#include "CPAEngine.h"
#include "NodeRecordUtils.h"
#include "RangeMark.h"
#include "VarDataPairUtils.h"
#include "MacroUtils.h"

using namespace std;

//---------------------------------------------------------
// Constructor()

ContactMgrCore::ContactMgrCore()
{
  // Configuration Variables
  m_max_contacts    = 500;
  m_contact_max_age = 60;
  m_alert_verbose   = false;
  m_decay_start     = 15;
  m_decay_end       = 30;
  m_reject_range    = 2000;

  m_early_warning_time = -1;

  // State Variables
  m_curr_utc = 0;
  m_osx = 0;
  m_osy = 0;
  m_osh = 0;
  m_osv = 0;

  m_hold_alerts_for_helm = false;
  m_helm_in_drive_noted  = false;

  m_ledger.setMaxContacts(0); // We handle culling on our own
}

//---------------------------------------------------------
// Procedure: setParam()

bool ContactMgrCore::setParam(string param, string value)
{
  param = tolower(param);

  bool handled = false;
  if(param == "alert")
    handled = handleConfigAlert(value, "general");
  else if(param == "ignore_name")
    handled = m_filter_set.addIgnoreName(value);
  else if(param == "match_name")
    handled = m_filter_set.addMatchName(value);
  else if(param == "ignore_group")
    handled = m_filter_set.addIgnoreGroup(value);
  else if(param == "match_group")
    handled = m_filter_set.addMatchGroup(value);
  else if(param == "ignore_type")
    handled = m_filter_set.addIgnoreType(value);
  else if(param == "match_type")
    handled = m_filter_set.addMatchType(value);
  else if(param == "match_region")
    handled = m_filter_set.addMatchRegion(value);
  else if(param == "ignore_region")
    handled = m_filter_set.addIgnoreRegion(value);
  else if(param == "strict_ignore")
    handled = m_filter_set.setStrictIgnore(value);
  else if(param == "max_contacts")
    handled = setUIntOnString(m_max_contacts, value);
  else if(param == "decay")
    handled = handleConfigDecay(value);
  else if(param == "reject_range")
    handled = handleConfigRejectRange(value);
  else if(param == "alert_verbose")
    handled = setBooleanOnString(m_alert_verbose, value);
  else if(param == "contact_max_age")
    handled = setPosDoubleOnString(m_contact_max_age, value);
  else if(param == "hold_alerts_for_helm")
    handled = setBooleanOnString(m_hold_alerts_for_helm, value);
  else if(param == "ownship_group") {
    setNonWhiteVarOnString(m_os_group, value);
    handled = m_filter_set.setOwnshipGroup(value);
  }
  else if(param == "ownship_type") {
    setNonWhiteVarOnString(m_os_type, value);
    handled = m_filter_set.setOwnshipType(value);
  }
  else if(param == "disable_var")
    handled = setNonWhiteVarOnString(m_disable_var, value);
  else if(param == "enable_var")
    handled = setNonWhiteVarOnString(m_enable_var, value);
  else if(param == "expunge_var")
    handled = setNonWhiteVarOnString(m_expunge_var, value);
  else if(param == "able_flag")
    handled = addVarDataPairOnString(m_able_flags, value);
  else if(param == "disable_flag")
    handled = addVarDataPairOnString(m_disable_flags, value);
  else if(param == "enable_flag")
    handled = addVarDataPairOnString(m_enable_flags, value);
  else if(param == "early_warning_flag")
    handled = addVarDataPairOnString(m_early_warning_flags, value);
  else if(param == "cease_warning_flag")
    handled = addVarDataPairOnString(m_cease_warning_flags, value);
  else if(param == "retire_flag")
    handled = addVarDataPairOnString(m_retire_flags, value);
  else if(param == "early_warning_time")
    handled = setNonNegDoubleOnString(m_early_warning_time, value);

  return(handled);
}

//---------------------------------------------------------
// Procedure: setOwnship()

void ContactMgrCore::setOwnship(string ownship)
{
  m_ownship = ownship;
  m_filter_set.addIgnoreName(m_ownship);
}

//---------------------------------------------------------
// Procedure: iterate()

void ContactMgrCore::iterate(double utc)
{
  m_curr_utc = utc;
  m_ledger.setCurrTimeUTC(m_curr_utc);
  m_ledger.extrapolate();

  updateRanges();

  checkForEarlyWarnings();
  checkForCeaseWarnings();

  if(!m_hold_alerts_for_helm || m_helm_in_drive_noted)
    checkForAlerts();

  checkForNewRetiredContacts();
}

//---------------------------------------------------------
// Procedure: getPostings()
//   Purpose: Hand over, and forget, the postings made since the
//            last call. Only used if cmgrPost() isn't overridden.

vector<VarDataPair> ContactMgrCore::getPostings()
{
  vector<VarDataPair> postings;
  postings.swap(m_postings);
  return(postings);
}

//---------------------------------------------------------
// Procedure: getConfigWarnings()

vector<string> ContactMgrCore::getConfigWarnings()
{
  vector<string> warnings;
  warnings.swap(m_config_warnings);
  return(warnings);
}

//---------------------------------------------------------
// Procedure: cmgrPost()

void ContactMgrCore::cmgrPost(const string& var, const string& sval)
{
  m_postings.push_back(VarDataPair(var, sval));
}

void ContactMgrCore::cmgrPost(const string& var, double dval)
{
  m_postings.push_back(VarDataPair(var, dval));
}

//---------------------------------------------------------
// Procedure: cmgrConfigWarning()

void ContactMgrCore::cmgrConfigWarning(const string& warning)
{
  m_config_warnings.push_back(warning);
}

//---------------------------------------------------------
// Procedure: handleMailNodeReport()
//   Example: NAME=alpha,TYPE=KAYAK,UTC_TIME=1267294386.51,
//            X=29.66,Y=-23.49, LAT=43.825089,LON=-70.330030, 
//            SPD=2.00,HDG=119.06,YAW=119.05677,DEPTH=0.00,     
//            LENGTH=4.0,MODE=DRIVE

bool ContactMgrCore::handleMailNodeReport(string report,
					 string& whynot)
{  
  NodeRecord new_record = string2NodeRecord(report);

  // Sanity Check 1: Record must have a vname
  // string vname = tolower(new_record.getName());
  string vname = new_record.getName();
  if(vname == "") {
    whynot = "Empty vname";
    return(false);
  }

  // Sanity Check 2: Ingore ownship records. Should not need to muck
  // with case sensitivity, but we do it to be conservative. Downside
  // is that an ownship named "charlie" will regard a contact named
  // "CHARLIE" as ownship and reject.
  if(tolower(vname) == tolower(m_ownship))
    return(true);

  // ==============================================================
  // Part 1: Check against ContactMgr level ExclusionFilterSet 
  // ==============================================================
  // Note: If this contact is filtered, add it to the set of filtered
  // vnames. If for some reason it had passed this check previously,
  // and failed now, we to retire it promptly. This may occur for
  // example if the contact moved in/out of a match/ignore region.

  bool ok_filt = m_filter_set.filterCheck(new_record, m_osx, m_osy);
  if(!ok_filt) {
    m_filtered_vnames.insert(new_record.getName());
    return(true);
  }

  // ==============================================================
  // Part 2: Accept the contact, update data structures
  // ==============================================================
  bool prev_known_vehicle = m_ledger.hasVName(vname);

  string vname_added = m_ledger.processNodeRecord(new_record, whynot);
  //if(tolower(vname_added) != vname) {
  if(vname_added != vname) {
    whynot = "1:" + vname + "2:" + vname_added;
    return(false);
  }
  
  if(!prev_known_vehicle) {
    m_map_node_ranges_actual[vname] = 0;
    m_map_node_ranges_extrap[vname] = 0;
    m_map_node_cpa[vname] = 0;
    m_map_node_roc[vname] = 0;

    m_par.addVehicle(vname);

    if(m_alert_verbose) 
      cmgrPost("ALERT_VERBOSE", "new_contact="+vname);
  }
  
  // Check if the contact had been on the retired list (due to age)
  // and if so, resurrect it and remove it from the retired list.
  m_contacts_retired.remove(vname);

  return(true);
}

//---------------------------------------------------------
// Procedure: handleMailHelmState()
                     
void ContactMgrCore::handleMailHelmState(string value)
{
  if(value == "DRIVE")
    m_helm_in_drive_noted = true;
}

//---------------------------------------------------------
// Procedure: handleMailModEnableContact()
//   Example: XYZ_REENABLE_TARGET = 87993
//
// Notes: Re-enabling a behavior is a matter for the helm, by
//        way of a message to the BHV_ABLE_FILTER variable.
//        The contact manager can facilitate.
//        Assumed input is the ID of a contact.

bool ContactMgrCore::handleMailModEnableContact(string str,
					       string action)
{
  if((action != "enable") &&
     (action != "disable") &&
     (action != "expunge"))
    return(false);

  string contact_id;
  string msg;
  // First, handle special case, the value is just contact ID
  if(!strContains(str,"=") && !strContains(str,","))
    msg = "contact=" + str;
  else {
    contact_id = tokStringParse(str, "contact");
    if(contact_id != "")
      msg = "contact=" + contact_id;
    else { 
      string vsource = tokStringParse(str, "vsource");
      if(vsource != "")
	msg = "vsource=" + vsource;
    }
  }
  if(msg == "")
    return(false);

  msg += ",action=" + action;
  
  cmgrPost("BHV_ABLE_FILTER", msg);

  // Update list of (re)enabled contacts and post flags

  if((action == "enable") && (contact_id != "")) {
    addEnabledContact(contact_id);
    postFlags(m_able_flags);
    postFlags(m_enable_flags);
  }
  if((action == "disable") && (contact_id != "")) {
    addDisabledContact(contact_id);
    postFlags(m_able_flags);
    postFlags(m_disable_flags);
  }

  return(true);
}

//---------------------------------------------------------
// Procedure: handleConfigDecay()
//   Example: "30,60"

bool ContactMgrCore::handleConfigDecay(string decay_str)
{
  string left  = biteStringX(decay_str, ',');
  string right = decay_str;

  if(!isNumber(left) || !isNumber(right))
    return(false);
  
  double start = atof(left.c_str());
  double end   = atof(right.c_str());

  if((start >= 0) && (start <= end)) {
    m_decay_start = start;
    m_decay_end   = end;
    return(true);
  }  
  return(false);
}

//---------------------------------------------------------
// Procedure: handleConfigRejectRange()

bool ContactMgrCore::handleConfigRejectRange(string str)
{
  if(tolower(str) == "off") {
    m_reject_range = -1;
    return(true);
  }
  return(setPosDoubleOnString(m_reject_range, str));
}

//---------------------------------------------------------
// Procedure: handleConfigAlert()

bool ContactMgrCore::handleConfigAlert(string alert_str, string source)
{
  // Part 1: Get the alert id. Allow for an "empty" alert,
  // but call it "no_id".
  string alert_id = tokStringParse(alert_str, "id", ',', '=');
  if(alert_id == "")
    alert_id = "no_id";

  // Begin Apr 2025 mod to disable alerts after launch time
  if(strContains(tolower(alert_str), "action=disable")) {
    if(alert_id != "no_id")
      m_alerts_disabled.insert(alert_id);
    return(true);
  }
  if(strContains(tolower(alert_str), "action=enable")) {
    if(alert_id != "no_id")
      m_alerts_disabled.erase(alert_id);
    return(true);
  }
  // End Apr 2025 mod to disable alerts after launch time
  
  // Part 2: Add to the Platform Alert Record. If alert_id is
  // already known, it's just ignored.
  m_par.addAlertID(alert_id);

  // Part 3: Apply the source to the Alert
  if(source != "")
    m_map_alerts[alert_id].setAlertSource(source);
  
  // Part 4: Parse the configuration string
  string var, pattern;
  vector<string> svector = parseStringQ(alert_str, ',');
  for(unsigned int i=0; i<svector.size(); i++) {
    string left  = tolower(biteStringX(svector[i], '='));
    string right = svector[i];
    double dval  = atof(right.c_str());

    if(isQuoted(right))
      right = stripQuotes(right);

    bool ok = true;
    if(left == "var")
      var = right;
    else if((left == "val") || (left == "pattern")) 
      pattern = right;

    else if((left == "on_flag") || (left == "onflag")) 
      ok = m_map_alerts[alert_id].addAlertOnFlag(right);
    else if((left == "off_flag") || (left == "offflag"))
      ok = m_map_alerts[alert_id].addAlertOffFlag(right);
    else if((left == "alert_range") && isNumber(right))
      ok = m_map_alerts[alert_id].setAlertRange(dval);

    else if(left == "match_name")
      ok = m_map_alerts[alert_id].configFilter(left, right);
    else if(left == "ignore_name")
      ok = m_map_alerts[alert_id].configFilter(left, right);
    else if(left == "match_group")
      ok = m_map_alerts[alert_id].configFilter(left, right);
    else if(left == "ignore_group")
      ok = m_map_alerts[alert_id].configFilter(left, right);
    else if(left == "match_type")
      ok = m_map_alerts[alert_id].configFilter(left, right);
    else if(left == "ignore_type")
      ok = m_map_alerts[alert_id].configFilter(left, right);
    
    else if(left == "match_region")
      ok = m_map_alerts[alert_id].configFilter(left, right);
    else if(left == "ignore_region")
      ok = m_map_alerts[alert_id].configFilter(left, right);

    else if(left == "strict_ignore")
      ok = m_map_alerts[alert_id].configFilter(left, right);

    else if((left == "cpa_range") && isNumber(right))
      ok = m_map_alerts[alert_id].setAlertRangeFar(dval);
    else
      ok = false;

    
    if(!ok && (left != "id")) {
      cmgrConfigWarning("unhandled alert config component: " + left);
      return(false);
    }
  }

  // For backward compatibility sake we allow the user to specify an
  // on-flag with separate var,pattern fields.
  if((var != "") && (pattern != ""))
    m_map_alerts[alert_id].addAlertOnFlag(var + "=" + pattern);
  
  return(true);
}


//---------------------------------------------------------
// Procedure: checkForAlerts()
//   Purpose: Check each contact/alert pair and handle if the
//            alert condition changes.

void ContactMgrCore::checkForAlerts()
{
  //==============================================================
  // For each contact, check all alerts
  //==============================================================
  vector<string> vnames = m_ledger.getVNames();
  for(unsigned int i=0; i<vnames.size(); i++) {
    string     contact = vnames[i];
    NodeRecord record = m_ledger.getRecord(contact);
    
    //==============================================================
    // For each alert_id, check if alert should be posted for this contact
    //==============================================================
    map<string, CMAlert>::iterator q;
    for(q=m_map_alerts.begin(); q!=m_map_alerts.end(); q++) {
      string id = q->first;

      // Apr1725 mikerb: return/continue here if this id is disabled
      if(isAlertDisabled(id))
	continue;
      
      bool alert_applies = checkAlertApplies(contact, id);

      // If alert applies and currently not alerted, handle
      string transition;
      if(alert_applies && !m_par.getAlertedValue(contact, id)) {
        postOnAlerts(record, id);
	m_par.setAlertedValue(contact, id, true);
	transition = "off-->alerted";
      }
      else if(!alert_applies && m_par.getAlertedValue(contact, id)) {
        postOffAlerts(record, id);
	m_par.setAlertedValue(contact, id, false);
	transition = "alerted-->off";
      }

      string delta_str; 
      if(m_early_warning_time > 0) {
	if(m_map_utc_was_warned.count(contact) != 0) {
	  if(transition == "off-->alerted") {
	    double delta = m_curr_utc - m_map_utc_was_warned[contact];
	    delta_str = "after_warning=" + doubleToStringX(delta,2);
	  }
	}
      }
      
      
      if((m_alert_verbose) && (transition != "")) {
	string mvar = "ALERT_VERBOSE";
	string mval = "contact=" + contact;
	mval += ",alert_id=" + id;
	mval += "," + transition; 

	double alert_range = getAlertRange(id);
	double alert_range_cpa = getAlertRangeCPA(id);
	mval += ",alert_range=" + doubleToStringX(alert_range,1);
	mval += ",alert_range_cpa=" + doubleToStringX(alert_range_cpa,1);
	
	double range_actual = m_map_node_ranges_actual[contact];
	mval += ",range_actual=" + doubleToString(range_actual,1);	
	if(m_par.getAlertedValue(contact,id)) {
	  double range_cpa = m_map_node_cpa[contact];
	  mval += ",range_cpa=" + doubleToString(range_cpa,1);
	}
	if(delta_str != "")
	  mval += "," + delta_str;
	
	cmgrPost(mvar, mval);
      }
    }
  }
}


//---------------------------------------------------------
// Procedure: postWarningFlags()

void ContactMgrCore::postWarningFlags(const vector<VarDataPair>& flags,
				     string contact)
{
  for(unsigned int i=0; i<flags.size(); i++) {
    VarDataPair pair = flags[i];
    string moosvar = pair.get_var();
    string postval;
    
    // If posting is a double, just post. No macro expansion
    if(!pair.is_string()) {
      double dval = pair.get_ddata();
      postval = doubleToStringX(dval,4);
      cmgrPost(moosvar, dval);
    }
    // Otherwise if string posting, handle macro expansion
    else {
      postval = stripBlankEnds(pair.get_sdata());
      postval = macroExpand(postval, "CONTACT", contact);

      if(m_map_node_ranges_extrap.count(contact) != 0) {
	double rng = m_map_node_ranges_extrap[contact];
	postval = macroExpand(postval, "RNG_EXT", rng);
      }
      if(m_map_node_ranges_actual.count(contact) != 0) {
	double rng = m_map_node_ranges_actual[contact];
	postval = macroExpand(postval, "RNG", rng);
      }
      if(m_map_node_cpa.count(contact) != 0) {
	double cpa = m_map_node_cpa[contact];
	postval = macroExpand(postval, "CPA", cpa);
      }
      if(m_map_node_roc.count(contact) != 0) {
	double roc = m_map_node_roc[contact];
	postval = macroExpand(postval, "ROC", roc);
      }

      postval = macroExpand(postval, "UTC", m_curr_utc);

      if(postval != "")
        cmgrPost(moosvar, postval);
    }
    cmgrEvent(moosvar + "=" + postval);
  }
}

//---------------------------------------------------------
// Procedure: postRetireFlags()

void ContactMgrCore::postRetireFlags(const vector<VarDataPair>& flags,
				    string contact)
{
  for(unsigned int i=0; i<flags.size(); i++) {
    VarDataPair pair = flags[i];
    string moosvar = pair.get_var();
    string postval;
    
    // If posting is a double, just post. No macro expansion
    if(!pair.is_string()) {
      double dval = pair.get_ddata();
      postval = doubleToStringX(dval,4);
      cmgrPost(moosvar, dval);
    }
    // Otherwise if string posting, handle macro expansion
    else {
      postval = stripBlankEnds(pair.get_sdata());
      postval = macroExpand(postval, "CONTACT", contact);
      postval = macroExpand(postval, "UTC", m_curr_utc);
      if(postval != "")
        cmgrPost(moosvar, postval);
    }
    cmgrEvent(moosvar + "=" + postval);
  }
}

//---------------------------------------------------------
// Procedure: checkForCeaseWarnings()

void ContactMgrCore::checkForCeaseWarnings()
{
  vector<string> vnames = m_ledger.getVNames();
  for(unsigned int i=0; i<vnames.size(); i++) {
    string contact = vnames[i];

    // Check 1: If has been no early warning for this contact, skip
    if(m_map_range_was_warned.count(contact) == 0)
      continue;
    
    double contact_range_ext = m_map_node_ranges_extrap[contact]; 
    if(contact_range_ext > (m_map_range_was_warned[contact] * 1.05)) {
      m_map_range_was_warned.erase(contact);
      m_map_utc_was_warned.erase(contact);
      postWarningFlags(m_cease_warning_flags, contact);
    }
  }
}

//---------------------------------------------------------
// Procedure: checkForEarlyWarnings()

void ContactMgrCore::checkForEarlyWarnings()
{
  // Check 0: If early warnings disabled, just return now
  if(m_early_warning_time <= 0)
    return;
  
  vector<string> vnames = m_ledger.getVNames();

  for(unsigned int i=0; i<vnames.size(); i++) {
    string contact = vnames[i];
    
    // Check 1: If already warned about this contact, just skip now.
    if(m_map_range_was_warned.count(contact))
      continue;

    // Reset default to zero. This is used when/if drawing rng circle
    m_map_range_will_warn[contact] = 0;

    // Check 2: If no contact range is known, just skip now
    if(m_map_node_ranges_extrap.count(contact) == 0)
      continue;
    // Check 3: If no rate_of_closure known for this contact, skip
    if(m_map_node_roc.count(contact) == 0)
      continue;

    double contact_range = m_map_node_ranges_extrap[contact]; 
    double roc = m_map_node_roc[contact]; 
    
    // Check 4: If opening range to contact, just skip
    if(roc <= 0)
      continue;

    // Part 1: Determine the alert range for the contact. We want a warning
    // to be posted N seconds before the contact reaches this range. Where
    // N is m_early_warning_time.
    double max_alert_range = getMaxAlertRange();

    
    // Part 2: Determine Early warning range, based by rate_of_closure
    // and m_early_warning_time interval.
    if(max_alert_range > 0) {
      double rng_delta = roc * m_early_warning_time;
      double early_warning_rng = max_alert_range + rng_delta;
      m_map_range_will_warn[contact] = early_warning_rng;
      
      // Check 2: If contact inside warning range, warning warranted
      if(contact_range < early_warning_rng) {
	postWarningFlags(m_early_warning_flags, contact);
	m_map_range_was_warned[contact] = contact_range;
	m_map_utc_was_warned[contact] = m_curr_utc;
	m_map_range_will_warn[contact] = 0;
      }
    }
    else
      m_map_range_will_warn[contact] = 0;
  }    
}

//---------------------------------------------------------
// Procedure: checkForNewRetiredContacts()
//   Purpose: Check all contacts and see if any of them should be
//            moved onto the retired list based on contact_max_age.

void ContactMgrCore::checkForNewRetiredContacts()
{
  unsigned int starting_amt = m_ledger.size();
  
  //==============================================================
  // Part 1: Find new contacts that need to be retired.
  //==============================================================
  // To-be-retired set starts with the vnames flagged when receiving
  // node reports, that did not survive one of the filter criteria
  set<string> to_be_retired = m_filtered_vnames;

  vector<string> vnames = m_ledger.getVNames();
  for(unsigned int i=0; i<vnames.size(); i++) {
    string     contact = vnames[i];
    NodeRecord record  = m_ledger.getRecord(contact);

    // Possibly drop due to age
    double age = m_curr_utc - record.getTimeStamp();
    if(age > m_contact_max_age) 
      to_be_retired.insert(contact);

    // Possibly drop due to reject_range. 
    else if(m_map_node_ranges_actual.count(contact) && (m_reject_range > 0)) {
      // Reject range adjusted 5pct higher to avoid thrashing
      double adjusted_reject_range = m_reject_range * 1.05;
      if(m_map_node_ranges_actual[contact] > adjusted_reject_range) {
	to_be_retired.insert(contact);
      }
    }
  }

  //==============================================================
  // Part 1A: Contingent Culling based on staleness. If pruning is
  //          is still needed, remove based on lowere stale thresh.
  //==============================================================
  // Cull if needed
  if((to_be_retired.size() + m_max_contacts) < starting_amt) {
    vector<string> vnames = m_ledger.getVNames();
    for(unsigned int i=0; i<vnames.size(); i++) {
      string     contact = vnames[i];
      NodeRecord record  = m_ledger.getRecord(contact);
      
      // Possibly drop due to age
      double age = m_curr_utc - record.getTimeStamp();
      double lower_contact_max_age = m_contact_max_age / 2;
      if(age > lower_contact_max_age) 
	to_be_retired.insert(contact);      
    }
  }
  
  //==============================================================
  // Part 1B: Dire Contingent Culling. Keep only the N closest
  //==============================================================
  if((to_be_retired.size() + m_max_contacts) < starting_amt) {
    unsigned int to_cull = starting_amt - (to_be_retired.size() + m_max_contacts);
    list<string> ordered_contacts = getRangeOrderedContacts();

    unsigned int culled_so_far = 0;
    list<string>::reverse_iterator p;
    for(p=ordered_contacts.rbegin(); p!=ordered_contacts.rend(); p++) {
      string contact = *p;
      if(to_be_retired.count(contact) == 0) {
	to_be_retired.insert(contact);
	culled_so_far++;
      }
      if(culled_so_far >= to_cull)
	break;
    }
  }
  
  //==============================================================
  // Part 2: Go through the set of newly retired contacts and do
  //         (a) free up any memory associated with this contact
  //         (b) Merge newly retired contacts onto the front of 
  //         the list of previously retired contacts, since we
  //         always delete retired contacts from back of this list.
  //==============================================================
  set<string>::iterator q;
  for(q=to_be_retired.begin(); q!=to_be_retired.end(); q++) {
    string contact = *q;

    // (a) If contact currently in memory and will be removed,
    // then add to front of list of retired contacts

    if(m_ledger.hasVName(contact))
      m_contacts_retired.push_front(contact);

    // (b) Free up any memory associated with this contact
    m_ledger.clearNode(contact);
    m_map_node_ranges_actual.erase(contact);
    m_map_node_ranges_extrap.erase(contact);
    m_map_node_cpa.erase(contact);
    m_map_node_roc.erase(contact);
    m_par.removeVehicle(contact);

    postRetireFlags(m_retire_flags, contact);

    m_map_range_will_warn.erase(contact);
    m_map_range_was_warned.erase(contact);
    m_map_utc_was_warned.erase(contact);
  }

  // Part 3: Clear the set of filtered_vnames. This list should start
  // each iteration empty, with names added as node report mail is
  // processed. Now that we have used it here in this function to seed
  // the set of names to retire, it can be cleared now.
  m_filtered_vnames.clear();
}

//----------------------------------------------------------------
// Procedure: postOnAlerts()

void ContactMgrCore::postOnAlerts(NodeRecord record, string id)
{
  if(!knownAlert(id))
    return;

  vector<VarDataPair> pairs = getAlertOnFlags(id);
  for(unsigned int i=0; i<pairs.size(); i++)
    postAlert(record, pairs[i]);
  
}

//----------------------------------------------------------------
// Procedure: postOffAlerts()

void ContactMgrCore::postOffAlerts(NodeRecord record, string id)
{
  if(!knownAlert(id))
    return;

  vector<VarDataPair> pairs = getAlertOffFlags(id);
  for(unsigned int i=0; i<pairs.size(); i++)
    postAlert(record, pairs[i]);
  
}

//----------------------------------------------------------------
// Procedure: postAlert()

void ContactMgrCore::postAlert(NodeRecord record, VarDataPair pair)
{
  // Step 1: Get all potential macro info
  string x_str    = record.getStringValue("x");
  string y_str    = record.getStringValue("y");
  string lat_str  = record.getStringValue("lat");
  string lon_str  = record.getStringValue("lon");
  string spd_str  = record.getStringValue("speed");
  string hdg_str  = record.getStringValue("heading");
  string dep_str  = record.getStringValue("depth");
  string time_str = record.getStringValue("time");
  string name_str = record.getName();
  string type_str = record.getType();
  string group_str = record.getGroup();
  string vsource_str = record.getVSource();
  
  // Step 2: Get var to post, and expand macros if any
  string var = pair.get_var();

  var = findReplace(var, "$[X]", x_str);
  var = findReplace(var, "$[Y]", y_str);
  var = findReplace(var, "$[LAT]", lat_str);
  var = findReplace(var, "$[LON]", lon_str);
  var = findReplace(var, "$[SPD}", spd_str);
  var = findReplace(var, "$[HDG]", hdg_str);
  var = findReplace(var, "$[DEP]", dep_str);
  var = findReplace(var, "$[VNAME]", name_str);
  var = findReplace(var, "$[VTYPE]", type_str);
  var = findReplace(var, "$[UTIME]", time_str);
  var = findReplace(var, "$[UTC]", time_str);
  //var = findReplace(var, "%[VNAME]", tolower(name_str));
  var = findReplace(var, "%[VNAME]", name_str);
  var = findReplace(var, "%[VTYPE]", tolower(type_str));

  // Step 3: If posting is numerical just do it and be done!
  if(!pair.is_string()) {
    double dval = pair.get_ddata();
    cmgrPost(var, dval);
    cmgrEvent(var + "=" + doubleToStringX(dval, 3));
    return;
  }

  // Step 4: Otherwise handle a string posting with macros
  
  string msg = pair.get_sdata();
  
  msg = findReplace(msg, "$[X]", x_str);
  msg = findReplace(msg, "$[Y]", y_str);
  msg = findReplace(msg, "$[LAT]", lat_str);
  msg = findReplace(msg, "$[LON]", lon_str);
  msg = findReplace(msg, "$[SPD}", spd_str);
  msg = findReplace(msg, "$[HDG]", hdg_str);
  msg = findReplace(msg, "$[DEP]", dep_str);
  msg = findReplace(msg, "$[VNAME]", name_str);
  msg = findReplace(msg, "$[VTYPE]", type_str);
  msg = findReplace(msg, "$[GROUP]", group_str);
  msg = findReplace(msg, "$[UTIME]", time_str);
  //msg = findReplace(msg, "%[VNAME]", tolower(name_str));
  msg = findReplace(msg, "%[VNAME]", name_str);
  msg = findReplace(msg, "%[VTYPE]", tolower(type_str));
  msg = findReplace(msg, "%[GROUP]", tolower(group_str));

  cmgrPost(var, msg);
  cmgrEvent(var + "=" + msg);
}

//---------------------------------------------------------
// Procedure: updateRanges()

void ContactMgrCore::updateRanges()
{
  double alert_range_cpa_time = 36000; // 10 hours

  vector<string> vnames = m_ledger.getVNames();
  for(unsigned int i=0; i<vnames.size(); i++) {
    string     contact = vnames[i];
    NodeRecord record = m_ledger.getRecord(contact);

    // First figure out the raw range to the contact
    //double cnx = record.getX();
    //double cny = record.getY();
    //double cnh = record.getHeading();
    //double cns = record.getSpeed();
    //double cnt = record.getTimeStamp();

    double cnx = m_ledger.getX(contact);
    double cny = m_ledger.getY(contact);
    double cnh = m_ledger.getHeading(contact);
    double cns = m_ledger.getSpeed(contact);
    double cnt = m_ledger.getUTC(contact);

    // #1 Determine and store the actual point-to-point range between
    // ownship and the last absolute known position of the contact
    double range_actual = hypot((m_osx - cnx), (m_osy - cny));
    m_map_node_ranges_actual[contact] = range_actual;

    // #2 Determine and store the extrapolated range between ownship
    // and the contact position determined by its last known range and
    // extrapolation.
    LinearExtrapolator linex;
    linex.setDecay(m_decay_start, m_decay_end);
    linex.setPosition(cnx, cny, cns, cnh, cnt);

    double extrap_x = cnx;
    double extrap_y = cny;
    double range_extrap = range_actual;

    bool ok = linex.getPosition(extrap_x, extrap_y, m_curr_utc);
    if(ok) {
      cnx = extrap_x;
      cny = extrap_y;
      range_extrap = hypot((m_osx - cnx), (m_osy - cny));
    }
    m_map_node_ranges_extrap[contact] = range_extrap;

    // #3 Determine and store the cpa range between ownship and the
    // contact position determined by the contact's extrapolated position
    // and it's last known heading and speed.
    CPAEngine engine(cny, cnx, cnh, cns, m_osy, m_osx);      
    double range_cpa = engine.evalCPA(m_osh, m_osv,
				      alert_range_cpa_time);
    double roc = engine.evalROC(m_osh, m_osv);
    m_map_node_cpa[contact] = range_cpa;
    m_map_node_roc[contact] = roc;
  }
}

//---------------------------------------------------------
// Procedure: checkAlertApplies()

bool ContactMgrCore::checkAlertApplies(string contact, string id) 
{
  //=========================================================
  // Part 1: Sanity checks
  //=========================================================
  // Return false immediately if alert or contact are unknown
  if(!knownAlert(id))
    return(false);
  if(!m_map_alerts.at(id).valid())
    return(false);
  if(!m_ledger.hasVName(contact))
    return(false);

  // Return false immediately if age of node record exceeds max age
  NodeRecord record = m_ledger.getRecord(contact);
  double age = m_curr_utc - record.getTimeStamp();
  if(age > m_contact_max_age)
    return(false);

  //=========================================================
  // Part 2: Apply the alert's exclusion filter
  //=========================================================
  bool okfilter = m_map_alerts.at(id).filterCheck(record, m_osx, m_osy);
  if(!okfilter)
    return(false);

  //=========================================================
  // Part 3: Check range and cpa_range of ownship to contact
  //=========================================================
  double alert_range     = getAlertRange(id);
  double alert_range_cpa = getAlertRangeCPA(id);

  // If alert range is not positive, regarded as having the range
  // criteria OFF. Likely this alert depends only on the region.
  if(alert_range > 0) {
    double contact_range_abs = m_map_node_ranges_actual[contact]; 
    double contact_range_cpa = m_map_node_cpa[contact];    
    
    if(contact_range_abs > alert_range_cpa)
      return(false);
    
    if((contact_range_abs > alert_range) && (contact_range_cpa > alert_range))
      return(false);
  }
  
  // If none of the above no-apply conditions hold, return true!
  return(true);  
}

//---------------------------------------------------------
// Procedure: knownAlert()

bool ContactMgrCore::knownAlert(string alert_id) const
{
  map<string, CMAlert>::const_iterator p;
  p = m_map_alerts.find(alert_id);
  if(p == m_map_alerts.end())
    return(false);
  return(true);
}

//---------------------------------------------------------
// Procedure: getAlertRange()
//      Note: Use this method to access map, not map directly. This 
//            allows intelligent defaults to be applied if missing key.

double ContactMgrCore::getAlertRange(string alert_id) const
{
  if(!knownAlert(alert_id))
    return(-1);
  return(m_map_alerts.at(alert_id).getAlertRange());
}

//---------------------------------------------------------
// Procedure: getAlertRangeCPA()
//      Note: Use this method to access map, not map directly. This 
//            allows intelligent defaults to be applied if missing key.

double ContactMgrCore::getAlertRangeCPA(string alert_id) const
{
  if(!knownAlert(alert_id))
    return(-1);
  return(m_map_alerts.at(alert_id).getAlertRangeFar());
}

//---------------------------------------------------------
// Procedure: getMaxAlertRange()
//      Note: For all current alerts, determine the largest range.
//            If an alert has two ranges, use the larger of two.

double ContactMgrCore::getMaxAlertRange() const
{
  double max_range = -1;

  map<string, CMAlert>::const_iterator p;
  for(p=m_map_alerts.begin(); p!=m_map_alerts.end(); p++) {
    string  alert_id = p->first;
    if(isAlertDisabled(alert_id))
      continue;
    
    CMAlert alert = p->second;
    double range = alert.getAlertRangeFar();
    if(range > max_range)
      max_range = range;
  }

  return(max_range);
}

//---------------------------------------------------------
// Procedure: getAlertOnFlags()

vector<VarDataPair> ContactMgrCore::getAlertOnFlags(string alert_id) const
{
  vector<VarDataPair> empty;
  if(!knownAlert(alert_id))
    return(empty);
  return(m_map_alerts.at(alert_id).getAlertOnFlags());
}

//---------------------------------------------------------
// Procedure: getAlertOffFlags()

vector<VarDataPair> ContactMgrCore::getAlertOffFlags(string alert_id) const
{
  vector<VarDataPair> empty;
  if(!knownAlert(alert_id))
    return(empty);
  return(m_map_alerts.at(alert_id).getAlertOffFlags());
}

//---------------------------------------------------------
// Procedure: getRangeOrderedContacts()
//   Purpose: Get a list of all contact names, sorted by range,
//            with the closest contact at the front of the list.

list<string> ContactMgrCore::getRangeOrderedContacts() const
{
  list<string> to_be_returned_ordered_list;

  list<RangeMark> working_list;
  map<string,double>::const_iterator p;
  for(p=m_map_node_ranges_actual.begin(); p!=m_map_node_ranges_actual.end(); p++) {
    string vname = p->first;
    double range = p->second;
    RangeMark range_mark(vname, range);
    working_list.push_front(range_mark);
  }

  working_list.sort();

  list<RangeMark>::iterator q;
  for(q=working_list.begin(); q!=working_list.end(); q++) {
    RangeMark range_mark = *q;
    to_be_returned_ordered_list.push_back(range_mark.getContact());
  }

  return(to_be_returned_ordered_list);
}

//---------------------------------------------------------
// Procedure: addDisabledContact()
//   Purpose: Maintain a list of contacts disabled. This list
//            used primarily for informational purposes, and
//            its size can be used as a way of confirming that
//            a disabled contact has been added.
//      Note: We are always paranoid about memory growth, so
//            this contact list is limited to 25 most recent
//            contacts.

void ContactMgrCore::addDisabledContact(string contact_id)
{
  // If contact on enabled list, remove it
  m_enabled_contacts.remove(contact_id);

  // If contact already is on list, remove so we can put at front
  m_disabled_contacts.remove(contact_id);
  m_disabled_contacts.push_front(contact_id);

  // If list is too large, removed the oldest.
  if(m_disabled_contacts.size() > 25)
    m_disabled_contacts.pop_back();
}

//---------------------------------------------------------
// Procedure: addEnabledContact()
//      Note: See comments for addDisabledContact()

void ContactMgrCore::addEnabledContact(string contact_id)
{
  // If contact on disabled list, remove it
  m_disabled_contacts.remove(contact_id);

  // If contact already is on list, remove so we can put at front
  m_enabled_contacts.remove(contact_id);
  m_enabled_contacts.push_front(contact_id);

  // If list is too large, removed the oldest.
  if(m_enabled_contacts.size() > 250)
    m_enabled_contacts.pop_back();
}

//------------------------------------------------------------
// Procedure: postFlags()
//   Purpose: For any set of flags, post each. For flags that
//            contain a macro, expand the macro first.

void ContactMgrCore::postFlags(const vector<VarDataPair>& flags)
{
  for(unsigned int i=0; i<flags.size(); i++) {
    VarDataPair pair = flags[i];
    string moosvar = pair.get_var();
    string postval;

    // If posting is a double, just post. No macro expansion
    if(!pair.is_string()) {
      double dval = pair.get_ddata();
      postval = doubleToStringX(dval,4);
      cmgrPost(moosvar, dval);
    }
    // Otherwise if string posting, handle macro expansion
    else {
      postval = stripBlankEnds(pair.get_sdata());
      postval = expandMacros(postval);
      if(postval != "")
        cmgrPost(moosvar, postval);
    }
    cmgrEvent(moosvar + "=" + postval);
  }
}

//------------------------------------------------------------
// Procedure: expandMacros()
//   Purpose: For any set of flags, post each. For flags that
//            contain a macro, expand the macro first.

string ContactMgrCore::expandMacros(string sdata) const
{
  // Handle $[RECENT_DISABLED]
  if(m_disabled_contacts.size() > 0) {
    string most_recently_disabled;
    most_recently_disabled = m_disabled_contacts.front();
    sdata = macroExpand(sdata, "RECENT_DISABLED", most_recently_disabled);
  }

  // Handle $[RECENT_ENABLED]
  if(m_enabled_contacts.size() > 0) {
    string most_recently_enabled;
    most_recently_enabled = m_enabled_contacts.front();
    sdata = macroExpand(sdata, "RECENT_ENABLED", most_recently_enabled);
  }

  // Handle $[TOTAL_DISABLED]
  unsigned int total_disabled = m_disabled_contacts.size();
  sdata = macroExpand(sdata, "TOTAL_DISABLED", total_disabled);

  // Handle $[ALL_DISABLED] (check first before building content)
  if(strContains(sdata, "ALL_DISABLED")) {
    string all_disabled = stringListToString(m_disabled_contacts);
    sdata = macroExpand(sdata, "ALL_DISABLED", all_disabled);
  }

  return(sdata);
}

//------------------------------------------------------------
// Procedure: isAlertDisabled()

bool ContactMgrCore::isAlertDisabled(string alert_id) const
{
  return(m_alerts_disabled.count(alert_id) != 0);
}

