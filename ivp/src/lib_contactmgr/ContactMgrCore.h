/*****************************************************************/
/*    NAME: Michael Benjamin                                     */
/*    ORGN: Dept of Mechanical Engineering, MIT, Cambridge MA    */
/*    FILE: ContactMgrCore.h                                     */
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

#ifndef CONTACT_MGR_CORE_HEADER
#define CONTACT_MGR_CORE_HEADER

#include <list>
#include <map>
#include <set>
#include <string>
#include <vector>
#include "ContactLedger.h"
#include "NodeRecord.h"
#include "PlatformAlertRecord.h"
#include "CMAlert.h"
#include "ExFilterSet.h"
#include "VarDataPair.h"

// The contact handling and alerts of pContactMgrV20, without MOOS:
// the contact ledger, alerts and alert requests, early and cease
// warnings, retiring contacts, and enabling or disabling the
// behaviors for a contact. Postings go out through cmgrPost(), which
// by default holds them for getPostings(), so a helm outside MOOS can
// raise the same alerts as pContactMgrV20.

class ContactMgrCore
{
 public:
  ContactMgrCore();
  virtual ~ContactMgrCore() {}

 public: // Configuration
  // Returns true if the parameter is one of the contact manager's
  // and its value is valid
  bool setParam(std::string param, std::string value);
  void setOwnship(std::string);
  bool handleConfigAlert(std::string, std::string);

 public: // Mail
  void setCurrTimeUTC(double utc) {m_curr_utc = utc;}
  void setOwnshipX(double v)       {m_osx = v;}
  void setOwnshipY(double v)       {m_osy = v;}
  void setOwnshipHeading(double v) {m_osh = v;}
  void setOwnshipSpeed(double v)   {m_osv = v;}

  bool handleMailNodeReport(std::string, std::string&);
  void handleMailHelmState(std::string);
  bool handleMailModEnableContact(std::string, std::string);

 public: // Iteration
  // Extrapolates the contacts to the given time, then checks for
  // warnings, alerts and contacts to retire
  void iterate(double utc);

  void updateRanges();
  void checkForEarlyWarnings();
  void checkForCeaseWarnings();
  void checkForAlerts();
  void checkForNewRetiredContacts();

 public: // Output, when cmgrPost() etc. aren't overridden
  std::vector<VarDataPair> getPostings();
  std::vector<std::string> getConfigWarnings();

 public: // Getters
  const ContactLedger& getLedger() const {return(m_ledger);}
  ContactLedger&       getLedger()       {return(m_ledger);}

  double getAlertRange(std::string id) const;
  double getAlertRangeCPA(std::string id) const;
  double getMaxAlertRange() const;

  std::vector<VarDataPair> getAlertOnFlags(std::string id) const;
  std::vector<VarDataPair> getAlertOffFlags(std::string id) const;

  bool knownAlert(std::string id) const;
  bool isAlertDisabled(std::string alert_id) const;

  std::list<std::string> getRangeOrderedContacts() const;

 protected: // Where postings, events and warnings go
  virtual void cmgrPost(const std::string& var, const std::string& sval);
  virtual void cmgrPost(const std::string& var, double dval);
  virtual void cmgrEvent(const std::string&) {}
  virtual void cmgrConfigWarning(const std::string&);

 protected:
  bool handleConfigDecay(std::string);
  bool handleConfigRejectRange(std::string);

  void postOnAlerts(NodeRecord, std::string id);
  void postOffAlerts(NodeRecord, std::string id);
  void postAlert(NodeRecord, VarDataPair);

  bool checkAlertApplies(std::string contact, std::string id);

  void postWarningFlags(const std::vector<VarDataPair>&,
			std::string contact);
  void postRetireFlags(const std::vector<VarDataPair>&,
		       std::string);

  void addDisabledContact(std::string id);
  void addEnabledContact(std::string id);

  void postFlags(const std::vector<VarDataPair>&);
  std::string expandMacros(std::string) const;

 protected: // Main record of alerts, each keyed on the alert_id
  std::map<std::string, CMAlert> m_map_alerts;

  std::set<std::string> m_alerts_disabled;

 protected: // Configuration parameters

  // Global Exclusion filters
  double       m_reject_range;
  ExFilterSet  m_filter_set;
  unsigned int m_max_contacts;

  std::string  m_ownship;
  double       m_contact_max_age;

  bool         m_alert_verbose;
  double       m_decay_start;
  double       m_decay_end;

 protected: // State variables
  double m_curr_utc;

  double m_osx;
  double m_osy;
  double m_osh;
  double m_osv;

  std::string m_os_group;
  std::string m_os_type;

  std::set<std::string> m_filtered_vnames;

  ContactLedger m_ledger;

  // Keyed on vname (contact name)
  std::map<std::string, double> m_map_node_ranges_actual;
  std::map<std::string, double> m_map_node_ranges_extrap;
  std::map<std::string, double> m_map_node_cpa;
  std::map<std::string, double> m_map_node_roc;

  // A matrix: vehicle_name X alert_id. Cell val is Boolean indicating
  // if the alert is active or resolved, for the given vehicle.
  PlatformAlertRecord  m_par;

  std::list<std::string> m_contacts_retired;

  bool  m_hold_alerts_for_helm;
  bool  m_helm_in_drive_noted;

 protected: // Rel 24.8.x For users using cmgr for dis/enabling bhvs
  std::string  m_disable_var;
  std::string  m_enable_var;
  std::string  m_expunge_var;
  std::list<std::string> m_disabled_contacts;
  std::list<std::string> m_enabled_contacts;

  std::vector<VarDataPair> m_able_flags;
  std::vector<VarDataPair> m_disable_flags;
  std::vector<VarDataPair> m_enable_flags;

 protected: // Rel 24.8.x For users wanting early warning system
  double m_early_warning_time;  // Config param by default -1/off

  // Key is contact name. To range at which warning was generated
  std::map<std::string, double> m_map_range_will_warn;
  std::map<std::string, double> m_map_range_was_warned;
  std::map<std::string, double> m_map_utc_was_warned;

  std::vector<VarDataPair>  m_early_warning_flags;
  std::vector<VarDataPair>  m_cease_warning_flags;

  // Rel 24.8.x Support flags on event of retired contact
  std::vector<VarDataPair> m_retire_flags;

 private:
  std::vector<VarDataPair> m_postings;
  std::vector<std::string> m_config_warnings;
};

#endif
