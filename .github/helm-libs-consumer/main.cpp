// Uses the helm libraries from outside the MOOS-IvP tree:
//
//  1. The static behavior factory creates behaviors from lib_behaviors-marine,
//     lib_behaviors-colregs, and lib_dep_behaviors, which pulls every helm
//     library into the link.
//  2. The solver finds the peak of a one-function problem.
//  3. The helm engine decides a course from a behavior file, as pHelmIvP does.
//  4. The contact ledger places a contact using a geodesy supplied by the
//     caller, and a contact manager alert filters it.
//
// Prints one line per problem and exits non-zero if there were any.

#include <cmath>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <set>
#include <string>

#include "BFactoryStatic.h"
#include "BehaviorSet.h"
#include "CMAlert.h"
#include "ContactLedger.h"
#include "HelmEngine.h"
#include "InfoBuffer.h"
#include "IvPBehavior.h"
#include "IvPDomain.h"
#include "IvPFunction.h"
#include "IvPProblem.h"
#include "LedgerSnap.h"
#include "Populator_BehaviorSet.h"
#include "ZAIC_PEAK.h"

int main()
{
    int errors = 0;

    IvPDomain domain;
    domain.addDomain("course", 0, 359, 360);
    domain.addDomain("speed", 0, 4, 41);

    BFactoryStatic factory;
    factory.setDomain(domain);
    for (const std::string type : {"BHV_Waypoint", "BHV_AvdColregsV22", "BHV_OpRegionV26",
                                   "BHV_ConstantHeading"})
    {
        IvPBehavior* behavior = factory.newBehavior(type);
        if (!behavior)
        {
            std::cerr << "failed: no " << type << " from BFactoryStatic\n";
            ++errors;
        }
        delete behavior;
    }

    IvPDomain course;
    course.addDomain("course", 0, 359, 360);
    ZAIC_PEAK zaic(course, "course");
    zaic.setSummit(90);
    zaic.setBaseWidth(90);
    zaic.setValueWrap(true);

    IvPProblem problem;
    problem.setDomain(course);
    problem.addOF(zaic.extractIvPFunction());
    problem.solve();
    double result = problem.getResult("course");
    if (std::fabs(result - 90) > 1)
    {
        std::cerr << "failed: solved course " << result << ", not 90\n";
        ++errors;
    }

    const std::string bhv_file =
        (std::filesystem::temp_directory_path() / "helm_libs_consumer.bhv").string();
    std::ofstream(bhv_file) << "Behavior = BHV_ConstantHeading\n"
                               "{\n"
                               "  name     = east\n"
                               "  heading  = 90\n"
                               "  duration = no-time-limit\n"
                               "}\n";
    InfoBuffer info_buffer;
    LedgerSnap ledger_snap;
    const double now = 1000;
    info_buffer.setCurrTime(now);
    ledger_snap.setCurrTimeUTC(now);
    for (const std::string var : {"NAV_X", "NAV_Y", "NAV_HEADING", "NAV_SPEED"})
        info_buffer.setValue(var, 0);

    Populator_BehaviorSet populator(domain, &info_buffer, &ledger_snap);
    BehaviorSet* behaviors = populator.populate(std::set<std::string>{bhv_file});
    double decision = NAN;
    if (!behaviors)
    {
        std::cerr << "failed: no behaviors from " << bhv_file << "\n";
        ++errors;
    }
    else
    {
        // pHelmIvP does this itself for the behaviors in its files
        for (unsigned int i = 0; i < behaviors->size(); ++i)
        {
            behaviors->getBehavior(i)->setParam("us", "consumer");
            behaviors->getBehavior(i)->onSetParamComplete();
        }
        HelmEngine engine(domain, &info_buffer, &ledger_snap);
        engine.setBehaviorSet(behaviors);
        HelmReport report = engine.determineNextDecision(behaviors, now);
        decision = report.getDecision("course");
        if (report.getHalted() || !report.hasDecision("course") || std::fabs(decision - 90) > 1)
        {
            std::cerr << "failed: helm engine decided " << report.getDecisionSummary()
                      << ", not course 90\n";
            ++errors;
        }
        delete behaviors;
    }

    // 1 degree of latitude or longitude as 100 km, which is enough here
    ContactLedger ledger;
    ledger.setGeodesy([](double lat, double lon, double& x, double& y)
                      { x = lon * 1e5, y = lat * 1e5; },
                      [](double x, double y, double& lat, double& lon)
                      { lat = y / 1e5, lon = x / 1e5; });
    ledger.setCurrTimeUTC(now);
    std::string whynot;
    ledger.processNodeReport("NAME=gilda,TYPE=kayak,LAT=0.001,LON=0.002,SPD=1,HDG=0,TIME=1000",
                             whynot);
    if (!ledger.hasVName("gilda") || std::fabs(ledger.getX("gilda", false) - 200) > 1e-6)
    {
        std::cerr << "failed: contact gilda not at x=200 " << whynot << "\n";
        ++errors;
    }
    else
    {
        CMAlert alert;
        alert.configFilter("match_type", "kayak");
        if (!alert.filterCheck(ledger.getRecord("gilda", false)))
        {
            std::cerr << "failed: alert for type kayak rejected gilda\n";
            ++errors;
        }
    }

    if (errors == 0)
        std::cout << "Helm libraries OK: behaviors created, solved course " << result
                  << ", helm engine decided course " << decision << ", contact placed\n";
    return errors == 0 ? 0 : 1;
}
