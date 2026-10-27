// Uses the helm libraries from outside the MOOS-IvP tree:
//
//  1. The static behavior factory creates behaviors from lib_behaviors-marine,
//     lib_behaviors-colregs, and lib_dep_behaviors, which pulls every helm
//     library into the link.
//  2. The solver finds the peak of a one-function problem.
//
// Prints one line per problem and exits non-zero if there were any.

#include <cmath>
#include <iostream>
#include <string>

#include "BFactoryStatic.h"
#include "IvPBehavior.h"
#include "IvPDomain.h"
#include "IvPFunction.h"
#include "IvPProblem.h"
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

    if (errors == 0)
        std::cout << "Helm libraries OK: behaviors created, solved course " << result << "\n";
    return errors == 0 ? 0 : 1;
}
