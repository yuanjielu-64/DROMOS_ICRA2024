/*
 * Copyright (C) 2018 Erion Plaku
 * All Rights Reserved
 * 
 *       Created by Erion Plaku
 *       Computational Robotics Group
 *       Department of Electrical Engineering and Computer Science
 *       Catholic University of America
 *
 *       www.robotmotionplanning.org
 *
 * Code should not be distributed or used without written permission from the
 * copyright holder.
 */
#include "Programs/Setup.hpp"
#include "Utils/Timer.hpp"
#include "Utils/Stats.hpp"
#include "Components/TourGeneratorExact.hpp"
#include <fstream>
#include <iomanip>

using namespace Antipatrea;

extern "C" int RunPlanner(int argc, char **argv) {
    std::ifstream in("stop.txt");
    int check = 1;

    if (in && (in >> check) && check == 1) {
        Logger::m_out << "...forced stop : remove stop.txt to continue" << std::endl;

        in.close();

        return 0;

    }
    if (in)
        in.close();

    Setup setup;
    Params params;

    params.ReadFromFile(argv[1]);
    //params.ReadFromFile(fnameProblem);    
    params.ProcessArgs(2, argc - 1, argv);

    auto tmax = params.GetValueAsDouble("PlannerMaxRuntime", 10.0);
    auto recordCannotBeSolved = params.GetValueAsBool("PlannerRecordCannotBeSolved", true);
    std::string statsFileName(params.GetValue("PlannerStatsFile", "stats.txt"));

    int solved = 0;
    Timer::Clock clk;
    double tcurr = 0;
    int count = 0;
    double tint = 2.0;

    Logger::m_out << "...results will be written to <" << statsFileName << ">" << std::endl;

    Timer::Start(clk);

    setup.SetupFromParams(params);
    setup.RunConstructDecomposition(params);
    if (setup.GetMP() && setup.GetMP()->m_usePredictions)
        setup.GetDecomposition()->Predictions();

    bool canBeSolved = true;

    while (canBeSolved && (tcurr = Timer::Elapsed(clk)) < tmax && setup.GetMP()->IsSolved() == false) {
        setup.GetMP()->Solve(1000, 7.0, canBeSolved);
        if (tcurr > count * tint) {
            Logger::m_out << "...not solved yet at time " << Timer::Elapsed(clk) << std::endl;
            ++count;

            Logger::m_out << *(Stats::GetSingleton()) << std::endl;

        }
    }
    tcurr = Timer::Elapsed(clk);
    Stats::GetSingleton()->AddValue("TimeSolve", tcurr);

    std::ofstream out(statsFileName.c_str(), std::ofstream::out | std::ofstream::app);
    Solution sol;
    solved = setup.GetMP()->GetSolution(sol);

    const int markAs = solved ? 1 : (canBeSolved == false ? -1 : 0);

    auto ss = setup.GetMP()->GetState(0);
    auto goals = setup.GetProblem()->GetGoals();
    double p[3];

    double totalTime = 0.0;
    totalTime = Stats::GetSingleton()->GetValue("TimeSolve")
                + Stats::GetSingleton()->GetValue("TimeAddRegions")
                + Stats::GetSingleton()->GetValue("TimeConnectRegions");

    out << markAs << " "
        << params.GetValue("UseMP", "unknown") << " "
        << totalTime << " "
        << std::setprecision(5) << Stats::GetSingleton()->GetValue("TimeForPRM")<< " "
        << std::setprecision(5) << Stats::GetSingleton()->GetValue("TimeSolve") << " "
        << sol.GetCost() << " "
        << sol.GetEndTime() << " "

        << ss[0] << " "
        << ss[1] << " ";

    for (auto & goal : *goals) {
        goal->GetRepresentativePoint(p);
        out << p[0] << " " << p[1] << " ";
    }
    out << std::endl;

    out.close();

    Logger::m_out << "[solved                  = " << markAs << "] " << std::endl
                  << "[runtimeForPRM           = " << Stats::GetSingleton()->GetValue("TimeForPRM") << std::endl
                  << "[runtimeForSolve         = " << Stats::GetSingleton()->GetValue("TimeSolve") << "] " << std::endl
                  << "[SolDistance             = " << sol.GetCost() << "] " << std::endl
                  << "[SolEndTime              = " << sol.GetEndTime() << "]" << std::endl;

    return 0;

}

