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
#include "Planners/Dromos.hpp"
#include "Utils/Timer.hpp"
#include "Utils/Stats.hpp"
#include "Components/predictLKH.h"
#include <iostream>

namespace Antipatrea {

    bool Dromos::Solve(const int nrIters, const double tmax, bool &canBeSolved) {
        Timer::Clock clk;
        std::vector<double> pos;
        Group *group;

        Timer::Start(clk);

        if (m_tree.GetNrVertices() == 0) {
            Logger::m_out << "DROMOS" << std::endl;
            if (Start() == false) {
                canBeSolved = false;
                return false;
            }
        }

        int oldNrGroups = m_groups.GetNrGroups();
        int counter = 0;

        pos.resize(GetSimulator()->GetPositionAllocator()->GetDim());

        for (int i = 0; i < nrIters && Timer::Elapsed(clk) < tmax &&  !IsSolved(); ++i) {
            if (m_shouldWait)
                ExtendWait();
            else {
                GetGroupSelector()->SetGroups(&m_groups);
                group = GetGroupSelector()->SelectGroup();

                const int dimPos = GetSimulator()->GetPositionAllocator()->GetDim();

                if (RandomUniformReal() > 0.50 && group->m_guide.size() > dimPos) {
                    const int n = (int) (group->m_guide.size() / dimPos);
                    const int which = RandomUniformInteger(1, std::min(4, n - 1));
                    const double *target = &group->m_guide[which * dimPos];
                    RandomPointInsideSphere(dimPos, target, 2, &pos[0]);
                } else
                    GetSimulator()->SamplePosition(&pos[0]);

                auto vid = SelectVertex(*group, &pos[0]);

                ExtendFrom(SelectVertex(*group, &pos[0]), &pos[0]);

                m_groups.UpdateWeightGroup(*group, group->GetWeight() * GetDiscountSelect());

            }
        }

        return IsSolved();
    }

    bool Dromos::CompleteGroup(Group &group, MPTreeVertex &v) {
        Timer::Clock clk;
        Timer::Start(clk);

        auto tkey = dynamic_cast<GroupKeyTour *>(group.GetKey());

        if (tkey == NULL)
            return false;

        bool ok = false;
        auto tgen = GetTourGenerator();
        auto kreg = tkey->GetRegionKey();
        auto kgoals = tkey->GetUnreachedGoalsKey();
        auto unreached = kgoals->GetUnreachedGoals();
        auto ng = unreached->size();
        auto decomp = GetDecomposition();
        auto bounds = GetProblem()->GetTimeBounds();
        auto goals = GetProblem()->GetGoals();
        double cost;

        auto sim = GetSimulator();

        auto regStart = dynamic_cast<Region *>(decomp->GetGraph()->GetVertex(kreg->GetId()));
        auto data = dynamic_cast<Region *>(decomp->GetGraph()->GetVertex(kreg->GetId()))->GetPathDataToGoals();

        tgen->SetNrSites(1 + ng);
        tgen->SetStartTime(tkey->GetTour()->GetStartTime());
        tgen->SetBounds(0, 0.0, INFINITY);
        tgen->SetDuration(0, 0, 0.0);
        for (int i = 0; i < ng; ++i) {
            auto regGoal = (*goals)[(*unreached)[i]];

            tgen->SetDuration(i + 1, i + 1, 0.0);
            tgen->SetBounds(i + 1, 0, INFINITY);//(*bounds)[2 * (*unreached)[i]], (*bounds)[2 * (*unreached)[i] + 1]);
            cost = (*data)[(*unreached)[i]]->m_cost;
            tgen->SetDuration(0, i + 1, cost);
            tgen->SetDuration(i + 1, 0, cost);

            if (cost == INFINITY) {
                Logger::m_out << "infinite cost" << std::endl;
                Stats::GetSingleton()->AddValue("TimeCompleteGroup", Timer::Elapsed(clk));

                return false;
            }
        }

        for (int i = 0; i < ng; ++i) {

            auto regGoal1 = (*goals)[(*unreached)[i]];

            data = (*goals)[(*unreached)[i]]->GetPathDataToGoals();
            for (int j = i + 1; j < ng; ++j) {
                auto regGoal2 = (*goals)[(*unreached)[j]];

                cost = (*data)[(*unreached)[j]]->m_cost;
                tgen->SetDuration(i + 1, j + 1, cost);
                tgen->SetDuration(j + 1, i + 1, cost);

                if (cost == INFINITY) {
                    Logger::m_out << "inifinite cost" << std::endl;
                    Stats::GetSingleton()->AddValue("TimeCompleteGroup", Timer::Elapsed(clk));

                    return false;
                }
            }
        }

        if (m_usePredictions) {
//	    Logger::m_out << "using predictions...............ng = " << ng << std::endl;

            //setting prediction values from start to every unreached goal
            for (int i = 0; i < ng; ++i) {
                tgen->SetDuration(0, i + 1, regStart->m_predictedCombinedToGoals[(*unreached)[i]]);
                tgen->SetDuration(i + 1, 0, INFINITY);
            }

            //setting prediction values from each goal to each goal
            for (int i = 0; i < ng; ++i) {
                auto regGoal = (*goals)[(*unreached)[i]];
                for (int j = 0; j < ng; ++j)
                    tgen->SetDuration(i + 1, j + 1, regGoal->m_predictedCombinedToGoals[(*unreached)[j]]);
            }
        }

        // check if tour from parent can be used
        if (v.GetParent()) {
            auto parent = dynamic_cast<MPTreeVertex *>(v.GetParent());
            auto pkey = dynamic_cast<GroupKeyTour *>(parent->GetGroup()->GetKey());
            if (kgoals->SameContent(*(pkey->GetUnreachedGoalsKey()))) {
                tkey->GetTour()->m_order = pkey->GetTour()->m_order;
                ok = tgen->FromOrderToTimes(*(tkey->GetTour()));
                //   if (ok)
                //    Logger::m_out << "using parent order " << *(tkey->GetTour()) << std::endl;

            } else {
//                Logger::m_out << "parent unreached goals:" << (*(pkey->GetUnreachedGoalsKey())) << std::endl
//                              << "parent tour " << *(pkey->GetTour()) << std::endl
//                              << "vertex unreached goals:" << *kgoals << std::endl
//                              << "vertex reached goal: " << v.GetReachedGoal() << std::endl;

            }

        }

        /*
          if(!ok && m_groups.GetNrGroups() > 1)
          {
          int nrTries = 0;


          //check to see if any previous tours apply
          for(int i = m_groups.GetNrGroups() - 1; i >= 0 && !ok; --i)
          {
          auto other = m_groups.GetGroup(i);
          auto otherKey =  dynamic_cast<GroupKeyTour *>(other->GetKey());
          if (kgoals->SameContent(*(otherKey->GetUnreachedGoalsKey())))
          {
          tkey->GetTour()->m_order = otherKey->GetTour()->m_order;
          ok = tgen->FromOrderToTimes(*(tkey->GetTour()));
          ++nrTries;
          //
          //
          //   Logger::m_out << "unreached " << *kgoals << std::endl
          //	    << "start times = " << tkey->GetTour()->GetStartTime() << " " << otherKey->GetTour()->GetStartTime() << " ok = " << ok << std::endl;
          //Logger::m_out << "tour " << *(otherKey->GetTour()) << std::endl;
          }

          }

          if(ok)
          Logger::m_out <<" cache hit...................................." << nrTries << " : " << m_groups.GetNrGroups() << std::endl;
          //   else
          //  Logger::m_out <<" cache mis...................................." << nrTries << " : " << m_groups.GetNrGroups() << std::endl;

          }*/


        if (!ok) {
            ok = tgen->GenerateTour(*(tkey->GetTour()));
            Stats::GetSingleton()->AddValue("NrCallsGenerateTour", 1);
            if (m_shouldWait)
                Stats::GetSingleton()->AddValue("NrCallsGenerateTourWaiting", 1);
            //  Stats::GetSingleton()->AddValue("SanityCheck", tgen->SanityCheck() == false);

        }

        if (ok) {
            const double dur = tkey->GetTour()->GetEndTime() - tkey->GetTour()->GetStartTime();
            const int nrSites = tkey->GetTour()->GetNrSites();

            if (dur < Constants::EPSILON && nrSites > 1) {
                Logger::m_out << "unreached:";
                for (int i = 0; i < ng; ++i)
                    Logger::m_out << (*unreached)[i] << " ";
                Logger::m_out << std::endl;
                Logger::m_out << "goals to regions: ";
                for (int i = 0; i < (int) goals->size(); ++i)
                    Logger::m_out << "(" << i << "," << (*goals)[i]->GetKey() << ") ";
                Logger::m_out << std::endl;

                Logger::m_out << "what....... " << dur << "nrSites = " << nrSites << std::endl;
                Logger::m_out << *tkey << std::endl;
                Logger::m_out << "tour generator " << std::endl;
                Logger::m_out << *tgen << std::endl;

                group.SetWeight(Constants::GROUP_MAX_WEIGHT);
            } else
                group.SetWeight(10000000000.0 / (dur * pow(2, nrSites)));

             if(tkey->GetTour()->m_order.size() > 1)
             {

             auto xgid = (*unreached)[tkey->GetTour()->m_order[1] - 1];
             auto xgdata = (*regStart->GetPathDataToGoals())[xgid];

             const int dimPos = GetSimulator()->GetPositionAllocator()->GetDim();
             group.m_guide.clear();
             RegularizePointsAlongPath(xgdata->m_pts.size() / dimPos, &xgdata->m_pts[0], 3.0, dimPos, group.m_guide);
             }

            Stats::GetSingleton()->AddValue("TimeCompleteGroup", Timer::Elapsed(clk));

            return true;
        }

        Stats::GetSingleton()->AddValue("TimeCompleteGroup", Timer::Elapsed(clk));

        return false;
    }

}

