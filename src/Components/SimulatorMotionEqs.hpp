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

#ifndef Antipatrea__SimulatorMotionEqs_HPP_
#define Antipatrea__SimulatorMotionEqs_HPP_

#include "Components/Simulator.hpp"

namespace Antipatrea
{
class SimulatorMotionEqs : public Simulator
{
  public:
    SimulatorMotionEqs(void) : Simulator()
    {
    }

    virtual ~SimulatorMotionEqs(void)
    {
    }

    virtual void MotionEqs(const double s[],
                           const double t,
                           const double u[],
                           double ds[]) = 0;

    virtual double SimulateOneStep(void);
};
}

#endif
