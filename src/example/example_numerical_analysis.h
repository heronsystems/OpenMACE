//!
//! This is an example of using NumericalAnalysis library to do an optimization problem
//! In this example we are performing numerical analysis on a R->R problem (simple)
//!

#ifndef EXAMPLE_NUMERICAL_ANALYSIS_H
#define EXAMPLE_NUMERICAL_ANALYSIS_H

#include "common/numerical_analysis.h"

//!
//! \brief Enivronment that problem is to operate in.
//! In this case set up min/max bounds.
//!
class TestEnvironment
{
public:
    double min;
    double max;
};

//!
//! \brief Solution the problem is to generate. In this case it is the domain/range of the evaluation.
//!
class TestSolutionMetrics
{
public:
    double x;
    double value;
};


//!
//! \brief Class defining the actual problem
//!
//! In this case, an optimizer that scans an infinitesimally shrinking gap for a solution.
//! Pretty dirty optimizer, but shows the point.
//!
//! After performing one "step", the intermdiate solution is to be passed up and evaluated for compleatness.
//! It is up to the implimentor to determine when this should be done.
//!
class TestAlgorithm : public NumericalAnalysis::BaseNumericalProblem<TestEnvironment, TestSolutionMetrics>
{
public:
    std::string Solve(const TestEnvironment &envirionment, NumericalAnalysis::BaseAnalyzer<TestSolutionMetrics> *intermediate)
    {
        // y = x^2
        auto func = [](double value){return (value+0.2) * (value+0.2);};

        //std::this_thread::sleep_for(std::chrono::seconds(100));

        const double SCANS_IN_RANGE = 10;

        double minB = envirionment.min;
        double maxB = envirionment.max;
        while(true)
        {
            if(this->IsStopped() == true)
            {
                return "";
            }

            double step = (maxB - minB) / SCANS_IN_RANGE;
            double minV;
            double minI;
            bool set = false;
            for(double i = minB ; i < maxB ; i = i+step)
            {
                if(set == false)
                {
                    minV = func(i);
                    minI = i;
                    set = true;
                }

                double V = func(i);
                if(V < minV)
                {
                    minV = V;
                    minI = i;
                }
            }

            minB = minI - step;
            maxB = minI + step;

            TestSolutionMetrics metrics;
            metrics.x = minI;
            metrics.value = minV;
            intermediate->NewIntermediateSolution(metrics);
        }
    }
};



//!
//! \brief Example analyzer that determines if numerical analysis has converged/diverged upon a solution.
//!
//! In this example congergence is determined when delta of current intermdiate soultion to past intermediate solution is very small.
//!
class TestConditions : public NumericalAnalysis::BaseAnalyzer_IntermediateHistory<TestSolutionMetrics>
{
    virtual NumericalAnalysis::AnalysisStates CheckConvergence()
    {
        std::vector<TestSolutionMetrics> past = NumericalAnalysis::BaseAnalyzer_IntermediateHistory<TestSolutionMetrics>::m_PreviousOutputs;

        if(past.size() > 2)
        {
            double delta = past.at(past.size() - 2).value - past.at(past.size() - 1).value;
            if(std::fabs(delta) < 0.00000001)
            {
                return NumericalAnalysis::AnalysisStates::CONVERGED;
            }
        }

        return NumericalAnalysis::AnalysisStates::UNKNOWN;
    }
};

#endif // EXAMPLE_NUMERICAL_ANALYSIS_H
