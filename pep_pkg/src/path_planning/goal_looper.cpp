#include <iostream>
#include <array>
#include <cmath>

#define DIST_THRESHOLD 10

class GoalLooper
{
private:
    std::array<std::array<double, 2>, 6> goals;
    int currentGoalIndex;

    double distance(const std::array<double, 2> &pos1, const std::array<double, 2> &pos2)
    {
        return sqrt(pow(pos1[0] - pos2[0], 2) + pow(pos1[1] - pos2[1], 2));
    }

    std::array<double, 2> getCurrentGoal()
    {
        return goals[currentGoalIndex];
    }

public:
    GoalLooper(const std::array<std::array<double, 2>, 6> &goals)
    {
        this->currentGoalIndex = 0;
        this->goals = goals;
    }

    std::array<double, 2> loopGoals(std::array<double, 2> &position)
    {
        if (distance(position, getCurrentGoal()) < DIST_THRESHOLD)
        {
            currentGoalIndex = (currentGoalIndex + 1) % goals.size();
        }
        return getCurrentGoal();
    }
};