#pragma once

struct PolicyCommand
{
    double x = 0.0;
    double y = 0.0;
    double yaw = 0.0;
    bool goal_balance_hold = false;

    bool update(int mode, double forward, double lateral, double yaw_rate)
    {
        const bool was_holding = goal_balance_hold;
        if (mode == 4) goal_balance_hold = true;
        if (mode == 3) goal_balance_hold = false;
        x = goal_balance_hold ? 0.0 : forward;
        y = goal_balance_hold ? 0.0 : lateral;
        yaw = goal_balance_hold ? 0.0 : yaw_rate;
        return was_holding != goal_balance_hold;
    }
};
