#include "rl_quadruped_controller/control/PolicyCommand.h"

#include <iostream>
#include <stdexcept>

void require(bool condition, const char* message)
{
    if (!condition) throw std::runtime_error(message);
}

int main()
{
    PolicyCommand command;
    const auto apply = [&](int mode) { return command.update(mode, 0.28, -0.17, -0.43); };
    const auto is_stopped = [&] { return command.x == 0 && command.y == 0 && command.yaw == 0; };
    const auto is_walking = [&] { return command.x == 0.28 && command.y == -0.17 && command.yaw == -0.43; };

    require(!apply(0) && is_walking(), "normal locomotion lost a velocity command");
    require(apply(4) && is_stopped(), "hold pulse did not inhibit all three velocities immediately");
    for (int step = 0; step < 2000; ++step) {
        require(!apply(step % 2 ? 4 : 0) && is_stopped(), "repeated or absent pulses changed the hold");
    }
    require(!apply(1) && is_stopped(), "passive request leaked motion before the state transition");
    require(!apply(2) && is_stopped(), "fixed stand request leaked motion before the state transition");
    require(apply(3) && is_walking(), "explicit resume did not restore the three velocities");
    require(!apply(3) && is_walking(), "duplicate resume pulse changed the substate");
    require(apply(4) && is_stopped(), "second hold failed");
    command = {};
    require(!command.goal_balance_hold && is_stopped(), "new RL session inherited an old hold");
    std::cout << "PASS: balance hold latched zero velocity on all axes until explicit resume\n";
}
