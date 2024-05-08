#pragma once

namespace a2rl_bs_msgs::topics
{

    constexpr char const *T_VN_INS = "a2rl/vn/ins";

    constexpr char const *T_OBSERVER_EGO_STATE = "a2rl/observer/ego_state";
    constexpr char const *T_OBSERVER_EGO_STATE_LOWFREQ = "a2rl/observer/ego_state/low_freq";
    constexpr char const *T_OBSERVER_EGO_LOC = "a2rl/observer/ego_loc";
    constexpr char const *T_OBSERVER_EGO_LOC_LOWFREQ = "a2rl/observer/ego_loc/low_freq";
    constexpr char const *T_OBSERVER_STATUS = "a2rl/observer/status";

    constexpr char const *T_PLANNER_TRAJECTORY = "a2rl/planner/trajectory";
    constexpr char const *T_PLANNER_STATUS = "a2rl/planner/status";

    constexpr char const *T_CONTROLLER_CONTROL = "a2rl/controller/control";
    constexpr char const *T_CONTROLLER_DEBUG = "a2rl/controller/debug";

    constexpr char const *T_RACE_CONTROL = "a2rl/remote/race_control";

} // namespace a2rl_bs_msgs::topics
