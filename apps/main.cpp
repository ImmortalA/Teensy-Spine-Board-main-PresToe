/*
 * test_spine — PC-side demo for T-Motor AK MIT over UDP ↔ Teensy ↔ CAN.
 *
 * Flow: SpineBoard runs two threads (UDP receive from Teensy, UDP send with optional wait-for-feedback).
 * This file fills joint commands from a simple mode menu; topology (boards / buses / nodes) lives in host_cfg.
 * Keep host_cfg consistent with teensy/teensy.ino CONFIG and with spine_board.cpp wire constants.
 */

#include "spine_board.h"
#include <memory>
#include <cmath>
#include <chrono>
#include <limits>
#include <thread>
#include <vector>

/* One entry per actuator: which SpineBoard instance and which (bus, node) slot in its bus_list. */
struct ActuatorInfo
{
    int board; /* index into _spine_boards */
    int bus;   /* 0 .. num_buses-1 (logical CAN bus in UDP layout) */
    int node;  /* 0 .. num_nodes-1 along that bus */
};

namespace host_cfg {
constexpr int kNumTeensyBoards = 1;
/* Logical CAN buses used per board (SpineBoard second ctor arg). ≤ kWireNumLogicalBuses. */
constexpr int kNumCanBuses = 1;
/* Nodes per bus (first ctor arg). Must match firmware NUM_ACTIVE_NODES_BUS* totals. */
constexpr int kNodesPerBus = 2;
constexpr ActuatorType kDefaultActuator = ActuatorType::AK_60_6;
/* Protocol capacity: 2 buses × 3 nodes × 8 B — same numbers as teensy.ino / spine_board.cpp. */
constexpr int kWireNumLogicalBuses = 2;
constexpr int kWireMaxNodesPerBus = 3;
}  // namespace host_cfg

static_assert(host_cfg::kNumCanBuses >= 1 && host_cfg::kNumCanBuses <= host_cfg::kWireNumLogicalBuses,
              "kNumCanBuses");
static_assert(host_cfg::kNodesPerBus >= 1 && host_cfg::kNodesPerBus <= host_cfg::kWireMaxNodesPerBus,
              "kNodesPerBus");

/* Flatten (board × bus × node) into the list the control loop iterates. */
static std::vector<ActuatorInfo> makeActuatorMap()
{
    std::vector<ActuatorInfo> m;
    for (int b = 0; b < host_cfg::kNumTeensyBoards; ++b)
        for (int bus = 0; bus < host_cfg::kNumCanBuses; ++bus)
            for (int node = 0; node < host_cfg::kNodesPerBus; ++node)
                m.push_back({b, bus, node});
    return m;
}

static std::vector<std::vector<std::vector<ActuatorParams>>> makeActuatorParams()
{
    std::vector<std::vector<std::vector<ActuatorParams>>> out;
    for (int b = 0; b < host_cfg::kNumTeensyBoards; ++b)
    {
        std::vector<std::vector<ActuatorParams>> board;
        for (int bus = 0; bus < host_cfg::kNumCanBuses; ++bus)
        {
            std::vector<ActuatorParams> row;
            for (int n = 0; n < host_cfg::kNodesPerBus; ++n)
                row.push_back(getActuatorParams(host_cfg::kDefaultActuator));
            board.push_back(row);
        }
        out.push_back(board);
    }
    return out;
}

int main()
{
    const std::vector<ActuatorInfo> ACTUATOR_INFO_MAP = makeActuatorMap();
    const std::vector<std::vector<std::vector<ActuatorParams>>> ACTUATOR_PARAMS = makeActuatorParams();

    /* Linux: bind UDP to this interface; must exist (`ip link`). PC IP often 192.168.0.100 on same /24 as Teensy. */
    static std::string BOARD_INTERFACE_NAME = "enp8s0";
    static std::string ACTUATOR_TEENSY_BOARD_IPS[host_cfg::kNumTeensyBoards] = {"192.168.0.101"};
    static int ACTUATOR_TEENSY_BOARD_PORTS[host_cfg::kNumTeensyBoards] = {8003}; /* Teensy udp.begin port */

    std::vector<std::unique_ptr<SpineBoard>> _spine_boards;
    for (size_t i = 0; i < host_cfg::kNumTeensyBoards; i++)
    {
        _spine_boards.push_back(
            std::make_unique<SpineBoard>(ACTUATOR_TEENSY_BOARD_IPS[i],
                                         BOARD_INTERFACE_NAME,
                                         ACTUATOR_TEENSY_BOARD_PORTS[i],
                                         host_cfg::kNodesPerBus,
                                         host_cfg::kNumCanBuses,
                                         "board_" + std::to_string(i)));
    }

    for (size_t id = 0; id < _spine_boards.size(); ++id)
    {
        auto &board = *_spine_boards[id];
        board.setActuatorParams(ACTUATOR_PARAMS[id]);
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
        board.start();
        board.setThreadAffinityAndPriority(static_cast<int>(id));
    }

    /* Trajectory sample period for this thread only; MIT UDP rate is limited by Teensy round-trip when wait-for-feedback is on. */
    int dt_us = 10000;
    float dt = dt_us / 1e6f;
    while (true)
    {
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
        bool all_initialized = true;
        for (size_t id = 0; id < _spine_boards.size(); ++id)
            all_initialized = all_initialized && _spine_boards[id]->boardInitialized;
        if (all_initialized)
            break;
    }

    std::cout << "Select test mode (T-Motor AK — MIT impedance control on CAN):\n";
    std::cout << "  0: Enable only (no MIT commands from host)\n";
    std::cout << "  1: Position step to 0.2 rad\n";
    std::cout << "  2: Position sine (amp=1 rad, 1 rad/s)\n";
    std::cout << "  3: Position sine + velocity FF (same MIT frame)\n";
    std::cout << "  4: Velocity hold v_des=0.2 rad/s\n";
    std::cout << "Mode: " << std::flush;

    int mode = 0;
    if (!(std::cin >> mode))
        mode = 0;
    std::cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n');

    if (mode == 0)
    {
        std::cout << "Running SpineBoard... Motor enabled. No MIT commands sent." << std::endl;
        std::cout << "Press Enter to exit." << std::flush;
        std::cin.get();
    }
    else
    {
        /* Align with firmware: one MIT UDP per completed feedback round (see spine_board start()). */
        for (auto &board : _spine_boards)
        {
            board->setWaitForFeedbackAfterSend(true);
            board->setAllowCommandSend(true);
        }

        std::cout << "Running control test (Ctrl+C to stop)...\n";

        const float MIT_SINE_AMP = 1.0f;
        const float MIT_SINE_OMEGA = 1.0f;
        const float MIT_TORQUE_FF = 0.0f;
        const float PRIVATE_SINE_AMP = 1.0f;
        const float PRIVATE_VELOCITY = 0.2f;

        float p_center = 0.0f;
        bool center_initialized = false;

        int iter = 0;
        while (true)
        {
            /* Copy current state, write new commands into bus_list copies, then push back to SpineBoard. */
            std::vector<std::vector<bus>> boards_bus_lists(host_cfg::kNumTeensyBoards);
            for (size_t b = 0; b < host_cfg::kNumTeensyBoards; b++)
                boards_bus_lists[b] = _spine_boards[b]->getBusList();

            for (size_t i = 0; i < ACTUATOR_INFO_MAP.size(); i++)
            {
                auto info = ACTUATOR_INFO_MAP[i];
                auto &bus_list = boards_bus_lists[static_cast<size_t>(info.board)];

                if (!center_initialized && (mode == 2 || mode == 3))
                {
                    p_center = bus_list[info.bus].state.j[info.node].p;
                    center_initialized = true;
                }

                float p_target = 0.0f;
                float v_target = 0.0f;
                float t_ff = 0.0f;

                if (mode == 1)
                {
                    p_target = 0.2f;
                }
                else if (mode == 2)
                {
                    float t = iter * dt;
                    p_target = p_center + PRIVATE_SINE_AMP * std::sin(t);
                }
                else if (mode == 3)
                {
                    float t = iter * dt;
                    p_target = p_center + MIT_SINE_AMP * std::sin(MIT_SINE_OMEGA * t);
                    v_target = MIT_SINE_AMP * MIT_SINE_OMEGA * std::cos(MIT_SINE_OMEGA * t);
                    t_ff = MIT_TORQUE_FF;
                }
                else if (mode == 4)
                {
                    v_target = PRIVATE_VELOCITY;
                }

                bus_list[info.bus].command.j[info.node].p_des = p_target;
                bus_list[info.bus].command.j[info.node].v_des = v_target;
                bus_list[info.bus].command.j[info.node].kp = 25.0f;
                bus_list[info.bus].command.j[info.node].kd = 2.5f;
                bus_list[info.bus].command.j[info.node].t_ff = t_ff;

                if (iter % 100 == 0)
                {
                    std::cout << "p=" << bus_list[info.bus].state.j[info.node].p
                              << " p_des=" << p_target << " v_des=" << v_target
                              << " t_ff=" << t_ff << std::endl;
                }
            }

            for (size_t b = 0; b < host_cfg::kNumTeensyBoards; b++)
                _spine_boards[b]->setBusList(boards_bus_lists[b]);

            iter++;
            std::this_thread::sleep_for(std::chrono::microseconds(dt_us));
        }
    }

    for (size_t id = 0; id < _spine_boards.size(); ++id)
        _spine_boards[id]->end();

    return 0;
}
