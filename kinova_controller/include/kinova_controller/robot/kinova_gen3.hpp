#pragma once

//Kortex(kinova) API
#include <KDetailedException.h>
#include <BaseClientRpc.h>
#include <BaseCyclicClientRpc.h>
#include <ActuatorConfigClientRpc.h>
#include <SessionClientRpc.h>
#include <SessionManager.h>
#include <RouterClient.h>
#include <TransportClientUdp.h>
#include <TransportClientTcp.h>
#include <google/protobuf/util/json_util.h>

#include "kinova_controller/robot/kinova_gen3_base.hpp"

#define PORT 10000
#define PORT_REAL_TIME 10001

class KinovaGen3: public KinovaGen3Base
{
    public:
    KinovaGen3();
    ~KinovaGen3();

    virtual bool init(std::shared_ptr<RobotState> robot_state_arg) override;
    //destroy
    virtual void destroy() override;
    virtual void update_state() override;
    virtual bool set_joint_torque(const Eigen::VectorXd &u) override;

    private:
    bool move2home_position();
    std::function<void(Kinova::Api::Base::ActionNotification)>
    create_event_listener_by_promise(std::promise<Kinova::Api::Base::ActionEvent>& finish_promise);
    bool set_current_mode();
    Gen3State get_gen3_state(){return gen3_state;}
    void convert_joint_position(Eigen::Matrix<double, 7, 1>& joint_positions);


    // configuration
    std::string gen3_ip;
    std::shared_ptr<RobotState> robot_state;
    Gen3State gen3_state;
    std::array<float, 7> input_current_limit={10,10,10,10,6,6,6};

    //filter
    double sampling_time;
    LPF lpf_dq;
    double lpf_dq_cutoff_freq;
    LPF lpf_tau_j;
    double lpf_tau_j_cutoff_freq;

    //for convert joint position
    std::array<int, 7> is_limit = {0,1,0,1,0,1,0};

    // Kortex API
    //CORE VARIABLES OF CONTROL GEN3
    bool is_initialized_{false};
    Kinova::Api::Base::BaseClient *base{nullptr};
    Kinova::Api::BaseCyclic::BaseCyclicClient *base_cyclic{nullptr};
    Kinova::Api::ActuatorConfig::ActuatorConfigClient *actuator_config{nullptr};
    Kinova::Api::SessionManager* session_manager{nullptr};
    Kinova::Api::SessionManager* session_manager_real_time{nullptr};
    Kinova::Api::RouterClient* router{nullptr};
    Kinova::Api::TransportClientTcp* transport{nullptr};
    Kinova::Api::RouterClient* router_real_time{nullptr};
    Kinova::Api::TransportClientUdp* transport_real_time{nullptr};
    Kinova::Api::BaseCyclic::Feedback base_feedback;
    Kinova::Api::BaseCyclic::Command  base_command;

};
