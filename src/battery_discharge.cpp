#include "brass_gazebo_battery/battery_discharge.hh"

#include <ros/console.h>
#include <ros/ros.h>

#include "brass_gazebo_battery/ROS_debugging.h"
#include "gazebo/common/Battery.hh"
#include "gazebo/common/Plugin.hh"
#include "gazebo/common/Time.hh"
#include "gazebo/physics/physics.hh"
#include "std_msgs/Float64.h"
#include "cmath"

enum power { OFF = 0, ON = 1 };

template <typename T>
T max(T x, T y) {
    return x < y ? y : x;
}

using namespace gazebo;

GZ_REGISTER_MODEL_PLUGIN(BatteryPlugin);

BatteryPlugin::BatteryPlugin() {
    this->c = 0.0;
    this->r = 0.0;
    this->tau = 0.0;

    this->e0 = 0.0;
    this->e1 = 0.0;

    this->q0 = 0.0;
    this->q = 0.0;
    this->qt = 0.0;

    this->iraw = 0.0;
    this->ismooth = 0.0;

#ifdef BATTERY_DEBUG
    ROS_DEBUG_STREAM_NAMED("battery discharge", "Constructed BatteryPlugin and initialized parameters.");
#endif

    ROS_INFO_STREAM("BRASS CP1 battery is constructed.");
}

BatteryPlugin::~BatteryPlugin() {
#ifdef BATTERY_DEBUG
    ROS_DEBUG_STREAM_NAMED("battery discharge", "Destructing BatteryPlugin and removing the ros node.");
#endif
    this->rosNode->shutdown();
}

void BatteryPlugin::Load(physics::ModelPtr _model, sdf::ElementPtr _sdf) {
#ifdef BATTERY_DEBUG
    ROS_DEBUG_STREAM_NAMED("battery discharge", "Loading the BatteryPlugin");
#endif

    // check if the ros is up!
    if (!ros::isInitialized()) {
        ROS_INFO_STREAM("Initializing ROS...");
        int argc = 0;
        char **argv = NULL;
        ros::init(argc, argv, _sdf->Get<std::string>("ros_node"), ros::init_options::NoSigintHandler);
    }

    this->model = _model;
    this->world = _model->GetWorld();

    this->sim_time_now = this->world->SimTime().Double();

    // Create ros node and publish stuff there!
    this->rosNode.reset(new ros::NodeHandle(_sdf->Get<std::string>("ros_node")));
    if (this->rosNode->ok()) {
        ROS_GREEN_STREAM("ROS node is up");
    }

    // Publish a topic for motor power and charge level
    this->motor_power = this->rosNode->advertise<std_msgs::Bool>(this->model->GetName() + "/battery_discharged", 1);
    this->charge_state = this->rosNode->advertise<std_msgs::Float64>(this->model->GetName() + "/charge_level", 1);
    this->charge_state_mwh =
        this->rosNode->advertise<std_msgs::Float64>(this->model->GetName() + "/charge_level_mwh", 1);
    this->temperature_state = this->rosNode->advertise<std_msgs::Float64>(this->model->GetName() + "/temperature", 1);

    this->reset_battery = this->rosNode->subscribe(this->model->GetName() + "/reset_battery", 1,
                                                   &BatteryPlugin::BatteryResetCallback, this);

    this->set_charging =
        this->rosNode->advertiseService(this->model->GetName() + "/set_charging", &BatteryPlugin::SetCharging, this);
    this->set_charging_rate = this->rosNode->advertiseService(this->model->GetName() + "/set_charge_rate",
                                                              &BatteryPlugin::SetChargingRate, this);
    this->set_charge =
        this->rosNode->advertiseService(this->model->GetName() + "/set_charge", &BatteryPlugin::SetCharge, this);
    this->set_coefficients = this->rosNode->advertiseService(this->model->GetName() + "/set_model_coefficients",
                                                             &BatteryPlugin::SetModelCoefficients, this);

    std::string linkName = _sdf->Get<std::string>("link_name");
    this->link = this->model->GetLink(linkName);

    this->e0 = _sdf->Get<double>("constant_coef");
    this->e1 = _sdf->Get<double>("linear_coef");
    this->q0 = _sdf->Get<double>("initial_charge");
    this->qt = _sdf->Get<double>("charge_rate");
    this->c = _sdf->Get<double>("capacity");
    this->r = _sdf->Get<double>("resistance");
    this->tau = _sdf->Get<double>("smooth_current_tau");

    std::string batteryName = _sdf->Get<std::string>("battery_name");

    if (this->link->BatteryCount() > 0) {
        // Creates the battery
        this->battery = this->link->Battery(batteryName);
        ROS_GREEN_STREAM("Created a battery");
    } else {
        ROS_RED_STREAM("There is no battery specification in the link");
    };

    // Specifying a custom update function
    this->battery->SetUpdateFunc(std::bind(&BatteryPlugin::OnUpdateVoltage, this, std::placeholders::_1));

    this->sim_time_now = this->world->SimTime().Double();

#ifdef BATTERY_DEBUG
    ROS_DEBUG_STREAM_NAMED("battery discharge", "Loaded the BatteryPlugin at time:" << this->sim_time_now);
#endif

    ROS_GREEN_STREAM("Plugin is fully loaded.");
}

// This is for in initialization purposes and is called once after Load
// So explanation about Gazebo Init are here:
// http://playerstage.sourceforge.net/doc/Gazebo-manual-0.5-html/plugin_models.html
void BatteryPlugin::Init() {
    ROS_GREEN_STREAM("Init Battery");
    this->q = this->q0;
    this->charging = false;
    last_time_step = ros::Time::now();
}

void BatteryPlugin::Reset() {
    this->iraw = 0.0;
    this->ismooth = 0.0;
    this->Init();
}

double BatteryPlugin::OnUpdateVoltage(const common::BatteryPtr &_battery) {
    double dt = (ros::Time::now() - last_time_step).toSec();
    ;
    double totalpower = 0.0;
    double k = dt / this->tau;

    if (fabs(_battery->Voltage()) < 1e-3) return 0.0;

    for (auto powerLoad : _battery->PowerLoads()) totalpower += powerLoad.second;

    // current = power(Watts)/Voltage
    this->iraw = totalpower / _battery->Voltage();

    this->ismooth = this->ismooth + k * (this->iraw - this->ismooth);

    if (!this->charging)
    {
        this->q = this->q - GZ_SEC_TO_HOUR(dt * this->ismooth);
    }
    else
    {
        this->q = this->q + GZ_SEC_TO_HOUR(dt * this->qt);
    }

    this->sim_time_now = this->world->SimTime().Double();

#ifdef BATTERY_DEBUG
    ROS_DEBUG_STREAM_NAMED("battery discharge", "Current charge:" << this->q << ", at:" << this->sim_time_now);
#endif
    //    ROS_INFO_STREAM(this->q);

    this->et = this->e0 + this->e1 * (1 - this->q / this->c) - this->r * this->ismooth;

#ifdef BATTERY_DEBUG
    ROS_DEBUG_STREAM_NAMED("battery discharge", "Current voltage:" << this->et << ", at:" << this->sim_time_now);
#endif

    // Turn off the motor
    if (this->q <= 0) {
        this->sim_time_now = this->world->SimTime().Double();

#ifdef BATTERY_DEBUG
        ROS_DEBUG_STREAM_NAMED("battery discharge", "Out of juice at:" << this->sim_time_now);
#endif

        this->q = 0;
        std_msgs::Bool power_msg;
        power_msg.data = true;
        lock.lock();
        this->motor_power.publish(power_msg);
        lock.unlock();
    } else if (this->q >= this->c) {
        this->q = this->c;
    }

    std_msgs::Float64 charge_msg, charge_msg_mwh;
    charge_msg.data = this->q;
    charge_msg_mwh.data = this->q * 1000 * this->et;
    if (charge_msg_mwh.data < 0.0) {
        charge_msg_mwh.data = 0.0;
    }

    // Estimate the current temperature of the battery.
    std_msgs::Float64 temp_msg;
    temp_msg.data = ((pow(this->iraw,2)) * this->r * THERMAL_RESISTANCE) + 25;

    lock.lock();
    this->charge_state.publish(charge_msg);
    this->charge_state_mwh.publish(charge_msg_mwh);
    this->temperature_state.publish(temp_msg);
    lock.unlock();
    // Store the last time the code calculated its voltage.
    last_time_step = ros::Time::now();
    return et;
}

bool BatteryPlugin::SetCharging(brass_gazebo_battery::SetCharging::Request &req,
                                brass_gazebo_battery::SetCharging::Response &res) {
    lock.lock();
    this->charging = req.charging;
    if (this->charging) {
#ifdef BATTERY_DEBUG
        ROS_DEBUG_STREAM_NAMED("battery discharge", "Bot is charging");
#endif
        ROS_GREEN_STREAM("Bot is charging");
    } else {
#ifdef BATTERY_DEBUG
        ROS_DEBUG_STREAM_NAMED("battery discharge", "Bot disconnected from the charging station");
#endif
        ROS_GREEN_STREAM("Bot disconnected from the charging station");
    }
    lock.unlock();
    res.result = true;
    return true;
}

bool BatteryPlugin::SetChargingRate(brass_gazebo_battery::SetChargingRate::Request &req,
                                    brass_gazebo_battery::SetChargingRate::Response &res) {
    lock.lock();
    this->qt = req.charge_rate;
#ifdef BATTERY_DEBUG
    ROS_DEBUG_STREAM_NAMED("battery discharge", "Charging rate has been changed to: " << this->qt);
#endif
    ROS_GREEN_STREAM("Charging rate has been changed to: " << this->qt);
    lock.unlock();
    res.result = true;
    return true;
}

bool BatteryPlugin::SetCharge(brass_gazebo_battery::SetCharge::Request &req,
                              brass_gazebo_battery::SetCharge::Response &res) {
    lock.lock();
    if (req.charge <= this->c) {
        this->q = req.charge;
#ifdef BATTERY_DEBUG
        ROS_DEBUG_STREAM_NAMED("battery discharge", "Received charge:" << this->q);
#endif
        ROS_GREEN_STREAM("A new charge is set: " << this->q);
    } else {
        this->q = this->c;
        ROS_RED_STREAM("The charge cannot be higher than the capacity of the battery");
    }
    lock.unlock();
    res.result = true;
    return true;
}

bool BatteryPlugin::SetModelCoefficients(brass_gazebo_battery::SetCoef::Request &req,
                                         brass_gazebo_battery::SetCoef::Response &res) {
    lock.lock();
    this->e0 = req.constant_coef;
    this->e1 = req.linear_coef;
#ifdef BATTERY_DEBUG
    ROS_DEBUG_STREAM_NAMED("battery discharge",
                           "Power model is changed, new coefficients (constant, linear):" << this->e0 << this->e1);
#endif
    ROS_GREEN_STREAM("Power model is changed, new coefficients (constant, linear):" << this->e0 << this->e1);
    lock.unlock();
    res.result = true;
    return true;
}

void BatteryPlugin::BatteryResetCallback(const std_msgs::Empty &) {
    // Reset to the start current value.
    this->q = this->q0;
    this->charging = false;
    std_msgs::Bool power_msg;
    power_msg.data = false;
    lock.lock();
    this->motor_power.publish(power_msg);
    lock.unlock();
}