// ─────────────────────────────────────────────────────────────────────────────
//  Master_parking_v2.cpp  –  Estacionamiento en PARALELO (ROS 2 Humble)
//
//  Toda la lógica está en parking_core.hpp (sin ROS, probada en
//  test/test_parking.cpp). Este nodo solo lee tópicos, llama a step() a 20 Hz
//  y publica.
//
//  Suscribe:
//    /distance_center_line            std_msgs/Int16   (lane_detection)
//    /sensors/ultrasonic/{front,back,right}  sensor_msgs/Range
//    /imu                             sensor_msgs/Imu  (opcional, use_imu)
//  Publica:
//    /AutoModelMini/manual_control/steering|speed   std_msgs/Int16
//    /lights/{right_signal,blinkers,rear}           std_msgs/Bool
//    /parking/state                                 std_msgs/String
//
//  En simulación: use_sim_time:=true.
// ─────────────────────────────────────────────────────────────────────────────
#include <chrono>
#include <cmath>
#include <memory>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/int16.hpp"
#include "std_msgs/msg/string.hpp"
#include "sensor_msgs/msg/range.hpp"
#include "sensor_msgs/msg/imu.hpp"

#include "control/parking_core.hpp"

class MasterParkingV2 : public rclcpp::Node
{
public:
    MasterParkingV2() : rclcpp::Node("Master_parking_v2")
    {
        declare_params();
        core_ = std::make_unique<avim::ParkingCore>(P_);
        debug_ = declare_parameter("debug", false);
        const double rate = declare_parameter("rate_hz", 20.0);
        sonar_timeout_s_ = declare_parameter("sonar_timeout_s", 0.5);

        dist_sub_ = create_subscription<std_msgs::msg::Int16>("/distance_center_line", 1,
            [this](std_msgs::msg::Int16::SharedPtr m) { dist_px_ = m->data; dist_t_ = now_s(); });

        auto sonar = [this](const std::string& name, double* dst, double* t) {
            return create_subscription<sensor_msgs::msg::Range>(
                "/sensors/ultrasonic/" + name, rclcpp::SensorDataQoS(),
                [this, dst, t](sensor_msgs::msg::Range::SharedPtr m) {
                    double r = m->range;
                    if (!std::isfinite(r) || r <= 0.0 || r >= m->max_range - 0.01) r = INFINITY;
                    *dst = r; *t = now_s();
                });
        };
        subs_.push_back(sonar("front", &sF_, &sF_t_));
        subs_.push_back(sonar("back",  &sB_, &sB_t_));
        subs_.push_back(sonar("right", &sR_, &sR_t_));

        const std::string imu_topic = declare_parameter("imu_topic", std::string("/imu"));
        use_imu_ = declare_parameter("use_imu", true);
        if (use_imu_)
            imu_sub_ = create_subscription<sensor_msgs::msg::Imu>(imu_topic, rclcpp::SensorDataQoS(),
                [this](sensor_msgs::msg::Imu::SharedPtr m) { yaw_rate_ = m->angular_velocity.z; imu_t_ = now_s(); });

        steer_pub_ = create_publisher<std_msgs::msg::Int16>("/AutoModelMini/manual_control/steering", 10);
        speed_pub_ = create_publisher<std_msgs::msg::Int16>("/AutoModelMini/manual_control/speed", 10);
        right_pub_ = create_publisher<std_msgs::msg::Bool>("/lights/right_signal", 1);
        blink_pub_ = create_publisher<std_msgs::msg::Bool>("/lights/blinkers", 1);
        rear_pub_  = create_publisher<std_msgs::msg::Bool>("/lights/rear", 1);
        state_pub_ = create_publisher<std_msgs::msg::String>("/parking/state", rclcpp::QoS(1).transient_local());

        timer_ = create_wall_timer(std::chrono::duration<double>(1.0 / rate), [this]() { step(); });
        RCLCPP_INFO(get_logger(), "Master_parking_v2 listo | radio de giro %.2f m | %s",
                    core_->turn_radius(), use_imu_ ? "con IMU" : "sin IMU");
    }

private:
    avim::ParkParams P_;
    std::unique_ptr<avim::ParkingCore> core_;
    bool debug_ = false, use_imu_ = true;
    double sonar_timeout_s_ = 0.5;
    int dist_px_ = 0; double dist_t_ = -1e9;
    double sF_ = INFINITY, sB_ = INFINITY, sR_ = INFINITY, sF_t_ = -1e9, sB_t_ = -1e9, sR_t_ = -1e9;
    double yaw_rate_ = 0, imu_t_ = -1e9;
    bool right_on_ = false, blink_on_ = false, rear_on_ = false;
    std::string last_state_;

    rclcpp::Subscription<std_msgs::msg::Int16>::SharedPtr dist_sub_;
    std::vector<rclcpp::Subscription<sensor_msgs::msg::Range>::SharedPtr> subs_;
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
    rclcpp::Publisher<std_msgs::msg::Int16>::SharedPtr steer_pub_, speed_pub_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr right_pub_, blink_pub_, rear_pub_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr state_pub_;
    rclcpp::TimerBase::SharedPtr timer_;

    double now_s() { return this->now().seconds(); }

    void declare_params()
    {
        auto d = [this](const char* n, double& v) { v = declare_parameter(n, v); };
        auto i = [this](const char* n, int& v)    { v = (int)declare_parameter(n, (int64_t)v); };
        d("wheelbase_m", P_.wheelbase_m);           d("car_front_m", P_.car_front_m);
        d("car_rear_m", P_.car_rear_m);             d("car_half_width_m", P_.car_half_width_m);
        d("sonar_right_x_m", P_.sonar_right_x_m);   d("sonar_side_y_m", P_.sonar_side_y_m);
        d("steer_center_cmd", P_.steer_center_cmd); d("steer_deg_per_cmd", P_.steer_deg_per_cmd);
        d("steer_offset_deg", P_.steer_offset_deg); d("max_steer_deg", P_.max_steer_deg);
        d("steer_right_cmd", P_.steer_right_cmd);   d("steer_left_cmd", P_.steer_left_cmd);
        d("turn_radius_m", P_.turn_radius_m);       d("steer_settle_s", P_.steer_settle_s);
        d("steer_lag_s", P_.steer_lag_s);
        d("speed_mps_per_cmd", P_.speed_mps_per_cmd);
        i("search_cmd", P_.search_cmd);             i("approach_cmd", P_.approach_cmd);
        i("maneuver_cmd", P_.maneuver_cmd);         i("creep_cmd", P_.creep_cmd);
        d("stop_lag_s", P_.stop_lag_s);
        d("lk_thr_0", P_.lk_thr[0]); d("lk_thr_1", P_.lk_thr[1]); d("lk_thr_2", P_.lk_thr[2]); d("lk_thr_3", P_.lk_thr[3]);
        d("lk_kp_0", P_.lk_kp[0]);   d("lk_kp_1", P_.lk_kp[1]);   d("lk_kp_2", P_.lk_kp[2]);
        d("lk_kp_3", P_.lk_kp[3]);   d("lk_kp_4", P_.lk_kp[4]);
        d("lk_min_cmd", P_.lk_min_cmd);             d("lk_max_cmd", P_.lk_max_cmd);
        d("side_car_max_m", P_.side_car_max_m);     d("side_free_min_m", P_.side_free_min_m);
        d("min_spot_m", P_.min_spot_m);             d("max_spot_m", P_.max_spot_m);
        d("start_past_m", P_.start_past_m);         d("target_offset_m", P_.target_offset_m);
        d("ref_radius_m", P_.ref_radius_m);         d("wall_margin_m", P_.wall_margin_m);
        d("arc_deg", P_.arc_deg);
        d("back_stop_m", P_.back_stop_m);           d("front_stop_m", P_.front_stop_m);
        d("side_stop_m", P_.side_stop_m);
        d("straight_tol_deg", P_.straight_tol_deg); i("max_fixes", P_.max_fixes);
        d("center_tol_m", P_.center_tol_m);         d("center_timeout_s", P_.center_timeout_s);
        d("speed_lag_s", P_.speed_lag_s);
        // corrección final de profundidad con el ultrasónico derecho (sin IMU)
        d("depth_tol_m", P_.depth_tol_m);           i("max_depth_fixes", P_.max_depth_fixes);
        d("final_wall_margin_m", P_.final_wall_margin_m);
        d("depth_bias_m", P_.depth_bias_m);
    }

    void step()
    {
        const double t = now_s();
        avim::ParkInputs in;
        in.t = t;
        in.has_dist = (t - dist_t_) < 0.5;
        in.dist_px = dist_px_;
        auto fresh = [&](double v, double ts) { return (t - ts) < sonar_timeout_s_ ? v : INFINITY; };
        in.sonar_front = fresh(sF_, sF_t_);
        in.sonar_back  = fresh(sB_, sB_t_);
        in.sonar_right = fresh(sR_, sR_t_);
        in.sonar_right_t = sR_t_;
        in.has_yaw_rate = use_imu_ && (t - imu_t_) < 0.2;
        in.yaw_rate = yaw_rate_;

        const avim::ParkOutputs o = core_->step(in);

        std_msgs::msg::Int16 a; a.data = (int16_t)o.steer_cmd;
        std_msgs::msg::Int16 s; s.data = (int16_t)o.speed_cmd;
        steer_pub_->publish(a); speed_pub_->publish(s);
        light(right_pub_, right_on_, o.right_signal);
        light(blink_pub_, blink_on_, o.blinkers);
        light(rear_pub_, rear_on_, o.speed_cmd >= 0);     // freno / reversa

        if (!o.event.empty()) RCLCPP_INFO(get_logger(), "[%s] %s", o.state, o.event.c_str());
        if (last_state_ != o.state) {
            std_msgs::msg::String m; m.data = o.state; state_pub_->publish(m);
            last_state_ = o.state;
        }
        if (debug_)
            RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 500,
                "%-8s steer=%3d speed=%4d | dist=%d | sonar F=%.2f B=%.2f R=%.2f | rumbo=%+.1f recorrido=%.2f | %s",
                o.state, o.steer_cmd, o.speed_cmd, in.dist_px, in.sonar_front, in.sonar_back, in.sonar_right,
                core_->heading_deg(), core_->travel(), in.has_yaw_rate ? "IMU" : "sinIMU");
    }

    static void light(const rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr& pub, bool& cur, bool want) {
        if (cur == want) return;
        cur = want;
        std_msgs::msg::Bool m; m.data = want; pub->publish(m);
    }
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<MasterParkingV2>());
    rclcpp::shutdown();
    return 0;
}
