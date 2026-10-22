// ─────────────────────────────────────────────────────────────────────────────
//  Master_overtake.cpp  –  Master de rebase ROBUSTO para el AVIM (ROS 2 Humble)
//
//  Reemplaza a Master_static para la prueba de obstáculos estáticos.
//  Toda la lógica vive en overtake_core.hpp (sin ROS) y se probó en un banco
//  de pruebas cinemático (test_overtake.cpp) con cientos de escenarios.
//  Este nodo solo: lee tópicos → arma Inputs → core.step() → publica.
//
//  Suscribe:
//    /distance_center_line      std_msgs/Int16                (lane_detection)
//    /lane/left_line            nav_msgs/Path                 (lane_detection, nuevo)
//    /lane/right_line           nav_msgs/Path                 (lane_detection, nuevo)
//    /objects_points            object_detection/PointsObjects (object_detection)
//    /sensors/ultrasonic/{front,back,left,right}  sensor_msgs/Range
//    /stop_sign_detected        std_msgs/Bool     (detector viejo: alto inmediato)
//    /stop_sign_position        geometry_msgs/PointStamped (detector v2: alto en el cruce)
//    /stop_sign_distance        std_msgs/Float32  (igual, si no hay /stop_sign_position)
//  Publica:
//    /AutoModelMini/manual_control/steering   std_msgs/Int16
//    /AutoModelMini/manual_control/speed      std_msgs/Int16
//    /lights/{left_signal,right_signal,blinkers,rear}   std_msgs/Bool
//    /overtake/state            std_msgs/String  (estado actual, para depurar)
//
//  En simulación lánzalo con use_sim_time:=true (los tiempos son de Gazebo).
// ─────────────────────────────────────────────────────────────────────────────
#include <chrono>
#include <cstdio>
#include <cmath>
#include <limits>
#include <memory>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/float32.hpp"
#include "geometry_msgs/msg/point_stamped.hpp"
#include "std_msgs/msg/int16.hpp"
#include "std_msgs/msg/int8.hpp"
#include "std_msgs/msg/string.hpp"
#include "sensor_msgs/msg/range.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "nav_msgs/msg/path.hpp"
#include "object_detection/msg/points_objects.hpp"

#include "control/overtake_core.hpp"

using namespace std::chrono_literals;

class MasterOvertake : public rclcpp::Node
{
public:
    MasterOvertake() : rclcpp::Node("Master_overtake")
    {
        declare_params();
        core_ = std::make_unique<avim::OvertakeCore>(P_);

        rate_hz_ = this->declare_parameter("rate_hz", 20.0);
        debug_   = this->declare_parameter("debug", false);

        // ── Suscripciones ────────────────────────────────────────────────
        // El ciclo de control corre con cada /distance_center_line (como tu
        // Master: menos retraso). El timer solo lo cubre si ese tópico se detiene.
        dist_sub_ = create_subscription<std_msgs::msg::Int16>(
            "/distance_center_line", 1,
            [this](std_msgs::msg::Int16::SharedPtr m) { dist_px_ = m->data; dist_t_ = now_s(); step(); });

        // IMU (giroscopio): hace la odometría mucho más confiable. En Gazebo el
        // modelo publica /imu; si no existe, se usa el modelo de la dirección.
        const std::string imu_topic = this->declare_parameter("imu_topic", std::string("/imu"));
        use_imu_ = this->declare_parameter("use_imu", true);
        if (use_imu_)
            imu_sub_ = create_subscription<sensor_msgs::msg::Imu>(
                imu_topic, rclcpp::SensorDataQoS(),
                [this](sensor_msgs::msg::Imu::SharedPtr m) {
                    yaw_rate_ = m->angular_velocity.z; imu_t_ = now_s();
                    if (!imu_seen_) { imu_seen_ = true; RCLCPP_INFO(get_logger(), "IMU detectada: odometria con giro medido"); }
                });

        left_sub_ = create_subscription<nav_msgs::msg::Path>(
            "/lane/left_line", 1,
            [this](nav_msgs::msg::Path::SharedPtr m) { left_ = path_to_car(*m); left_t_ = now_s(); lane_new_ = true; });
        right_sub_ = create_subscription<nav_msgs::msg::Path>(
            "/lane/right_line", 1,
            [this](nav_msgs::msg::Path::SharedPtr m) { right_ = path_to_car(*m); right_t_ = now_s(); lane_new_ = true; });

        // continua (1) / punteada (2) de cada línea
        ltype_sub_ = create_subscription<std_msgs::msg::Int8>("/lane/left_type", 1,
            [this](std_msgs::msg::Int8::SharedPtr m) { ltype_ = m->data; ltype_t_ = now_s(); });
        rtype_sub_ = create_subscription<std_msgs::msg::Int8>("/lane/right_type", 1,
            [this](std_msgs::msg::Int8::SharedPtr m) { rtype_ = m->data; rtype_t_ = now_s(); });

        obj_sub_ = create_subscription<object_detection::msg::PointsObjects>(
            "/objects_points", 1,
            std::bind(&MasterOvertake::objects_cb, this, std::placeholders::_1));

        auto sonar = [this](const std::string& name, double* dst, double* t) {
            return create_subscription<sensor_msgs::msg::Range>(
                "/sensors/ultrasonic/" + name, rclcpp::SensorDataQoS(),
                [this, dst, t](sensor_msgs::msg::Range::SharedPtr m) {
                    double r = m->range;
                    // Sin eco: Gazebo da inf; el driver real da max_range
                    if (!std::isfinite(r) || r <= 0.0 || r >= m->max_range - 0.01) r = INFINITY;
                    *dst = r; *t = now_s();
                });
        };
        sonar_subs_.push_back(sonar("front", &sF_, &sF_t_));
        sonar_subs_.push_back(sonar("back",  &sB_, &sB_t_));
        sonar_subs_.push_back(sonar("left",  &sL_, &sL_t_));
        sonar_subs_.push_back(sonar("right", &sR_, &sR_t_));

        stop_sub_ = create_subscription<std_msgs::msg::Bool>(
            "/stop_sign_detected", 1,
            [this](std_msgs::msg::Bool::SharedPtr m) { if (m->data) stop_t_ = now_s(); });
        // Detector v2: distancia (m) de la cámara a la señal, cada cuadro en que la ve
        stop_dist_sub_ = create_subscription<std_msgs::msg::Float32>(
            "/stop_sign_distance", 5,
            [this](std_msgs::msg::Float32::SharedPtr m) {
                if (stop_pos_seen_) return;           // se usa /stop_sign_position (trae también el lateral)
                stop_dist_ = m->data; stop_lat_ = 0.0; stop_dist_new_ = true;
                if (!stop_dist_seen_) RCLCPP_INFO(get_logger(), "Recibo /stop_sign_distance: alto en el cruce (no al ver la senal)");
                stop_dist_seen_ = true; });
        stop_pos_sub_ = create_subscription<geometry_msgs::msg::PointStamped>(
            "/stop_sign_position", 5,
            [this](geometry_msgs::msg::PointStamped::SharedPtr m) {
                stop_dist_ = m->point.x; stop_lat_ = m->point.y; stop_dist_new_ = true;
                if (!stop_dist_seen_) RCLCPP_INFO(get_logger(), "Recibo /stop_sign_position: alto en el cruce (no al ver la senal)");
                stop_dist_seen_ = stop_pos_seen_ = true; });

        // ── Publicadores ─────────────────────────────────────────────────
        steer_pub_ = create_publisher<std_msgs::msg::Int16>("/AutoModelMini/manual_control/steering", 10);
        speed_pub_ = create_publisher<std_msgs::msg::Int16>("/AutoModelMini/manual_control/speed", 10);
        left_sig_pub_  = create_publisher<std_msgs::msg::Bool>("/lights/left_signal", 1);
        right_sig_pub_ = create_publisher<std_msgs::msg::Bool>("/lights/right_signal", 1);
        blink_pub_     = create_publisher<std_msgs::msg::Bool>("/lights/blinkers", 1);
        rear_pub_      = create_publisher<std_msgs::msg::Bool>("/lights/rear", 1);
        // transient_local: lane_detection recibe el estado actual aunque arranque después
        state_pub_     = create_publisher<std_msgs::msg::String>("/overtake/state", rclcpp::QoS(1).transient_local());

        timer_ = create_wall_timer(std::chrono::duration<double>(1.0 / rate_hz_), [this]() {
            if (now_s() - last_step_t_ > 1.5 / rate_hz_) step();
        });

        RCLCPP_INFO(get_logger(),
            "Master_overtake listo (%.0f Hz, use_sim_time=%s, lane_keep_mode=%d)",
            rate_hz_, get_parameter("use_sim_time").as_bool() ? "true" : "false", P_.lane_keep_mode);
    }

private:
    // ── Parámetros: todos los campos de avim::Params ─────────────────────
    avim::Params P_;
    void declare_params()
    {
        auto d = [this](const char* n, double& v) { v = this->declare_parameter(n, v); };
        auto i = [this](const char* n, int& v)    { v = (int)this->declare_parameter(n, (int64_t)v); };
        d("wheelbase_m", P_.wheelbase_m);           d("car_front_m", P_.car_front_m);
        d("car_rear_m", P_.car_rear_m);             d("car_half_width_m", P_.car_half_width_m);
        d("lane_cam_x_m", P_.lane_cam_x_m);         d("depth_cam_x_m", P_.depth_cam_x_m);
        d("depth_hfov_deg", P_.depth_hfov_deg);     d("depth_range_m", P_.depth_range_m);
        d("sonar_right_x_m", P_.sonar_right_x_m);
        d("lane_width_m", P_.lane_width_m);         d("obstacle_length_m", P_.obstacle_length_m);
        d("steer_center_cmd", P_.steer_center_cmd); d("steer_deg_per_cmd", P_.steer_deg_per_cmd);
        d("steer_offset_deg", P_.steer_offset_deg); d("steer_cmd_min", P_.steer_cmd_min);
        d("steer_cmd_max", P_.steer_cmd_max);       d("max_steer_deg", P_.max_steer_deg);
        d("speed_mps_per_cmd", P_.speed_mps_per_cmd);
        i("cruise_cmd", P_.cruise_cmd);             d("kp_speed", P_.kp_speed);
        i("approach_cmd", P_.approach_cmd);         i("change_cmd", P_.change_cmd);
        i("pass_cmd", P_.pass_cmd);                 i("reverse_cmd", P_.reverse_cmd);
        d("lk_thr_0", P_.lk_thr[0]); d("lk_thr_1", P_.lk_thr[1]); d("lk_thr_2", P_.lk_thr[2]); d("lk_thr_3", P_.lk_thr[3]);
        d("lk_kp_0", P_.lk_kp[0]);   d("lk_kp_1", P_.lk_kp[1]);   d("lk_kp_2", P_.lk_kp[2]);
        d("lk_kp_3", P_.lk_kp[3]);   d("lk_kp_4", P_.lk_kp[4]);
        d("lk_min_cmd", P_.lk_min_cmd);             d("lk_max_cmd", P_.lk_max_cmd);
        i("lane_keep_mode", P_.lane_keep_mode);
        d("lookahead_m", P_.lookahead_m);
        d("detect_range_m", P_.detect_range_m);     d("slow_gap_m", P_.slow_gap_m);
        d("trigger_gap_m", P_.trigger_gap_m);       d("min_change_gap_m", P_.min_change_gap_m);
        d("feasible_margin_m", P_.feasible_margin_m); d("feasible_lag_m", P_.feasible_lag_m);
        d("gap_uncertainty_m", P_.gap_uncertainty_m);
        d("backoff_target_m", P_.backoff_target_m); d("keep_gap_m", P_.keep_gap_m);
        d("emergency_gap_m", P_.emergency_gap_m);   i("confirm_frames", P_.confirm_frames);
        d("mem_match_m", P_.mem_match_m);           i("mem_min_hits", P_.mem_min_hits);
        d("mem_unseen_s", P_.mem_unseen_s);         d("mem_max_s", P_.mem_max_s);
        d("in_lane_frac", P_.in_lane_frac);         d("left_lane_check_m", P_.left_lane_check_m);
        d("abort_return_gap_m", P_.abort_return_gap_m);
        d("return_check_m", P_.return_check_m);
        d("return_visible_from_m", P_.return_visible_from_m); d("return_visible_m", P_.return_visible_m);
        d("objects_timeout_s", P_.objects_timeout_s); d("lane_timeout_s", P_.lane_timeout_s);
        d("side_detect_m", P_.side_detect_m);       d("side_clear_extra_m", P_.side_clear_extra_m);
        d("odom_pass_margin_m", P_.odom_pass_margin_m); d("side_sonar_min_free_m", P_.side_sonar_min_free_m);
        d("change_length_m", P_.change_length_m);   d("return_length_m", P_.return_length_m);
        d("change_done_lat_m", P_.change_done_lat_m); d("change_done_head", P_.change_done_head);
        i("change_done_frames", P_.change_done_frames);
        d("anchor_tol_m", P_.anchor_tol_m);         d("fit_max_x_m", P_.fit_max_x_m);
        d("anchor_timeout_s", P_.anchor_timeout_s); d("settle_timeout_s", P_.settle_timeout_s);
        d("change_timeout_s", P_.change_timeout_s); d("pass_timeout_s", P_.pass_timeout_s);
        d("backoff_timeout_s", P_.backoff_timeout_s); d("front_sonar_stop_m", P_.front_sonar_stop_m);
        d("blocked_timeout_s", P_.blocked_timeout_s);
        d("blind_max_s", P_.blind_max_s);           d("anchor_tol_growth", P_.anchor_tol_growth);
        d("return_max_kappa", P_.return_max_kappa); d("steer_lag_s", P_.steer_lag_s);
        d("type_tol_m", P_.type_tol_m);             i("type_confirm_frames", P_.type_confirm_frames);
        i("wrong_lane_frames", P_.wrong_lane_frames); d("maneuver_max_steer_deg", P_.maneuver_max_steer_deg);
        P_.use_line_types = this->declare_parameter("use_line_types", P_.use_line_types);
        P_.auto_speed_scale = this->declare_parameter("auto_speed_scale", P_.auto_speed_scale);
        P_.auto_steer_gain  = this->declare_parameter("auto_steer_gain",  P_.auto_steer_gain);
        d("stop_time_s", P_.stop_time_s);           d("stop_cooldown_s", P_.stop_cooldown_s);
        d("stop_cam_x_m", P_.stop_cam_x_m);         d("stop_before_m", P_.stop_before_m);
        d("stop_slow_m", P_.stop_slow_m);           i("stop_approach_cmd", P_.stop_approach_cmd);
        i("stop_final_cmd", P_.stop_final_cmd);     d("stop_same_sign_m", P_.stop_same_sign_m);
        d("stop_new_sign_m", P_.stop_new_sign_m);   d("stop_overshoot_m", P_.stop_overshoot_m);
        d("stop_lag_s", P_.stop_lag_s);
        d("sign_ignore_m", P_.sign_ignore_m);       d("cross_length_m", P_.cross_length_m);
        d("stop_freeze_m", P_.stop_freeze_m);
        d("sonar_confirm_m", P_.sonar_confirm_m);
        sonar_timeout_s_ = this->declare_parameter("sonar_timeout_s", 0.5);
        min_angle_dev_deg_ = this->declare_parameter("objects_max_angle_dev_deg", 60.0);
    }

    // ── Conversión de mensajes al marco del EJE TRASERO (m, x adelante, y izq.)
    std::vector<avim::Vec2> path_to_car(const nav_msgs::msg::Path& p) const
    {
        std::vector<avim::Vec2> v;
        v.reserve(p.poses.size());
        for (const auto& ps : p.poses)   // marco "lane_camera_ground": origen bajo la cámara
            v.push_back({ ps.pose.position.x + P_.lane_cam_x_m, ps.pose.position.y });
        return v;
    }

    void objects_cb(const object_detection::msg::PointsObjects::SharedPtr msg)
    {
        // Cada punto: x = profundidad (cm), y = ángulo (°; 90 = al frente, >90 = derecha)
        objects_.clear();
        for (const auto& p : msg->points) {
            const double z = p.x / 100.0;
            const double dev = 90.0 - p.y;                  // + = izquierda
            if (z <= 0.0 || std::fabs(dev) > min_angle_dev_deg_) continue;
            const double lat = z * std::tan(dev * M_PI / 180.0);
            objects_.push_back({ z + P_.depth_cam_x_m, lat });
        }
        objects_t_ = now_s();
        objects_new_ = true;
    }

    // ── Ciclo de control ─────────────────────────────────────────────────
    void step()
    {
        const double t = now_s();
        if (t <= 0.0) return;                            // todavía sin /clock
        if (t - last_step_t_ < 0.02) return;             // máx. ~50 Hz
        last_step_t_ = t;

        avim::Inputs in;
        in.t = t;
        in.has_dist = (t - dist_t_) < 0.3;
        in.dist_px  = dist_px_;

        in.lane_new = lane_new_;
        lane_new_ = false;
        if (t - left_t_  < 0.15) in.left_line  = left_;
        if (t - right_t_ < 0.15) in.right_line = right_;
        in.left_type  = (t - ltype_t_ < 0.15) ? ltype_ : 0;
        in.right_type = (t - rtype_t_ < 0.15) ? rtype_ : 0;

        in.objects_fresh = (t - objects_t_) < P_.objects_timeout_s;
        in.objects_new   = objects_new_;
        objects_new_ = false;
        if (in.objects_fresh) in.obstacles = objects_;

        auto fresh = [&](double v, double ts) { return (t - ts) < sonar_timeout_s_ ? v : INFINITY; };
        in.sonar_front = fresh(sF_, sF_t_);
        in.sonar_back  = fresh(sB_, sB_t_);
        in.sonar_left  = fresh(sL_, sL_t_);
        in.sonar_right = fresh(sR_, sR_t_);

        // Con el detector v2 (publica la distancia) se usa la distancia; el Bool
        // solo cuenta si nunca llegó una distancia (detector viejo). Se espera
        // 0.3 s tras el Bool por si la distancia llega justo después.
        in.stop_sign = !stop_dist_seen_ && (t - stop_t_) > 0.3 && (t - stop_t_) < 0.8;
        in.stop_dist = stop_dist_new_ ? stop_dist_ : NAN;
        in.stop_lat  = stop_lat_;
        stop_dist_new_ = false;

        in.has_yaw_rate = use_imu_ && (t - imu_t_) < 0.2;
        in.yaw_rate = yaw_rate_;

        const avim::Outputs o = core_->step(in);

        publish_cmd(o.steer_cmd, o.speed_cmd);
        set_light(left_sig_pub_,  left_on_,  o.left_signal);
        set_light(right_sig_pub_, right_on_, o.right_signal);
        set_light(blink_pub_,     blink_on_, o.blinkers);
        set_light(rear_pub_,      rear_on_,  o.speed_cmd == 0 || (last_speed_ < 0 && o.speed_cmd > last_speed_ + 50));
        last_speed_ = o.speed_cmd;

        if (!o.event.empty()) RCLCPP_INFO(get_logger(), "[%s] %s", o.state, o.event.c_str());
        if (std::string(o.state) != last_state_) {
            std_msgs::msg::String s; s.data = o.state; state_pub_->publish(s);
            last_state_ = o.state;
        }
        // Diagnóstico: "obstáculo" pegado al cofre que el ultrasónico de enfrente
        // no ve => casi seguro es el PISO o el cofre (filtro de suelo de
        // object_detection mal calibrado / parámetros que no se pasaron)
        for (const auto& ob : in.obstacles) {
            const double gap = std::hypot(ob.x, ob.y) - P_.car_front_m;
            if (gap < 0.35 && std::fabs(ob.y) < 0.15 && in.sonar_front > 0.8) {
                RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
                    "Obstaculo a %.2f m de la defensa (lat %.2f) pero el ultrasonico de enfrente "
                    "no ve nada: probablemente es el PISO/cofre. Revisa horizon_row, camera_height_m "
                    "y v_end_ratio de object_detection (ros2 topic echo /objects_points con el carro quieto).",
                    gap, ob.y);
                break;
            }
        }

        if (debug_)
            RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 500,
                "%-12s steer=%3d speed=%4d | dist=%s%d L=%zu%s R=%zu%s obs=%zu | sonar F=%.2f B=%.2f L=%.2f R=%.2f | %s vesc=%.2f gdir=%.2f | carro lat=%+.2f rumbo=%+.0f modelo=%.1fs | alto en %s | obst %s",
                o.state, o.steer_cmd, o.speed_cmd, in.has_dist ? "" : "(viejo)", in.dist_px,
                in.left_line.size(), type_txt(in.left_type), in.right_line.size(), type_txt(in.right_type), in.obstacles.size(),
                in.sonar_front, in.sonar_back, in.sonar_left, in.sonar_right,
                in.has_yaw_rate ? "IMU" : "sinIMU", core_->speed_scale(), core_->steer_gain(),
                core_->debug_lateral(), core_->debug_heading_deg(), core_->debug_model_age(t),
                stop_txt(core_->stop_remaining()).c_str(), core_->debug_obs_txt().c_str());
    }

    void publish_cmd(int steer, int speed)
    {
        std_msgs::msg::Int16 a; a.data = (int16_t)steer;
        std_msgs::msg::Int16 s; s.data = (int16_t)speed;
        if (steer_pub_) steer_pub_->publish(a);
        if (speed_pub_) speed_pub_->publish(s);
    }
    static void set_light(const rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr& pub, bool& cur, bool want)
    {
        if (cur == want) return;
        cur = want;
        std_msgs::msg::Bool m; m.data = want; pub->publish(m);
    }
    double now_s() { return this->now().seconds(); }
    static std::string stop_txt(double rem) {
        if (!std::isfinite(rem)) return "-";
        char b[24]; std::snprintf(b, sizeof b, "%.2fm", rem); return b; }
    static const char* type_txt(int t) { return t == 2 ? "p" : (t == 1 ? "c" : ""); }   // p = punteada, c = continua

    // ── Estado ───────────────────────────────────────────────────────────
    std::unique_ptr<avim::OvertakeCore> core_;
    double rate_hz_ = 20.0, sonar_timeout_s_ = 0.5, min_angle_dev_deg_ = 60.0;
    bool   debug_ = false;

    int    dist_px_ = 0;            double dist_t_ = -1e9;
    std::vector<avim::Vec2> left_, right_;
    double left_t_ = -1e9, right_t_ = -1e9;
    bool   lane_new_ = false;
    std::vector<avim::Obstacle> objects_;
    double objects_t_ = -1e9;       bool objects_new_ = false;
    double sF_ = INFINITY, sB_ = INFINITY, sL_ = INFINITY, sR_ = INFINITY;
    double sF_t_ = -1e9, sB_t_ = -1e9, sL_t_ = -1e9, sR_t_ = -1e9;
    double stop_t_ = -1e9;
    double stop_dist_ = NAN, stop_lat_ = 0.0;
    bool   stop_dist_new_ = false, stop_dist_seen_ = false, stop_pos_seen_ = false;
    int    ltype_ = 0, rtype_ = 0;  double ltype_t_ = -1e9, rtype_t_ = -1e9;
    bool   use_imu_ = true, imu_seen_ = false;
    double yaw_rate_ = 0.0, imu_t_ = -1e9, last_step_t_ = -1e9;

    bool left_on_ = false, right_on_ = false, blink_on_ = false, rear_on_ = false;
    int  last_speed_ = 0;
    std::string last_state_;

    rclcpp::Subscription<std_msgs::msg::Int16>::SharedPtr dist_sub_;
    rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr left_sub_, right_sub_;
    rclcpp::Subscription<object_detection::msg::PointsObjects>::SharedPtr obj_sub_;
    std::vector<rclcpp::Subscription<sensor_msgs::msg::Range>::SharedPtr> sonar_subs_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr stop_sub_;
    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr stop_dist_sub_;
    rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr stop_pos_sub_;
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
    rclcpp::Subscription<std_msgs::msg::Int8>::SharedPtr ltype_sub_, rtype_sub_;
    rclcpp::Publisher<std_msgs::msg::Int16>::SharedPtr steer_pub_, speed_pub_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr left_sig_pub_, right_sig_pub_, blink_pub_, rear_pub_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr state_pub_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<MasterOvertake>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
