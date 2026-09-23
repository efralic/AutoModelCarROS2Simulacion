// ─────────────────────────────────────────────────────────────────────────────
//  Master.cpp  –  Prueba 1 / Prueba 3 (obstáculos dinámicos)
//  Migrado de ROS 1 (Noetic) → ROS 2 (Humble / Iron)
//
//  Cambios principales respecto a la versión ROS 1:
//   • ros::NodeHandle  →  herencia de rclcpp::Node
//   • nh_.subscribe / advertise  →  create_subscription / create_publisher
//   • ROS_INFO / ROS_WARN  →  RCLCPP_INFO / RCLCPP_WARN (con get_logger())
//   • Tipos de mensajes: std_msgs::Int16  →  std_msgs::msg::Int16
//   • Firmas de callbacks: ConstPtr  →  SharedPtr  /  valor → const ref
//   • ros::init + ros::spin  →  rclcpp::init + rclcpp::spin
//   • Custom msg: object_detection::points_objects
//                 → object_detection::msg::PointsObjects
// ─────────────────────────────────────────────────────────────────────────────

#include <chrono>
#include <map>
#include <memory>
#include <string>
#include <thread>
#include <vector>
#include <cmath>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/int16.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "object_detection/msg/points_objects.hpp"   // msg custom (ROS 2)

// ── Estados ───────────────────────────────────────────────────────────────────
const int LANE_DRIVING    = 0;
const int FOLLOWING       = 1;
const int MOVING_LEFT     = 2;
const int PASSING         = 3;
const int MOVING_RIGHT    = 4;
const int MOVING_RIGHT_LANE = 5;
const int STOP_AT_SIGN    = 6;

static std::map<int, std::string> task_names {
    { LANE_DRIVING,     "Lane driving"              },
    { FOLLOWING,        "Following"                 },
    { MOVING_LEFT,      "Moving to left lane"       },
    { PASSING,          "Passing obstacle"          },
    { MOVING_RIGHT,     "Returning right lane"      },
    { MOVING_RIGHT_LANE,"Returning right lane/lane" },
    { STOP_AT_SIGN,     "Stopping at STOP sign"     },
};


class Task {
public:
    std::string name;
    int ID;
    explicit Task(int id) : ID(id), name(task_names[id]) {}
};


// ─────────────────────────────────────────────────────────────────────────────
//  Clase Master  –  hereda de rclcpp::Node (reemplaza ros::NodeHandle)
// ─────────────────────────────────────────────────────────────────────────────
class Master : public rclcpp::Node
{
public:
    Master(Task task, bool PASSING_ENABLED, int MAX_WAIT_TIME, float DIST_TO_KEEP)
    : rclcpp::Node("Master")
    {
        // ── Subscriptores ────────────────────────────────────────────────────
        // ROS 1: nh_.subscribe(topic, queue, &Cb, this)
        // ROS 2: create_subscription<T>(topic, qos, callback)
        distance_center_ = this->create_subscription<std_msgs::msg::Int16>(
            "/distance_center_line", 1,
            std::bind(&Master::dist_center_clbk, this, std::placeholders::_1));

        object_detection_sub_ = this->create_subscription<
            object_detection::msg::PointsObjects>(
            "/objects_points", 1,
            std::bind(&Master::object_detec_clbk, this, std::placeholders::_1));

        stop_sign_sub_ = this->create_subscription<std_msgs::msg::Bool>(
            "/stop_sign_detected", 1,
            std::bind(&Master::stop_sign_callback, this, std::placeholders::_1));

        // ── Publicadores ─────────────────────────────────────────────────────
        angle_pub_        = this->create_publisher<std_msgs::msg::Int16>(
                                "/AutoModelMini/manual_control/steering", 1000);
        speed_pub_        = this->create_publisher<std_msgs::msg::Int16>(
                                "/AutoModelMini/manual_control/speed",    1000);
        left_signal_pub_  = this->create_publisher<std_msgs::msg::Bool>(
                                "/lights/left_signal",  1);
        right_signal_pub_ = this->create_publisher<std_msgs::msg::Bool>(
                                "/lights/right_signal", 1);
        blinkers_pub_     = this->create_publisher<std_msgs::msg::Bool>(
                                "/lights/blinkers",     1);
        rear_lights_pub_  = this->create_publisher<std_msgs::msg::Bool>(
                                "/lights/rear",         1);

        // ── Parámetros ───────────────────────────────────────────────────────
        num_found_objects_    = 0;
        dist_now_ = dist_last_ = 0;
        kp_angle_ = 1.15f;   kd_angle_ = 0.045f;
        kp_speed_ = 2.4462f;
        u_angle_ = u_speed_ = speed_pid_ = angle_pd_ = 0;
        count_ = count_pass_ = 0;
        mid_speed_            = 1035;
        vel_decreasing_factor_= -15;
        dist_to_keep_         = DIST_TO_KEEP;
        max_waiting_time_     = MAX_WAIT_TIME;
        passing_enabled_      = PASSING_ENABLED;
        stop_sign_detected_   = false;
        stop_initiated_       = false;
        left_signal_on_  = right_signal_on_ = blinkers_on_ = rear_lights_on_ = false;
        last_task_            = std::make_unique<Task>(LANE_DRIVING);

        add_task(task);
    }

private:
    // ── Declaración de miembros ───────────────────────────────────────────────
    rclcpp::Subscription<std_msgs::msg::Int16>::SharedPtr           distance_center_;
    rclcpp::Subscription<object_detection::msg::PointsObjects>::SharedPtr object_detection_sub_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr            stop_sign_sub_;

    rclcpp::Publisher<std_msgs::msg::Int16>::SharedPtr angle_pub_;
    rclcpp::Publisher<std_msgs::msg::Int16>::SharedPtr speed_pub_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr  left_signal_pub_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr  right_signal_pub_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr  blinkers_pub_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr  rear_lights_pub_;

    std::vector<geometry_msgs::msg::Point> found_objects_;
    int   num_found_objects_;
    int   dist_now_, dist_last_;
    int   angle_pd_, speed_pid_;
    float kp_angle_, kd_angle_, kp_speed_;
    int   u_angle_, u_speed_;
    bool  passing_enabled_;
    int   time_long_;
    std::vector<Task> task_pile_;
    std::unique_ptr<Task> last_task_;
    int   count_, count_pass_;
    int   max_waiting_time_;
    float dist_to_keep_;
    int   vel_decreasing_factor_;
    int   mid_speed_;
    bool  stop_sign_detected_, stop_initiated_;
    bool  left_signal_on_, right_signal_on_, blinkers_on_, rear_lights_on_;

    std::chrono::steady_clock::time_point start_, end_, stop_start_time_;

    // ── Helpers de luces ──────────────────────────────────────────────────────
    void set_left_signal(bool state) {
        if (left_signal_on_ == state) return;
        left_signal_on_ = state;
        auto msg = std_msgs::msg::Bool(); msg.data = state;
        left_signal_pub_->publish(msg);
        RCLCPP_INFO(get_logger(), state ? "<- Left signal ON" : "<- Left signal OFF");
    }
    void set_right_signal(bool state) {
        if (right_signal_on_ == state) return;
        right_signal_on_ = state;
        auto msg = std_msgs::msg::Bool(); msg.data = state;
        right_signal_pub_->publish(msg);
        RCLCPP_INFO(get_logger(), state ? "-> Right signal ON" : "-> Right signal OFF");
    }
    void set_blinkers(bool state) {
        if (blinkers_on_ == state) return;
        blinkers_on_ = state;
        auto msg = std_msgs::msg::Bool(); msg.data = state;
        blinkers_pub_->publish(msg);
        RCLCPP_INFO(get_logger(), state ? "Blinkers ON" : "Blinkers OFF");
    }
    void update_rear_brake_lights() {
        bool braking = (speed_pid_ == 0);
        if (rear_lights_on_ == braking) return;
        rear_lights_on_ = braking;
        auto msg = std_msgs::msg::Bool(); msg.data = braking;
        rear_lights_pub_->publish(msg);
        RCLCPP_INFO(get_logger(), braking ? "Brake lights ON" : "Brake lights OFF");
    }

    // ── Callbacks ─────────────────────────────────────────────────────────────
    // ROS 1: void cb(const std_msgs::Bool::ConstPtr& msg)
    // ROS 2: void cb(const std_msgs::msg::Bool::SharedPtr msg)
    void stop_sign_callback(const std_msgs::msg::Bool::SharedPtr msg) {
        stop_sign_detected_ = msg->data;
        if (stop_sign_detected_)
            RCLCPP_INFO(get_logger(), "STOP SIGN DETECTED!");
    }

    // ROS 1: void cb(const std_msgs::Int16& msg)   ← valor
    // ROS 2: void cb(const std_msgs::msg::Int16::SharedPtr msg)  ← SharedPtr
    void dist_center_clbk(const std_msgs::msg::Int16::SharedPtr msg) {
        dist_now_ = static_cast<int>(msg->data);
        run();
        publish_policies();
    }

    void object_detec_clbk(
        const object_detection::msg::PointsObjects::SharedPtr msg)
    {
        found_objects_.clear();
        if (msg->points.empty()) { num_found_objects_ = 0; return; }
        std::map<float, float> points_order;
        for (auto & p : msg->points)
            points_order.emplace(p.x, p.y);
        auto it = points_order.begin();
        geometry_msgs::msg::Point pt;
        pt.x = it->first; pt.y = it->second;
        found_objects_.push_back(pt);
        num_found_objects_ = static_cast<int>(found_objects_.size());
    }

    // ── Task management ───────────────────────────────────────────────────────
    void add_task(Task t)  { task_pile_.push_back(t); }
    void remove_task()     { if (task_pile_.size() > 1) task_pile_.pop_back(); }
    Task get_current_task(){ return task_pile_.back(); }

    void task_assigner() {
        Task current = get_current_task();
        if (stop_sign_detected_ && current.ID == LANE_DRIVING) {
            add_task(Task(STOP_AT_SIGN));
            stop_sign_detected_ = false;
            return;
        }
        if (num_found_objects_ > 0) {
            for (auto & obs : found_objects_) {
                if (obs.y > 70.0 && obs.y < 110.0 && obs.x < 119.0) {
                    if (current.ID == LANE_DRIVING) {
                        add_task(Task(FOLLOWING));
                        last_task_ = std::make_unique<Task>(FOLLOWING);
                        break;
                    }
                }
                if (current.ID == FOLLOWING && count_ > max_waiting_time_) {
                    count_pass_++;
                    count_ = 0;
                    add_task(Task(MOVING_RIGHT_LANE));
                    add_task(Task(MOVING_RIGHT));
                    add_task(Task(PASSING));
                    add_task(Task(MOVING_LEFT));
                    break;
                }
            }
        }
    }

    void task_solver() {
        Task current = get_current_task();
        RCLCPP_INFO(get_logger(), "[Current task]: %s", current.name.c_str());

        // STOP AT SIGN
        if (current.ID == STOP_AT_SIGN) {
            if (!stop_initiated_) {
                stop_initiated_  = true;
                stop_start_time_ = std::chrono::steady_clock::now();
                set_blinkers(true);
                RCLCPP_INFO(get_logger(), "Stopping at STOP sign");
            }
            on_lane_stop();
            update_rear_brake_lights();
            auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(
                std::chrono::steady_clock::now() - stop_start_time_).count();
            if (elapsed > 2000) {
                set_blinkers(false);
                remove_task();
                stop_initiated_ = false;
                RCLCPP_INFO(get_logger(), "Resuming after STOP sign");
            }
            return;
        }

        // LANE DRIVING
        if (current.ID == LANE_DRIVING) {
            set_left_signal(false); set_right_signal(false); set_blinkers(false);
            for (auto & obs : found_objects_)
                if (obs.y > 55.0 && obs.y < 125.0 && obs.x <= 200.0)
                    { mid_speed_ = 535; break; }
            on_lane();
        }
        // FOLLOWING
        else if (current.ID == FOLLOWING) {
            if (count_pass_ > 4) count_pass_ = 0;
            if (num_found_objects_ == 0) { on_lane(); remove_task(); }
            else {
                for (auto & obs : found_objects_) {
                    if ((obs.y >= 225.0 && obs.y <= 315.0) ||
                        last_task_->ID == MOVING_RIGHT_LANE)
                    { on_lane(); count_ = 0; remove_task(); break; }
                    else if (obs.x > dist_to_keep_ + 10.0f) {
                        count_ = 0;
                        (count_pass_ == 2 || count_pass_ == 3) ?
                            on_lane_front_object_cross(obs.x) :
                            on_lane_front_object(obs.x);
                        break;
                    }
                    else if (obs.x < dist_to_keep_ - 10.0f) {
                        if (passing_enabled_) count_++;
                        (count_pass_ == 2 || count_pass_ == 3) ?
                            on_lane_front_object_cross(obs.x) :
                            on_lane_front_object(obs.x);
                        break;
                    }
                    else { if (passing_enabled_) count_++; on_lane_stop(); break; }
                }
            }
        }
        // MOVING LEFT
        else if (current.ID == MOVING_LEFT) {
            set_left_signal(true); set_right_signal(false);
            for (auto & obs : found_objects_) {
                if (obs.y >= 60.0 && obs.y <= 135.0) {
                    speed_pid_ = (count_pass_ == 4) ? -250 : -200;
                    angle_pd_  = (count_pass_ == 4) ? 170  : 180;
                    break;
                }
                else if ((obs.y >= 0.0 && obs.y < 60.0) ||
                         (obs.y >= 315.0 && obs.y <= 360.0))
                { set_left_signal(false); on_lane(); remove_task(); break; }
            }
        }
        // PASSING
        else if (current.ID == PASSING) {
            set_left_signal(true); set_right_signal(false);
            for (auto & obs : found_objects_) {
                if (obs.y >= 270.0 && obs.y <= 345.0) {
                    speed_pid_ = -300; angle_pd_ = 20;
                    remove_task();
                    start_ = std::chrono::steady_clock::now();
                    break;
                }
                else {
                    if (count_pass_ > 2) { speed_pid_ = -200; angle_pd_ = 65; }
                    else on_lane();
                }
            }
        }
        // MOVING RIGHT
        else if (current.ID == MOVING_RIGHT) {
            set_right_signal(true); set_left_signal(false);
            time_long_ = 2500;
            if      (count_pass_ == 4) time_long_ = 3500;
            else if (count_pass_ == 2) time_long_ = 1700;
            else if (count_pass_ == 3) time_long_ = 1850;
            end_ = std::chrono::steady_clock::now();
            if (std::chrono::duration_cast<std::chrono::milliseconds>(
                    end_ - start_).count() > time_long_) {
                set_right_signal(false);
                on_lane_right();
                remove_task();
                start_ = std::chrono::steady_clock::now();
            }
            else { speed_pid_ = -200; angle_pd_ = 35; }
        }
        // MOVING RIGHT LANE
        else if (current.ID == MOVING_RIGHT_LANE) {
            set_right_signal(true); set_left_signal(false);
            end_ = std::chrono::steady_clock::now();
            if (std::chrono::duration_cast<std::chrono::milliseconds>(
                    end_ - start_).count() > 7500) {
                set_right_signal(false);
                on_lane_right();
                remove_task();
                mid_speed_ = 1035;
            }
            else on_lane_right();
        }

        last_task_ = std::make_unique<Task>(current.ID);
        update_rear_brake_lights();
    }

    void run() { task_assigner(); task_solver(); }

    // ── Movimiento ────────────────────────────────────────────────────────────
    void on_lane() {
        u_angle_  = static_cast<int>(kp_angle_ * dist_now_ +
                                     kd_angle_ * (dist_now_ - dist_last_));
        angle_pd_ = std::max(45, std::min(135, 90 + u_angle_));
        u_speed_  = static_cast<int>(kp_speed_ * dist_now_);
        speed_pid_= std::max(-mid_speed_, std::min(0, -mid_speed_ + std::abs(u_speed_)));
        dist_last_= dist_now_;
    }
    void on_lane_right() {
        u_angle_  = static_cast<int>(kp_angle_ * dist_now_ +
                                     kd_angle_ * (dist_now_ - dist_last_));
        angle_pd_ = std::max(0, std::min(180, 90 + u_angle_));
        speed_pid_= -150;
        dist_last_= dist_now_;
    }
    void on_lane_stop() {
        u_angle_  = static_cast<int>(kp_angle_ * dist_now_ +
                                     kd_angle_ * (dist_now_ - dist_last_));
        angle_pd_ = std::max(45, std::min(135, 90 + u_angle_));
        speed_pid_= 0;
        dist_last_= dist_now_;
    }
    void on_lane_front_object(float obstacle_dist) {
        u_angle_  = static_cast<int>(kp_angle_ * dist_now_ +
                                     kd_angle_ * (dist_now_ - dist_last_));
        angle_pd_ = std::max(45, std::min(135, 90 + u_angle_));
        speed_pid_= std::max(-535, std::min(250, decrement_speed(obstacle_dist)));
        dist_last_= dist_now_;
    }
    void on_lane_front_object_cross(float obstacle_dist) {
        speed_pid_= std::max(-535, std::min(250, decrement_speed(obstacle_dist)));
        angle_pd_ = 90;
    }
    int decrement_speed(float obstacle_dist) {
        return vel_decreasing_factor_ *
               (static_cast<int>(obstacle_dist) - static_cast<int>(dist_to_keep_));
    }

    void publish_policies() {
        auto a_msg = std_msgs::msg::Int16(); a_msg.data = angle_pd_;
        auto s_msg = std_msgs::msg::Int16(); s_msg.data = speed_pid_;
        angle_pub_->publish(a_msg);
        speed_pub_->publish(s_msg);
    }
};


int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    // ROS 1: Master *control = new Master(Task(LANE_DRIVING), true, 5, 115);
    //        ros::spin();
    // ROS 2: se usa make_shared y rclcpp::spin
    rclcpp::spin(std::make_shared<Master>(Task(LANE_DRIVING), true, 5, 115));
    rclcpp::shutdown();
    return 0;
}
