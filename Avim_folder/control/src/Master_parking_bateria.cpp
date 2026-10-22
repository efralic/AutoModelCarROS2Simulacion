// ─────────────────────────────────────────────────────────────────────────────
//  Master_parking_bateria.cpp  –  Prueba 4: estacionamiento en batería
//  Migrado de ROS 1 (Noetic) → ROS 2 (Humble / Iron)
//
//  Cambios adicionales respecto a los otros Masters:
//   • ros::Timer watchdog_timer_  →  rclcpp::TimerBase::SharedPtr watchdog_timer_
//   • nh_.createTimer(ros::Duration(0.1), &Cb, this)
//         → create_wall_timer(100ms, std::bind(&Cb, this))
//   • Firma del watchdog: (const ros::TimerEvent&)  →  ()  [sin argumento]
//   • ros::Time last_cmd_time_  →  rclcpp::Time last_cmd_time_
//   • (ros::Time::now() - last_cmd_time_).toSec()
//         → (this->now() - last_cmd_time_).seconds()
//   • sensor_msgs::Range  →  sensor_msgs::msg::Range
// ─────────────────────────────────────────────────────────────────────────────

#include <chrono>
#include <map>
#include <memory>
#include <string>
#include <vector>
#include <cmath>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/int16.hpp"
#include "sensor_msgs/msg/range.hpp"
#include "object_detection/msg/points_objects.hpp"

// ── Estados ───────────────────────────────────────────────────────────────────
const int LANE_DRIVING   = 0;
const int ALIGN_TO_SPACE = 1;
const int PARKING_IN     = 2;
const int STOP           = 3;

static std::map<int, std::string> task_names {
    { 0, "Lane driving"              },
    { 1, "Align to parking space"    },
    { 2, "Parking in"                },
    { 3, "Parked - stop"             },
};

// ── Umbrales ultrasonido ──────────────────────────────────────────────────────
static constexpr float NEAR_THRESHOLD  = 0.20f;
static constexpr float SPACE_THRESHOLD = 0.35f;
static constexpr float WALL_THRESHOLD  = 0.10f;
static constexpr float LEFT_SAFE_MIN   = 0.08f;
static constexpr long  ALIGN_TIME_MS   = 600;

// ── Velocidades ───────────────────────────────────────────────────────────────
static constexpr int BASE_SPEED = -380;
static constexpr int MIN_SPEED  = -200;


class Task {
public:
    std::string name;
    int ID;
    explicit Task(int id) : ID(id), name(task_names[id]) {}
};


class Master : public rclcpp::Node
{
public:
    explicit Master(Task task)
    : rclcpp::Node("Master_parking")
    {
        // ── Subscriptores ────────────────────────────────────────────────────
        distance_center_sub_ = this->create_subscription<std_msgs::msg::Int16>(
            "/distance_center_line", 1,
            std::bind(&Master::dist_center_clbk, this, std::placeholders::_1));

        ultrasonic_right_sub_ = this->create_subscription<sensor_msgs::msg::Range>(
            "/sensors/ultrasonic/right", 1,
            std::bind(&Master::ultra_right_clbk, this, std::placeholders::_1));

        ultrasonic_back_sub_ = this->create_subscription<sensor_msgs::msg::Range>(
            "/sensors/ultrasonic/back", 1,
            std::bind(&Master::ultra_back_clbk, this, std::placeholders::_1));

        ultrasonic_left_sub_ = this->create_subscription<sensor_msgs::msg::Range>(
            "/sensors/ultrasonic/left", 1,
            std::bind(&Master::ultra_left_clbk, this, std::placeholders::_1));

        // ── Publicadores ─────────────────────────────────────────────────────
        angle_pub_ = this->create_publisher<std_msgs::msg::Int16>(
                         "/AutoModelMini/manual_control/steering", 1000);
        speed_pub_ = this->create_publisher<std_msgs::msg::Int16>(
                         "/AutoModelMini/manual_control/speed",    1000);

        // ── Watchdog timer ────────────────────────────────────────────────────
        // ROS 1: nh_.createTimer(ros::Duration(0.1), &Master::watchdog_clbk, this)
        // ROS 2: create_wall_timer(100ms, callback_sin_argumento)
        watchdog_timer_ = this->create_wall_timer(
            std::chrono::milliseconds(100),
            std::bind(&Master::watchdog_clbk, this));

        // ── Init ──────────────────────────────────────────────────────────────
        dist_now_ = angle_last_ = 0;
        kp_angle_ = 0.825f;  kd_angle_ = 0.0297f;  kp_speed_ = 1.2645f;
        u_angle_ = angle_pd_ = speed_pid_ = u_speed_ = 0;
        ultra_right_ = ultra_back_ = ultra_left_ = 4.0f;
        saw_car_on_right_    = false;
        space_detected_once_ = false;

        // ROS 1: last_cmd_time_ = ros::Time::now()
        // ROS 2: last_cmd_time_ = this->now()
        last_cmd_time_ = this->now();

        add_task(task);
        RCLCPP_INFO(get_logger(),
                    "Master_parking listo - esperando coche a la derecha...");
    }

private:
    rclcpp::Subscription<std_msgs::msg::Int16>::SharedPtr       distance_center_sub_;
    rclcpp::Subscription<sensor_msgs::msg::Range>::SharedPtr    ultrasonic_right_sub_;
    rclcpp::Subscription<sensor_msgs::msg::Range>::SharedPtr    ultrasonic_back_sub_;
    rclcpp::Subscription<sensor_msgs::msg::Range>::SharedPtr    ultrasonic_left_sub_;

    rclcpp::Publisher<std_msgs::msg::Int16>::SharedPtr angle_pub_;
    rclcpp::Publisher<std_msgs::msg::Int16>::SharedPtr speed_pub_;

    // ROS 1: ros::Timer watchdog_timer_
    // ROS 2: rclcpp::TimerBase::SharedPtr watchdog_timer_
    rclcpp::TimerBase::SharedPtr watchdog_timer_;

    // ROS 1: ros::Time last_cmd_time_
    // ROS 2: rclcpp::Time last_cmd_time_
    rclcpp::Time last_cmd_time_;

    int   dist_now_, angle_last_;
    float kp_angle_, kd_angle_, kp_speed_;
    int   u_angle_, angle_pd_, speed_pid_, u_speed_;

    float ultra_right_, ultra_back_, ultra_left_;
    bool  saw_car_on_right_;
    bool  space_detected_once_;

    std::vector<Task> task_pile_;
    std::chrono::steady_clock::time_point align_start_;

    // ── Watchdog ──────────────────────────────────────────────────────────────
    // ROS 1: void watchdog_clbk(const ros::TimerEvent&)
    // ROS 2: void watchdog_clbk()  ← sin argumento
    void watchdog_clbk() {
        // ROS 1: (ros::Time::now() - last_cmd_time_).toSec()
        // ROS 2: (this->now() - last_cmd_time_).seconds()
        double elapsed = (this->now() - last_cmd_time_).seconds();
        if (elapsed > 0.3) {
            RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000,
                "Sin datos de lane_detection (%.2f s) - velocidad minima", elapsed);
            speed_pid_ = MIN_SPEED;
            angle_pd_  = 90;
            publish_policies();
        }
    }

    // ── Callbacks ultrasonido ─────────────────────────────────────────────────
    void ultra_right_clbk(const sensor_msgs::msg::Range::SharedPtr msg)
        { ultra_right_ = msg->range; }
    void ultra_back_clbk(const sensor_msgs::msg::Range::SharedPtr msg)
        { ultra_back_  = msg->range; }
    void ultra_left_clbk(const sensor_msgs::msg::Range::SharedPtr msg)
        { ultra_left_  = msg->range; }

    void dist_center_clbk(const std_msgs::msg::Int16::SharedPtr msg) {
        last_cmd_time_ = this->now();   // reinicia el watchdog
        dist_now_ = static_cast<int>(msg->data);
        run();
        publish_policies();
    }

    // ── Task management ───────────────────────────────────────────────────────
    void add_task(Task t)   { task_pile_.push_back(t); }
    void remove_task()      { if (task_pile_.size() > 1) task_pile_.pop_back(); }
    Task get_current_task() { return task_pile_.back(); }

    void task_assigner() {
        if (get_current_task().ID != LANE_DRIVING) return;
        if (space_detected_once_) return;

        if (!saw_car_on_right_) {
            if (ultra_right_ < NEAR_THRESHOLD) {
                saw_car_on_right_ = true;
                RCLCPP_INFO(get_logger(),
                    "Coche derecho detectado (%.2f m) - esperando cajón...", ultra_right_);
            }
            return;
        }
        if (ultra_right_ > SPACE_THRESHOLD) {
            space_detected_once_ = true;
            align_start_ = std::chrono::steady_clock::now();
            RCLCPP_INFO(get_logger(),
                "Cajón detectado (derecho=%.2f m) - alineando eje trasero...", ultra_right_);
            add_task(Task(ALIGN_TO_SPACE));
        }
    }

    void task_solver() {
        Task current = get_current_task();
        RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 500,
            "[Estado]: %s | der=%.2fm | tra=%.2fm | izq=%.2fm",
            current.name.c_str(), ultra_right_, ultra_back_, ultra_left_);

        if (current.ID == LANE_DRIVING) {
            on_lane();
        }
        else if (current.ID == ALIGN_TO_SPACE) {
            on_lane();
            long elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(
                std::chrono::steady_clock::now() - align_start_).count();
            if (elapsed >= ALIGN_TIME_MS) {
                RCLCPP_INFO(get_logger(), "Alineado - iniciando entrada al cajón");
                remove_task();
                add_task(Task(PARKING_IN));
            }
        }
        else if (current.ID == PARKING_IN) {
            if (ultra_left_ < LEFT_SAFE_MIN) {
                RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 300,
                    "Muy cerca del coche izquierdo (%.2f m) - pausando reversa", ultra_left_);
                speed_pid_ = 0; angle_pd_ = 90;
                return;
            }
            angle_pd_  = 45;
            speed_pid_ = 250;
            if (ultra_back_ < WALL_THRESHOLD) {
                RCLCPP_INFO(get_logger(),
                    "Banqueta detectada (%.2f m) - estacionamiento completo", ultra_back_);
                speed_pid_ = 0; angle_pd_ = 90;
                remove_task();
                add_task(Task(STOP));
            }
        }
        else if (current.ID == STOP) {
            speed_pid_ = 0; angle_pd_ = 90;
            RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 2000, "Estacionado correctamente.");
        }
    }

    void run() { task_assigner(); task_solver(); }

    void on_lane() {
        u_angle_  = static_cast<int>(kp_angle_ * dist_now_ +
                                     kd_angle_ * (dist_now_ - angle_last_));
        angle_pd_ = std::max(45, std::min(135, 90 + u_angle_));
        int speed_reduction = static_cast<int>(kp_speed_ * std::abs(dist_now_));
        speed_pid_= std::max(MIN_SPEED, std::min(BASE_SPEED, BASE_SPEED + speed_reduction));
        angle_last_ = angle_pd_;
    }

    void publish_policies() {
        auto a = std_msgs::msg::Int16(); a.data = angle_pd_;
        auto s = std_msgs::msg::Int16(); s.data = speed_pid_;
        angle_pub_->publish(a);
        speed_pub_->publish(s);
    }
};


int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<Master>(Task(LANE_DRIVING)));
    rclcpp::shutdown();
    return 0;
}
