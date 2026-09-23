// ─────────────────────────────────────────────────────────────────────────────
//  Master_parking_paralelo.cpp  –  Estacionamiento en paralelo
//  Migrado de ROS 1 (Noetic) → ROS 2 (Humble / Iron)
//
//  NOTA: stop_car() usa std::this_thread::sleep_for() dentro de un callback,
//  lo cual es un patrón bloqueante. Se conserva la lógica original pero
//  idealmente debería migrarse a un timer o máquina de estados no bloqueante.
// ─────────────────────────────────────────────────────────────────────────────

#include <chrono>
#include <map>
#include <memory>
#include <string>
#include <thread>
#include <vector>
#include <cmath>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/int16.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "object_detection/msg/points_objects.hpp"

// ── Estados ───────────────────────────────────────────────────────────────────
const int LANE_DRIVING      = 0;
const int PASSING           = 1;
const int STOP              = 2;
const int MOVING_LEFT       = 3;
const int STOP_MOVING_RIGHT = 4;
const int MOVING_RIGHT      = 5;
const int MOVING_BACK       = 6;

static std::map<int, std::string> task_names {
    { LANE_DRIVING,       "Lane driving"       },
    { PASSING,            "Passing cars"       },
    { STOP,               "Stop moving"        },
    { MOVING_LEFT,        "Moving to left lane"},
    { STOP_MOVING_RIGHT,  "Moving along"       },
    { MOVING_RIGHT,       "Returning right"    },
    { MOVING_BACK,        "Back"               },
};


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
    : rclcpp::Node("Master_parking_paralelo")
    {
        distance_center_ = this->create_subscription<std_msgs::msg::Int16>(
            "/distance_center_line", 1,
            std::bind(&Master::dist_center_clbk, this, std::placeholders::_1));

        object_detection_sub_ = this->create_subscription<
            object_detection::msg::PointsObjects>(
            "/objects_points", 1,
            std::bind(&Master::object_detec_clbk, this, std::placeholders::_1));

        angle_pub_ = this->create_publisher<std_msgs::msg::Int16>(
                         "/AutoModelMini/manual_control/steering", 1000);
        speed_pub_ = this->create_publisher<std_msgs::msg::Int16>(
                         "/AutoModelMini/manual_control/speed",    1000);

        num_found_objects_ = 0;
        dist_now_ = angle_last_ = 0;
        kp_angle_ = 0.825f;  kd_angle_ = 0.0297f;  kp_speed_ = 1.2645f;
        u_angle_ = angle_pd_ = speed_pid_ = u_speed_ = 0;
        count_pass_ = 0;

        add_task(task);
    }

private:
    rclcpp::Subscription<std_msgs::msg::Int16>::SharedPtr               distance_center_;
    rclcpp::Subscription<object_detection::msg::PointsObjects>::SharedPtr object_detection_sub_;

    rclcpp::Publisher<std_msgs::msg::Int16>::SharedPtr angle_pub_;
    rclcpp::Publisher<std_msgs::msg::Int16>::SharedPtr speed_pub_;

    std::vector<geometry_msgs::msg::Point> found_objects_;
    int   num_found_objects_;
    int   dist_now_, angle_last_;
    float kp_angle_, kd_angle_, kp_speed_;
    int   u_angle_, angle_pd_, speed_pid_, u_speed_;
    int   count_pass_;

    std::vector<Task> task_pile_;
    std::chrono::steady_clock::time_point start_, end_;

    // ── Callbacks ─────────────────────────────────────────────────────────────
    void dist_center_clbk(const std_msgs::msg::Int16::SharedPtr msg) {
        dist_now_ = static_cast<int>(msg->data);
        run();
        publish_policies();
    }

    void object_detec_clbk(
        const object_detection::msg::PointsObjects::SharedPtr msg)
    {
        found_objects_.clear();
        for (auto & p : msg->points) {
            geometry_msgs::msg::Point pt;
            pt.x = p.x; pt.y = p.y;
            found_objects_.push_back(pt);
        }
        num_found_objects_ = static_cast<int>(found_objects_.size());
    }

    // ── Task management ───────────────────────────────────────────────────────
    void add_task(Task t)   { task_pile_.push_back(t); }
    void remove_task()      { if (task_pile_.size() > 1) task_pile_.pop_back(); }
    Task get_current_task() { return task_pile_.back(); }

    void task_assigner() {
        Task current = get_current_task();
        if (num_found_objects_ > 0) {
            for (auto & obs : found_objects_) {
                if (count_pass_ <= 0 && obs.y <= 360.0 && obs.y >= 270.0) {
                    if (current.ID == LANE_DRIVING) {
                        // Apila la secuencia de maniobra completa
                        add_task(Task(STOP));
                        add_task(Task(MOVING_BACK));
                        add_task(Task(STOP_MOVING_RIGHT));
                        add_task(Task(MOVING_RIGHT));
                        add_task(Task(MOVING_LEFT));
                        add_task(Task(PASSING));
                        count_pass_++;
                        start_ = std::chrono::steady_clock::now();
                        break;
                    }
                }
            }
        }
    }

    void task_solver() {
        Task current = get_current_task();
        RCLCPP_INFO(get_logger(), "[Current task]: %s", current.name.c_str());

        if (current.ID == LANE_DRIVING) {
            on_lane();
        }
        else if (current.ID == PASSING) {
            end_ = std::chrono::steady_clock::now();
            if (num_found_objects_ == 0) {
                on_lane();
            }
            else {
                for (auto & obs : found_objects_) {
                    long elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(
                        end_ - start_).count();
                    if (elapsed > 3750 &&
                        (obs.y == 0.0 || (obs.y <= 360.0 && obs.y >= 300.0)))
                    {
                        stop_car();
                        remove_task();
                        break;
                    }
                    else {
                        on_lane();
                    }
                    found_objects_.clear();
                }
            }
        }
        else if (current.ID == MOVING_LEFT) {
            angle_pd_  = 0;
            speed_pid_ = 150;
            for (auto & obs : found_objects_) {
                if (obs.y <= 315.0 && obs.y >= 200.0) {
                    speed_pid_ = 0;
                    remove_task();
                    break;
                }
                found_objects_.clear();
            }
        }
        else if (current.ID == MOVING_RIGHT) {
            angle_pd_  = 180;
            speed_pid_ = 50;
            for (auto & obs : found_objects_) {
                if (obs.x <= 21.0 && obs.y <= 345.0 && obs.y >= 180.0) {
                    speed_pid_ = 0;
                    remove_task();
                    start_ = std::chrono::steady_clock::now();
                    break;
                }
                found_objects_.clear();
            }
        }
        else if (current.ID == STOP_MOVING_RIGHT) {
            end_ = std::chrono::steady_clock::now();
            speed_pid_ = -100;
            angle_pd_  = 0;
            if (std::chrono::duration_cast<std::chrono::milliseconds>(
                    end_ - start_).count() > 1850) {
                speed_pid_ = 0; angle_pd_ = 90;
                remove_task();
                start_ = std::chrono::steady_clock::now();
            }
        }
        else if (current.ID == MOVING_BACK) {
            end_ = std::chrono::steady_clock::now();
            speed_pid_ = 150;
            angle_pd_  = 0;
            if (std::chrono::duration_cast<std::chrono::milliseconds>(
                    end_ - start_).count() > 600) {
                speed_pid_ = 0; angle_pd_ = 90;
                remove_task();
            }
        }
        else if (current.ID == STOP) {
            speed_pid_ = 0;
        }
    }

    void run() { task_assigner(); task_solver(); }

    void on_lane() {
        u_angle_  = static_cast<int>(kp_angle_ * dist_now_ +
                                     kd_angle_ * (dist_now_ - angle_last_));
        angle_pd_ = 90 + u_angle_;
        if (angle_pd_ <= 45)  angle_pd_ = 45;
        else if (angle_pd_ >= 135) angle_pd_ = 135;
        u_speed_  = static_cast<int>(kp_speed_ * dist_now_);
        speed_pid_= -435 + std::abs(u_speed_);
        if (speed_pid_ < -435) speed_pid_ = -435;
        else if (speed_pid_ > 0) speed_pid_ = 0;
        angle_last_ = angle_pd_;
    }

    // NOTA: stop_car() es bloqueante por diseño original.
    // En ROS 2 esto bloquea el executor mientras frena.
    // Para producción, considerar reemplazar con un timer.
    void stop_car() {
        u_angle_  = static_cast<int>(kp_angle_ * dist_now_ +
                                     kd_angle_ * (dist_now_ - angle_last_));
        angle_pd_ = 90 + u_angle_;
        if (angle_pd_ <= 45)  angle_pd_ = 45;
        else if (angle_pd_ >= 135) angle_pd_ = 135;
        angle_last_ = angle_pd_;
        while (speed_pid_ < 0) {
            speed_pid_ += 5;
            std::this_thread::sleep_for(std::chrono::milliseconds(2));
        }
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
