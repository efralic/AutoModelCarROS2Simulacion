// ─────────────────────────────────────────────────────────────────────────────
//  object_detection.cpp  –  ROS 2 (Humble)
//  Sensor: Yahboom/Angstrong Nuwa-HP60C  (reemplaza Orbbec Astra Pro)
//
//  CAMBIOS RESPECTO A LA VERSIÓN PREVIA (Astra Pro):
//    • Tópico:  /camera/depth/image_raw
//              → /ascamera_hp60c/camera_publisher/depth0/image_raw
//    • FOV horizontal:  60.0° → 73.8°
//    • Rango mínimo:    0.6 m → 0.2 m   (mejor para obstáculos cercanos)
//    • Rango máximo:    4.0 m  (igual)
//    • Mapeo píxel → ángulo:
//        angle_deg = 90° + (u − 320) × (73.8° / 640) = 90° + (u−320) × 0.1153
//        Rango resultante:  ~53.1°  ...  ~126.9°
//
//  INVARIANTE (NO cambia):
//    • Algoritmo DBSCAN
//    • transform_point  (centroide 800×800 → distancia + ángulo polar)
//    • Formato de salida /objects_points
//    • Masters NO necesitan modificarse
// ─────────────────────────────────────────────────────────────────────────────

#include <cmath>
#include <map>
#include <vector>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "object_detection/msg/points_objects.hpp"

#include <opencv2/opencv.hpp>

// ── Constantes DBSCAN ───────────────────────────────────────────────────────
#define UNCLASSIFIED -1
#define NOISE        -2
#define SUCCESS       0
#define FAILURE      -3

// ── Parámetros de la Nuwa-HP60C ─────────────────────────────────────────────
static constexpr float CAMERA_FOV_H_DEG = 73.8f;   // FOV horizontal real
static constexpr int   CAMERA_WIDTH     = 640;
static constexpr int   CAMERA_HEIGHT    = 480;

// Rango de detección (en mm, la cámara publica en mm)
static constexpr float RANGE_MIN_MM     = 200.0f;  // 0.2 m  (Nuwa)
static constexpr float RANGE_MAX_MM     = 4000.0f; // 4.0 m  (Nuwa)

// Submuestreo de la imagen de profundidad
static constexpr int   DEPTH_STEP       = 8;

// ── DBSCAN ──────────────────────────────────────────────────────────────────
class PointC {
public:
    float x, y;
    int   clusterID;
    PointC(float x_, float y_) : x(x_), y(y_), clusterID(UNCLASSIFIED) {}
};

class DBSCAN {
public:
    std::vector<PointC> m_points;

    DBSCAN(unsigned int minPts, float eps,
           const std::vector<cv::Point> & points_in)
    : m_minPoints(minPts), m_epsilon(eps)
    {
        for (auto & p : points_in)
            m_points.emplace_back((float)p.x, (float)p.y);
    }

    int run() {
        int clusterID = 0;
        for (auto & pt : m_points)
            if (pt.clusterID == UNCLASSIFIED)
                if (expandCluster(pt, clusterID) != FAILURE)
                    clusterID++;
        return 0;
    }

    void getCluster(std::map<int, std::vector<cv::Point>> & out) {
        for (auto & p : m_points)
            out[p.clusterID].push_back(cv::Point((int)p.x, (int)p.y));
    }

private:
    unsigned int m_minPoints;
    float        m_epsilon;

    inline double dist(const PointC & a, const PointC & b) {
        return std::sqrt((a.x-b.x)*(a.x-b.x) + (a.y-b.y)*(a.y-b.y));
    }

    std::vector<int> calcCluster(const PointC & p) {
        std::vector<int> idx;
        for (int i = 0; i < (int)m_points.size(); i++)
            if (dist(p, m_points[i]) <= m_epsilon)
                idx.push_back(i);
        return idx;
    }

    int expandCluster(PointC & point, int clusterID) {
        auto seeds = calcCluster(point);
        if (seeds.size() < m_minPoints) { point.clusterID = NOISE; return FAILURE; }

        int indexCore = 0, index = 0;
        for (auto idx : seeds) {
            m_points[idx].clusterID = clusterID;
            if (m_points[idx].x == point.x && m_points[idx].y == point.y)
                indexCore = index;
            index++;
        }
        seeds.erase(seeds.begin() + indexCore);

        for (size_t i = 0, n = seeds.size(); i < n; i++) {
            auto neighbors = calcCluster(m_points[seeds[i]]);
            if (neighbors.size() >= m_minPoints) {
                for (auto ni : neighbors) {
                    if (m_points[ni].clusterID == UNCLASSIFIED ||
                        m_points[ni].clusterID == NOISE) {
                        if (m_points[ni].clusterID == UNCLASSIFIED) {
                            seeds.push_back(ni);
                            n = seeds.size();
                        }
                        m_points[ni].clusterID = clusterID;
                    }
                }
            }
        }
        return SUCCESS;
    }
};

// ────────────────────────────────────────────────────────────────────────────
//  ObjectDetection – nodo ROS 2
// ────────────────────────────────────────────────────────────────────────────
class ObjectDetection : public rclcpp::Node
{
public:
    ObjectDetection() : rclcpp::Node("ObjectDetection")
    {
        // Parámetros configurables
        this->declare_parameter("minimum_points",  1);
        this->declare_parameter("epsilon",         95);
        this->declare_parameter("depth_topic",
            "/ascamera_hp60c/camera_publisher/depth0/image_raw");
        this->declare_parameter("depth_step",      DEPTH_STEP);

        MINIMUM_POINTS_ = this->get_parameter("minimum_points").as_int();
        EPSILON_        = this->get_parameter("epsilon").as_int();
        depth_step_     = this->get_parameter("depth_step").as_int();
        std::string depth_topic = this->get_parameter("depth_topic").as_string();

        // Publicador (formato idéntico al original)
        pub_ = this->create_publisher<object_detection::msg::PointsObjects>(
                   "objects_points", 1);

        // Suscriptor a imagen de profundidad de la Nuwa-HP60C
        depth_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
            depth_topic, 1,
            std::bind(&ObjectDetection::depth_callback, this,
                      std::placeholders::_1));

        RCLCPP_INFO(get_logger(),
            "ObjectDetection [Nuwa-HP60C] | topic: %s", depth_topic.c_str());
        RCLCPP_INFO(get_logger(),
            "FOV=%.1f° | Rango %.0f-%.0f mm | DBSCAN eps=%d minPts=%d | paso=%d px",
            CAMERA_FOV_H_DEG, RANGE_MIN_MM, RANGE_MAX_MM,
            EPSILON_, MINIMUM_POINTS_, depth_step_);
    }

private:
    rclcpp::Publisher<object_detection::msg::PointsObjects>::SharedPtr pub_;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr           depth_sub_;

    int MINIMUM_POINTS_;
    int EPSILON_;
    int depth_step_;

    std::vector<cv::Point>                  points_;
    std::map<int, std::vector<cv::Point>>   clusters_points_;
    std::vector<cv::Point>                  points_centroids_;

    // ─────────────────────────────────────────────────────────────────────
    //  depth_callback
    //
    //  La Nuwa-HP60C publica depth como sensor_msgs::msg::Image con:
    //    encoding = "16UC1"   (uint16, valores en milímetros)
    //    width    = 640
    //    height   = 480
    //
    //  Mapeo píxel → ángulo (FOV = 73.8°):
    //    angle_deg = 90° + (u - 320) × (73.8°/640)
    //              = 90° + (u - 320) × 0.1153
    //  Rango: u=0 → 53.1°  |  u=320 → 90°  |  u=640 → 126.9°
    // ─────────────────────────────────────────────────────────────────────
    void depth_callback(const sensor_msgs::msg::Image::SharedPtr msg)
    {
        if (msg->encoding != "16UC1") {
            RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
                "Encoding inesperado: %s (esperado 16UC1)",
                msg->encoding.c_str());
            return;
        }

        int width  = (int)msg->width;
        int height = (int)msg->height;
        float cx   = width / 2.0f;
        float fov_per_pixel = CAMERA_FOV_H_DEG / (float)width;

        std::vector<std::pair<float,float>> scan_points;  // (angle_deg, range_cm)

        for (int v = 0; v < height; v += depth_step_) {
            const uint16_t* row_ptr = reinterpret_cast<const uint16_t*>(
                msg->data.data() + v * msg->step);
            for (int u = 0; u < width; u += depth_step_) {
                uint16_t depth_mm = row_ptr[u];

                // Filtrar valores inválidos o fuera de rango
                if (depth_mm == 0) continue;
                if ((float)depth_mm < RANGE_MIN_MM) continue;
                if ((float)depth_mm > RANGE_MAX_MM) continue;

                float angle_deg = 90.0f + (u - cx) * fov_per_pixel;
                float range_cm  = (float)depth_mm / 10.0f;

                scan_points.emplace_back(angle_deg, range_cm);
            }
        }

        if (scan_points.empty()) return;

        // Pipeline idéntico al original (no cambia)
        get_object_points(scan_points);

        if (!points_centroids_.empty()) {
            object_detection::msg::PointsObjects points_msg;
            points_msg.another_field = (uint8_t)points_centroids_.size();

            for (auto & centroid : points_centroids_) {
                geometry_msgs::msg::Point pt;
                transform_point(centroid, pt);
                pt.z = 0.0;
                points_msg.points.push_back(pt);
                RCLCPP_INFO(get_logger(),
                    "Obstáculo: dist=%.1f cm, angulo=%.1f deg",
                    pt.x, pt.y);
            }
            pub_->publish(points_msg);
        }
    }

    // (angle_deg, range_cm) → cartesiano 800×800 → DBSCAN → centroides
    void get_object_points(const std::vector<std::pair<float,float>> & sp)
    {
        points_.clear();
        clusters_points_.clear();

        for (auto & [angle_deg, range_cm] : sp) {
            float r   = range_cm;
            float rad = (180.0f - angle_deg) * (float)M_PI / 180.0f;
            float x   = 400.0f + r * std::cos(rad);
            float y   = 400.0f + r * std::sin(rad);
            x = std::max(0.0f, std::min(800.0f, x));
            y = std::max(0.0f, std::min(800.0f, y));
            points_.emplace_back((int)x, (int)y);
        }

        if (points_.empty()) return;

        DBSCAN db(MINIMUM_POINTS_, (float)EPSILON_, points_);
        db.run();
        db.getCluster(clusters_points_);
        get_centroids_objects(clusters_points_);
    }

    // Centroide (800×800) → (distancia_cm, ángulo_deg)  (idéntico al original)
    void transform_point(const cv::Point & point,
                         geometry_msgs::msg::Point & point_msg)
    {
        float adj = (float)(point.y - 400);
        float opp = (float)(point.x - 400);
        if (adj == 0.0f) adj = 0.000001f;

        if      (point.x >= 0   && point.x <= 400 && point.y >= 0   && point.y <= 400)
            point_msg.y = std::atan(opp / adj) * 57.295779f;
        else if (point.x >= 0   && point.x <= 400 && point.y >= 400 && point.y <= 800)
            point_msg.y = 180.0f + std::atan(opp / adj) * 57.295779f;
        else if (point.x >= 400 && point.x <= 800 && point.y >= 400 && point.y <= 800)
            point_msg.y = 180.0f + std::atan(opp / adj) * 57.295779f;
        else if (point.x >= 400 && point.x <= 800 && point.y >= 0   && point.y <= 400)
            point_msg.y = 360.0f + std::atan(opp / adj) * 57.295779f;

        if (point_msg.y < 0) point_msg.y = -point_msg.y;
        point_msg.x = cv::norm(point - cv::Point(400, 400));
    }

    void get_centroids_objects(
        const std::map<int, std::vector<cv::Point>> & cp)
    {
        points_centroids_.clear();
        for (auto & [id, cluster] : cp) {
            if (id == NOISE || cluster.empty()) continue;
            int sx = 0, sy = 0;
            for (auto & p : cluster) { sx += p.x; sy += p.y; }
            int n = (int)cluster.size();
            points_centroids_.emplace_back(sx / n, sy / n);
        }
    }
};

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ObjectDetection>());
    rclcpp::shutdown();
    return 0;
}
