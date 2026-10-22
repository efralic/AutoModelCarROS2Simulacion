// ─────────────────────────────────────────────────────────────────────────────
//  object_detection_parking.cpp  –  ROS 2 (Humble)
//  Sensor: Yahboom/Angstrong Nuwa-HP60C (reemplaza Orbbec Astra Pro)
//  Prueba 4: Estacionamiento
//
//  CAMBIOS RESPECTO A LA VERSIÓN PREVIA (Astra Pro):
//    • Tópico:  /camera/depth/image_raw
//              → /ascamera_hp60c/camera_publisher/depth0/image_raw
//    • FOV horizontal:  60.0° → 73.8°
//    • RANGE_MIN_MM:    180 → 200  (rango mínimo oficial de Nuwa)
//    • RANGE_MAX_MM:    440        (sin cambio — específico del parking)
//
//  DIFERENCIAS respecto a object_detection.cpp:
//    • RANGE_MAX_MM = 440  (44 cm — detecta solo el cajón cercano)
//    • RANGE_MIN_MM = 200  (20 cm — rango mínimo de Nuwa)
//    • EPSILON      = 10   (clusters más pequeños y compactos)
//    → El rango corto hace que el nodo "vea" solo los coches vecinos
//      del cajón de estacionamiento, ignorando el entorno lejano.
//
//  INVARIANTE (idéntico a object_detection.cpp):
//    • DBSCAN, transform_point, get_centroids_objects → sin cambios
//    • Formato de salida /objects_points → sin cambios
//    • Master_parking → sin cambios
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
static constexpr float CAMERA_FOV_H_DEG = 73.8f;
static constexpr int   CAMERA_WIDTH     = 640;

// ── Rango para estacionamiento (rango CORTO — solo coches vecinos) ──────────
static constexpr float RANGE_MAX_MM = 440.0f;   // 44 cm
static constexpr float RANGE_MIN_MM = 200.0f;   // 20 cm (mínimo Nuwa)

// Submuestreo: menos pasos porque el rango es muy corto y hay pocos puntos
static constexpr int   DEPTH_STEP   = 4;


// ── DBSCAN (idéntico al de object_detection.cpp) ────────────────────────────
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
           const std::vector<cv::Point> & pts)
    : m_minPoints(minPts), m_epsilon(eps)
    {
        for (auto & p : pts)
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
            auto nb = calcCluster(m_points[seeds[i]]);
            if (nb.size() >= m_minPoints) {
                for (auto ni : nb) {
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
//  ObjectDetectionParking – nodo ROS 2
// ────────────────────────────────────────────────────────────────────────────
class ObjectDetectionParking : public rclcpp::Node
{
public:
    ObjectDetectionParking() : rclcpp::Node("ObjectDetectionParking")
    {
        this->declare_parameter("minimum_points", 1);
        this->declare_parameter("epsilon",        10);
        this->declare_parameter("depth_topic",
            "/ascamera_hp60c/camera_publisher/depth0/image_raw");
        this->declare_parameter("depth_step",     DEPTH_STEP);

        MINIMUM_POINTS_ = this->get_parameter("minimum_points").as_int();
        EPSILON_        = this->get_parameter("epsilon").as_int();
        depth_step_     = this->get_parameter("depth_step").as_int();
        std::string depth_topic = this->get_parameter("depth_topic").as_string();

        pub_ = this->create_publisher<object_detection::msg::PointsObjects>(
                   "objects_points", 1);

        depth_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
            depth_topic, 1,
            std::bind(&ObjectDetectionParking::depth_callback, this,
                      std::placeholders::_1));

        RCLCPP_INFO(get_logger(),
            "ObjectDetectionParking [Nuwa-HP60C] | topic: %s", depth_topic.c_str());
        RCLCPP_INFO(get_logger(),
            "FOV=%.1f° | Rango %.0f-%.0f mm | DBSCAN eps=%d | paso=%d px",
            CAMERA_FOV_H_DEG, RANGE_MIN_MM, RANGE_MAX_MM,
            EPSILON_, depth_step_);
    }

private:
    rclcpp::Publisher<object_detection::msg::PointsObjects>::SharedPtr pub_;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr           depth_sub_;

    int MINIMUM_POINTS_, EPSILON_, depth_step_;

    std::vector<cv::Point>                 points_;
    std::map<int, std::vector<cv::Point>>  clusters_points_;
    std::vector<cv::Point>                 points_centroids_;

    // ─────────────────────────────────────────────────────────────────────
    //  depth_callback
    //  Misma lógica que object_detection.cpp pero con RANGE corto (20-44 cm)
    //  para detectar solo los coches vecinos del cajón de estacionamiento.
    //
    //  Mapeo píxel → ángulo (Nuwa-HP60C FOV = 73.8°):
    //    angle_deg = 90° + (u - 320) × (73.8°/640)
    //  Rango resultante: 53.1° (izq) ... 90° (centro) ... 126.9° (der)
    // ─────────────────────────────────────────────────────────────────────
    void depth_callback(const sensor_msgs::msg::Image::SharedPtr msg)
    {
        if (msg->encoding != "16UC1") {
            RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
                "Encoding inesperado: %s (esperado 16UC1)", msg->encoding.c_str());
            return;
        }

        int   width        = (int)msg->width;
        float cx           = width / 2.0f;
        float fov_per_pixel = CAMERA_FOV_H_DEG / (float)width;

        std::vector<std::pair<float, float>> scan_points;

        for (int v = 0; v < (int)msg->height; v += depth_step_) {
            const uint16_t* row = reinterpret_cast<const uint16_t*>(
                                      msg->data.data() + v * msg->step);
            for (int u = 0; u < width; u += depth_step_) {
                uint16_t d = row[u];
                if (d == 0) continue;
                // Rango corto: solo coches a 20-44 cm
                if ((float)d < RANGE_MIN_MM || (float)d > RANGE_MAX_MM) continue;

                float angle_deg = 90.0f + (u - cx) * fov_per_pixel;
                float range_cm  = (float)d / 10.0f;
                scan_points.emplace_back(angle_deg, range_cm);
            }
        }

        if (scan_points.empty()) return;

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
                    "Parking obstacle: dist=%.1f cm, angulo=%.1f deg",
                    pt.x, pt.y);
            }
            pub_->publish(points_msg);
        }
    }

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
    rclcpp::spin(std::make_shared<ObjectDetectionParking>());
    rclcpp::shutdown();
    return 0;
}
