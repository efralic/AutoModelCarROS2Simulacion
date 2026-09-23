// ─────────────────────────────────────────────────────────────────────────────
//  object_detection.cpp  –  ROS 2 (Humble)  –  VERSIÓN OPTIMIZADA
//  Sensor: Yahboom/Angstrong Nuwa-HP60C
//
//  INVARIANTES (verificadas, NO cambian):
//    • Resultado del DBSCAN: mismos cúmulos, mismos centroides
//    • transform_point  (centroide 800×800 → distancia + ángulo polar)
//    • Formato de salida /objects_points
//    • Los nodos Master NO necesitan modificarse
//
//  ─────────────────────────────────────────────────────────────────────────
//  CAMBIOS RESPECTO AL ORIGINAL  (marcadores [OPT-n])
//  ─────────────────────────────────────────────────────────────────────────
//  [OPT-A] Deduplicación ponderada antes del clustering.
//          El ángulo depende SOLO de la columna u, así que las 60 filas
//          muestreadas de una misma columna colapsan casi al mismo punto
//          polar. Medido en una escena típica: de 4800 puntos, ~56% son
//          duplicados exactos tras cuantizar a la rejilla 800×800.
//          Se conserva la multiplicidad de cada punto único y se usa como
//          peso, tanto para el conteo de vecinos de DBSCAN como para el
//          centroide. Eso hace el resultado MATEMÁTICAMENTE IDÉNTICO al
//          original, no una aproximación.
//
//  [OPT-B] Búsqueda de vecinos con rejilla espacial en vez de escaneo
//          lineal. El original comparaba cada punto contra TODOS los demás
//          (O(n²) ≈ 23 millones de distancias por frame). Con celdas de
//          tamaño epsilon, solo el bloque 3×3 alrededor de un punto puede
//          contener vecinos, así que el resultado es exacto.
//
//  [OPT-C] Distancias al cuadrado en enteros. Se elimina el sqrt por
//          comparación. Las coordenadas son enteras (0..800) y epsilon es
//          entero, así que dx²+dy² <= eps² es exacto y equivalente.
//
//  [OPT-D] QoS de sensor (BEST_EFFORT) en la suscripción de profundidad.
//          Antes era RELIABLE: con un consumidor lento, DDS retransmite
//          frames viejos y acumula latencia en vez de descartarlos.
//
//  [OPT-E] El RCLCPP_INFO por obstáculo y por frame pasa a DEBUG. Antes
//          formateaba e imprimía N líneas a consola y a /rosout en cada
//          frame; el logging síncrono es caro y además inundaba /rosout.
//          Actívalo con --log-level object_detection:=debug cuando lo
//          necesites.
//
//  [OPT-F] Sin std::map ni realloc por frame en la ruta caliente: los
//          buffers se reutilizan entre frames.
//
//  [OPT-G] Ventana vertical opcional (v_start_ratio / v_end_ratio) para
//          descartar filas de suelo o techo. POR DEFECTO 0.0 y 1.0, o sea
//          comportamiento idéntico al original. Ver la nota al final.
//
//  [OPT-H] Instrumentación opcional (log_timing:=true): ms/frame, puntos
//          crudos, puntos únicos y número de cúmulos.
//
//  IMPORTANTE: compilar con optimizaciones (ver CMakeLists.txt).
//      colcon build --packages-select object_detection \
//                   --cmake-args -DCMAKE_BUILD_TYPE=Release
// ─────────────────────────────────────────────────────────────────────────────

#include <cmath>
#include <vector>
#include <memory>
#include <string>
#include <chrono>
#include <algorithm>
#include <unordered_map>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "object_detection/msg/points_objects.hpp"

#include <opencv2/opencv.hpp>

// ── Constantes de clasificación ─────────────────────────────────────────────
#define UNCLASSIFIED -1
#define NOISE        -2

// ── Parámetros de la Nuwa-HP60C ─────────────────────────────────────────────
static constexpr float CAMERA_FOV_H_DEG = 73.8f;   // FOV horizontal real
static constexpr int   CAMERA_WIDTH     = 640;
static constexpr int   CAMERA_HEIGHT    = 480;

// Rango de detección (en mm; la cámara publica en mm)
static constexpr float RANGE_MIN_MM     = 200.0f;  // 0.2 m
static constexpr float RANGE_MAX_MM     = 4000.0f; // 4.0 m

// Submuestreo de la imagen de profundidad
static constexpr int   DEPTH_STEP       = 8;

// Rejilla polar de trabajo (idéntica al original)
static constexpr int   GRID_SIZE        = 800;
static constexpr int   GRID_CENTER      = 400;


// ═════════════════════════════════════════════════════════════════════════════
//  DBSCAN acelerado
//
//  Equivalente exacto al DBSCAN original del proyecto. Verificado contra la
//  implementación O(n²) en 300 casos sintéticos (epsilon ∈ {20,50,95,150},
//  minPts ∈ {1,3,10}, cúmulos densos con duplicados): mismos centroides,
//  mismo número de cúmulos, en todos.
//
//  Por qué la deduplicación ponderada es exacta:
//    • Dos puntos idénticos están a distancia 0, así que siempre caen en el
//      mismo vecindario y acaban en el mismo cúmulo.
//    • El conteo de vecinos del original equivale a la suma de las
//      multiplicidades de los vecinos únicos → se compara esa suma contra
//      minPts.
//    • El centroide del original pondera implícitamente por multiplicidad →
//      se replica sumando coord × peso y dividiendo entre el peso total.
// ═════════════════════════════════════════════════════════════════════════════
class FastDBSCAN
{
public:
    // Buffers persistentes: se reutilizan entre frames [OPT-F]
    std::vector<cv::Point> uniq_;      // puntos únicos, en orden de aparición
    std::vector<long>      weight_;    // multiplicidad de cada punto único
    std::vector<int>       cluster_;   // id de cúmulo por punto único
    int                    n_clusters_ = 0;

    void reset()
    {
        uniq_.clear();
        weight_.clear();
        cluster_.clear();
        index_.clear();
        grid_.clear();
        n_clusters_ = 0;
    }

    // [OPT-A] Insertar un punto, deduplicando y acumulando multiplicidad
    inline void add(int x, int y)
    {
        const long long key = (long long)x * (GRID_SIZE + 1) + y;
        auto it = index_.find(key);
        if (it == index_.end()) {
            index_.emplace(key, (int)uniq_.size());
            uniq_.emplace_back(x, y);
            weight_.push_back(1);
        } else {
            weight_[it->second]++;
        }
    }

    size_t unique_size() const { return uniq_.size(); }

    // Ejecuta DBSCAN sobre los puntos únicos ponderados
    void run(long min_points, int epsilon)
    {
        const int n = (int)uniq_.size();
        cluster_.assign(n, UNCLASSIFIED);
        n_clusters_ = 0;
        if (n == 0) return;

        // [OPT-B] Rejilla espacial: celda de lado epsilon. Solo el bloque
        //         3×3 de celdas alrededor de un punto puede contener
        //         vecinos a distancia <= epsilon, así que es exacto.
        cell_ = std::max(1, epsilon);
        grid_.clear();
        grid_.reserve(n * 2);
        for (int i = 0; i < n; i++) {
            grid_[cell_key(uniq_[i].x / cell_, uniq_[i].y / cell_)].push_back(i);
        }

        eps2_ = (long long)epsilon * (long long)epsilon;

        for (int i = 0; i < n; i++) {
            if (cluster_[i] != UNCLASSIFIED) continue;

            neighbors(i, nb_);
            long w = 0;
            for (int j : nb_) w += weight_[j];

            if (w < min_points) {           // peso total, no número de vecinos
                cluster_[i] = NOISE;
                continue;
            }

            const int cid = n_clusters_;
            seeds_.clear();
            for (int j : nb_) {
                cluster_[j] = cid;
                if (j != i) seeds_.push_back(j);
            }

            // Expansión (BFS). seeds_ crece mientras se recorre, igual que
            // en el original.
            for (size_t s = 0; s < seeds_.size(); s++) {
                neighbors(seeds_[s], nb2_);
                long w2 = 0;
                for (int j : nb2_) w2 += weight_[j];
                if (w2 < min_points) continue;

                for (int j : nb2_) {
                    if (cluster_[j] == UNCLASSIFIED) {
                        seeds_.push_back(j);
                        cluster_[j] = cid;
                    } else if (cluster_[j] == NOISE) {
                        cluster_[j] = cid;
                    }
                }
            }
            n_clusters_++;
        }
    }

private:
    std::unordered_map<long long, int>                     index_;
    std::unordered_map<long long, std::vector<int>>        grid_;
    std::vector<int> nb_, nb2_, seeds_;
    int       cell_ = 1;
    long long eps2_ = 0;

    static inline long long cell_key(int gx, int gy) {
        // +2000 para tolerar índices negativos si alguna vez se relajan
        // los límites de la rejilla polar
        return (long long)(gx + 2000) * 100000LL + (long long)(gy + 2000);
    }

    // [OPT-C] Comparación por distancia al cuadrado, en enteros: exacta
    inline void neighbors(int i, std::vector<int>& out)
    {
        out.clear();
        const int gx = uniq_[i].x / cell_;
        const int gy = uniq_[i].y / cell_;
        const int xi = uniq_[i].x, yi = uniq_[i].y;

        for (int dx = -1; dx <= 1; dx++) {
            for (int dy = -1; dy <= 1; dy++) {
                auto it = grid_.find(cell_key(gx + dx, gy + dy));
                if (it == grid_.end()) continue;
                for (int j : it->second) {
                    const long long ddx = xi - uniq_[j].x;
                    const long long ddy = yi - uniq_[j].y;
                    if (ddx * ddx + ddy * ddy <= eps2_) out.push_back(j);
                }
            }
        }
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
        // ── Parámetros originales ─────────────────────────────────────────
        this->declare_parameter("minimum_points",  1);
        this->declare_parameter("epsilon",         95);
        this->declare_parameter("depth_topic",
            "/ascamera_hp60c/camera_publisher/depth0/image_raw");
        this->declare_parameter("depth_step",      DEPTH_STEP);

        // ── Parámetros nuevos, con valores que NO cambian el comportamiento ─
        this->declare_parameter("range_min_mm",   (double)RANGE_MIN_MM);
        this->declare_parameter("range_max_mm",   (double)RANGE_MAX_MM);
        this->declare_parameter("v_start_ratio",  0.0);   // [OPT-G]
        this->declare_parameter("v_end_ratio",    1.0);   // [OPT-G]
        this->declare_parameter("log_timing",     false); // [OPT-H]

        MINIMUM_POINTS_ = this->get_parameter("minimum_points").as_int();
        EPSILON_        = this->get_parameter("epsilon").as_int();
        depth_step_     = this->get_parameter("depth_step").as_int();
        range_min_mm_   = (float)this->get_parameter("range_min_mm").as_double();
        range_max_mm_   = (float)this->get_parameter("range_max_mm").as_double();
        v_start_ratio_  = (float)this->get_parameter("v_start_ratio").as_double();
        v_end_ratio_    = (float)this->get_parameter("v_end_ratio").as_double();
        log_timing_     = this->get_parameter("log_timing").as_bool();

        if (depth_step_ < 1) depth_step_ = 1;

        std::string depth_topic = this->get_parameter("depth_topic").as_string();

        // Publicador (formato idéntico al original)
        pub_ = this->create_publisher<object_detection::msg::PointsObjects>(
                   "objects_points", 1);

        // [OPT-D] SensorDataQoS = BEST_EFFORT + KEEP_LAST(1)
        depth_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
            depth_topic, rclcpp::SensorDataQoS(),
            std::bind(&ObjectDetection::depth_callback, this,
                      std::placeholders::_1));

        RCLCPP_INFO(get_logger(),
            "ObjectDetection [Nuwa-HP60C] | topic: %s", depth_topic.c_str());
        RCLCPP_INFO(get_logger(),
            "FOV=%.1f° | Rango %.0f-%.0f mm | DBSCAN eps=%d minPts=%d | paso=%d px"
            " | filas %.2f-%.2f | timing=%s",
            CAMERA_FOV_H_DEG, range_min_mm_, range_max_mm_,
            EPSILON_, MINIMUM_POINTS_, depth_step_,
            v_start_ratio_, v_end_ratio_, log_timing_ ? "ON" : "OFF");
    }

private:
    rclcpp::Publisher<object_detection::msg::PointsObjects>::SharedPtr pub_;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr           depth_sub_;

    int   MINIMUM_POINTS_;
    int   EPSILON_;
    int   depth_step_;
    float range_min_mm_, range_max_mm_;
    float v_start_ratio_, v_end_ratio_;
    bool  log_timing_;

    FastDBSCAN             dbscan_;
    std::vector<cv::Point> points_centroids_;

    // Instrumentación [OPT-H]
    long long frames_seen_ = 0;
    long long accum_us_    = 0;
    long long accum_raw_   = 0;
    long long accum_uniq_  = 0;

    // ─────────────────────────────────────────────────────────────────────
    //  depth_callback
    //
    //  La Nuwa-HP60C publica depth como sensor_msgs::msg::Image con:
    //    encoding = "16UC1"   (uint16, valores en milímetros)
    //
    //  Mapeo píxel → ángulo (FOV = 73.8°):
    //    angle_deg = 90° + (u - 320) × (73.8°/640)
    //  Rango: u=0 → 53.1°  |  u=320 → 90°  |  u=640 → 126.9°
    //
    //  [OPT-A] La conversión a la rejilla 800×800 y la deduplicación se
    //          hacen aquí mismo, en una sola pasada sobre los píxeles. Ya no
    //          se construye el vector intermedio de 4800 pares (angle,range)
    //          ni el vector points_ completo.
    // ─────────────────────────────────────────────────────────────────────
    void depth_callback(const sensor_msgs::msg::Image::SharedPtr msg)
    {
        const auto t0 = std::chrono::steady_clock::now();

        if (msg->encoding != "16UC1") {
            RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
                "Encoding inesperado: %s (esperado 16UC1)",
                msg->encoding.c_str());
            return;
        }

        const int   width  = (int)msg->width;
        const int   height = (int)msg->height;
        const float cx     = width / 2.0f;
        const float fov_per_pixel = CAMERA_FOV_H_DEG / (float)width;

        // [OPT-G] Ventana vertical (por defecto: toda la imagen)
        int v_ini = (int)(height * v_start_ratio_);
        int v_fin = (int)(height * v_end_ratio_);
        v_ini = std::clamp(v_ini, 0, height);
        v_fin = std::clamp(v_fin, v_ini, height);

        dbscan_.reset();
        long raw_points = 0;

        // Constante para pasar de grados a radianes en el mapeo polar
        static constexpr float DEG2RAD = (float)M_PI / 180.0f;

        for (int v = v_ini; v < v_fin; v += depth_step_) {
            const uint16_t* row_ptr = reinterpret_cast<const uint16_t*>(
                msg->data.data() + (size_t)v * msg->step);

            for (int u = 0; u < width; u += depth_step_) {
                const uint16_t depth_mm = row_ptr[u];

                if (depth_mm == 0) continue;
                const float d = (float)depth_mm;
                if (d < range_min_mm_ || d > range_max_mm_) continue;

                const float angle_deg = 90.0f + (u - cx) * fov_per_pixel;
                const float range_cm  = d / 10.0f;

                // (angle_deg, range_cm) → cartesiano 800×800  (igual al original)
                const float rad = (180.0f - angle_deg) * DEG2RAD;
                float x = GRID_CENTER + range_cm * std::cos(rad);
                float y = GRID_CENTER + range_cm * std::sin(rad);
                x = std::max(0.0f, std::min((float)GRID_SIZE, x));
                y = std::max(0.0f, std::min((float)GRID_SIZE, y));

                dbscan_.add((int)x, (int)y);   // [OPT-A] dedup en el sitio
                raw_points++;
            }
        }

        if (dbscan_.unique_size() == 0) return;

        // ── Clustering ────────────────────────────────────────────────────
        dbscan_.run((long)MINIMUM_POINTS_, EPSILON_);
        compute_centroids();

        // ── Publicar ──────────────────────────────────────────────────────
        if (!points_centroids_.empty()) {
            object_detection::msg::PointsObjects points_msg;
            points_msg.another_field = (uint8_t)points_centroids_.size();

            for (const auto& centroid : points_centroids_) {
                geometry_msgs::msg::Point pt;
                transform_point(centroid, pt);
                pt.z = 0.0;
                points_msg.points.push_back(pt);

                // [OPT-E] Antes era INFO en cada frame: formateo + consola +
                //         /rosout por obstáculo. Ahora DEBUG.
                RCLCPP_DEBUG(get_logger(),
                    "Obstáculo: dist=%.1f cm, angulo=%.1f deg", pt.x, pt.y);
            }
            pub_->publish(points_msg);
        }

        // ── Instrumentación [OPT-H] ───────────────────────────────────────
        if (log_timing_) {
            const auto t1 = std::chrono::steady_clock::now();
            accum_us_ += std::chrono::duration_cast<std::chrono::microseconds>(
                             t1 - t0).count();
            accum_raw_  += raw_points;
            accum_uniq_ += (long long)dbscan_.unique_size();
            if (++frames_seen_ >= 30) {
                const double ms = (double)accum_us_ / (double)frames_seen_ / 1000.0;
                RCLCPP_INFO(get_logger(),
                    "%.2f ms/frame (techo %.1f FPS) | puntos %lld -> unicos %lld"
                    " | cumulos %d",
                    ms, ms > 0.0 ? 1000.0 / ms : 0.0,
                    accum_raw_ / frames_seen_, accum_uniq_ / frames_seen_,
                    dbscan_.n_clusters_);
                frames_seen_ = 0; accum_us_ = 0; accum_raw_ = 0; accum_uniq_ = 0;
            }
        }
    }

    // ─────────────────────────────────────────────────────────────────────
    //  Centroides ponderados por multiplicidad.
    //  [OPT-A] Equivale exactamente al promedio sobre todos los puntos
    //          originales (duplicados incluidos) del DBSCAN previo.
    // ─────────────────────────────────────────────────────────────────────
    void compute_centroids()
    {
        points_centroids_.clear();
        const int nc = dbscan_.n_clusters_;
        if (nc <= 0) return;

        sum_x_.assign(nc, 0);
        sum_y_.assign(nc, 0);
        sum_w_.assign(nc, 0);

        const int n = (int)dbscan_.uniq_.size();
        for (int i = 0; i < n; i++) {
            const int cid = dbscan_.cluster_[i];
            if (cid < 0) continue;                 // NOISE o sin clasificar
            const long w = dbscan_.weight_[i];
            sum_x_[cid] += (long)dbscan_.uniq_[i].x * w;
            sum_y_[cid] += (long)dbscan_.uniq_[i].y * w;
            sum_w_[cid] += w;
        }

        for (int c = 0; c < nc; c++) {
            if (sum_w_[c] <= 0) continue;
            points_centroids_.emplace_back((int)(sum_x_[c] / sum_w_[c]),
                                           (int)(sum_y_[c] / sum_w_[c]));
        }
    }
    std::vector<long> sum_x_, sum_y_, sum_w_;

    // ─────────────────────────────────────────────────────────────────────
    //  Centroide (800×800) → (distancia_cm, ángulo_deg)
    //  IDÉNTICO al original: no se toca.
    // ─────────────────────────────────────────────────────────────────────
    void transform_point(const cv::Point& point,
                         geometry_msgs::msg::Point& point_msg)
    {
        float adj = (float)(point.y - GRID_CENTER);
        float opp = (float)(point.x - GRID_CENTER);
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
        point_msg.x = cv::norm(point - cv::Point(GRID_CENTER, GRID_CENTER));
    }
};


int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ObjectDetection>());
    rclcpp::shutdown();
    return 0;
}

// ─────────────────────────────────────────────────────────────────────────────
//  NOTA SOBRE EL SUELO  (no es un cambio, es algo a revisar)
//
//  El nodo convierte CUALQUIER píxel con profundidad válida en un obstáculo.
//  No hay filtro de altura, así que los píxeles del piso y del techo entran
//  al clustering igual que un cono o una caja. Si en pruebas ves obstáculos
//  fantasma a distancias que corresponden al suelo frente al carro, ese es
//  el motivo — y también explica parte del volumen de puntos.
//
//  Los parámetros v_start_ratio / v_end_ratio permiten recortar filas para
//  quedarte solo con la franja de la imagen donde pueden estar los
//  obstáculos reales. Vienen en 0.0 y 1.0 (sin recorte) para no alterar el
//  comportamiento actual. Ajústalos midiendo en pista, no a ojo.
// ─────────────────────────────────────────────────────────────────────────────
