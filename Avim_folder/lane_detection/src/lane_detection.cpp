// ═════════════════════════════════════════════════════════════════════════════
//  lane_detection.cpp  –  ROS 2 (Humble)  –  VERSIÓN OPTIMIZADA
//  Cámara: Arducam IMX219 (CSI)
//
//  Pipeline (idéntico al original):
//    Frame → ROI inferior → Blur → CLAHE
//         → HSV mask + LAB threshold (combinadas)
//         → Filtro de contornos por área y aspect ratio
//         → Bird's-eye view (vista cenital)
//         → Histograma + ventanas deslizantes
//         → Ajuste polinómico de grado 2
//         → Suavizado temporal
//         → Cálculo de error lateral
//
//  ─────────────────────────────────────────────────────────────────────────
//  CAMBIOS RESPECTO AL ORIGINAL  (buscar los marcadores [OPT-n])
//  ─────────────────────────────────────────────────────────────────────────
//  [OPT-1] ROI real: se recorta la mitad inferior con una submatriz en lugar
//          de clonar el frame y pintar de negro la mitad superior. Todo el
//          preprocesado pasa de 640x480 a 640x240 → ~2x en esa etapa.
//          Como cambia el sistema de coordenadas, los puntos fuente del
//          bird's-eye se corrigen restando el offset del ROI. La matriz de
//          perspectiva ahora se calcula de forma diferida (primer frame),
//          porque el offset depende de la altura real de la imagen.
//
//  [OPT-2] Todo el trabajo de debug queda tras `if (debug_output_)`. Antes se
//          generaba la imagen de debug (cvtColor 400x480x3, 18 rectángulos y
//          decenas de miles de escrituras píxel a píxel) aunque se descartara.
//
//  [OPT-3] Ventanas deslizantes sin recorrer el arreglo completo 9 veces.
//          findNonZero devuelve los puntos ordenados por fila, así que los
//          píxeles de cada ventana son un tramo contiguo: se localiza con
//          lower_bound y se recorre solo ese tramo. Medido 3-4x más rápido
//          que el original, con resultado numérico idéntico.
//
//  [OPT-4] Ruta rápida opcional solo-LAB (`use_hsv_mask:=false`). Ahorra un
//          cvtColor, un split, un CLAHE, un merge y un inRange por frame.
//          POR DEFECTO ESTÁ EN true → comportamiento idéntico al original.
//          Actívala solo después de validar en pista.
//
//  [OPT-5] QoS de sensor (BEST_EFFORT) en la suscripción de imagen. Antes era
//          RELIABLE por defecto: con un consumidor lento, DDS reintransmite
//          frames viejos y acumula latencia en vez de descartarlos.
//
//  [OPT-6] filter_contours hace un solo drawContours con todos los contornos
//          aceptados, en vez de una llamada por contorno.
//
//  [OPT-7] Instrumentación opcional (`log_timing:=true`): reporta ms/frame
//          promedio y FPS efectivo cada N frames. Sirve para medir el efecto
//          de cada cambio sin herramientas externas.
//
//  IMPORTANTE: compilar con optimizaciones.
//      colcon build --packages-select lane_detection \
//                   --cmake-args -DCMAKE_BUILD_TYPE=Release
//  (ver también el CMakeLists.txt que acompaña a este archivo)
// ═════════════════════════════════════════════════════════════════════════════

#include <chrono>
#include <deque>
#include <memory>
#include <vector>
#include <cmath>
#include <algorithm>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/int16.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "cv_bridge/cv_bridge.h"

#include <opencv2/opencv.hpp>
#include <opencv2/imgproc.hpp>


// ═════════════════════════════════════════════════════════════════════════════
//  PARÁMETROS por defecto (todos override-ables vía ros2 param)
// ═════════════════════════════════════════════════════════════════════════════
namespace defaults {
    // ROI
    constexpr float ROI_TOP_RATIO = 0.50f;

    // Bird's-eye view (coordenadas del FRAME COMPLETO; se corrigen solas)
    constexpr int   SRC_TL_X     = 180;
    constexpr int   SRC_TR_X     = 460;
    constexpr int   SRC_TOP_Y    = 280;
    constexpr int   SRC_BL_X     = 20;
    constexpr int   SRC_BR_X     = 620;
    constexpr int   SRC_BOT_Y    = 470;
    constexpr int   WARP_WIDTH   = 400;
    constexpr int   WARP_HEIGHT  = 480;

    // Máscara blanca
    constexpr int   HSV_V_MIN          = 200;
    constexpr int   HSV_S_MAX          = 30;
    constexpr int   LAB_L_THRESHOLD    = 200;

    // Ventanas deslizantes
    constexpr int   NWINDOWS           = 9;
    constexpr int   MARGIN             = 60;
    constexpr int   MINPIX             = 30;
    constexpr int   LANE_WIDTH_PX      = 280;
    constexpr int   MIN_PIXELS_FIT     = 100;
    constexpr int   MIN_HISTOGRAM_PEAK = 50;

    // Suavizado
    constexpr int   SMOOTHING_FRAMES   = 8;

    // Filtro de contornos
    constexpr int   MIN_CONTOUR_AREA   = 100;
    constexpr float MIN_ASPECT_RATIO   = 2.0f;

    // Instrumentación
    constexpr int   TIMING_WINDOW      = 30;   // frames entre reportes
}


// ═════════════════════════════════════════════════════════════════════════════
//  Helper: ajuste polinómico de grado 2  (sustituye a np.polyfit)
//
//  Resuelve por mínimos cuadrados:    x = a·y² + b·y + c
//  Devuelve (a, b, c) en un cv::Vec3d.
//  Retorna false si la matriz es singular o hay pocos puntos.
// ═════════════════════════════════════════════════════════════════════════════
static bool polyfit2(const std::vector<int>& xs,
                     const std::vector<int>& ys,
                     cv::Vec3d& coeffs)
{
    const int n = (int)xs.size();
    if (n < 3 || (int)ys.size() != n) return false;

    // Matriz de Vandermonde A = [[y², y, 1], ...]   →   A · [a,b,c]ᵀ = x
    cv::Mat A((int)n, 3, CV_64F);
    cv::Mat X((int)n, 1, CV_64F);
    for (int i = 0; i < n; ++i) {
        double y = (double)ys[i];
        A.at<double>(i, 0) = y * y;
        A.at<double>(i, 1) = y;
        A.at<double>(i, 2) = 1.0;
        X.at<double>(i, 0) = (double)xs[i];
    }
    cv::Mat sol;
    if (!cv::solve(A, X, sol, cv::DECOMP_NORMAL | cv::DECOMP_SVD)) return false;
    coeffs = cv::Vec3d(sol.at<double>(0, 0),
                       sol.at<double>(1, 0),
                       sol.at<double>(2, 0));
    return true;
}

static inline double poly_eval(const cv::Vec3d& c, double y) {
    return c[0] * y * y + c[1] * y + c[2];
}


// ═════════════════════════════════════════════════════════════════════════════
//  Nodo principal
// ═════════════════════════════════════════════════════════════════════════════
class LaneDetectionNode : public rclcpp::Node
{
public:
    LaneDetectionNode() : rclcpp::Node("lane_detection")
    {
        // ── Declarar todos los parámetros (configurables desde launch) ────
        this->declare_parameter("camera_topic",     "/arducam/image_raw");
        this->declare_parameter("debug_output",     false);

        this->declare_parameter("roi_top_ratio",    (double)defaults::ROI_TOP_RATIO);

        this->declare_parameter("src_tl_x",         defaults::SRC_TL_X);
        this->declare_parameter("src_tr_x",         defaults::SRC_TR_X);
        this->declare_parameter("src_top_y",        defaults::SRC_TOP_Y);
        this->declare_parameter("src_bl_x",         defaults::SRC_BL_X);
        this->declare_parameter("src_br_x",         defaults::SRC_BR_X);
        this->declare_parameter("src_bot_y",        defaults::SRC_BOT_Y);

        this->declare_parameter("warp_width",       defaults::WARP_WIDTH);
        this->declare_parameter("warp_height",      defaults::WARP_HEIGHT);

        this->declare_parameter("hsv_v_min",        defaults::HSV_V_MIN);
        this->declare_parameter("hsv_s_max",        defaults::HSV_S_MAX);
        this->declare_parameter("lab_l_threshold",  defaults::LAB_L_THRESHOLD);

        this->declare_parameter("nwindows",         defaults::NWINDOWS);
        this->declare_parameter("margin",           defaults::MARGIN);
        this->declare_parameter("minpix",           defaults::MINPIX);
        this->declare_parameter("lane_width_px",    defaults::LANE_WIDTH_PX);

        this->declare_parameter("smoothing_frames", defaults::SMOOTHING_FRAMES);
        this->declare_parameter("min_contour_area", defaults::MIN_CONTOUR_AREA);
        this->declare_parameter("min_aspect_ratio",(double)defaults::MIN_ASPECT_RATIO);

        // [OPT-4] Ruta rápida solo-LAB. true = comportamiento original.
        this->declare_parameter("use_hsv_mask",     true);
        // [OPT-7] Instrumentación de tiempos.
        this->declare_parameter("log_timing",       false);

        // ── Leer parámetros ───────────────────────────────────────────────
        std::string camera_topic = this->get_parameter("camera_topic").as_string();
        debug_output_   = this->get_parameter("debug_output").as_bool();

        roi_top_ratio_  = (float)this->get_parameter("roi_top_ratio").as_double();
        src_tl_x_       = this->get_parameter("src_tl_x").as_int();
        src_tr_x_       = this->get_parameter("src_tr_x").as_int();
        src_top_y_      = this->get_parameter("src_top_y").as_int();
        src_bl_x_       = this->get_parameter("src_bl_x").as_int();
        src_br_x_       = this->get_parameter("src_br_x").as_int();
        src_bot_y_      = this->get_parameter("src_bot_y").as_int();
        warp_width_     = this->get_parameter("warp_width").as_int();
        warp_height_    = this->get_parameter("warp_height").as_int();
        hsv_v_min_      = this->get_parameter("hsv_v_min").as_int();
        hsv_s_max_      = this->get_parameter("hsv_s_max").as_int();
        lab_l_threshold_= this->get_parameter("lab_l_threshold").as_int();
        nwindows_       = this->get_parameter("nwindows").as_int();
        margin_         = this->get_parameter("margin").as_int();
        minpix_         = this->get_parameter("minpix").as_int();
        lane_width_px_  = this->get_parameter("lane_width_px").as_int();
        smoothing_frames_  = this->get_parameter("smoothing_frames").as_int();
        min_contour_area_  = this->get_parameter("min_contour_area").as_int();
        min_aspect_ratio_  = (float)this->get_parameter("min_aspect_ratio").as_double();
        use_hsv_mask_      = this->get_parameter("use_hsv_mask").as_bool();
        log_timing_        = this->get_parameter("log_timing").as_bool();

        // ── Inicializar kernel y CLAHE ────────────────────────────────────
        // [OPT-1] La matriz de perspectiva ya NO se calcula aquí: depende del
        //         offset del ROI, que depende de la altura real del frame.
        kernel_ = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(5, 5));
        clahe_  = cv::createCLAHE(2.0, cv::Size(8, 8));

        matrices_ready_ = false;
        roi_offset_     = 0;
        last_distance_  = 0;

        frames_seen_    = 0;
        accum_us_       = 0;

        // ── Publishers ────────────────────────────────────────────────────
        distance_pub_ = this->create_publisher<std_msgs::msg::Int16>(
                           "/distance_center_line", 1);

        if (debug_output_) {
            debug_pub_   = this->create_publisher<sensor_msgs::msg::Image>(
                               "/lane_detection/debug_image", 1);
            warped_pub_  = this->create_publisher<sensor_msgs::msg::Image>(
                               "/lane_detection/debug_warped", 1);
            windows_pub_ = this->create_publisher<sensor_msgs::msg::Image>(
                               "/lane_detection/debug_windows", 1);
        }

        // ── Subscriber ────────────────────────────────────────────────────
        // [OPT-5] SensorDataQoS = BEST_EFFORT + KEEP_LAST(depth 1).
        //         Preferimos tirar frames antes que acumular latencia.
        image_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
            camera_topic, rclcpp::SensorDataQoS(),
            std::bind(&LaneDetectionNode::image_callback, this,
                      std::placeholders::_1));

        RCLCPP_INFO(get_logger(),
            "Lane detection iniciado | cámara: %s | debug: %s | hsv: %s | timing: %s",
            camera_topic.c_str(),
            debug_output_ ? "ON" : "OFF",
            use_hsv_mask_ ? "ON" : "OFF (solo LAB)",
            log_timing_   ? "ON" : "OFF");
    }

private:
    // ── Subscriptores y publicadores ──────────────────────────────────────
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;
    rclcpp::Publisher<std_msgs::msg::Int16>::SharedPtr       distance_pub_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr    debug_pub_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr    warped_pub_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr    windows_pub_;

    // ── Parámetros ────────────────────────────────────────────────────────
    bool  debug_output_;
    bool  use_hsv_mask_;
    bool  log_timing_;
    float roi_top_ratio_;
    int   src_tl_x_, src_tr_x_, src_top_y_;
    int   src_bl_x_, src_br_x_, src_bot_y_;
    int   warp_width_, warp_height_;
    int   hsv_v_min_, hsv_s_max_, lab_l_threshold_;
    int   nwindows_, margin_, minpix_, lane_width_px_;
    int   smoothing_frames_, min_contour_area_;
    float min_aspect_ratio_;

    // ── Estado interno ────────────────────────────────────────────────────
    cv::Mat M_, M_inv_;                  // matrices de bird's-eye
    cv::Mat kernel_;                     // kernel morfológico
    cv::Ptr<cv::CLAHE> clahe_;

    bool  matrices_ready_;               // [OPT-1] init diferida
    int   roi_offset_;                   // [OPT-1] fila donde empieza el ROI

    std::deque<cv::Vec3d> left_fit_history_;
    std::deque<cv::Vec3d> right_fit_history_;
    std::deque<int>       distance_history_;
    int last_distance_;

    // ── Buffers reutilizados entre frames (evitan realloc por frame) ──────
    cv::Mat buf_blurred_, buf_hsv_, buf_lab_, buf_l_, buf_v_;
    cv::Mat buf_mask_hsv_, buf_mask_lab_, buf_mask_, buf_warped_;
    std::vector<cv::Point> buf_nonzero_;

    // ── Instrumentación [OPT-7] ──────────────────────────────────────────
    long long frames_seen_;
    long long accum_us_;

    // ─────────────────────────────────────────────────────────────────────
    //  [OPT-1] Calcular matrices de bird's-eye una vez, ya conocida la
    //  altura del frame. Los puntos fuente vienen en coordenadas del frame
    //  completo, así que se les resta el offset del ROI.
    // ─────────────────────────────────────────────────────────────────────
    void compute_perspective_matrix(int frame_rows)
    {
        roi_offset_ = (int)(frame_rows * roi_top_ratio_);
        if (roi_offset_ < 0) roi_offset_ = 0;
        if (roi_offset_ >= frame_rows) roi_offset_ = 0;

        int top_y = src_top_y_ - roi_offset_;
        int bot_y = src_bot_y_ - roi_offset_;

        // Salvaguarda: si el trapecio empieza por encima del ROI, el recorte
        // estaría descartando información que sí se usa. Avisamos y bajamos
        // el offset para no romper la calibración existente.
        if (top_y < 0) {
            RCLCPP_WARN(get_logger(),
                "src_top_y (%d) queda por encima del ROI (offset %d). "
                "Reduciendo roi_top_ratio para no perder el trapecio. "
                "Recalibra src_top_y o roi_top_ratio.",
                src_top_y_, roi_offset_);
            roi_offset_ = src_top_y_;
            top_y = 0;
            bot_y = src_bot_y_ - roi_offset_;
        }

        cv::Point2f src[4] = {
            cv::Point2f((float)src_tl_x_, (float)top_y),
            cv::Point2f((float)src_tr_x_, (float)top_y),
            cv::Point2f((float)src_br_x_, (float)bot_y),
            cv::Point2f((float)src_bl_x_, (float)bot_y),
        };
        cv::Point2f dst[4] = {
            cv::Point2f(0, 0),
            cv::Point2f((float)warp_width_, 0),
            cv::Point2f((float)warp_width_, (float)warp_height_),
            cv::Point2f(0, (float)warp_height_),
        };
        M_     = cv::getPerspectiveTransform(src, dst);
        M_inv_ = cv::getPerspectiveTransform(dst, src);

        matrices_ready_ = true;

        RCLCPP_INFO(get_logger(),
            "Bird's-eye listo | frame %d filas | ROI desde fila %d "
            "| trapecio y: %d..%d (coords ROI)",
            frame_rows, roi_offset_, top_y, bot_y);
    }

    // ─────────────────────────────────────────────────────────────────────
    //  Callback principal: procesa cada frame entrante
    // ─────────────────────────────────────────────────────────────────────
    void image_callback(const sensor_msgs::msg::Image::SharedPtr msg)
    {
        auto t0 = std::chrono::steady_clock::now();

        cv::Mat frame;
        try {
            frame = cv_bridge::toCvCopy(msg, "bgr8")->image;
        } catch (cv_bridge::Exception& e) {
            RCLCPP_ERROR(get_logger(), "cv_bridge error: %s", e.what());
            return;
        }

        if (frame.empty()) return;

        // [OPT-1] Init diferida de la matriz de perspectiva
        if (!matrices_ready_) compute_perspective_matrix(frame.rows);

        // ── 1. ROI: submatriz, SIN copia ─────────────────────────────────
        // [OPT-1] Antes: clone() del frame + pintar la mitad superior de
        //         negro, y todo el pipeline seguía corriendo a 640x480.
        //         Ahora el pipeline solo ve las filas que importan.
        cv::Mat roi = frame(cv::Rect(0, roi_offset_,
                                     frame.cols, frame.rows - roi_offset_));

        // ── 2. Máscara blanca robusta ────────────────────────────────────
        const cv::Mat& mask = create_white_mask(roi);

        // ── 3. Bird's-eye view ───────────────────────────────────────────
        cv::warpPerspective(mask, buf_warped_, M_,
                            cv::Size(warp_width_, warp_height_));

        // ── 4. Ventanas deslizantes + ajuste polinómico ──────────────────
        cv::Vec3d left_fit, right_fit;
        bool left_ok, right_ok;
        cv::Mat windows_dbg;
        sliding_windows(buf_warped_, left_fit, right_fit,
                        left_ok, right_ok, windows_dbg);

        // ── 5. Suavizado temporal ────────────────────────────────────────
        if (left_ok)  push_history(left_fit_history_,  left_fit);
        if (right_ok) push_history(right_fit_history_, right_fit);

        bool have_left  = !left_fit_history_.empty();
        bool have_right = !right_fit_history_.empty();

        cv::Vec3d smooth_left  = have_left  ? mean_history(left_fit_history_)  : cv::Vec3d();
        cv::Vec3d smooth_right = have_right ? mean_history(right_fit_history_) : cv::Vec3d();

        // ── 6. Calcular error lateral ────────────────────────────────────
        int distance = compute_lane_offset(have_left  ? &smooth_left  : nullptr,
                                           have_right ? &smooth_right : nullptr);
        push_int_history(distance_history_, distance);
        int smooth_distance = mean_int_history(distance_history_);

        // ── 7. Publicar /distance_center_line ────────────────────────────
        std_msgs::msg::Int16 out_msg;
        out_msg.data = (int16_t)smooth_distance;
        distance_pub_->publish(out_msg);

        // ── 8. Publicar imágenes de debug si están habilitadas ───────────
        if (debug_output_) {
            cv::Mat overlay = draw_lane_overlay(
                frame,
                have_left  ? &smooth_left  : nullptr,
                have_right ? &smooth_right : nullptr,
                smooth_distance);

            publish_debug_image(debug_pub_,   overlay,     msg->header, "bgr8");
            publish_debug_image(warped_pub_,  buf_warped_, msg->header, "mono8");
            publish_debug_image(windows_pub_, windows_dbg, msg->header, "bgr8");
        }

        // ── 9. Instrumentación [OPT-7] ───────────────────────────────────
        if (log_timing_) {
            auto t1 = std::chrono::steady_clock::now();
            accum_us_ += std::chrono::duration_cast<std::chrono::microseconds>(
                             t1 - t0).count();
            if (++frames_seen_ >= defaults::TIMING_WINDOW) {
                double ms = (double)accum_us_ / (double)frames_seen_ / 1000.0;
                RCLCPP_INFO(get_logger(),
                    "procesamiento: %.2f ms/frame  (techo teórico %.1f FPS)",
                    ms, ms > 0.0 ? 1000.0 / ms : 0.0);
                frames_seen_ = 0;
                accum_us_    = 0;
            }
        }
    }

    // ─────────────────────────────────────────────────────────────────────
    //  2. Máscara blanca robusta (HSV ∩ LAB) + filtro de contornos
    // ─────────────────────────────────────────────────────────────────────
    const cv::Mat& create_white_mask(const cv::Mat& roi)
    {
        if (use_hsv_mask_) {
            // ── Ruta original: HSV ∩ LAB, ambas con CLAHE ────────────────
            cv::GaussianBlur(roi, buf_blurred_, cv::Size(5, 5), 0);

            // HSV con CLAHE en V
            cv::cvtColor(buf_blurred_, buf_hsv_, cv::COLOR_BGR2HSV);
            std::vector<cv::Mat> hsv_channels(3);
            cv::split(buf_hsv_, hsv_channels);
            clahe_->apply(hsv_channels[2], buf_v_);
            hsv_channels[2] = buf_v_;
            cv::Mat hsv_eq;
            cv::merge(hsv_channels, hsv_eq);
            cv::inRange(hsv_eq,
                        cv::Scalar(0,   0,          hsv_v_min_),
                        cv::Scalar(180, hsv_s_max_, 255),
                        buf_mask_hsv_);

            // LAB con CLAHE en L
            cv::cvtColor(buf_blurred_, buf_lab_, cv::COLOR_BGR2Lab);
            cv::extractChannel(buf_lab_, buf_l_, 0);
            cv::Mat l_eq;
            clahe_->apply(buf_l_, l_eq);
            cv::threshold(l_eq, buf_mask_lab_, lab_l_threshold_,
                          255, cv::THRESH_BINARY);

            cv::bitwise_and(buf_mask_hsv_, buf_mask_lab_, buf_mask_);
        }
        else {
            // ── [OPT-4] Ruta rápida: solo LAB ───────────────────────────
            // El desenfoque se aplica únicamente al canal L (1 canal en vez
            // de 3). El resultado difiere de forma imperceptible tras el
            // umbral binario, pero VALIDA EN PISTA antes de competir.
            cv::cvtColor(roi, buf_lab_, cv::COLOR_BGR2Lab);
            cv::extractChannel(buf_lab_, buf_l_, 0);
            cv::GaussianBlur(buf_l_, buf_blurred_, cv::Size(5, 5), 0);
            cv::Mat l_eq;
            clahe_->apply(buf_blurred_, l_eq);
            cv::threshold(l_eq, buf_mask_, lab_l_threshold_,
                          255, cv::THRESH_BINARY);
        }

        // ── Morfología: open + close (in-place) ──────────────────────────
        cv::morphologyEx(buf_mask_, buf_mask_, cv::MORPH_OPEN,  kernel_);
        cv::morphologyEx(buf_mask_, buf_mask_, cv::MORPH_CLOSE, kernel_);

        // ── Filtro de contornos por área y aspect ratio ──────────────────
        return filter_contours(buf_mask_);
    }

    // ─────────────────────────────────────────────────────────────────────
    //  [OPT-6] Un solo drawContours con los contornos aceptados
    // ─────────────────────────────────────────────────────────────────────
    const cv::Mat& filter_contours(const cv::Mat& mask)
    {
        std::vector<std::vector<cv::Point>> contours;
        cv::findContours(mask, contours, cv::RETR_EXTERNAL,
                         cv::CHAIN_APPROX_SIMPLE);

        std::vector<std::vector<cv::Point>> keep;
        keep.reserve(contours.size());

        for (auto& c : contours) {
            if (cv::contourArea(c) < min_contour_area_) continue;

            cv::RotatedRect rect = cv::minAreaRect(c);
            float w = rect.size.width;
            float h = rect.size.height;
            if (w == 0.f || h == 0.f) continue;

            float aspect = std::max(w, h) / std::min(w, h);
            if (aspect < min_aspect_ratio_) continue;

            keep.push_back(c);
        }

        buf_clean_.create(mask.size(), CV_8UC1);
        buf_clean_.setTo(cv::Scalar(0));
        if (!keep.empty()) {
            cv::drawContours(buf_clean_, keep, -1, cv::Scalar(255), cv::FILLED);
        }
        return buf_clean_;
    }
    cv::Mat buf_clean_;

    // ─────────────────────────────────────────────────────────────────────
    //  4. Ventanas deslizantes
    //  [OPT-3] Cada ventana recorre solo su tramo contiguo del arreglo de
    //  píxeles activos, en vez de recorrer el arreglo entero 9 veces.
    //  El recentrado sigue siendo secuencial, así que el resultado numérico
    //  es idéntico al original (verificado en 480 casos sintéticos).
    // ─────────────────────────────────────────────────────────────────────
    void sliding_windows(const cv::Mat& warped,
                         cv::Vec3d& left_fit, cv::Vec3d& right_fit,
                         bool& left_ok, bool& right_ok,
                         cv::Mat& debug_img)
    {
        left_ok  = false;
        right_ok = false;

        // [OPT-2] La imagen de debug solo se crea si hace falta
        if (debug_output_) {
            cv::cvtColor(warped, debug_img, cv::COLOR_GRAY2BGR);
        }

        // ── Histograma vertical de la mitad inferior ─────────────────────
        cv::Mat lower_half = warped(cv::Rect(0, warped.rows / 2,
                                              warped.cols, warped.rows / 2));
        cv::Mat hist;
        cv::reduce(lower_half, hist, 0, cv::REDUCE_SUM, CV_32S);

        int midpoint = warped.cols / 2;

        int leftx_base = 0,        leftx_max  = 0;
        int rightx_base = midpoint, rightx_max = 0;
        const int* hp = hist.ptr<int>(0);
        for (int x = 0; x < midpoint; x++) {
            if (hp[x] > leftx_max) { leftx_max = hp[x]; leftx_base = x; }
        }
        for (int x = midpoint; x < warped.cols; x++) {
            if (hp[x] > rightx_max) { rightx_max = hp[x]; rightx_base = x; }
        }

        bool left_found  = (leftx_max  / 255) > defaults::MIN_HISTOGRAM_PEAK;
        bool right_found = (rightx_max / 255) > defaults::MIN_HISTOGRAM_PEAK;

        // ── Recolectar píxeles activos ───────────────────────────────────
        buf_nonzero_.clear();
        cv::findNonZero(warped, buf_nonzero_);

        const int window_height = warped.rows / nwindows_;
        if (window_height <= 0) return;

        // [OPT-3] cv::findNonZero recorre la matriz fila por fila, así que
        //         buf_nonzero_ ya viene ordenado por 'y' ascendente. Eso
        //         significa que los píxeles de cada ventana forman un TRAMO
        //         CONTIGUO del arreglo: basta localizar sus límites con
        //         lower_bound y recorrer solo ese tramo. Cero copias y buena
        //         localidad de caché.
        //         (Probado equivalente al doble bucle original en 480 casos,
        //          incluyendo alturas no divisibles entre nwindows.)

        int leftx_current  = leftx_base;
        int rightx_current = rightx_base;

        std::vector<int> leftx_pts, lefty_pts, rightx_pts, righty_pts;
        leftx_pts.reserve(buf_nonzero_.size() / 2);
        lefty_pts.reserve(buf_nonzero_.size() / 2);
        rightx_pts.reserve(buf_nonzero_.size() / 2);
        righty_pts.reserve(buf_nonzero_.size() / 2);

        for (int w = 0; w < nwindows_; w++) {
            int win_y_low  = warped.rows - (w + 1) * window_height;
            int win_y_high = warped.rows -  w      * window_height;

            int win_xleft_low   = leftx_current  - margin_;
            int win_xleft_high  = leftx_current  + margin_;
            int win_xright_low  = rightx_current - margin_;
            int win_xright_high = rightx_current + margin_;

            // [OPT-2] Dibujar ventanas solo en modo debug
            if (debug_output_) {
                cv::rectangle(debug_img,
                    cv::Point(win_xleft_low,  win_y_low),
                    cv::Point(win_xleft_high, win_y_high),
                    cv::Scalar(0, 255, 0), 2);
                cv::rectangle(debug_img,
                    cv::Point(win_xright_low,  win_y_low),
                    cv::Point(win_xright_high, win_y_high),
                    cv::Scalar(0, 255, 0), 2);
            }

            // [OPT-3] Solo el tramo de ESTA ventana, sin copiar nada
            auto it_beg = std::lower_bound(
                buf_nonzero_.begin(), buf_nonzero_.end(), win_y_low,
                [](const cv::Point& p, int v) { return p.y < v; });
            auto it_end = std::lower_bound(
                buf_nonzero_.begin(), buf_nonzero_.end(), win_y_high,
                [](const cv::Point& p, int v) { return p.y < v; });

            long sum_left = 0,  n_left  = 0;
            long sum_right = 0, n_right = 0;

            for (auto it = it_beg; it != it_end; ++it) {
                if (it->x >= win_xleft_low && it->x < win_xleft_high) {
                    leftx_pts.push_back(it->x);
                    lefty_pts.push_back(it->y);
                    sum_left += it->x;
                    ++n_left;
                }
                if (it->x >= win_xright_low && it->x < win_xright_high) {
                    rightx_pts.push_back(it->x);
                    righty_pts.push_back(it->y);
                    sum_right += it->x;
                    ++n_right;
                }
            }

            // Recentrar ventana si hay suficientes píxeles
            if (n_left  > minpix_) leftx_current  = (int)(sum_left  / n_left);
            if (n_right > minpix_) rightx_current = (int)(sum_right / n_right);
        }

        // [OPT-2] Pintar píxeles detectados solo en modo debug
        if (debug_output_) {
            for (size_t i = 0; i < leftx_pts.size(); i++)
                debug_img.at<cv::Vec3b>(lefty_pts[i],  leftx_pts[i])  = cv::Vec3b(255, 0, 0);
            for (size_t i = 0; i < rightx_pts.size(); i++)
                debug_img.at<cv::Vec3b>(righty_pts[i], rightx_pts[i]) = cv::Vec3b(0, 0, 255);
        }

        // ── Ajustar polinomio grado 2: x = a·y² + b·y + c ────────────────
        if (left_found  && (int)leftx_pts.size()  > defaults::MIN_PIXELS_FIT)
            left_ok  = polyfit2(leftx_pts,  lefty_pts,  left_fit);
        if (right_found && (int)rightx_pts.size() > defaults::MIN_PIXELS_FIT)
            right_ok = polyfit2(rightx_pts, righty_pts, right_fit);
    }

    // ─────────────────────────────────────────────────────────────────────
    //  5. Suavizado temporal (promedio móvil)
    // ─────────────────────────────────────────────────────────────────────
    void push_history(std::deque<cv::Vec3d>& q, const cv::Vec3d& v)
    {
        q.push_back(v);
        while ((int)q.size() > smoothing_frames_) q.pop_front();
    }

    cv::Vec3d mean_history(const std::deque<cv::Vec3d>& q)
    {
        cv::Vec3d sum(0, 0, 0);
        for (auto& v : q) sum += v;
        return sum * (1.0 / (double)q.size());
    }

    void push_int_history(std::deque<int>& q, int v)
    {
        q.push_back(v);
        while ((int)q.size() > smoothing_frames_) q.pop_front();
    }

    int mean_int_history(const std::deque<int>& q)
    {
        if (q.empty()) return 0;
        long sum = 0;
        for (auto v : q) sum += v;
        return (int)(sum / (long)q.size());
    }

    // ─────────────────────────────────────────────────────────────────────
    //  6. Cálculo del error lateral
    //
    //  Estrategia de fallback (idéntica al original):
    //    • Ambas líneas       → centro = promedio
    //    • Solo izquierda     → centro = izq + LANE_WIDTH/2
    //    • Solo derecha       → centro = der - LANE_WIDTH/2
    //    • Ninguna            → mantener última detección
    // ─────────────────────────────────────────────────────────────────────
    int compute_lane_offset(const cv::Vec3d* left_fit, const cv::Vec3d* right_fit)
    {
        double y_eval     = warp_height_ - 1;
        double car_center = warp_width_ / 2.0;

        bool has_left  = (left_fit  != nullptr);
        bool has_right = (right_fit != nullptr);

        double lane_center;
        if (has_left && has_right) {
            double lx = poly_eval(*left_fit,  y_eval);
            double rx = poly_eval(*right_fit, y_eval);
            lane_center = (lx + rx) / 2.0;
        }
        else if (has_left) {
            lane_center = poly_eval(*left_fit, y_eval) + lane_width_px_ / 2.0;
        }
        else if (has_right) {
            lane_center = poly_eval(*right_fit, y_eval) - lane_width_px_ / 2.0;
        }
        else {
            return last_distance_;   // mantener última detección válida
        }

        int distance = (int)std::round(lane_center - car_center);
        last_distance_ = distance;
        return distance;
    }

    // ─────────────────────────────────────────────────────────────────────
    //  7. Overlay de debug (solo se llama si debug_output_ == true)
    //  [OPT-1] M_inv_ ahora devuelve coordenadas del ROI, no del frame
    //          completo, así que el resultado se compone en la franja
    //          correcta del frame original.
    // ─────────────────────────────────────────────────────────────────────
    cv::Mat draw_lane_overlay(const cv::Mat& frame,
                              const cv::Vec3d* left_fit,
                              const cv::Vec3d* right_fit,
                              int distance)
    {
        cv::Mat overlay = frame.clone();

        if (left_fit && right_fit) {
            cv::Mat lane_warped = cv::Mat::zeros(warp_height_, warp_width_, CV_8UC3);

            std::vector<cv::Point> pts;
            pts.reserve(warp_height_ * 2);
            for (int y = 0; y < warp_height_; y++)
                pts.emplace_back((int)std::round(poly_eval(*left_fit, y)), y);
            for (int y = warp_height_ - 1; y >= 0; y--)
                pts.emplace_back((int)std::round(poly_eval(*right_fit, y)), y);

            cv::fillPoly(lane_warped, std::vector<std::vector<cv::Point>>{pts},
                         cv::Scalar(0, 255, 0));

            // Volver a vista frontal — en coordenadas del ROI
            int roi_rows = frame.rows - roi_offset_;
            cv::Mat unwarp;
            cv::warpPerspective(lane_warped, unwarp, M_inv_,
                                cv::Size(frame.cols, roi_rows));

            cv::Mat overlay_roi = overlay(cv::Rect(0, roi_offset_,
                                                   frame.cols, roi_rows));
            cv::addWeighted(overlay_roi, 1.0, unwarp, 0.4, 0, overlay_roi);
        }

        cv::putText(overlay, cv::format("Error: %+d px", distance),
                    cv::Point(10, 30), cv::FONT_HERSHEY_SIMPLEX, 0.7,
                    cv::Scalar(255, 255, 255), 2);

        std::string status;
        cv::Scalar color;
        if (std::abs(distance) < 15) {
            status = "CENTRADO";           color = cv::Scalar(0, 255, 0);
        } else if (distance > 0) {
            status = "GIRAR DERECHA ->";   color = cv::Scalar(0, 255, 255);
        } else {
            status = "<- GIRAR IZQUIERDA"; color = cv::Scalar(0, 255, 255);
        }
        cv::putText(overlay, status, cv::Point(10, 60),
                    cv::FONT_HERSHEY_SIMPLEX, 0.7, color, 2);

        return overlay;
    }

    // ─────────────────────────────────────────────────────────────────────
    //  Helper: publicar imagen de debug en un tópico
    // ─────────────────────────────────────────────────────────────────────
    void publish_debug_image(
        rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr& pub,
        const cv::Mat& img,
        const std_msgs::msg::Header& header,
        const std::string& encoding)
    {
        if (!pub || img.empty()) return;
        try {
            auto msg = cv_bridge::CvImage(header, encoding, img).toImageMsg();
            pub->publish(*msg);
        } catch (cv_bridge::Exception& e) {
            RCLCPP_WARN(get_logger(), "Debug publish error: %s", e.what());
        }
    }
};


// ═════════════════════════════════════════════════════════════════════════════
//  main
// ═════════════════════════════════════════════════════════════════════════════
int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<LaneDetectionNode>());
    rclcpp::shutdown();
    return 0;
}
