// ═════════════════════════════════════════════════════════════════════════════
//  lane_detection.cpp  –  ROS 2 (Humble / Iron)
//  Cámara: Arducam IMX219 (CSI)  →  /arducam/image_raw
//
//  Pipeline (mismo que la versión validada en Python):
//    Frame → ROI inferior → Blur → CLAHE
//         → HSV mask + LAB threshold (combinadas)
//         → Filtro de contornos por área y aspect ratio
//         → Bird's-eye view (vista cenital)
//         → Histograma + ventanas deslizantes
//         → Ajuste polinómico de grado 2
//         → Suavizado temporal
//         → Cálculo de error lateral
//
//  Salida:
//    /distance_center_line  (std_msgs/Int16)  → consumido por los Masters
//    /lane_detection/debug_image      (sensor_msgs/Image, si debug_output=true)
//    /lane_detection/debug_warped     (sensor_msgs/Image, si debug_output=true)
//    /lane_detection/debug_windows    (sensor_msgs/Image, si debug_output=true)
//
//  Migración Python → C++ ROS 2:
//    • Clase LaneDetector → hereda de rclcpp::Node
//    • cv2.* → cv::* (OpenCV C++)
//    • collections.deque → std::deque<cv::Vec3f>
//    • np.polyfit → cv::solve con matriz de Vandermonde
//    • cv2.imshow → publicar en tópicos de debug
// ═════════════════════════════════════════════════════════════════════════════

#include <chrono>
#include <deque>
#include <memory>
#include <vector>
#include <cmath>

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

    // Bird's-eye view (calibrar en pista)
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

        // ── Inicializar matrices de bird's-eye y kernel/CLAHE ─────────────
        compute_perspective_matrix();
        kernel_ = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(5, 5));
        clahe_  = cv::createCLAHE(2.0, cv::Size(8, 8));

        last_distance_ = 0;

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
        image_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
            camera_topic, 1,
            std::bind(&LaneDetectionNode::image_callback, this,
                      std::placeholders::_1));

        RCLCPP_INFO(get_logger(),
            "Lane detection iniciado | cámara: %s | debug: %s",
            camera_topic.c_str(), debug_output_ ? "ON" : "OFF");
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

    std::deque<cv::Vec3d> left_fit_history_;
    std::deque<cv::Vec3d> right_fit_history_;
    std::deque<int>       distance_history_;
    int last_distance_;

    // ─────────────────────────────────────────────────────────────────────
    //  Calcular matriz de bird's-eye una sola vez al inicio
    // ─────────────────────────────────────────────────────────────────────
    void compute_perspective_matrix()
    {
        cv::Point2f src[4] = {
            cv::Point2f(src_tl_x_, src_top_y_),
            cv::Point2f(src_tr_x_, src_top_y_),
            cv::Point2f(src_br_x_, src_bot_y_),
            cv::Point2f(src_bl_x_, src_bot_y_),
        };
        cv::Point2f dst[4] = {
            cv::Point2f(0, 0),
            cv::Point2f(warp_width_, 0),
            cv::Point2f(warp_width_, warp_height_),
            cv::Point2f(0, warp_height_),
        };
        M_     = cv::getPerspectiveTransform(src, dst);
        M_inv_ = cv::getPerspectiveTransform(dst, src);
    }

    // ─────────────────────────────────────────────────────────────────────
    //  Callback principal: procesa cada frame entrante
    // ─────────────────────────────────────────────────────────────────────
    void image_callback(const sensor_msgs::msg::Image::SharedPtr msg)
    {
        cv::Mat frame;
        try {
            frame = cv_bridge::toCvCopy(msg, "bgr8")->image;
        } catch (cv_bridge::Exception& e) {
            RCLCPP_ERROR(get_logger(), "cv_bridge error: %s", e.what());
            return;
        }

        if (frame.empty()) return;

        // ── 1. ROI ────────────────────────────────────────────────────────
        cv::Mat roi = apply_roi(frame);

        // ── 2. Máscara blanca robusta ────────────────────────────────────
        cv::Mat mask = create_white_mask(roi);

        // ── 3. Bird's-eye view ───────────────────────────────────────────
        cv::Mat warped;
        cv::warpPerspective(mask, warped, M_,
                            cv::Size(warp_width_, warp_height_));

        // ── 4. Ventanas deslizantes + ajuste polinómico ──────────────────
        cv::Vec3d left_fit, right_fit;
        bool left_ok, right_ok;
        cv::Mat windows_dbg;
        sliding_windows(warped, left_fit, right_fit, left_ok, right_ok, windows_dbg);

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
            publish_debug_image(warped_pub_,  warped,      msg->header, "mono8");
            publish_debug_image(windows_pub_, windows_dbg, msg->header, "bgr8");
        }
    }

    // ─────────────────────────────────────────────────────────────────────
    //  1. ROI: pintar de negro la mitad superior
    // ─────────────────────────────────────────────────────────────────────
    cv::Mat apply_roi(const cv::Mat& frame)
    {
        cv::Mat roi = frame.clone();
        int top_limit = (int)(frame.rows * roi_top_ratio_);
        if (top_limit > 0) {
            roi(cv::Rect(0, 0, frame.cols, top_limit)) = cv::Scalar(0, 0, 0);
        }
        return roi;
    }

    // ─────────────────────────────────────────────────────────────────────
    //  2. Máscara blanca robusta (HSV ∩ LAB) + filtro de contornos
    // ─────────────────────────────────────────────────────────────────────
    cv::Mat create_white_mask(const cv::Mat& frame)
    {
        cv::Mat blurred;
        cv::GaussianBlur(frame, blurred, cv::Size(5, 5), 0);

        // ── HSV con CLAHE en V ────────────────────────────────────────────
        cv::Mat hsv;
        cv::cvtColor(blurred, hsv, cv::COLOR_BGR2HSV);
        std::vector<cv::Mat> hsv_channels(3);
        cv::split(hsv, hsv_channels);
        cv::Mat v_eq;
        clahe_->apply(hsv_channels[2], v_eq);
        hsv_channels[2] = v_eq;
        cv::Mat hsv_eq;
        cv::merge(hsv_channels, hsv_eq);
        cv::Mat mask_hsv;
        cv::inRange(hsv_eq,
                    cv::Scalar(0,   0,            hsv_v_min_),
                    cv::Scalar(180, hsv_s_max_,   255),
                    mask_hsv);

        // ── LAB con CLAHE en L ───────────────────────────────────────────
        cv::Mat lab;
        cv::cvtColor(blurred, lab, cv::COLOR_BGR2Lab);
        std::vector<cv::Mat> lab_channels(3);
        cv::split(lab, lab_channels);
        cv::Mat l_eq;
        clahe_->apply(lab_channels[0], l_eq);
        cv::Mat mask_lab;
        cv::threshold(l_eq, mask_lab, lab_l_threshold_, 255, cv::THRESH_BINARY);

        // ── AND lógico (intersección) ────────────────────────────────────
        cv::Mat mask;
        cv::bitwise_and(mask_hsv, mask_lab, mask);

        // ── Morfología: open + close ─────────────────────────────────────
        cv::morphologyEx(mask, mask, cv::MORPH_OPEN,  kernel_);
        cv::morphologyEx(mask, mask, cv::MORPH_CLOSE, kernel_);

        // ── Filtro de contornos por área y aspect ratio ──────────────────
        return filter_contours(mask);
    }

    cv::Mat filter_contours(const cv::Mat& mask)
    {
        std::vector<std::vector<cv::Point>> contours;
        cv::findContours(mask, contours, cv::RETR_EXTERNAL,
                         cv::CHAIN_APPROX_SIMPLE);

        cv::Mat clean = cv::Mat::zeros(mask.size(), CV_8UC1);
        for (auto& c : contours) {
            double area = cv::contourArea(c);
            if (area < min_contour_area_) continue;

            cv::RotatedRect rect = cv::minAreaRect(c);
            float w = rect.size.width;
            float h = rect.size.height;
            if (w == 0 || h == 0) continue;

            float aspect = std::max(w, h) / std::min(w, h);
            if (aspect < min_aspect_ratio_) continue;

            cv::drawContours(clean,
                             std::vector<std::vector<cv::Point>>{c},
                             -1, cv::Scalar(255), cv::FILLED);
        }
        return clean;
    }

    // ─────────────────────────────────────────────────────────────────────
    //  4. Ventanas deslizantes (algoritmo estándar de Udacity)
    // ─────────────────────────────────────────────────────────────────────
    void sliding_windows(const cv::Mat& warped,
                         cv::Vec3d& left_fit, cv::Vec3d& right_fit,
                         bool& left_ok, bool& right_ok,
                         cv::Mat& debug_img)
    {
        // Imagen de debug a color
        cv::cvtColor(warped, debug_img, cv::COLOR_GRAY2BGR);

        // ── Histograma vertical de la mitad inferior ─────────────────────
        cv::Mat lower_half = warped(cv::Rect(0, warped.rows / 2,
                                              warped.cols, warped.rows / 2));
        cv::Mat hist;
        cv::reduce(lower_half, hist, 0, cv::REDUCE_SUM, CV_32S);
        // hist es 1 fila × warped.cols columnas

        int midpoint = warped.cols / 2;

        int leftx_base = 0,  leftx_max  = 0;
        int rightx_base = midpoint, rightx_max = 0;
        for (int x = 0; x < midpoint; x++) {
            int v = hist.at<int>(0, x);
            if (v > leftx_max) { leftx_max = v; leftx_base = x; }
        }
        for (int x = midpoint; x < warped.cols; x++) {
            int v = hist.at<int>(0, x);
            if (v > rightx_max) { rightx_max = v; rightx_base = x; }
        }

        // El histograma suma valores de 255 → reescalamos al "tamaño en píxeles"
        bool left_found  = (leftx_max  / 255) > defaults::MIN_HISTOGRAM_PEAK;
        bool right_found = (rightx_max / 255) > defaults::MIN_HISTOGRAM_PEAK;

        // ── Recolectar todos los píxeles activos del warped ──────────────
        std::vector<cv::Point> nonzero;
        cv::findNonZero(warped, nonzero);

        int window_height = warped.rows / nwindows_;
        int leftx_current  = leftx_base;
        int rightx_current = rightx_base;

        std::vector<int> leftx_pts, lefty_pts;
        std::vector<int> rightx_pts, righty_pts;

        for (int w = 0; w < nwindows_; w++) {
            int win_y_low  = warped.rows - (w + 1) * window_height;
            int win_y_high = warped.rows -  w      * window_height;

            int win_xleft_low   = leftx_current  - margin_;
            int win_xleft_high  = leftx_current  + margin_;
            int win_xright_low  = rightx_current - margin_;
            int win_xright_high = rightx_current + margin_;

            // Dibujar ventanas en debug
            cv::rectangle(debug_img,
                cv::Point(win_xleft_low,  win_y_low),
                cv::Point(win_xleft_high, win_y_high),
                cv::Scalar(0, 255, 0), 2);
            cv::rectangle(debug_img,
                cv::Point(win_xright_low,  win_y_low),
                cv::Point(win_xright_high, win_y_high),
                cv::Scalar(0, 255, 0), 2);

            // Recolectar píxeles en cada ventana
            std::vector<int> good_left_x, good_right_x;
            for (auto& pt : nonzero) {
                if (pt.y >= win_y_low && pt.y < win_y_high) {
                    if (pt.x >= win_xleft_low && pt.x < win_xleft_high) {
                        leftx_pts.push_back(pt.x);
                        lefty_pts.push_back(pt.y);
                        good_left_x.push_back(pt.x);
                    }
                    if (pt.x >= win_xright_low && pt.x < win_xright_high) {
                        rightx_pts.push_back(pt.x);
                        righty_pts.push_back(pt.y);
                        good_right_x.push_back(pt.x);
                    }
                }
            }

            // Recentrar ventana si hay suficientes píxeles
            if ((int)good_left_x.size() > minpix_) {
                long sum = 0;
                for (auto x : good_left_x) sum += x;
                leftx_current = (int)(sum / good_left_x.size());
            }
            if ((int)good_right_x.size() > minpix_) {
                long sum = 0;
                for (auto x : good_right_x) sum += x;
                rightx_current = (int)(sum / good_right_x.size());
            }
        }

        // Pintar píxeles detectados (debug)
        for (size_t i = 0; i < leftx_pts.size(); i++)
            debug_img.at<cv::Vec3b>(lefty_pts[i],  leftx_pts[i])  = cv::Vec3b(255, 0, 0);
        for (size_t i = 0; i < rightx_pts.size(); i++)
            debug_img.at<cv::Vec3b>(righty_pts[i], rightx_pts[i]) = cv::Vec3b(0, 0, 255);

        // ── Ajustar polinomio grado 2: x = a·y² + b·y + c ────────────────
        left_ok  = false;
        right_ok = false;
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
    //  Estrategia de fallback (idéntica a la versión Python):
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
            double lx = poly_eval(*left_fit, y_eval);
            lane_center = lx + lane_width_px_ / 2.0;
        }
        else if (has_right) {
            double rx = poly_eval(*right_fit, y_eval);
            lane_center = rx - lane_width_px_ / 2.0;
        }
        else {
            return last_distance_;   // mantener última detección válida
        }

        int distance = (int)std::round(lane_center - car_center);
        last_distance_ = distance;
        return distance;
    }

    // ─────────────────────────────────────────────────────────────────────
    //  7. Overlay de debug (pinta el carril sobre la imagen original)
    // ─────────────────────────────────────────────────────────────────────
    cv::Mat draw_lane_overlay(const cv::Mat& frame,
                              const cv::Vec3d* left_fit,
                              const cv::Vec3d* right_fit,
                              int distance)
    {
        cv::Mat overlay = frame.clone();

        if (left_fit && right_fit) {
            cv::Mat lane_warped = cv::Mat::zeros(warp_height_, warp_width_, CV_8UC3);

            // Generar polígono del carril
            std::vector<cv::Point> pts;
            for (int y = 0; y < warp_height_; y++) {
                int lx = (int)std::round(poly_eval(*left_fit,  y));
                pts.push_back(cv::Point(lx, y));
            }
            for (int y = warp_height_ - 1; y >= 0; y--) {
                int rx = (int)std::round(poly_eval(*right_fit, y));
                pts.push_back(cv::Point(rx, y));
            }
            cv::fillPoly(lane_warped, std::vector<std::vector<cv::Point>>{pts},
                         cv::Scalar(0, 255, 0));

            // Volver a vista frontal
            cv::Mat unwarp;
            cv::warpPerspective(lane_warped, unwarp, M_inv_,
                                cv::Size(frame.cols, frame.rows));
            cv::addWeighted(overlay, 1.0, unwarp, 0.4, 0, overlay);
        }

        // Texto con error lateral
        cv::putText(overlay,
                    cv::format("Error: %+d px", distance),
                    cv::Point(10, 30),
                    cv::FONT_HERSHEY_SIMPLEX, 0.7,
                    cv::Scalar(255, 255, 255), 2);

        // Indicador de dirección
        std::string status;
        cv::Scalar color;
        if (std::abs(distance) < 15) {
            status = "CENTRADO";          color = cv::Scalar(0, 255, 0);
        } else if (distance > 0) {
            status = "GIRAR DERECHA ->";  color = cv::Scalar(0, 255, 255);
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
        if (!pub) return;
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
