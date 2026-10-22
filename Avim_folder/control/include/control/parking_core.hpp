// ─────────────────────────────────────────────────────────────────────────────
//  parking_core.hpp  –  Estacionamiento en PARALELO (lógica sin ROS)
//
//  Secuencia (como en la figura de la prueba):
//    SEARCH   avanza por su carril (tu on_lane con /distance_center_line) y
//             mide con el ULTRASÓNICO DERECHO: carro – hueco – carro.
//             Si el hueco mide lo suficiente => lugar encontrado.
//    APPROACH sigue derecho hasta dejar el eje trasero start_past_m adelante
//             de la defensa trasera del carro de enfrente (A).
//    REV_IN   reversa con la dirección toda a la DERECHA hasta girar arc_deg (B).
//    REV_STR  reversa derecho un tramo (calculado para llegar al centro del lugar).
//    REV_OUT  reversa con la dirección toda a la IZQUIERDA hasta quedar derecho (C).
//    FWD_FIX  si quedó chueco (lo frenó el ultrasónico trasero): adelante con la
//             dirección a la DERECHA hasta enderezarse; se repite si hace falta (D).
//    CENTER   adelante/atrás derecho hasta quedar a la misma distancia del carro
//             de enfrente y del de atrás (ultrasónicos delantero y trasero) (E).
//             Antes de centrarse revisa la PROFUNDIDAD con el ultrasónico derecho:
//             compara la distancia a la pared con la que debería tener (medida al
//             pasar junto al hueco). Si quedó lejos de la pared (no necesita IMU):
//    DEPTH_BACK  reversa derecho hasta el carro de atrás (para tener todo el espacio),
//    DEPTH_FWD / DEPTH_FWD2  una "S" hacia adelante (derecha, luego izquierda),
//    DEPTH_IN + REV_OUT      una "S" en reversa (derecha, luego izquierda).
//             Cada S recorre el carro hacia la pared y lo deja derecho. Luego
//             CENTER otra vez; se repite hasta max_depth_fixes veces.
//    PARKED   detenido, intermitentes.
//
//  Seguridad: en toda la maniobra, si el ultrasónico trasero (en reversa) o el
//  delantero (hacia adelante) ve algo a menos de *_stop_m, se detiene y pasa al
//  siguiente paso. En REV_OUT, además, se cuida la esquina trasera derecha
//  contra la pared (ultrasónico derecho + rumbo).
//
//  El rumbo se mide con el IMU (si hay) o con el comando de dirección (con el
//  retardo del servo y de la velocidad). Las distancias, con la velocidad
//  comandada (speed_mps_per_cmd, calibrada).
// ─────────────────────────────────────────────────────────────────────────────
#pragma once
#include <algorithm>
#include <cmath>
#include <cstdio>
#include <string>

namespace avim {

struct ParkParams {
    // Geometría del carro (desde el EJE TRASERO)
    double wheelbase_m      = 0.25;
    double car_front_m      = 0.34;
    double car_rear_m       = 0.09;
    double car_half_width_m = 0.10;
    double sonar_right_x_m  = 0.125;   // ultrasónico derecho, delante del eje trasero
    double sonar_side_y_m   = 0.105;   // ultrasónico derecho, a la derecha del centro

    // Dirección: comando (0-180) <-> ángulo de rueda
    double steer_center_cmd  = 90.0;
    double steer_deg_per_cmd = 0.1689;  // calibrar_odometria.py
    double steer_offset_deg  = 0.314;
    double max_steer_deg     = 14.9;    // ángulo REAL máximo de las ruedas
    double steer_right_cmd   = 0.0;     // dirección toda a la derecha (carro real: el mínimo del servo)
    double steer_left_cmd    = 180.0;   // toda a la izquierda
    double turn_radius_m     = 0.0;     // radio de giro mínimo medido (0 = calcularlo con max_steer_deg)
    double steer_settle_s    = 0.6;     // espera para que el servo gire antes de moverse
    double steer_lag_s       = 0.15;    // retraso del servo (solo para el rumbo SIN IMU)
    double speed_lag_s       = 0.15;    // el carro tarda en acelerar/frenar (rumbo y tramo recto SIN IMU; 0 = no)

    // Velocidad: comando (negativo = adelante, positivo = reversa)
    double speed_mps_per_cmd = 0.00122;
    int    search_cmd   = 250;          // buscando lugar
    int    approach_cmd = 180;          // llegando al punto de inicio
    int    maneuver_cmd = 150;          // en la maniobra
    int    creep_cmd    = 120;          // acomodándose
    double stop_lag_s   = 0.001;         // el carro tarda en frenar

    // Seguimiento de carril (tu on_lane)
    double lk_thr[4] = { 8.0, 15.0, 25.0, 35.0 };
    double lk_kp[5]  = { 0.10, 0.40, 0.55, 0.70, 0.85 };
    double lk_min_cmd = 45.0, lk_max_cmd = 135.0;

    // Búsqueda del lugar con el ultrasónico derecho
    double side_car_max_m = 0.26;       // lectura menor => hay un carro al lado
    double side_free_min_m = 0.30;      // lectura mayor => hueco (histéresis)
    double min_spot_m     = 0.62;       // largo mínimo del hueco
    double max_spot_m     = 1.80;       // más largo => no es un lugar entre dos carros
    double start_past_m   = 0.30;       // eje trasero adelante de la defensa trasera del carro de enfrente (con radio 0.94 m)
    double ref_radius_m   = 0.94;       // radio con el que se ajustó start_past_m (con otro radio se corrige solo)
    double wall_margin_m  = 0.06;       // holgura con la pared/banqueta del fondo del lugar (en la maniobra)
    double target_offset_m = 0.0;       // 0 = alinearse con los carros estacionados (medido con el sonar)

    // Maniobra
    double arc_deg        = 28.0;       // giro del primer arco (B)
    double back_stop_m    = 0.10;       // ultrasónico trasero: no acercarse más
    double front_stop_m   = 0.10;       // ultrasónico delantero
    double side_stop_m    = 0.03;       // ultrasónico derecho (pared / banqueta) en reversa
    double straight_tol_deg = 2.0;      // "derecho"
    int    max_fixes      = 4;          // correcciones adelante/atrás
    double center_tol_m   = 0.03;
    double center_timeout_s = 8.0;

    // Corrección final de profundidad (lazo cerrado con el ultrasónico DERECHO):
    // al terminar, compara la distancia a la pared con la que debería tener
    // (medida al pasar junto al hueco). Si quedó lejos de la pared, hace una
    // "S" hacia adelante y otra en reversa para recorrerse hacia la pared.
    // No necesita IMU.
    double depth_tol_m    = 0.012;      // más lejos que esto de lo ideal => corregir
    int    max_depth_fixes = 3;         // máximo de ciclos de corrección (0 = desactivado)
    double final_wall_margin_m = 0.04;  // holgura con la pared al terminar la corrección
    double depth_bias_m   = 0.0;        // + = quedar más metido (ajuste fino a mano)
};

struct ParkInputs {
    double t = 0;
    bool   has_dist = false; int dist_px = 0;
    double sonar_front = INFINITY, sonar_back = INFINITY, sonar_right = INFINITY;
    double sonar_right_t = -1;          // hora de la última lectura del sonar derecho
    bool   has_yaw_rate = false; double yaw_rate = 0;   // rad/s, + = izquierda
};

struct ParkOutputs {
    int  steer_cmd = 90, speed_cmd = 0;
    bool right_signal = false, blinkers = false;
    const char* state = "SEARCH";
    std::string event;
};

enum class PState { SEARCH, APPROACH, PAUSE, REV_IN, REV_STR, REV_OUT, FWD_FIX, CENTER, DEPTH_BACK, DEPTH_FWD, DEPTH_FWD2, DEPTH_IN, PARKED };

inline const char* pstate_name(PState s) {
    switch (s) {
        case PState::SEARCH: return "SEARCH";   case PState::APPROACH: return "APPROACH";
        case PState::PAUSE: return "PAUSE";     case PState::REV_IN: return "REV_IN";
        case PState::REV_STR: return "REV_STR"; case PState::REV_OUT: return "REV_OUT";
        case PState::FWD_FIX: return "FWD_FIX"; case PState::CENTER: return "CENTER";
        case PState::DEPTH_FWD: return "DEPTH_FWD"; case PState::DEPTH_FWD2: return "DEPTH_FWD2";
        case PState::DEPTH_IN: return "DEPTH_IN";   case PState::DEPTH_BACK: return "DEPTH_BACK";
        case PState::PARKED: return "PARKED";
    }
    return "?";
}

class ParkingCore {
public:
    explicit ParkingCore(const ParkParams& p = ParkParams()) : P(p) {}
    PState state() const { return st_; }
    double heading_deg() const { return th_ * 180.0 / M_PI; }
    double travel() const { return travel_; }
    double spot_len() const { return spot_len_; }
    double spot_offset() const { return d_spot_; }
    double turn_radius() const {
        if (P.turn_radius_m > 0) return P.turn_radius_m;
        return P.wheelbase_m / std::tan(P.max_steer_deg * M_PI / 180.0);
    }

    ParkOutputs step(const ParkInputs& in)
    {
        ParkOutputs out;
        const double dt = (t_prev_ < 0) ? 0.0 : std::clamp(in.t - t_prev_, 0.0, 0.2);
        t_prev_ = in.t;
        odometry(dt, in);
        const double sF = clean(in.sonar_front), sB = clean(in.sonar_back), sR = clean(in.sonar_right);
        const bool sR_new = in.sonar_right_t > sR_t_;
        if (sR_new) sR_t_ = in.sonar_right_t;

        double steer = P.steer_center_cmd; int speed = 0;
        switch (st_) {
        // ── Buscar el lugar ──────────────────────────────────────────────
        case PState::SEARCH: {
            lane_keep(in, steer, speed, P.search_cmd);
            if (sR_new) scan_side(sR, out);
            break;
        }
        // ── Seguir hasta el punto de inicio (A) ──────────────────────────
        case PState::APPROACH: {
            lane_keep(in, steer, speed, P.approach_cmd);
            out.right_signal = true;
            const double v = std::fabs(last_speed_ * P.speed_mps_per_cmd);
            if (travel_ + v * P.stop_lag_s >= start_travel_) {
                plan();
                next(PState::REV_IN, in, out, "en el punto de inicio");
            }
            break;
        }
        // ── Pausa: detenido mientras el servo gira ───────────────────────
        case PState::PAUSE: {
            out.right_signal = true;
            steer = pause_steer_;
            speed = 0;
            if (in.t - t_enter_ > P.steer_settle_s) go(after_pause_, in, out, "");
            break;
        }
        // ── B: reversa a la derecha ──────────────────────────────────────
        case PState::REV_IN: {
            out.right_signal = true;
            steer = P.steer_right_cmd; speed = P.maneuver_cmd;
            if (th_ >= arc_ - yaw_lead() || sB < P.back_stop_m) {
                replan();       // con el radio de giro REAL medido en este arco
                next(PState::REV_STR, in, out, sB < P.back_stop_m ? "atras muy cerca" : "primer arco listo");
            }
            break;
        }
        // ── Tramo recto en reversa ───────────────────────────────────────
        case PState::REV_STR: {
            out.right_signal = true;
            steer = P.steer_center_cmd; speed = P.maneuver_cmd;
            const double done = imu_ok_ ? seg_start_ - travel_ + lag_dist() : seg_start_real_ - travel_real_ + coast_dist();
            if (done >= straight_ || sB < P.back_stop_m || sR < P.side_stop_m || straight_ <= 0.005)
                next(PState::REV_OUT, in, out, sB < P.back_stop_m ? "atras muy cerca" : sR < P.side_stop_m ? "pared muy cerca" : "tramo recto listo");
            break;
        }
        // ── C: reversa a la izquierda hasta quedar derecho ───────────────
        case PState::REV_OUT: {
            out.right_signal = true;
            // normalmente la trompa apunta a la izquierda (rumbo +) => reversa a la IZQUIERDA;
            // si quedó apuntando a la pared (rumbo -), reversa a la derecha
            steer = fix_sign_ > 0 ? P.steer_left_cmd : P.steer_right_cmd; speed = P.maneuver_cmd;
            // esquina trasera derecha: va (sonar_right_x + car_rear)·sen(rumbo) más metida que el sonar
            const bool wall = fix_sign_ > 0 && std::isfinite(sR) &&
                (sR < P.side_stop_m || sR * std::cos(th_) - (P.sonar_right_x_m + P.car_rear_m) * std::sin(th_) < P.side_stop_m);
            if (fix_sign_ * th_ <= yaw_lead() || sB < P.back_stop_m || wall) {
                if (std::fabs(th_) * 180.0 / M_PI <= P.straight_tol_deg || fixes_ >= P.max_fixes)
                    next(PState::CENTER, in, out, "derecho");
                else
                    next(PState::FWD_FIX, in, out, "atras muy cerca, enderezar hacia adelante");
            }
            break;
        }
        // ── D: adelante a la derecha para enderezarse ────────────────────
        case PState::FWD_FIX: {
            out.right_signal = true;
            steer = fix_sign_ > 0 ? P.steer_right_cmd : P.steer_left_cmd; speed = -P.maneuver_cmd;
            if (fix_sign_ * th_ <= yaw_lead() || sF < P.front_stop_m) {
                ++fixes_;
                if (std::fabs(th_) * 180.0 / M_PI <= P.straight_tol_deg || fixes_ >= P.max_fixes)
                    next(PState::CENTER, in, out, "derecho");
                else
                    next(PState::REV_OUT, in, out, "enfrente muy cerca, otra vez en reversa");
            }
            break;
        }
        // ── E: centrarse entre el carro de enfrente y el de atrás ────────
        case PState::CENTER: {
            out.blinkers = true;
            steer = P.steer_center_cmd;
            // 1) detenido: ¿quedó lejos de la pared? (ultrasónico derecho)
            if (!depth_checked_ && depth_possible()) {
                speed = 0;
                if (in.t - t_enter_ > 0.25 && sR_new && std::isfinite(sR) && sR < 0.6) { side_sum_ += sR; ++side_n_; }
                if (in.t - t_enter_ > 0.8) {
                    depth_checked_ = true;
                    if (side_n_ >= 2) {
                        const double excess = side_sum_ / side_n_ - side_target();   // + = quedó lejos de la pared
                        last_excess_ = excess;
                        if (excess > P.depth_tol_m) { start_depth(excess, sF, sB, in, out); break; }
                    }
                    t_enter_ = in.t;        // ahora sí, a centrarse
                }
                break;
            }
            // 2) centrarse entre el carro de enfrente y el de atrás
            const bool both = std::isfinite(sF) && std::isfinite(sB) && sF < 1.0 && sB < 1.0;
            const double err = both ? 0.5 * (sF - sB) : 0.0;          // + = hay más lugar adelante
            if (!both || std::fabs(err) < P.center_tol_m || in.t - t_enter_ > P.center_timeout_s) {
                speed = 0;
                if (++center_ok_ >= 3) {
                    char b[120];
                    if (last_excess_ > -1.0)
                        std::snprintf(b, sizeof b, "%s, a %.1f cm de lo ideal de la pared", both ? "centrado" : "estacionado", last_excess_ * 100);
                    else
                        std::snprintf(b, sizeof b, "%s", both ? "centrado" : "estacionado (sin referencia adelante/atras)");
                    go(PState::PARKED, in, out, b);
                }
            } else {
                center_ok_ = 0;
                speed = (err > 0) ? -P.creep_cmd : P.creep_cmd;
                if ((speed < 0 && sF < P.front_stop_m) || (speed > 0 && sB < P.back_stop_m)) speed = 0;
            }
            break;
        }
        // ── Profundidad 1: reversa derecho hasta el carro de atrás (para tener todo el espacio) ──
        case PState::DEPTH_BACK: {
            out.blinkers = true;
            steer = P.steer_center_cmd; speed = P.creep_cmd;
            if (room_back(sB) <= lag_dist()) {
                plan_fwd_s();
                char b[120];
                std::snprintf(b, sizeof b, "S adelante de %.1f° (espacio adelante %.2f m)", depth_fwd_arc_ * 180.0 / M_PI, room_front(sF));
                next(PState::DEPTH_FWD, in, out, b);
            }
            break;
        }
        // ── Profundidad 2: S hacia ADELANTE: derecha (DEPTH_FWD) y luego izquierda (DEPTH_FWD2) ──
        //    (si no cabe la S, solo avanza derecho y la S en reversa hace todo)
        case PState::DEPTH_FWD:
        case PState::DEPTH_FWD2: {
            out.blinkers = true;
            const bool s_curve = depth_fwd_arc_ > 0;
            const bool second = (st_ == PState::DEPTH_FWD2);
            steer = !s_curve ? P.steer_center_cmd : second ? P.steer_left_cmd : P.steer_right_cmd;
            speed = -P.creep_cmd;
            const bool blocked = room_front(sF) <= lag_dist() || sR < P.side_stop_m ||
                                 (!s_curve && travel_ - seg_start_ >= depth_fwd_len_ - lag_dist());
            if (s_curve && !second && !blocked) {
                if (th_seg_ - th_ >= depth_fwd_arc_ - yaw_lead()) next(PState::DEPTH_FWD2, in, out, "arco adelante listo");
                break;
            }
            if (s_curve && second && !blocked && th_ < -yaw_lead()) break;      // aún no queda derecho
            // S en reversa con lo que falta y el espacio que queda atrás
            const double R = turn_radius(), back = room_back(sB);
            double a = std::acos(std::clamp(1.0 - depth_excess_ / (2.0 * R), -1.0, 1.0));
            a = std::min(a, std::asin(std::clamp((back - lag_dist()) / (2.0 * R), 0.0, 1.0)));
            depth_arc_ = std::max(0.0, a);           // rumbo (absoluto) al que llega la reversa a la derecha
            char b[140];
            if (depth_arc_ < 2.0 * M_PI / 180.0 && std::fabs(th_) * 180.0 / M_PI <= P.straight_tol_deg) {
                std::snprintf(b, sizeof b, "sin S en reversa (espacio atras %.2f m)", back);
                next(PState::CENTER, in, out, b);
            } else {
                std::snprintf(b, sizeof b, "S en reversa de %.1f° (espacio atras %.2f m)", depth_arc_ * 180.0 / M_PI, back);
                next(PState::DEPTH_IN, in, out, b);
            }
            break;
        }
        // ── Profundidad 3: reversa a la derecha; luego REV_OUT (izquierda) hasta quedar derecho ──
        case PState::DEPTH_IN: {
            out.blinkers = true;
            steer = P.steer_right_cmd; speed = P.creep_cmd;
            const bool wall = th_ > 0 && sR < P.side_stop_m;     // con la trompa a la pared, la reversa lo aleja
            if (th_ >= depth_arc_ - yaw_lead() || room_back(sB) <= 0.0 || wall)
                next(PState::REV_OUT, in, out, room_back(sB) <= 0.0 ? "atras muy cerca" : wall ? "pared muy cerca" : "arco listo");
            break;
        }
        case PState::PARKED: {
            out.blinkers = true;
            steer = P.steer_center_cmd; speed = 0;
            break;
        }
        }
        out.steer_cmd = (int)std::lround(std::clamp(steer, 0.0, 180.0));
        out.speed_cmd = speed;
        out.state = pstate_name(st_);
        last_steer_ = out.steer_cmd; last_speed_ = speed;
        return out;
    }

private:
    ParkParams P;
    PState st_ = PState::SEARCH, after_pause_ = PState::SEARCH;
    double t_prev_ = -1, t_enter_ = 0, sR_t_ = -1;
    double travel_ = 0, th_ = 0, delta_ = 0;
    int    last_steer_ = 90, last_speed_ = 0;
    double pause_steer_ = 90;
    // búsqueda
    bool   seen_car_ = false, in_gap_ = false;
    int    car_frames_ = 0, free_frames_ = 0;
    double gap_start_ = 0, gap_sum_ = 0; int gap_n_ = 0;
    double car_recent_[6] = {0}; int car_n_ = 0;          // últimas lecturas "hay carro" (la más reciente manda)
    double spot_len_ = 0, d_spot_ = 0.34, start_travel_ = 0, gap_end_ = 0;
    // maniobra
    double arc_ = 0, straight_ = 0, seg_start_ = 0;
    int    fixes_ = 0, center_ok_ = 0;
    int    fix_sign_ = 1;               // REV_OUT/FWD_FIX: +1 = la trompa apunta a la izquierda
    double th_seg_ = 0, v_lag_ = 0, travel_real_ = 0, seg_start_real_ = 0;
    bool   imu_ok_ = false;
    // profundidad (lazo cerrado con el sonar derecho)
    double wall_read_ = -1;              // lectura del sonar derecho hacia la pared, al pasar junto al hueco
    double side_sum_ = 0; int side_n_ = 0;
    int    depth_fixes_ = 0;
    double depth_excess_ = 0, depth_arc_ = 0, depth_fwd_arc_ = 0, depth_fwd_len_ = 0;
    double depth_ref_ = 0, depth_ff0_ = 0, depth_fb0_ = 0, side_gap_ = 0, last_excess_ = -9;
    bool   depth_checked_ = false;
    // espacio libre adelante / atrás (lo que se puede mover sin pasar de *_stop_m):
    // lo medido al empezar la corrección menos lo recorrido, y el sonar si lo ve más cerca
    double room_front(double sF) const {
        double f = depth_ff0_ - (travel_ - depth_ref_);
        if (std::isfinite(sF) && sF < 1.0) f = std::min(f, sF);
        return f - P.front_stop_m - 0.02;
    }
    double room_back(double sB) const {
        double r = depth_fb0_ + (travel_ - depth_ref_);
        if (std::isfinite(sB) && sB < 1.0) r = std::min(r, sB);
        return r - P.back_stop_m - 0.02;
    }
    void start_depth(double excess, double sF, double sB, const ParkInputs& in, ParkOutputs& out) {
        ++depth_fixes_; fixes_ = 0;
        depth_excess_ = excess; side_gap_ = side_sum_ / side_n_;
        // libre adelante y atrás: sonares, revisados con el largo del hueco medido
        // (por si un sonar no "ve" al carro y lee algo más lejos)
        const double free_tot = std::max(0.0, spot_len_ - (P.car_front_m + P.car_rear_m));
        double ff = (std::isfinite(sF) && sF < 1.0) ? sF : -1, fb = (std::isfinite(sB) && sB < 1.0) ? sB : -1;
        if (ff < 0 && fb < 0) ff = fb = 0.5 * free_tot;
        else if (ff < 0) ff = std::max(0.0, free_tot - fb);
        else if (fb < 0) fb = std::max(0.0, free_tot - ff);
        ff = std::min(ff, free_tot - fb + 0.15);
        fb = std::min(fb, free_tot - ff + 0.15);
        depth_ff0_ = ff; depth_fb0_ = fb; depth_ref_ = travel_;
        char b[120];
        std::snprintf(b, sizeof b, "quedo %.1f cm lejos de la pared: corregir (%d)", excess * 100, depth_fixes_);
        if (room_back(sB) > 0.04) next(PState::DEPTH_BACK, in, out, b);
        else { plan_fwd_s(); next(PState::DEPTH_FWD, in, out, b); }
    }
    // S hacia adelante: la mitad de lo que falta, lo que quepa adelante y sin que
    // la esquina delantera derecha (que se acerca a la pared con el carro inclinado) la toque
    void plan_fwd_s() {
        const double R = turn_radius(), room = room_front(INFINITY);
        double af = std::acos(std::clamp(1.0 - 0.5 * depth_excess_ / (2.0 * R), -1.0, 1.0));
        af = std::min(af, std::asin(std::clamp((room - lag_dist()) / (2.0 * R), 0.0, 1.0)));
        // al final del primer arco la esquina se metió car_front·sen(a) + R(1-cos a): dejar 2 cm
        while (af > 0 && P.car_front_m * std::sin(af) + R * (1.0 - std::cos(af)) > side_gap_ - 0.02) af -= 0.25 * M_PI / 180.0;
        depth_fwd_arc_ = (af >= 2.0 * M_PI / 180.0) ? af : 0.0;
        if (depth_fwd_arc_ > 0) depth_excess_ = std::max(0.0, depth_excess_ - 2.0 * R * (1.0 - std::cos(af)));
        depth_fwd_len_ = std::max(0.0, std::min(room, 0.6));     // sin S: avanzar derecho
    }
    bool depth_possible() const { return wall_read_ > 0 && depth_fixes_ < P.max_depth_fixes; }
    // lo que debería leer el sonar derecho estacionado = pared - lugar
    double d_final_ = 0.34;              // a dónde debe quedar al final (alineado con los estacionados)
    double side_target() const { return wall_read_ - d_final_ - P.depth_bias_m; }

    static double clean(double r) { return (std::isfinite(r) && r > 0.0) ? r : INFINITY; }

    void odometry(double dt, const ParkInputs& in) {
        const double v = -last_speed_ * P.speed_mps_per_cmd;               // + = adelante
        // ángulo real de las ruedas (el servo tarda)
        const double d_cmd = (P.steer_deg_per_cmd * (last_steer_ - P.steer_center_cmd) + P.steer_offset_deg) * M_PI / 180.0;
        // tope de las ruedas: el del radio medido (turn_radius_m) si lo hay, para que cuadre con el plan
        const double dmax = (P.turn_radius_m > 0) ? std::atan(P.wheelbase_m / P.turn_radius_m) : P.max_steer_deg * M_PI / 180.0;
        delta_ += (std::clamp(d_cmd, -dmax, dmax) - delta_) * std::min(1.0, dt / std::max(0.01, P.steer_lag_s));
        travel_ += v * dt;
        v_lag_ += (v - v_lag_) * std::min(1.0, dt / std::max(0.01, P.speed_lag_s));   // velocidad real (tarda en acelerar)
        travel_real_ += v_lag_ * dt;
        imu_ok_ = in.has_yaw_rate;
        th_ += (in.has_yaw_rate ? in.yaw_rate : v_lag_ / P.wheelbase_m * std::tan(delta_)) * dt;
    }
    double lag_dist() const { return std::fabs(last_speed_ * P.speed_mps_per_cmd) * P.stop_lag_s; }
    // lo que todavía avanza el carro (con el modelo de retardo) si se frena ahora
    double coast_dist() const { return P.speed_lag_s > 0.01 ? std::fabs(v_lag_) * P.speed_lag_s : lag_dist(); }
    double yaw_lead() const {      // lo que gira el carro mientras frena
        return lag_dist() / turn_radius();
    }

    void lane_keep(const ParkInputs& in, double& steer, int& speed, int cruise) {
        if (!in.has_dist) { steer = P.steer_center_cmd; speed = -cruise; return; }
        const double e = -(double)in.dist_px, ae = std::fabs(e);
        double kp = P.lk_kp[4];
        for (int i = 0; i < 4; ++i) if (ae < P.lk_thr[i]) { kp = P.lk_kp[i]; break; }
        steer = std::clamp(P.steer_center_cmd + kp * e, P.lk_min_cmd, P.lk_max_cmd);
        speed = -cruise;
    }

    // carro – hueco – carro con el ultrasónico derecho
    void scan_side(double sR, ParkOutputs& out) {
        const double here = travel_ + P.sonar_right_x_m;      // posición del sonar (en distancia recorrida)
        const bool car = sR < P.side_car_max_m, freep = sR > P.side_free_min_m;
        car_frames_ = car ? car_frames_ + 1 : 0;
        free_frames_ = freep ? free_frames_ + 1 : 0;
        if (car && car_frames_ >= 2) {
            if (!in_gap_) { car_recent_[car_n_ % 6] = sR; ++car_n_; }
            if (in_gap_) {
                // fin del hueco: carro de enfrente
                in_gap_ = false;
                gap_end_ = here;
                spot_len_ = gap_end_ - gap_start_;
                char b[160];
                if (spot_len_ >= P.min_spot_m && spot_len_ <= P.max_spot_m) {
                    // distancia a la fila: lecturas del carro de atrás junto al hueco (el carro ya va derecho)
                    double side = P.side_car_max_m * 0.5;
                    if (car_n_ > 0) {
                        const int n = std::min(car_n_, 6); double sum = 0;
                        for (int k = 0; k < n; ++k) sum += car_recent_[k];
                        side = sum / n;
                    }
                    d_spot_ = (P.target_offset_m > 0) ? P.target_offset_m
                                                      : P.sonar_side_y_m + side + P.car_half_width_m;
                    d_final_ = d_spot_;
                    // sin pasarse de la pared / banqueta del fondo (si el sonar la vio)
                    if (gap_n_ >= 2) {
                        wall_read_ = gap_sum_ / gap_n_;
                        const double wall = P.sonar_side_y_m + wall_read_;
                        d_spot_  = std::min(d_spot_,  wall - P.car_half_width_m - P.wall_margin_m);
                        d_final_ = std::min(d_final_, wall - P.car_half_width_m - P.final_wall_margin_m);
                    }
                    // punto de inicio: con otro radio de giro la maniobra es más larga/corta
                    start_travel_ = gap_end_ + P.start_past_m - (path_len(P.ref_radius_m) - path_len(turn_radius()));
                    std::snprintf(b, sizeof b, "lugar de %.2f m (carros a %.2f m a la derecha)", spot_len_, d_spot_);
                    st_ = PState::APPROACH; out.event = std::string("SEARCH -> APPROACH (") + b + ")";
                } else {
                    std::snprintf(b, sizeof b, "hueco de %.2f m: no sirve", spot_len_);
                    out.event = b;
                }
                car_recent_[0] = sR; car_n_ = 1;
            }
            seen_car_ = true;
        } else if (freep && free_frames_ >= 2 && seen_car_ && !in_gap_) {
            in_gap_ = true;
            gap_start_ = here; gap_sum_ = 0; gap_n_ = 0;
        }
        if (in_gap_ && freep && std::isfinite(sR) && sR < 1.0) { gap_sum_ += sR; ++gap_n_; }
    }

    // Largo (a lo largo del carril) de la trayectoria con radio R
    double path_len(double R) const {
        double a = P.arc_deg * M_PI / 180.0, s = 0.0;
        const double lat2 = 2.0 * R * (1.0 - std::cos(a));
        if (lat2 >= d_spot_) a = std::acos(std::max(-1.0, 1.0 - d_spot_ / (2.0 * R)));
        else s = (d_spot_ - lat2) / std::sin(a);
        return 2.0 * R * std::sin(a) + s * std::cos(a);
    }
    // Trayectoria: arco a la derecha (arc_), tramo recto, arco a la izquierda.
    // Desplazamiento lateral = 2R(1 - cos a) + s·sen a = d_spot
    void plan() {
        const double R = turn_radius();
        double a = P.arc_deg * M_PI / 180.0;
        const double lat2 = 2.0 * R * (1.0 - std::cos(a));
        if (lat2 >= d_spot_) {                  // basta con dos arcos más chicos
            a = std::acos(std::max(-1.0, 1.0 - d_spot_ / (2.0 * R)));
            straight_ = 0.0;
        } else {
            straight_ = (d_spot_ - lat2) / std::sin(a);
        }
        arc_ = a;
    }

    // Al terminar el primer arco: radio de giro medido = distancia / ángulo
    // (con IMU es muy confiable). Se recalcula el tramo recto para llegar a la
    // fila de los estacionados aunque el carro gire más o menos de lo supuesto.
    double r_meas_ = 0;
    void replan() {
        if (!imu_ok_) return;                              // sin IMU el ángulo sale del mismo modelo: no aporta
        const double dist = seg_start_ - travel_;          // lo recorrido en reversa en el arco
        const double a = th_;
        if (a < 10.0 * M_PI / 180.0 || dist < 0.05) return;
        const double R0 = turn_radius();
        r_meas_ = std::clamp(dist / a, 0.5 * R0, 1.8 * R0);
        straight_ = std::max(0.0, (d_spot_ - 2.0 * r_meas_ * (1.0 - std::cos(a))) / std::sin(a));
    }
    void next(PState s, const ParkInputs& in, ParkOutputs& out, const char* why) {
        // siempre se detiene y espera a que gire el servo antes del siguiente paso
        after_pause_ = s;
        if (s == PState::REV_OUT || s == PState::FWD_FIX) fix_sign_ = (th_ >= 0) ? 1 : -1;
        pause_steer_ = (s == PState::REV_IN || s == PState::DEPTH_IN) ? P.steer_right_cmd
                     : (s == PState::FWD_FIX) ? (fix_sign_ > 0 ? P.steer_right_cmd : P.steer_left_cmd)
                     : (s == PState::REV_OUT) ? (fix_sign_ > 0 ? P.steer_left_cmd : P.steer_right_cmd)
                     : (s == PState::DEPTH_FWD) ? (depth_fwd_arc_ > 0 ? P.steer_right_cmd : P.steer_center_cmd)
                     : (s == PState::DEPTH_FWD2) ? P.steer_left_cmd : P.steer_center_cmd;
        char b[320];
        std::snprintf(b, sizeof b, "%s -> %s (%s) | rumbo %.1f°, recorrido %.2f m", pstate_name(st_), pstate_name(s),
                      why, th_ * 180.0 / M_PI, travel_);
        out.event = b;
        if (s == PState::REV_STR && r_meas_ > 0) {
            char c[120];
            std::snprintf(c, sizeof c, " | radio medido %.2f m -> recto %.2f m", r_meas_, straight_);
            out.event += c;
        }
        if (s == PState::REV_IN) {
            char c[160];
            std::snprintf(c, sizeof c, " | plan: arco %.1f°, recto %.2f m, radio %.2f m", arc_ * 180.0 / M_PI, straight_, turn_radius());
            out.event += c;
            th_ = 0.0;                          // rumbo relativo al carril, al empezar
        }
        st_ = PState::PAUSE; t_enter_ = in.t;
    }
    void go(PState s, const ParkInputs& in, ParkOutputs& out, const char* why) {
        if (*why) {
            char b[256];
            std::snprintf(b, sizeof b, "%s -> %s (%s)", pstate_name(st_), pstate_name(s), why);
            out.event = b;
        }
        st_ = s; t_enter_ = in.t; seg_start_ = travel_; seg_start_real_ = travel_real_; th_seg_ = th_; center_ok_ = 0;
        side_sum_ = 0; side_n_ = 0;
        if (s == PState::CENTER) depth_checked_ = false;
    }
};

}  // namespace avim
