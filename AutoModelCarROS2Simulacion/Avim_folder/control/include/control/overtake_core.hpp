// ─────────────────────────────────────────────────────────────────────────────
//  overtake_core.hpp  –  Lógica de rebase del AVIM, SIN dependencias de ROS
//
//  La usan:
//    • Master_overtake.cpp  (nodo ROS 2, simulación y carro real)
//    • test_overtake.cpp    (banco de pruebas cinemático, fuera de ROS)
//
//  Idea central: ninguna fase termina por reloj. Cada una termina por lo que
//  miden los sensores; los tiempos solo existen como límites de seguridad.
//
//    LANE ──obstáculo EN el carril, cerca, carril izq. libre──▶ CHANGE_LEFT
//    CHANGE_LEFT ──centrado en el carril izquierdo──▶ PASS
//    PASS ──obstáculo rebasado (sonar derecho / odometría) y carril der. libre──▶ CHANGE_RIGHT
//    CHANGE_RIGHT ──centrado en el carril derecho──▶ LANE
//    (FOLLOW: obstáculo enfrente pero no se puede rebasar todavía)
//    (BACK_OFF: quedó demasiado cerca para girar; retrocede un poco)
//
//  Marco de coordenadas: origen en el EJE TRASERO, x hacia adelante,
//  y hacia la izquierda, en metros. Ángulo de dirección positivo = izquierda.
// ─────────────────────────────────────────────────────────────────────────────
#pragma once

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <limits>
#include <string>
#include <vector>

namespace avim {

struct Vec2 { double x = 0, y = 0; };

// Obstáculo ya convertido al marco del eje trasero
struct Obstacle { double x = 0, y = 0; };

// ── Parámetros (valores por defecto = simulación Gazebo) ────────────────────
struct Params {
    // Geometría del carro (modelo AutoNOMOS_mini)
    double wheelbase_m      = 0.25;
    double car_front_m      = 0.34;   // eje trasero -> defensa delantera
    double car_rear_m       = 0.09;   // eje trasero -> defensa trasera
    double car_half_width_m = 0.10;
    double lane_cam_x_m     = 0.095;  // cámara de carril, delante del eje trasero
    double depth_cam_x_m    = 0.095;  // cámara de profundidad
    double depth_hfov_deg   = 73.8;   // campo de visión horizontal de la cámara de profundidad
    double depth_range_m    = 3.0;    // hasta dónde se confía en que vería un obstáculo
    double sonar_right_x_m  = 0.125;  // ultrasónico derecho, delante del eje trasero

    // Pista
    double lane_width_m      = 0.40;
    double obstacle_length_m = 0.43;

    // Dirección: comando (0-180) <-> ángulo de rueda
    //   ángulo_deg = deg_per_cmd·(cmd - center) + offset_deg
    double steer_center_cmd  = 90.0;
    double steer_deg_per_cmd = 0.2526;   // plugin de Gazebo; en el carro real MEDIRLO
    double steer_offset_deg  = 0.573;
    double steer_cmd_min     = 0.0;
    double steer_cmd_max     = 180.0;
    double max_steer_deg     = 22.0;
    double maneuver_max_steer_deg = 22.0;  // ángulo máx. que pide el pure pursuit (22 = sin límite)
    double steer_lag_s       = 0.15;     // retraso del servo (solo para la odometría sin IMU)

    // Velocidad: comando (negativo = adelante) -> m/s
    double speed_mps_per_cmd = 0.00121;  // plugin de Gazebo; en el carro real MEDIRLO
    int    cruise_cmd        = 500;      // = mid_speed_ del Master original
    double kp_speed          = 2.4462;   // frenado por error de carril (original)
    int    approach_cmd      = 300;      // al acercarse a un obstáculo
    int    change_cmd        = 260;      // durante cambios de carril
    int    pass_cmd          = 300;      // en el carril izquierdo
    int    reverse_cmd       = 150;      // BACK_OFF (positivo = reversa)

    // Seguimiento de carril normal (tu on_lane, con /distance_center_line)
    double lk_thr[4] = { 8.0, 15.0, 25.0, 35.0 };
    double lk_kp[5]  = { 0.10, 0.40, 0.55, 0.85, 1.00 };
    double lk_min_cmd = 45.0, lk_max_cmd = 135.0;
    int    lane_keep_mode = 0;   // 0 = tu on_lane (/distance_center_line), 1 = pure pursuit con /lane/*_line

    // Pure pursuit (cambios de carril y carril izquierdo)
    double lookahead_m = 0.55;

    // Decisiones (distancias medidas desde la defensa delantera)
    double detect_range_m     = 2.2;   // ignora obstáculos más lejos
    double slow_gap_m         = 1.4;   // empieza a bajar velocidad
    double trigger_gap_m      = 0.85;  // inicia el rebase
    double min_change_gap_m   = 0.45;  // más cerca que esto nunca intenta girar
    double feasible_margin_m  = 0.04;  // holgura lateral con el obstáculo al rebasar
    double feasible_lag_m     = 0.20;  // distancia "perdida" mientras gira el volante
    double gap_uncertainty_m  = 0.10;  // la distancia medida puede ser mayor que la real
    double backoff_target_m   = 0.65;  // BACK_OFF retrocede hasta aquí
    double keep_gap_m         = 0.30;  // FOLLOW se detiene a esta distancia
    double emergency_gap_m    = 0.10;  // freno total
    int    confirm_frames     = 2;     // detecciones seguidas para confiar
    double mem_match_m        = 0.30;  // misma detección si está a menos de esto
    int    mem_min_hits       = 2;     // detecciones para considerarlo real
    double mem_unseen_s       = 0.6;   // dentro del campo de visión y sin verse => se olvida
    double mem_max_s          = 8.0;   // se olvida tras este tiempo sin verse
    double in_lane_frac       = 0.5;   // |desv. lateral| < frac·ancho => en ese carril
    double left_lane_check_m  = 1.2;   // carril izq. debe estar libre hasta gap + esto
    double abort_return_gap_m = 1.3;   // obstáculo en el carril derecho al regresar => abortar
    double return_check_m     = 2.4;   // carril der. libre tantos m antes de regresar (> return_length + margen)
    double return_max_kappa   = 0.35;  // regresar solo si la curvatura del carril es menor (1/m)
    double return_visible_from_m = 0.9; // tramo del carril derecho que debe estar a la vista
    double return_visible_m   = 2.2;   //   (lo más cercano queda fuera del campo de visión; eso lo cubre el sonar)
    double objects_timeout_s  = 0.30;  // sin /objects_points => no hay obstáculos
    double lane_timeout_s     = 1.0;   // usar el último carril conocido este tiempo

    // Rebase con ultrasónico derecho
    double side_detect_m      = 0.45;  // sonar derecho "ve" el obstáculo
    double side_clear_extra_m = 0.05;  // avanzar esto tras dejar de verlo
    double odom_pass_margin_m = 0.12;  // margen si se decide por odometría
    double side_sonar_min_free_m = 0.30; // sonar lateral libre para cambiarse

    // Cambio de carril: trayectoria suave (coseno) de un carril al otro a lo
    // largo de esta distancia recorrida. Menos = más brusco (y más riesgo de
    // perder las líneas); más = necesita más espacio antes del obstáculo.
    double change_length_m    = 1.20;  // ida (hay un obstáculo enfrente: no puede ser muy largo)
    double return_length_m    = 1.60;  // regreso (sin prisa: más suave, no pierde las líneas)
    double change_done_lat_m  = 0.06;
    double anchor_tol_m       = 0.12;  // línea vista vs. predicha: más lejos => se descarta
    bool   use_line_types     = true;  // usar continua/punteada de lane_detection
    double type_tol_m         = 0.45;
    int    type_confirm_frames = 4;    // cuadros seguidos para que el tipo corrija a la odometría
    int    wrong_lane_frames  = 8;     // cuadros seguidos en el carril izq. (por tipo de línea) para regresar  // con tipo conocido se acepta aunque la odometría esté así de mal
    double fit_max_x_m        = 1.30;  // usar las líneas solo hasta esta distancia
    double anchor_timeout_s   = 1.5;   // sin líneas confiables este tiempo => soltar el ancla (fuera de maniobra)
    double blind_max_s        = 6.0;   // en maniobra: confiar en el modelo movido con odometría hasta este tiempo
    double anchor_tol_growth  = 0.03;  // m de tolerancia extra por segundo sin ver líneas (máx. 0.17)
    bool   auto_speed_scale   = true;  // calibrar la escala de velocidad con el sonar delantero
    bool   auto_steer_gain    = true;  // sin IMU: calibrar la ganancia de dirección con las líneas
    double settle_timeout_s   = 3.0;   // máx. tiempo esperando que el detector vea el carril de casa
    double change_done_head   = 0.15;  // pendiente (rad aprox) del carril
    int    change_done_frames = 3;
    double change_timeout_s   = 6.0;
    double pass_timeout_s     = 8.0;
    double backoff_timeout_s  = 2.5;
    double front_sonar_stop_m = 0.12;
    double blocked_timeout_s  = 1.0;   // detenido por seguridad a media maniobra => BACK_OFF

    // Señal de alto. Con /stop_sign_distance el carro se detiene en el cruce
    // (la señal está a un lado, 5 cm antes de la cebra): con la distancia a
    // la señal se fija el punto de alto en la odometría y se sigue frenando
    // aunque la señal ya haya salido del campo de visión.
    double stop_time_s        = 2.0;   // tiempo detenido
    double stop_cooldown_s    = 5.0;   // solo para el modo viejo (/stop_sign_detected sin distancia)
    double stop_cam_x_m       = 0.095; // cámara del detector de señales, delante del eje trasero
    double stop_before_m      = 0.06;  // la defensa queda esto ANTES de la señal (+ = antes; la cebra empieza 5 cm después)
    double stop_slow_m        = 1.00;  // empezar a frenar a esta distancia del punto de alto
    int    stop_approach_cmd  = 250;   // velocidad al acercarse al alto
    int    stop_final_cmd     = 170;   // últimos 25 cm
    double stop_same_sign_m   = 0.60;  // tras un alto, ignorar la misma señal (puntos más cerca que esto)
    double stop_new_sign_m    = 0.50;  // una medición que difiere más que esto es OTRA señal
    double stop_overshoot_m   = 0.50;  // se pasó del punto (p. ej. rebasando) => olvidarlo
    double stop_lag_s         = 0.15;  // el carro tarda en frenar: ordenar el alto esto antes
    // La señal (poste + placa) también la ve object_detection como "obstáculo":
    // lo que esté a menos de esto de una señal detectada NO es obstáculo
    double sign_ignore_m      = 0.30;
    // Tras el alto el carro cruza la intersección: la cebra y las líneas del
    // otro camino confunden al detector de carril. Mientras recorre esto, el
    // modelo del carril no se actualiza (se mueve solo con odometría)
    double cross_length_m     = 1.50;
    // Obstáculo justo enfrente a menos de esto (m): debe verlo también el
    // ultrasónico de enfrente; si no lo ve, es un poste delgado (0 = no revisar)
    double sonar_confirm_m    = 1.60;
    double stop_freeze_m      = 0.40;  // congelar el modelo del carril a esta distancia del alto
};

// ── Entradas de un ciclo ────────────────────────────────────────────────────
struct Inputs {
    double t = 0;                       // tiempo (s), de simulación si use_sim_time
    bool   has_dist = false;            // hay /distance_center_line reciente
    int    dist_px  = 0;

    bool   lane_new = false;            // llegó geometría nueva este ciclo
    std::vector<Vec2> left_line;        // marco del EJE TRASERO, metros
    std::vector<Vec2> right_line;

    bool   objects_fresh = false;       // /objects_points reciente
    bool   objects_new   = false;       // llegó un mensaje nuevo desde el ciclo anterior
    std::vector<Obstacle> obstacles;    // marco del EJE TRASERO, metros

    // Ultrasónicos (m). Sin eco => infinito
    double sonar_front = INFINITY, sonar_back = INFINITY;
    double sonar_left  = INFINITY, sonar_right = INFINITY;

    bool   stop_sign = false;           // modo viejo: Bool sin distancia => alto inmediato
    // Distancia hacia adelante (m) de la cámara a la señal de alto, medida
    // NUEVA en este ciclo (NAN si no llegó ninguna). De /stop_sign_distance.
    double stop_dist = NAN;
    double stop_lat  = 0.0;             // posición lateral de la señal (m, + = izquierda), si se sabe

    // Giroscopio (IMU), opcional: velocidad de giro en rad/s (+ = izquierda).
    // Si existe, la odometría usa el giro MEDIDO en vez de suponerlo a partir
    // del comando de dirección (mucho más confiable en Gazebo y en el carro).
    bool   has_yaw_rate = false;
    double yaw_rate = 0.0;

    // Tipo de cada línea (de lane_detection): 0 = no se sabe, 1 = continua
    // (borde de la pista), 2 = punteada (centro). Dice en qué carril está el
    // carro aunque el detector confunda izquierda/derecha.
    int left_type = 0, right_type = 0;
};

struct Outputs {
    int  steer_cmd = 90;
    int  speed_cmd = 0;
    bool left_signal = false, right_signal = false, blinkers = false;
    const char* state = "LANE";
    std::string event;                  // texto para el log cuando cambia algo
};

enum class State { LANE, FOLLOW, BACK_OFF, CHANGE_LEFT, PASS, CHANGE_RIGHT, STOP };

inline const char* state_name(State s) {
    switch (s) {
        case State::LANE:         return "LANE";
        case State::FOLLOW:       return "FOLLOW";
        case State::BACK_OFF:     return "BACK_OFF";
        case State::CHANGE_LEFT:  return "CHANGE_LEFT";
        case State::PASS:         return "PASS";
        case State::CHANGE_RIGHT: return "CHANGE_RIGHT";
        case State::STOP:         return "STOP";
    }
    return "?";
}

// ─────────────────────────────────────────────────────────────────────────────
class OvertakeCore {
public:
    explicit OvertakeCore(const Params& p = Params()) : P(p) {}
    const Params& params() const { return P; }
    State state() const { return st_; }

    Outputs step(const Inputs& in)
    {
        Outputs out;
        const double dt = (t_prev_ < 0) ? 0.0 : std::clamp(in.t - t_prev_, 0.0, 0.2);
        t_prev_ = in.t;
        integrate_odometry(dt, in);

        // Detenido en el alto: el carro no se mueve, pero el detector de carril
        // ve la cebra y el modelo del carril se iría a cualquier lado. Se congela.
        // Lo mismo al cruzar la intersección: el modelo (con el que se decide
        // si un obstáculo está en el carril) se queda como antes del alto,
        // movido con odometría. La dirección sigue con /distance_center_line.
        if (crossing_ && odo_.travel > cross_end_travel_) crossing_ = false;
        // (y en los últimos centímetros antes del alto: la cámara ya ve la cebra)
        const bool near_stop = stop_pending_ && stop_target_ - odo_.travel < P.stop_freeze_m;
        const bool freeze_lane = st_ == State::STOP || crossing_ || near_stop;
        if (in.lane_new && !freeze_lane) update_lane_model(in);
        if (freeze_lane && lane_valid_) lane_t_ = in.t;
        if (crossing_) wrong_lane_frames_ = 0;
        update_stop_target(in);
        // metros que faltan para el alto, descontando lo que el carro avanza mientras frena
        const double v_now = std::max(0.0, -last_speed_cmd_ * P.speed_mps_per_cmd * speed_scale());
        const double stop_rem = stop_pending_ ? stop_target_ - odo_.travel - v_now * P.stop_lag_s : INFINITY;
        // Durante una maniobra el detector de carril casi nunca ve las líneas
        // (el carro va inclinado): se sigue con el modelo movido por odometría.
        const bool lane_ok = lane_valid_ && (in.t - lane_t_) < (maneuvering_now() ? P.blind_max_s : P.lane_timeout_s);

        // Obstáculos: memoria local (los obstáculos son fijos; se recuerdan con
        // odometría aunque salgan del campo de visión, p. ej. en curvas o al
        // retroceder). Luego se clasifican por carril (0 = actual, +1 = izq.).
        update_obstacle_memory(in);
        check_with_sonar(in);
        const std::vector<Obstacle> obs = remembered_obstacles();
        // carril actual, o lo que esté justo enfrente de la defensa
        double gap_cur  = nearest_gap(obs, lane_ok, lane_rel(0));
        if (st_ != State::PASS && st_ != State::CHANGE_LEFT) gap_cur = std::min(gap_cur, nearest_gap_corridor(obs));
        double gap_left = nearest_gap(obs, lane_ok, lane_rel(+1));
        double gap_right= nearest_gap(obs, lane_ok, lane_rel(-1));
        confirm_cur_ = (gap_cur < P.detect_range_m) ? confirm_cur_ + 1 : 0;
        const bool obstacle_ahead = confirm_cur_ >= P.confirm_frames;

        const double sR = clean(in.sonar_right), sL = clean(in.sonar_left);
        const double sF = clean(in.sonar_front), sB = clean(in.sonar_back);
        calibrate_speed(in, sF);

        int speed = 0;
        double steer = P.steer_center_cmd;

        switch (st_) {
        // ── Carril normal ────────────────────────────────────────────────
        case State::LANE: {
            if (in.stop_sign && in.t - stop_end_t_ > P.stop_cooldown_s) {
                enter_stop(in, out, "senal de alto (sin distancia)");
                break;
            }
            if (stop_rem <= 0.0) { enter_stop(in, out, "senal de alto: llegue al cruce"); break; }
            // Quedó en el carril izquierdo (un regreso que no terminó bien):
            // regresar al derecho
            if (!settle_ && !crossing_ && wrong_lane_frames_ >= P.wrong_lane_frames && lane_ok) {
                wrong_lane_frames_ = 0;
                const auto c = car_polyline();
                const auto home = offset_points(c, width());     // el carril de casa está un carril a la derecha
                if (home.size() >= 2) {
                    center_w_.clear();
                    for (const auto& p : home) center_w_.push_back(to_world(p));
                    target_lane_ = 1;
                    start_change_right(in, out, "estaba en el carril izquierdo");
                    break;
                }
            }
            if (settle_) {
                // Recién regresó al carril derecho: seguir el carril anclado hasta
                // que el detector también lo vea (si no, on_lane podría
                // regresar al carro hacia el carril izquierdo)
                pursue(lane_ok, steer);
                speed = -P.change_cmd;
                if (settle_ok_frames_ >= 3 || in.t - t_enter_ > P.settle_timeout_s || !lane_ok) settle_ = false;
            } else {
                lane_keep(in, lane_ok, steer, speed);
            }
            limit_for_stop(stop_rem, speed);
            // Si el alto está antes que el obstáculo, primero hacer el alto
            if (obstacle_ahead && stop_rem < gap_cur) {
                if (gap_cur < P.slow_gap_m) speed = -std::min(-speed, P.approach_cmd);
            } else if (obstacle_ahead) {
                if (gap_cur < P.slow_gap_m) speed = -std::min(-speed, P.approach_cmd);
                if (gap_cur < P.trigger_gap_m) {
                    if (gap_cur < P.min_change_gap_m || !change_left_feasible(gap_cur, lane_ok))
                                                           go(State::BACK_OFF, in.t, out, "demasiado cerca");
                    else if (left_free(gap_cur, gap_left, sL)) start_change_left(in, obs, out);
                    else                                   go(State::FOLLOW, in.t, out, "carril izquierdo ocupado");
                }
            }
            break;
        }
        // ── Obstáculo enfrente, sin poder rebasar todavía ────────────────
        case State::FOLLOW: {
            lane_keep(in, lane_ok, steer, speed);
            if (stop_rem <= 0.0) { enter_stop(in, out, "senal de alto: llegue al cruce"); break; }
            if (!obstacle_ahead && gap_cur >= P.detect_range_m) {
                go(State::LANE, in.t, out, "obstaculo ya no esta");
                if (anchor_hold_) { settle_ = true; settle_ok_frames_ = 0; anchor_hold_ = false; }
                break;
            }
            const double v = std::clamp((gap_cur - P.keep_gap_m) * 600.0, 0.0, (double)P.approach_cmd);
            speed = -(int)v;
            limit_for_stop(stop_rem, speed);
            if (left_free(gap_cur, gap_left, sL)) {
                if (gap_cur < P.min_change_gap_m || !change_left_feasible(gap_cur, lane_ok))
                    go(State::BACK_OFF, in.t, out, "libre pero muy cerca");
                else start_change_left(in, obs, out);
            }
            break;
        }
        // ── Retroceder para tener espacio de giro ────────────────────────
        case State::BACK_OFF: {
            steer = reverse_steer(lane_ok);
            speed = (sB > 0.15) ? P.reverse_cmd : 0;
            const bool far_enough = gap_cur >= P.backoff_target_m && change_left_feasible(gap_cur, lane_ok);
            if (far_enough || in.t - t_enter_ > P.backoff_timeout_s || sB <= 0.15) {
                if (left_free(gap_cur, gap_left, sL)) start_change_left(in, obs, out);
                else go(State::FOLLOW, in.t, out, "retroceso terminado");
            }
            break;
        }
        // ── Cambio al carril izquierdo ───────────────────────────────────
        case State::CHANGE_LEFT: {
            out.left_signal = true;
            pursue(lane_ok, steer);
            speed = -P.change_cmd;
            remember_obstacle_in_lane(obs, lane_ok, -1);    // el obstáculo ya queda a la derecha
            // el sonar derecho puede empezar a ver el obstáculo desde aquí
            if (sR < P.side_detect_m) { side_seen_ = true; side_clear_travel_ = -1; }
            else if (side_seen_ && side_clear_travel_ < 0) side_clear_travel_ = odo_.travel;
            if (changed_lane(lane_ok, dt)) {
                pass_start_x_ = odo_.travel;
                go(State::PASS, in.t, out, "en carril izquierdo");
            } else if (in.t - t_enter_ > P.change_timeout_s) {
                pass_start_x_ = odo_.travel;
                go(State::PASS, in.t, out, "CHANGE_LEFT por tiempo maximo");
            }
            break;
        }
        // ── Avanzar por el carril izquierdo hasta rebasar ────────────────
        case State::PASS: {
            out.left_signal = true;
            pursue(lane_ok, steer);
            speed = -P.pass_cmd;
            remember_obstacle_in_lane(obs, lane_ok, -1);    // otro coche más adelante en el carril derecho

            // Ultrasónico derecho: ver el obstáculo y luego dejar de verlo
            if (sR < P.side_detect_m) { side_seen_ = true; side_clear_travel_ = -1; }
            else if (side_seen_ && side_clear_travel_ < 0) side_clear_travel_ = odo_.travel;

            const bool sonar_done = side_seen_ && side_clear_travel_ >= 0 &&
                                    odo_.travel - side_clear_travel_ > P.side_clear_extra_m;
            const bool odom_done  = obstacle_end_behind();
            const bool side_busy  = sR < P.side_sonar_min_free_m;
            // Carril derecho libre Y visible: en una curva cerrada la cámara no
            // alcanza a ver el carril derecho más adelante; regresar "a ciegas"
            // es como se choca con un segundo obstáculo escondido en la curva
            // y además en recta o curva suave: regresar a media curva cerrada,
            // a ciegas (el detector pierde las líneas), es como más se sale
            const bool gentle = std::fabs(lane_curvature_near()) < P.return_max_kappa;
            const bool right_lane_free = gap_right > P.return_check_m && gentle &&
                                         lane_visible(0.0, P.return_visible_from_m, P.return_visible_m);

            // Obstáculo en el carril IZQUIERDO: regresar antes si se puede, si no frenar
            if (obstacle_ahead && gap_cur < P.trigger_gap_m) {
                if (!side_busy && right_lane_free && (sonar_done || odom_done)) {
                    start_change_right(in, out, "obstaculo en carril izquierdo");
                } else {
                    speed = (gap_cur < P.keep_gap_m) ? 0 : -P.approach_cmd;
                }
                break;
            }
            if ((sonar_done || (!side_seen_ && odom_done)) && !side_busy && right_lane_free) {
                start_change_right(in, out, sonar_done ? "rebasado (sonar)" : "rebasado (odometria)");
            } else if (in.t - t_enter_ > P.pass_timeout_s && !side_busy && right_lane_free) {
                start_change_right(in, out, "PASS por tiempo maximo");
            } else if (in.t - t_enter_ > 3.0 * P.pass_timeout_s && !side_busy && gap_right > P.return_check_m) {
                // no se logra "ver" el carril derecho (sin líneas, curva muy larga): regresar con cuidado
                start_change_right(in, out, "PASS por tiempo maximo (sin vista completa)");
            }
            break;
        }
        // ── Regreso al carril derecho ────────────────────────────────────
        case State::CHANGE_RIGHT: {
            out.right_signal = true;
            pursue(lane_ok, steer);
            speed = -P.change_cmd;
            // Apareció otro obstáculo en el carril derecho (p. ej. escondido en
            // la curva): abortar el regreso y volver al carril izquierdo
            if (obstacle_ahead && gap_cur < P.abort_return_gap_m) {
                if (!change_left_feasible(gap_cur, lane_ok)) {
                    target_lane_ = 0; anchor_hold_ = true;
                    go(State::BACK_OFF, in.t, out, "obstaculo al regresar, muy cerca");
                    speed = 0;
                    break;
                }
                if (gap_left > gap_cur + 0.3 && sL > P.side_sonar_min_free_m) {
                    start_change_left(in, obs, out, "obstaculo al regresar: sigo rebasando");
                    break;
                }
                speed = (gap_cur < P.keep_gap_m) ? 0 : -P.approach_cmd / 2;
            }
            if (changed_lane(lane_ok, dt) || in.t - t_enter_ > P.change_timeout_s) {
                const bool timeout = in.t - t_enter_ > P.change_timeout_s;
                go(State::LANE, in.t, out, timeout ? "CHANGE_RIGHT por tiempo maximo" : "de vuelta en carril derecho");
                settle_ = true; settle_ok_frames_ = 0;
            }
            break;
        }
        // ── Alto ─────────────────────────────────────────────────────────
        case State::STOP: {
            out.blinkers = true;
            lane_keep(in, lane_ok, steer, speed);
            speed = 0;
            if (in.t - t_enter_ > P.stop_time_s) {
                stop_end_t_ = in.t;
                crossing_ = true; cross_end_travel_ = odo_.travel + P.cross_length_m;
                go(State::LANE, in.t, out, "fin del alto");
            }
            break;
        }
        }

        // ── Seguridad, en cualquier estado ───────────────────────────────
        const double front_gap_any = nearest_gap_corridor(obs);
        bool blocked = speed < 0 && (sF < P.front_sonar_stop_m || front_gap_any < P.emergency_gap_m);
        // Detenido a propósito en PASS (algo enfrente en el carril izquierdo) por mucho tiempo
        if (st_ == State::PASS && speed == 0) { if (stopped_since_ < 0) stopped_since_ = in.t; }
        else stopped_since_ = -1;
        if (stopped_since_ >= 0 && in.t - stopped_since_ > 2.0) { blocked = true; speed = -1; }
        if (blocked) speed = 0;
        // Atorado a media maniobra (algo justo enfrente): retroceder y reintentar
        const bool maneuvering = st_ == State::CHANGE_LEFT || st_ == State::PASS || st_ == State::CHANGE_RIGHT;
        if (blocked && maneuvering) {
            if (blocked_since_ < 0) blocked_since_ = in.t;
            if (in.t - blocked_since_ > P.blocked_timeout_s) {
                target_lane_ = 0; settle_ = false; anchor_hold_ = true;
                go(State::BACK_OFF, in.t, out, "bloqueado a media maniobra");
                blocked_since_ = -1;
            }
        } else blocked_since_ = -1;

        out.steer_cmd = (int)std::lround(std::clamp(steer, P.steer_cmd_min, P.steer_cmd_max));
        out.speed_cmd = speed;
        // Para el log: qué obstáculo provocó el rebase / retroceso
        if (!out.event.empty() && (st_ == State::BACK_OFF || st_ == State::CHANGE_LEFT || st_ == State::FOLLOW)) {
            double bx = 1e9, by = 0;
            for (const auto& o : obs) if (o.x > 0 && o.x < bx) { bx = o.x; by = o.y; }
            double ds = INFINITY;
            for (const auto& g : signs_w_) { const Vec2 c = to_car(g.w); ds = std::min(ds, std::hypot(c.x - bx, c.y - by)); }
            char b[200];
            std::snprintf(b, sizeof b, " | obstaculo x=%.2f y=%+.2f gap=%.2f sonar=%.2f | senales=%zu, la mas cercana a %.2f m de el",
                          bx, by, gap_cur, sF, signs_w_.size(), ds);
            out.event += b;
        }
        out.state = state_name(st_);
        last_steer_cmd_ = out.steer_cmd;
        last_speed_cmd_ = out.speed_cmd;
        return out;
    }

    // Para depuración / pruebas
    int target_lane() const { return target_lane_; }
    // Para el log: posición lateral del carro respecto al carril DERECHO (m,
    // + = izquierda; 0.40 = centro del carril izquierdo) y su rumbo respecto
    // al carril (grados, + = apunta a la izquierda)
    double debug_lateral() const { return center_w_.empty() ? NAN : lateral_of(car_polyline(), {0.0, 0.0}); }
    double debug_heading_deg() const {
        if (center_w_.size() < 2) return NAN;
        const Vec2 tn = normal_near(car_polyline(), {0.0, 0.0});
        return -std::atan2(-tn.x, tn.y) * 180.0 / M_PI; }
    double debug_model_age(double t) const { return t - lane_t_; }
    Vec2 debug_target() const { return last_target_; }
    Vec2 debug_center(double d) const { return point_at(car_polyline(), d); }
    // y del modelo (marco del carro) en una x dada; NAN si no la cubre
    double debug_model_y(double x) const {
        const auto pl = car_polyline();
        for (size_t i = 1; i < pl.size(); ++i)
            if ((pl[i-1].x - x) * (pl[i].x - x) <= 0 && pl[i].x != pl[i-1].x)
                return pl[i-1].y + (x - pl[i-1].x) / (pl[i].x - pl[i-1].x) * (pl[i].y - pl[i-1].y);
        return NAN;
    }
    int anchor_rejects() const { return anchor_rejects_; }
    // Para el log: obstáculos en memoria (x adelante, y izquierda, desde el eje
    // trasero; "S" = descartado por ser señal) y señales conocidas
    std::string debug_obs_txt() const {
        std::string r; char b[64];
        for (const auto& m : mem_) {
            if (m.hits < P.mem_min_hits) continue;
            const Vec2 c = to_car(m.w);
            std::snprintf(b, sizeof b, "(%.2f,%+.2f%s)", c.x, c.y,
                          is_sign(m.w) ? "S" : (m.no_echo >= kNoEchoGhost ? "P" : "")); r += b;
        }
        for (const auto& g : signs_w_) { const Vec2 c = to_car(g.w); std::snprintf(b, sizeof b, "[alto %.2f,%+.2f]", c.x, c.y); r += b; }
        if (unconfirmed_) { std::snprintf(b, sizeof b, " sin_sonar=%d", unconfirmed_); r += b; }
        return r.empty() ? "-" : r;
    }
    // metros que faltan para el punto de alto (NAN si no hay alto pendiente)
    double stop_remaining() const { return stop_pending_ ? stop_target_ - odo_.travel : NAN; }
    double travel() const { return odo_.travel; }

private:
    Params P;
    State  st_ = State::LANE;
    double t_prev_ = -1, t_enter_ = 0, stop_end_t_ = -1e9;
    // Alto en el cruce: punto (en distancia recorrida) donde detenerse
    bool   stop_pending_ = false;
    double stop_target_ = 0, stop_w_ = 0;
    double last_stop_travel_ = -1e9;
    int    unconfirmed_ = 0;                  // objetos descartados porque el sonar no los ve
    double sonar_check_t_ = -1e9;
    bool   crossing_ = false;                 // cruzando la intersección tras un alto
    double cross_end_travel_ = 0;
    struct SignW { Vec2 w; double t; };
    std::vector<SignW> signs_w_;              // señales vistas (mundo de odometría)

    // Guarda dónde está cada señal para no tomarla como obstáculo
    void remember_sign(const Inputs& in) {
        std::vector<SignW> keep;
        for (const auto& g : signs_w_)
            if (to_car(g.w).x > -1.0 && in.t - g.t < 60.0) keep.push_back(g);
        signs_w_.swap(keep);
        if (!std::isfinite(in.stop_dist) || in.stop_dist <= 0.0) return;
        const Vec2 w = to_world({P.stop_cam_x_m + in.stop_dist, in.stop_lat});
        for (auto& g : signs_w_)
            if (std::hypot(g.w.x - w.x, g.w.y - w.y) < 0.4) {
                // la medición más reciente (más cerca, menos error de odometría) manda
                g.w = {0.3 * g.w.x + 0.7 * w.x, 0.3 * g.w.y + 0.7 * w.y}; g.t = in.t; return; }
        signs_w_.push_back({w, in.t});
    }
    bool sign_ahead() const {
        for (const auto& g : signs_w_) { const Vec2 c = to_car(g.w); if (c.x > -0.2 && c.x < 3.0) return true; }
        return false;
    }
    bool is_sign(const Vec2& w) const {
        for (const auto& g : signs_w_)
            if (std::hypot(g.w.x - w.x, g.w.y - w.y) < P.sign_ignore_m) return true;
        return false;
    }

    // Cada medición de distancia da un punto de alto (en distancia recorrida
    // por odometría). Se promedian, pesando más las cercanas (más precisas y
    // con menos error de odometría acumulado).
    void update_stop_target(const Inputs& in) {
        remember_sign(in);
        if (stop_pending_ && odo_.travel > stop_target_ + P.stop_overshoot_m) stop_pending_ = false;
        if (!std::isfinite(in.stop_dist) || in.stop_dist <= 0.0) return;
        // Distancia A LO LARGO DEL CARRIL: si el carro va algo girado respecto
        // al carril, la distancia "hacia enfrente" de la cámara no es la misma
        // (con la señal 35 cm a un lado y 6° de giro son ~4 cm de error)
        double psi = debug_heading_deg() * M_PI / 180.0;          // + = apunta a la izquierda
        if (!std::isfinite(psi) || std::fabs(psi) > 0.35) psi = 0.0;
        const double along = in.stop_dist * std::cos(psi) - in.stop_lat * std::sin(psi);
        const double tgt = odo_.travel + along - (P.car_front_m - P.stop_cam_x_m) - P.stop_before_m;
        if (tgt < last_stop_travel_ + P.stop_same_sign_m) return;      // la señal donde ya se hizo alto
        const double z = std::max(in.stop_dist, 0.3);
        const double w = 1.0 / (z * z);
        if (!stop_pending_ || std::fabs(tgt - stop_target_) > P.stop_new_sign_m) {
            // primera medición, u otra señal MÁS CERCA que la pendiente
            if (!stop_pending_ || tgt < stop_target_) { stop_target_ = tgt; stop_w_ = w; stop_pending_ = true; }
            return;
        }
        stop_w_ *= 0.85;                                               // olvidar poco a poco las lejanas
        stop_target_ = (stop_target_ * stop_w_ + tgt * w) / (stop_w_ + w);
        stop_w_ += w;
    }
    void limit_for_stop(double rem, int& speed) const {
        if (!(rem < P.stop_slow_m)) return;
        const int cap = (rem < 0.25) ? P.stop_final_cmd : P.stop_approach_cmd;
        if (speed < 0) speed = -std::min(-speed, cap);
    }
    void enter_stop(const Inputs& in, Outputs& out, const char* why) {
        last_stop_travel_ = odo_.travel;
        stop_pending_ = false;
        go(State::STOP, in.t, out, why);
    }
    int    confirm_cur_ = 0;
    int    last_steer_cmd_ = 90, last_speed_cmd_ = 0;
    double blocked_since_ = -1, stopped_since_ = -1;

    // ── Odometría por comandos (dead reckoning) ──────────────────────────
    struct Odom { double X = 0, Y = 0, th = 0, travel = 0; } odo_;

    double speed_scale_ = 1.0;          // velocidad real / modelo (se ajusta con el sonar)
    void integrate_odometry(double dt, const Inputs& in)
    {
        if (dt <= 0) return;
        // velocidad del modelo con un retardo de primer orden (el carro no
        // cambia de velocidad instantáneamente)
        const double v_cmd = -last_speed_cmd_ * P.speed_mps_per_cmd * speed_scale_;   // + adelante
        v_est_ += (v_cmd - v_est_) * std::min(1.0, dt / 0.25);
        const double v = v_est_;
        const double th_mid = odo_.th;
        if (in.has_yaw_rate) odo_.th += in.yaw_rate * dt;                 // giro medido (IMU)
        else {
            // sin IMU: ángulo de las ruedas supuesto, con retraso del servo
            delta_est_ += (cmd_to_rad(last_steer_cmd_) - delta_est_) * std::min(1.0, dt / std::max(0.01, P.steer_lag_s));
            const double raw = v / P.wheelbase_m * std::tan(delta_est_) * dt;
            yaw_raw_since_lane_ += raw;
            odo_.th += steer_gain_ * raw;
        }
        const double th_avg = 0.5 * (th_mid + odo_.th);
        odo_.X  += v * std::cos(th_avg) * dt;
        odo_.Y  += v * std::sin(th_avg) * dt;
        odo_.travel += v * dt;
    }
    double v_est_ = 0.0, delta_est_ = 0.0;

    // Autocalibración de la dirección (solo sin IMU): mientras se ven las
    // líneas, se compara cuánto giró de verdad el carro (según el carril) con
    // lo que dice el modelo, y se ajusta la ganancia steer_gain_.
    double steer_gain_ = 1.0, yaw_raw_since_lane_ = 0.0, sxx_ = 0.0, sxy_ = 0.0;
    static double heading_between(const std::vector<Vec2>& pl, double x0, double x1, bool& ok)
    {
        auto y_at = [&](double x, bool& f) {
            for (size_t i = 1; i < pl.size(); ++i)
                if ((pl[i-1].x - x) * (pl[i].x - x) <= 0 && pl[i].x != pl[i-1].x) {
                    f = true; return pl[i-1].y + (x - pl[i-1].x) / (pl[i].x - pl[i-1].x) * (pl[i].y - pl[i-1].y); }
            f = false; return 0.0; };
        bool f0, f1; const double y0 = y_at(x0, f0), y1 = y_at(x1, f1);
        ok = f0 && f1;
        return std::atan2(y1 - y0, x1 - x0);
    }
    void learn_steer_gain(const std::vector<Vec2>& meas, bool fresh_prev, bool has_imu)
    {
        const double raw = yaw_raw_since_lane_;
        yaw_raw_since_lane_ = 0.0;
        if (has_imu || !P.auto_steer_gain || !fresh_prev || v_est_ < 0.15) return;
        bool ok1, ok2;
        const double h_pred = heading_between(car_polyline(), 0.45, 0.85, ok1);
        const double h_meas = heading_between(meas, 0.45, 0.85, ok2);
        if (!ok1 || !ok2) return;
        const double real = steer_gain_ * raw + (h_pred - h_meas);    // giro real en este intervalo
        if (std::fabs(real - steer_gain_ * raw) > 0.15) return;        // salto raro del detector
        sxx_ = 0.995 * sxx_ + raw * raw;
        sxy_ = 0.995 * sxy_ + raw * real;
        if (sxx_ > 0.004) steer_gain_ = std::clamp(sxy_ / sxx_, 0.4, 1.6);
    }
public:
    double steer_gain() const { return steer_gain_; }
private:

    // Escala de velocidad: al acercarse en línea recta a un obstáculo fijo,
    // lo que baja la lectura del sonar delantero ES la velocidad real.
    double sf_ref_ = -1, sf_ref_t_ = 0, sf_ref_travel_ = 0;
    void calibrate_speed(const Inputs& in, double sF)
    {
        if (!P.auto_speed_scale) return;
        // (no con una señal de alto enfrente: el sonar rebota en el poste de
        // forma intermitente y la escala saldría mal)
        const bool ok = st_ == State::LANE && !stop_pending_ && !sign_ahead() &&
                        std::isfinite(sF) && sF > 0.35 && sF < 2.0 &&
                        std::fabs(last_steer_cmd_ - P.steer_center_cmd) < 15 && v_est_ > 0.15;
        if (!ok) { sf_ref_ = -1; return; }
        if (sf_ref_ < 0) { sf_ref_ = sF; sf_ref_t_ = in.t; sf_ref_travel_ = odo_.travel; return; }
        const double dist_model = odo_.travel - sf_ref_travel_;
        if (in.t - sf_ref_t_ < 0.6 || dist_model < 0.15) return;
        const double dist_real = sf_ref_ - sF;
        if (dist_real > 0.05) {
            const double ratio = std::clamp(dist_real / dist_model, 0.5, 2.0);
            speed_scale_ = std::clamp(speed_scale_ * (0.7 + 0.3 * ratio), 0.6, 1.6);
        }
        sf_ref_ = sF; sf_ref_t_ = in.t; sf_ref_travel_ = odo_.travel;
    }
public:
    double speed_scale() const { return speed_scale_; }
private:
    Vec2 to_world(const Vec2& p) const {
        const double c = std::cos(odo_.th), s = std::sin(odo_.th);
        return { odo_.X + c * p.x - s * p.y, odo_.Y + s * p.x + c * p.y };
    }
    Vec2 to_car(const Vec2& w) const {
        const double c = std::cos(odo_.th), s = std::sin(odo_.th);
        const double dx = w.x - odo_.X, dy = w.y - odo_.Y;
        return { c * dx + s * dy, -s * dx + c * dy };
    }

    double cmd_to_rad(double cmd) const {
        const double deg = P.steer_deg_per_cmd * (cmd - P.steer_center_cmd) + P.steer_offset_deg;
        return deg * M_PI / 180.0;
    }
    double rad_to_cmd(double rad) const {
        double deg = std::clamp(rad * 180.0 / M_PI, -P.max_steer_deg, P.max_steer_deg);
        return P.steer_center_cmd + (deg - P.steer_offset_deg) / P.steer_deg_per_cmd;
    }

    static double clean(double r) { return (std::isfinite(r) && r > 0.0) ? r : INFINITY; }

    // ── Modelo del carril (línea central en el mundo de odometría) ───────
    std::vector<Vec2> center_w_;     // puntos de la línea central, mundo
    bool   lane_valid_ = false;
    double lane_t_ = -1e9;
    double width_est_ = -1;

    struct Quad { double a = 0, b = 0, c = 0; bool ok = false; double xmin = 0, xmax = 0; };
    static Quad fit_quad(const std::vector<Vec2>& pts)
    {
        Quad q;
        if (pts.size() < 3) return q;
        double S[5] = {0,0,0,0,0}, T[3] = {0,0,0};
        q.xmin = 1e9; q.xmax = -1e9;
        for (const auto& p : pts) {
            double xp = 1;
            for (int k = 0; k < 5; ++k) { S[k] += xp; if (k < 3) T[k] += xp * p.y; xp *= p.x; }
            q.xmin = std::min(q.xmin, p.x); q.xmax = std::max(q.xmax, p.x);
        }
        // [S4 S3 S2][a]   [T2]
        // [S3 S2 S1][b] = [T1]
        // [S2 S1 S0][c]   [T0]
        const double m[3][3] = {{S[4],S[3],S[2]},{S[3],S[2],S[1]},{S[2],S[1],S[0]}};
        const double r[3] = {T[2],T[1],T[0]};
        auto det3 = [](const double A[3][3]) {
            return A[0][0]*(A[1][1]*A[2][2]-A[1][2]*A[2][1]) - A[0][1]*(A[1][0]*A[2][2]-A[1][2]*A[2][0])
                 + A[0][2]*(A[1][0]*A[2][1]-A[1][1]*A[2][0]); };
        const double D = det3(m);
        if (std::fabs(D) < 1e-12) {           // pocos puntos distintos: recta
            double sx=0, sy=0, sxx=0, sxy=0; const double n = (double)pts.size();
            for (const auto& p : pts) { sx += p.x; sy += p.y; sxx += p.x*p.x; sxy += p.x*p.y; }
            const double d = n*sxx - sx*sx; if (std::fabs(d) < 1e-12) return q;
            q.b = (n*sxy - sx*sy)/d; q.c = (sy - q.b*sx)/n; q.a = 0; q.ok = true; return q;
        }
        double sol[3];
        for (int k = 0; k < 3; ++k) {
            double A[3][3]; for (int i=0;i<3;++i) for (int j=0;j<3;++j) A[i][j] = (j==k) ? r[i] : m[i][j];
            sol[k] = det3(A) / D;
        }
        q.a = sol[0]; q.b = sol[1]; q.c = sol[2];
        // Curvatura físicamente razonable (radio >= ~0.5 m)
        q.a = std::clamp(q.a, -1.0, 1.0);
        q.ok = true;
        return q;
    }
    static double qy(const Quad& q, double x)  { return q.a*x*x + q.b*x + q.c; }
    static double qdy(const Quad& q, double x) { return 2*q.a*x + q.b; }

    // Curva paralela a una línea (lista de puntos ordenada), desplazada
    // "line_off" hacia la DERECHA de la línea (= la línea está line_off a la
    // izquierda del centro). Normal calculada con los vecinos.
    static std::vector<Vec2> offset_points(const std::vector<Vec2>& pts, double line_off)
    {
        std::vector<Vec2> c;
        const size_t n = pts.size();
        if (n < 2) return c;
        // Suavizado: parábola en un marco alineado con la cuerda de la línea
        // (así funciona aunque la línea esté muy inclinada respecto al carro)
        double ux = pts.back().x - pts.front().x, uy = pts.back().y - pts.front().y;
        const double un = std::hypot(ux, uy); if (un < 1e-6) return c;
        ux /= un; uy /= un;
        const Vec2 o = pts.front();
        std::vector<Vec2> loc; loc.reserve(n);
        for (const auto& p : pts) loc.push_back({ (p.x-o.x)*ux + (p.y-o.y)*uy, -(p.x-o.x)*uy + (p.y-o.y)*ux });
        Quad q = fit_quad(loc);
        if (!q.ok) return c;
        if (n < 4) q.a = 0;                                        // pocos puntos: recta
        for (double u = q.xmin; u <= q.xmax + 1e-9; u += 0.05) {
            const double v = qy(q, u), d = qdy(q, u), dn = std::sqrt(1 + d*d);
            // punto y normal izquierda en el marco local
            const double px = u, py = v, nx = -d / dn, ny = 1 / dn;
            const double cx = px - line_off * nx, cy = py - line_off * ny;
            c.push_back({ o.x + cx*ux - cy*uy, o.y + cx*uy + cy*ux });
        }
        return c;
    }

    // Combina 1 o 2 estimaciones de la línea central. Se toma como base la que
    // empieza más cerca del carro; si la otra es consistente se promedia el
    // desplazamiento (corrige errores del ancho de carril).
    std::vector<Vec2> combine_centers(const std::vector<std::vector<Vec2>>& c) const
    {
        if (c.size() == 1) return c[0];
        const auto d0 = [](const std::vector<Vec2>& v) { return v.empty() ? 1e9 : std::hypot(v[0].x, v[0].y); };
        const auto& base  = (d0(c[0]) <= d0(c[1])) ? c[0] : c[1];
        const auto& other = (d0(c[0]) <= d0(c[1])) ? c[1] : c[0];
        if (base.size() < 2) return other;
        if (other.size() < 2) return base;
        const double m = line_offset(base, other);                 // + = la otra está a la izquierda
        if (std::fabs(m) > 0.06) return base;                      // no coinciden: quedarse con la cercana
        std::vector<Vec2> out;
        for (size_t i = 0; i < base.size(); ++i) {
            const Vec2 a = base[i == 0 ? 0 : i - 1], b = base[i + 1 < base.size() ? i + 1 : base.size() - 1];
            double tx = b.x - a.x, ty = b.y - a.y; const double tn = std::hypot(tx, ty);
            if (tn < 1e-9) continue;
            out.push_back({ base[i].x - 0.5 * m * ty / tn, base[i].y + 0.5 * m * tx / tn });
        }
        return out;
    }

    // Desplazamiento lateral típico (mediana) de los puntos de una línea
    // respecto a una polilínea de referencia
    double line_offset(const std::vector<Vec2>& ref, const std::vector<Vec2>& pts) const
    {
        // Solo puntos que caen DENTRO de la referencia (no sobre su extensión
        // recta, que en curva cerrada es engañosa). Si casi no hay, usar los
        // puntos más cercanos al carro.
        std::vector<double> v, near;
        for (const auto& p : pts) {
            bool in = false; const double l = lateral_of(ref, p, &in);
            if (in) v.push_back(l);
            if (near.size() < 3) near.push_back(l);
        }
        if (v.size() < 2) v = near;
        if (v.empty()) return 0.0;
        std::nth_element(v.begin(), v.begin() + v.size()/2, v.end());
        return v[v.size()/2];
    }

    void update_lane_model(const Inputs& in)
    {
        // Solo la parte cercana de cada línea (lejos es menos precisa), ordenada
        // del carro hacia adelante
        auto near_part = [&](const std::vector<Vec2>& v) {
            std::vector<Vec2> o;
            for (const auto& p : v) if (p.x > 0 && std::hypot(p.x, p.y) <= P.fit_max_x_m) o.push_back(p);
            std::sort(o.begin(), o.end(), [](const Vec2& a, const Vec2& b) { return std::hypot(a.x, a.y) < std::hypot(b.x, b.y); });
            return o; };
        const std::vector<Vec2> lpts = near_part(in.left_line), rpts = near_part(in.right_line);
        const bool Lok = lpts.size() >= 3, Rok = rpts.size() >= 3;
        if (!Lok && !Rok) return;                         // conserva el modelo anterior
        if (Lok && Rok) {                                 // ancho del carril
            const auto rc = offset_points(rpts, 0.0);
            const double w = line_offset(rc, lpts);
            if (w > 0.6 * P.lane_width_m && w < 1.5 * P.lane_width_m)
                width_est_ = (width_est_ < 0) ? w : 0.9 * width_est_ + 0.1 * w;
        }
        const double W = width();

        // ¿Seguir "anclado" al carril de casa (el derecho)? Durante el rebase
        // el detector cambia de carril y confunde izquierda/derecha cuando solo
        // ve una línea. Por eso NO se confía en sus etiquetas: cada línea vista
        // se compara con el carril de casa predicho por odometría y se asigna a
        // la línea de pista más cercana (bordes y punteada están a ±W/2 + k·W).
        const bool anchored = anchoring() && lane_valid_ && !center_w_.empty() &&
                              (maneuvering_now() || in.t - lane_t_ < P.anchor_timeout_s);
        const double tol = std::min(0.17, P.anchor_tol_m + P.anchor_tol_growth * std::max(0.0, in.t - lane_t_));
        std::vector<std::vector<Vec2>> cands;
        if (!anchored) {
            // ¿El detector está viendo el carril IZQUIERDO? (continua a su
            // izquierda y punteada a su derecha)
            if (P.use_line_types) {
                if (Lok && Rok && in.left_type == 1 && in.right_type == 2) wrong_lane_frames_++;
                else if ((Lok && in.left_type == 2) || (Rok && in.right_type == 1)) wrong_lane_frames_ = 0;
            }
            // Carril normal: el carril de casa es el que reporta el detector
            if (Lok) cands.push_back(offset_points(lpts, +0.5 * W));
            if (Rok) cands.push_back(offset_points(rpts, -0.5 * W));
        } else {
            const auto pred = car_polyline();
            std::vector<std::vector<Vec2>> typed;
            bool disagree = false;
            for (int side = 0; side < 2; ++side) {
                const auto& pts = side ? rpts : lpts;
                if (side ? !Rok : !Lok) continue;
                const int type = side ? in.right_type : in.left_type;
                const double m = line_offset(pred, pts);
                const double k_near = std::round((m - 0.5 * W) / W);       // la más cercana a lo predicho
                double t_snap = NAN;
                if (P.use_line_types && type == 2) t_snap = 0.5 * W;        // punteada: SIEMPRE el centro
                else if (P.use_line_types && type == 1)                     // continua: uno de los bordes
                    t_snap = (m < 0.5 * W) ? -0.5 * W : 1.5 * W;
                if (std::isfinite(t_snap) && std::fabs(m - t_snap) < P.type_tol_m) {
                    const bool agrees = std::fabs((0.5 * W + k_near * W) - t_snap) < 1e-6;
                    // El tipo corrige a la odometría solo si lo dice varios cuadros
                    // seguidos (un cuadro mal clasificado no debe mover el carril)
                    if (agrees || type_disagree_frames_ >= P.type_confirm_frames) {
                        typed.push_back(offset_points(pts, t_snap));
                        if (!agrees) disagree = true;
                        continue;
                    }
                    disagree = true;
                    continue;          // espera confirmación: no usar esta línea todavía
                }
                const double k = k_near;
                const double snapped = 0.5 * W + k * W;
                // Solo existen 3 líneas: borde der. (k=-1), punteada (k=0), borde izq. (k=1)
                if (k < -1 || k > 1 || std::fabs(m - snapped) > tol) {
                    anchor_rejects_++;
                    continue; }
                cands.push_back(offset_points(pts, snapped));
            }
            type_disagree_frames_ = disagree ? type_disagree_frames_ + 1 : 0;
            if (!typed.empty()) cands.swap(typed);        // las líneas con tipo conocido mandan
        }
        if (cands.empty()) return;                       // nada confiable: seguir con odometría
        std::vector<Vec2> c_new = combine_centers(cands);
        if (!anchored)
            learn_steer_gain(c_new, lane_valid_ && !center_w_.empty() && in.t - lane_t_ < 0.15, in.has_yaw_rate);
        else yaw_raw_since_lane_ = 0.0;
        if (c_new.size() < 2) return;

        // Fusión con lo que ya se conocía: el tramo entre el carro y la primera
        // línea visible (la cámara no ve el piso justo enfrente) se toma del
        // modelo anterior movido con odometría, no de extrapolar la parábola.
        {
            const double x0 = c_new.front().x;
            std::vector<Vec2> fused;
            bool prev_ok = lane_valid_ && !center_w_.empty() &&
                           in.t - lane_t_ < (maneuvering_now() ? P.blind_max_s : P.anchor_timeout_s);
            // si la medición corrigió mucho al modelo, el tramo viejo ya no sirve
            if (prev_ok && std::fabs(lateral_of(car_polyline(), c_new.front())) > 0.10) prev_ok = false;
            if (prev_ok) {
                const auto pred = car_polyline();
                for (const auto& p : pred)
                    if (p.x >= -0.40 && p.x < x0 - 0.005 && (fused.empty() || p.x - fused.back().x > 0.03))
                        fused.push_back(p);
            }
            if (fused.empty() || fused.front().x > 0.0) {
                // sin historia: extrapolar en línea recta desde los primeros puntos medidos
                const Vec2 a = c_new[0], b = c_new[std::min<size_t>(2, c_new.size()-1)];
                const double m = (b.x - a.x > 1e-6) ? (b.y - a.y) / (b.x - a.x) : 0.0;
                std::vector<Vec2> ext;
                for (double x = -0.20; x < (fused.empty() ? x0 - 0.02 : fused.front().x - 0.02); x += 0.05)
                    ext.push_back({x, a.y + m * (x - a.x)});
                fused.insert(fused.begin(), ext.begin(), ext.end());
            }
            fused.insert(fused.end(), c_new.begin(), c_new.end());
            c_new.swap(fused);
        }

        // Mientras se asienta tras el rebase: ¿el detector ya volvió a ver el
        // carril de casa? (su centro coincide con el modelo anclado)
        if (anchored && st_ == State::LANE && settle_) {
            std::vector<std::vector<Vec2>> dc;
            if (Lok) dc.push_back(offset_points(lpts, +0.5 * W));
            if (Rok) dc.push_back(offset_points(rpts, -0.5 * W));
            const std::vector<Vec2> det = combine_centers(dc);
            // y además el carro ya va alineado y centrado (si no, on_lane, con su
            // dirección limitada, puede no alcanzar a corregir en una curva)
            const Vec2 tn = normal_near(c_new, {0.0, 0.0});
            const bool aligned = std::fabs(lateral_of(c_new, {0.0, 0.0})) < 0.05 &&
                                 std::fabs(std::atan2(-tn.x, tn.y)) < 0.10;
            const bool agree = det.size() >= 2 && aligned &&
                std::fabs(point_at(det, P.lookahead_m).y - point_at(c_new, P.lookahead_m).y) < 0.25 * W;
            settle_ok_frames_ = agree ? settle_ok_frames_ + 1 : 0;
        }

        // Sin dobleces: cada punto debe avanzar (al menos 2 cm) en la dirección
        // general de la línea (de su primer a su último punto)
        if (c_new.size() >= 2) {
            double dx = c_new.back().x - c_new.front().x, dy = c_new.back().y - c_new.front().y;
            const double dn = std::hypot(dx, dy);
            if (dn > 1e-6) {
                dx /= dn; dy /= dn;
                std::vector<Vec2> clean; double last = -1e9;
                for (const auto& p : c_new) {
                    const double pr = p.x * dx + p.y * dy;
                    if (pr < last + 0.02) continue;
                    clean.push_back(p); last = pr;
                }
                c_new.swap(clean);
            }
        }
        if (c_new.size() < 2) return;
        center_w_.clear();
        for (const auto& p : c_new) center_w_.push_back(to_world(p));
        lane_valid_ = true;
        lane_t_ = in.t;
    }

    std::vector<Vec2> car_polyline() const {
        std::vector<Vec2> v; v.reserve(center_w_.size());
        for (const auto& w : center_w_) v.push_back(to_car(w));
        return v;
    }
    // Punto de la polilínea a una distancia d (desde el eje trasero), con
    // extensión recta más allá del último punto
    static Vec2 point_at(const std::vector<Vec2>& pl, double d)
    {
        if (pl.empty()) return {d, 0};
        if (pl.size() == 1) return pl[0];
        for (size_t i = 1; i < pl.size(); ++i) {
            const double r0 = std::hypot(pl[i-1].x, pl[i-1].y), r1 = std::hypot(pl[i].x, pl[i].y);
            if (pl[i].x > 0 && r0 <= d && r1 >= d) {
                const double t = (r1 - r0 > 1e-9) ? (d - r0) / (r1 - r0) : 0;
                return { pl[i-1].x + t*(pl[i].x-pl[i-1].x), pl[i-1].y + t*(pl[i].y-pl[i-1].y) };
            }
        }
        const Vec2 a = pl[pl.size()-2], b = pl.back();
        const double sx = b.x - a.x, sy = b.y - a.y, n = std::hypot(sx, sy);
        if (n < 1e-9) return b;
        const double rb = std::hypot(b.x, b.y);
        if (rb >= d) {   // toda la polilínea está más allá: tomar el primer punto
            return pl.front();
        }
        // extender en línea recta hasta llegar a d
        double lo = 0, hi = 5.0;
        for (int k = 0; k < 40; ++k) {
            const double m = 0.5*(lo+hi);
            const Vec2 q{ b.x + sx/n*m, b.y + sy/n*m };
            (std::hypot(q.x, q.y) < d) ? lo = m : hi = m;
        }
        return { b.x + sx/n*lo, b.y + sy/n*lo };
    }
    // Normal izquierda de la polilínea cerca del punto p
    static Vec2 normal_near(const std::vector<Vec2>& pl, const Vec2& p)
    {
        if (pl.size() < 2) return {0, 1};
        size_t best = 1; double bd = 1e18;
        for (size_t i = 1; i < pl.size(); ++i) {
            const double mx = 0.5*(pl[i].x+pl[i-1].x), my = 0.5*(pl[i].y+pl[i-1].y);
            const double d = std::hypot(mx-p.x, my-p.y);
            if (d < bd) { bd = d; best = i; }
        }
        double sx = pl[best].x - pl[best-1].x, sy = pl[best].y - pl[best-1].y;
        const double n = std::hypot(sx, sy); if (n < 1e-9) return {0, 1};
        return { -sy/n, sx/n };
    }

    // Desviación lateral (con signo, + izquierda) de un punto respecto a la
    // línea central, medida sobre la normal del carril
    double lateral_of(const std::vector<Vec2>& pl, const Vec2& p, bool* inside = nullptr) const
    {
        if (inside) *inside = false;
        if (pl.size() < 2) return p.y;
        // segmento más cercano, extendiendo el último en línea recta
        double best = 1e18, lat = p.y;
        for (size_t i = 1; i < pl.size(); ++i) {
            const Vec2 a = pl[i-1], b = pl[i];
            const double sx = b.x-a.x, sy = b.y-a.y, L2 = sx*sx+sy*sy; if (L2 < 1e-12) continue;
            double t = ((p.x-a.x)*sx + (p.y-a.y)*sy) / L2;
            const double lo = (i == 1) ? -1e9 : 0.0;                 // primer tramo: se extiende hacia atrás
            const double hi = (i + 1 == pl.size()) ? 1e9 : 1.0;      // último tramo: hacia adelante
            const double t_raw = t;
            t = std::clamp(t, lo, hi);
            const double qx = a.x + t*sx, qy2 = a.y + t*sy;
            const double d = std::hypot(p.x-qx, p.y-qy2);
            if (d < best) {
                best = d; const double L = std::sqrt(L2); lat = ((p.y-qy2)*sx - (p.x-qx)*sy) / L;
                if (inside) *inside = t_raw >= 0.0 && t_raw <= 1.0;
            }
        }
        return lat;
    }

    // ── Carriles relativos: 0 = el que se sigue ahora, +1 izquierdo, -1 derecho
    // Carril objetivo respecto al carril que reporta el detector, en carriles
    // enteros (+1 = un carril a la izquierda del detectado). Cuando el detector
    // "brinca" al carril de al lado, se corrige restando ese brinco.
    // Carriles respecto al carril de casa (0 = derecho, 1 = izquierdo)
    int    target_lane_ = 0;
    int    wrong_lane_frames_ = 0;
    int    type_disagree_frames_ = 0;          // cuadros seguidos en que el tipo de línea contradice a la odometría         // cuadros seguidos viendo que está en el carril izquierdo
    int    anchor_rejects_ = 0;
    bool   settle_ = false;            // recién regresó: esperar a que el detector coincida
    bool   anchor_hold_ = false;       // BACK_OFF/FOLLOW a media maniobra: seguir anclado
    int    settle_ok_frames_ = 0;
    bool maneuvering_now() const {
        return st_ == State::CHANGE_LEFT || st_ == State::PASS || st_ == State::CHANGE_RIGHT ||
               (st_ == State::LANE && settle_) ||
               ((st_ == State::BACK_OFF || st_ == State::FOLLOW) && anchor_hold_);
    }
    bool anchoring() const {
        return st_ == State::CHANGE_LEFT || st_ == State::PASS || st_ == State::CHANGE_RIGHT ||
               (st_ == State::LANE && settle_) ||
               ((st_ == State::BACK_OFF || st_ == State::FOLLOW) && anchor_hold_);
    }
    double width() const { return (width_est_ > 0) ? width_est_ : P.lane_width_m; }

    // Durante un cambio de carril el objetivo no salta de golpe al otro
    // carril: sigue una curva suave (coseno) a lo largo de change_length_m.
    //   r = 0 al iniciar, r = 1 al terminar
    //   desplazamiento respecto al carril de casa = from + (to - from)·r
    //   (from = donde estaba el carro al empezar, to = carril objetivo)
    int    man_dir_ = 0;               // +1 cambio a la izq., -1 a la der., 0 nada
    double man_start_travel_ = 0, man_from_ = 0, man_len_ = 1;
    double man_progress(double ahead_m) const {
        if (man_dir_ == 0) return 1.0;
        const double u = std::clamp((odo_.travel - man_start_travel_ + ahead_m) / man_len_, 0.0, 1.0);
        return 0.5 * (1.0 - std::cos(M_PI * u));
    }
    double maneuver_offset(double ahead_m) const {
        const double to = target_lane_ * width();
        if (man_dir_ == 0) return to;
        return man_from_ + (to - man_from_) * man_progress(ahead_m);
    }
    // Inicia la curva suave desde la posición lateral actual hacia target
    void start_maneuver(int target, int dir, double base_len) {
        const bool have = lane_valid_ && !center_w_.empty();
        man_from_ = have ? lateral_of(car_polyline(), {0.0, 0.0}) : target_lane_ * width();
        target_lane_ = target; man_dir_ = dir; man_start_travel_ = odo_.travel;
        const double frac = std::clamp(std::fabs(target * width() - man_from_) / width(), 0.5, 1.0);
        man_len_ = base_len * frac;
    }
    // Desplazamiento lateral del carril k (0 = el carril objetivo del carro)
    double lane_rel(int k) const { return (target_lane_ + k) * width(); }

    // Línea de casa + extensión con curvatura constante (para clasificar
    // obstáculos más allá de donde se ven las líneas, p. ej. dentro de una curva)
    std::vector<Vec2> extended_polyline() const
    {
        auto pl = car_polyline();
        if (pl.size() < 3) return pl;
        const double kappa = end_curvature(pl);
        const Vec2 e = pl.back(), p = pl[pl.size()-2];
        double x = e.x, y = e.y, h = std::atan2(e.y - p.y, e.x - p.x);
        for (double sa = 0.15; sa < P.depth_range_m; sa += 0.15) {
            h += kappa * 0.15; x += 0.15 * std::cos(h); y += 0.15 * std::sin(h);
            pl.push_back({x, y});
        }
        return pl;
    }

    // Distancia libre (desde la defensa) al obstáculo más cercano en un carril
    double nearest_gap(const std::vector<Obstacle>& obs, bool lane_ok, double lane_off) const
    {
        const double W = (width_est_ > 0) ? width_est_ : P.lane_width_m;
        const auto pl = lane_ok ? extended_polyline() : std::vector<Vec2>{};
        double best = INFINITY;
        for (const auto& o : obs) {
            if (o.x <= 0) continue;
            const double lat = lane_ok ? lateral_of(pl, {o.x, o.y}) : o.y;   // sin carril: recta
            if (std::fabs(lat - lane_off) < P.in_lane_frac * W)
                best = std::min(best, std::hypot(o.x, o.y) - P.car_front_m);
        }
        return best;
    }
    // Lo que hay justo enfrente de la defensa (pasillo del ancho del carro)
    double nearest_gap_corridor(const std::vector<Obstacle>& obs) const
    {
        double best = INFINITY;
        for (const auto& o : obs)
            if (o.x > 0 && std::fabs(o.y) < P.car_half_width_m + 0.03)
                best = std::min(best, o.x - P.car_front_m);
        return best;
    }

    // ¿Alcanza a librar el obstáculo si empieza el cambio a la izquierda AHORA?
    // Simula la curva suave desde la posición lateral actual y revisa la
    // esquina delantera derecha cuando la defensa llega al obstáculo.
    bool change_left_feasible(double gap, bool lane_ok) const
    {
        if (!std::isfinite(gap)) return true;
        const double W = width();
        double from = 0.0, psi0 = 0.0;
        if (lane_ok && !center_w_.empty()) {
            const auto pl = car_polyline();
            from = lateral_of(pl, {0.0, 0.0});
            const Vec2 tn = normal_near(pl, {0.0, 0.0});
            psi0 = -std::atan2(-tn.x, tn.y);                           // rumbo del carro respecto al carril (+ izq.)
        }
        const double need = 0.5 * W - P.car_half_width_m + P.feasible_margin_m;   // borde izq. del obstáculo + margen
        // Si el carro apunta hacia la derecha, primero hay que enderezarlo:
        // se pierde distancia y algo de posición lateral
        const double r_min = P.wheelbase_m / std::tan(P.max_steer_deg * M_PI / 180.0);
        // gap se mide al centro de lo que ve la cámara; si ve también el
        // costado del obstáculo, la cara trasera está más cerca: margen
        double g = gap - P.feasible_lag_m - P.gap_uncertainty_m;       // retraso de dirección/rumbo
        from += P.feasible_lag_m * std::sin(psi0);
        if (psi0 < 0) { g -= r_min * (-psi0); from -= r_min * (1 - std::cos(psi0)); }
        const double L = P.change_length_m * std::clamp(std::fabs(W - from) / W, 0.5, 1.0);
        if (g <= 0) return from - P.car_half_width_m >= need;
        const double u = std::clamp(g / L, 0.0, 1.0);
        const double lat = from + (W - from) * 0.5 * (1 - std::cos(M_PI * u));
        const double psi = std::atan((W - from) * M_PI / (2 * L) * std::sin(M_PI * u));
        const double corner = lat + P.car_front_m * std::sin(psi) - P.car_half_width_m * std::cos(psi);
        return corner >= need;
    }

    bool left_free(double gap_cur, double gap_left, double sonar_left) const {
        return gap_left > gap_cur + P.left_lane_check_m && sonar_left > P.side_sonar_min_free_m;
    }

    // ── Memoria de obstáculos (mundo de odometría) ───────────────────────
    struct MemObs { Vec2 w; double t_seen = 0; int hits = 0; double unseen_in_fov = 0; int no_echo = 0; };
    std::vector<MemObs> mem_;
    double mem_t_prev_ = -1;

    bool in_depth_fov(const Vec2& c) const {
        const double dx = c.x - P.depth_cam_x_m;
        const double half = 0.5 * P.depth_hfov_deg * M_PI / 180.0 - 0.08;
        return dx > 0.30 && dx < P.depth_range_m && std::fabs(std::atan2(c.y, dx)) < half;
    }
    void update_obstacle_memory(const Inputs& in)
    {
        const double dt = (mem_t_prev_ < 0) ? 0.0 : std::clamp(in.t - mem_t_prev_, 0.0, 0.2);
        mem_t_prev_ = in.t;
        std::vector<bool> matched(mem_.size(), false);
        if (in.objects_fresh && in.objects_new) {
            for (const auto& o : in.obstacles) {
                const Vec2 w = to_world({o.x, o.y});
                int best = -1; double bd = P.mem_match_m;
                for (size_t i = 0; i < mem_.size(); ++i) {
                    const double d = std::hypot(mem_[i].w.x - w.x, mem_[i].w.y - w.y);
                    if (d < bd) { bd = d; best = (int)i; }
                }
                if (best >= 0) {
                    auto& m = mem_[best];
                    // la medición más reciente (y más cercana) manda
                    m.w = { 0.4 * m.w.x + 0.6 * w.x, 0.4 * m.w.y + 0.6 * w.y };
                    m.t_seen = in.t; m.hits++; m.unseen_in_fov = 0; matched[best] = true;
                } else {
                    mem_.push_back({w, in.t, 1, 0});
                    matched.push_back(true);
                }
            }
        }
        // Olvidar: lo que quedó atrás, lo que debería verse y no se ve, lo viejo
        std::vector<MemObs> keep;
        for (size_t i = 0; i < mem_.size(); ++i) {
            auto m = mem_[i];
            const Vec2 c = to_car(m.w);
            if (!matched[i] && in_depth_fov(c)) m.unseen_in_fov += dt;
            const bool behind = c.x < -(P.car_rear_m + 0.5);
            const bool ghost  = m.unseen_in_fov > P.mem_unseen_s;
            const bool old    = in.t - m.t_seen > P.mem_max_s;
            if (!behind && !ghost && !old) keep.push_back(m);
        }
        mem_.swap(keep);
    }
    // Postes delgados (otras señales del cruce, vistas por detrás): la cámara
    // de profundidad los ve, pero el ultrasónico de enfrente no. Un carro sí
    // lo ve. Si un objeto queda justo enfrente, dentro del haz del sonar, y el
    // sonar dice varias veces que ahí no hay nada, se marca como "sin eco" y
    // deja de contar (si después el sonar sí lo ve, vuelve a contar).
    static constexpr int kNoEchoGhost = 3;
    void check_with_sonar(const Inputs& in)
    {
        unconfirmed_ = 0;
        if (P.sonar_confirm_m <= 0) return;
        const double sF = clean(in.sonar_front);
        const bool fresh = (in.t - sonar_check_t_) > 0.09;        // el sonar va a 10 Hz
        if (fresh) sonar_check_t_ = in.t;
        for (auto& m : mem_) {
            const Vec2 c = to_car(m.w);
            const double gap = c.x - P.car_front_m;
            const bool in_beam = gap > 0.20 && gap < P.sonar_confirm_m &&
                                 std::fabs(c.y) < P.car_half_width_m + 0.13 * gap;
            if (in_beam && fresh) {
                if (sF > gap + 0.35) m.no_echo++;
                else if (std::fabs(sF - gap) < 0.30) m.no_echo = 0;
            }
            unconfirmed_ += m.no_echo >= kNoEchoGhost;
        }
    }
    std::vector<Obstacle> remembered_obstacles() const
    {
        std::vector<Obstacle> v;
        for (const auto& m : mem_) {
            if (m.hits < P.mem_min_hits) continue;
            if (P.sonar_confirm_m > 0 && m.no_echo >= kNoEchoGhost) continue;   // poste: el sonar no lo ve
            if (is_sign(m.w)) continue;                 // es la señal de alto, no un obstáculo
            const Vec2 c = to_car(m.w);
            v.push_back({c.x, c.y});
        }
        return v;
    }

    // ── Obstáculo que se está rebasando (en el mundo de odometría) ───────
    bool   have_target_ = false;
    Vec2   target_far_w_;           // extremo delantero estimado del obstáculo
    double pass_start_x_ = 0, side_clear_travel_ = -1;
    bool   side_seen_ = false;

    void remember_obstacle_in_lane(const std::vector<Obstacle>& obs, bool lane_ok, int k)
    {
        // Guarda el obstáculo más LEJANO que siga en el carril k (para PASS:
        // si hay otro coche más adelante, hay que rebasarlo también)
        const double W = (width_est_ > 0) ? width_est_ : P.lane_width_m;
        const auto pl = lane_ok ? extended_polyline() : std::vector<Vec2>{};
        for (const auto& o : obs) {
            if (o.x <= 0) continue;
            const double lat = lane_ok ? lateral_of(pl, {o.x, o.y}) : o.y;
            if (std::fabs(lat - lane_rel(k)) >= P.in_lane_frac * W) continue;
            if (std::hypot(o.x, o.y) - P.car_front_m > P.detect_range_m) continue;
            // extremo delantero: cara visible + largo, en dirección del carril
            Vec2 dir{1, 0};
            if (lane_ok && pl.size() >= 2) { const Vec2 n = normal_near(pl, {o.x, o.y}); dir = {n.y, -n.x}; }
            const Vec2 far_c{ o.x + dir.x * P.obstacle_length_m, o.y + dir.y * P.obstacle_length_m };
            const Vec2 far_w = to_world(far_c);
            if (!have_target_ || to_car(far_w).x > to_car(target_far_w_).x) {
                target_far_w_ = far_w; have_target_ = true;
            }
        }
    }
    bool obstacle_end_behind() const {
        if (!have_target_) return odo_.travel - pass_start_x_ > 1.2;   // sin dato: distancia fija
        return to_car(target_far_w_).x < -(P.car_rear_m + P.odom_pass_margin_m);
    }

    // Punto a "s" metros (a lo largo del carril) desde el carro, desplazado
    // lane_off a la izquierda de la línea de casa. Más allá del modelo se
    // extiende con la curvatura del último tramo.
    Vec2 lane_point(double s_along, double lane_off) const
    {
        const auto pl = car_polyline();
        if (pl.size() < 2) return { s_along, lane_off };
        size_t i0 = 0; double bd = 1e18;                         // punto más cercano al carro
        for (size_t i = 0; i < pl.size(); ++i) { const double d = std::hypot(pl[i].x, pl[i].y); if (d < bd) { bd = d; i0 = i; } }
        double acc = 0;
        auto at = [&](const Vec2& a, const Vec2& b, double u) {
            const double sx = b.x - a.x, sy = b.y - a.y, n = std::hypot(sx, sy);
            return Vec2{ a.x + u*sx - lane_off*sy/n, a.y + u*sy + lane_off*sx/n }; };
        for (size_t i = i0 + 1; i < pl.size(); ++i) {
            const double L = std::hypot(pl[i].x - pl[i-1].x, pl[i].y - pl[i-1].y);
            if (L < 1e-9) continue;
            if (acc + L >= s_along) return at(pl[i-1], pl[i], (s_along - acc) / L);
            acc += L;
        }
        // extensión con curvatura constante (estimada de los últimos ~40 cm)
        const Vec2 e = pl.back(), pe = pl[pl.size()-2];
        const double kappa = end_curvature(pl);
        double x = e.x, y = e.y, h = std::atan2(e.y - pe.y, e.x - pe.x);
        for (double r = s_along - acc; r > 0; r -= 0.05) {
            const double ds = std::min(0.05, r);
            h += kappa * ds; x += ds * std::cos(h); y += ds * std::sin(h);
        }
        return { x - lane_off * std::sin(h), y + lane_off * std::cos(h) };
    }
    // Curvatura del último tramo (~40 cm) de una polilínea, acotada
    static double end_curvature(const std::vector<Vec2>& pl)
    {
        if (pl.size() < 3) return 0.0;
        size_t j = pl.size() - 1; double back = 0;
        while (j > 0 && back < 0.4) { back += std::hypot(pl[j].x - pl[j-1].x, pl[j].y - pl[j-1].y); --j; }
        const size_t m = (j + pl.size() - 1) / 2;
        if (m == j || m == pl.size() - 1 || back < 0.05) return 0.0;
        const Vec2 e = pl.back();
        const double h1 = std::atan2(pl[m].y - pl[j].y, pl[m].x - pl[j].x);
        const double h2 = std::atan2(e.y - pl[m].y, e.x - pl[m].x);
        double dh = h2 - h1; while (dh > M_PI) dh -= 2*M_PI; while (dh < -M_PI) dh += 2*M_PI;
        return std::clamp(2.0 * dh / back, -0.85, 0.85);
    }

    // Curvatura del carril de casa junto al carro (1/m, + = curva a la izquierda)
    double lane_curvature_near() const
    {
        if (center_w_.empty()) return 0.0;
        const auto pl = extended_polyline();
        bool a, b;
        const double h1 = heading_between(pl, 0.0, 0.4, a), h2 = heading_between(pl, 0.4, 0.8, b);
        if (!a || !b) return 0.0;
        double dh = h2 - h1; while (dh > M_PI) dh -= 2*M_PI; while (dh < -M_PI) dh += 2*M_PI;
        return dh / 0.4;
    }

    // ¿La cámara de profundidad alcanza a ver ese tramo del carril?
    bool lane_visible(double lane_off, double s0, double s1) const
    {
        if (center_w_.empty()) return false;
        const double half = 0.5 * P.depth_hfov_deg * M_PI / 180.0 - 0.10;   // margen ~6°
        for (double sa = s0; sa <= s1 + 1e-9; sa += 0.2) {
            const Vec2 q = lane_point(sa, lane_off);
            const double dx = q.x - P.depth_cam_x_m;
            if (dx <= 0.05 || std::fabs(std::atan2(q.y, dx)) > half || std::hypot(dx, q.y) > P.depth_range_m) return false;
        }
        return true;
    }
    // Dirección en reversa: seguir la curvatura del carril y corregir el rumbo
    double reverse_steer(bool lane_ok) const
    {
        if (!lane_ok || center_w_.empty()) return P.steer_center_cmd;
        const Vec2 a = lane_point(-0.15, 0.0), b = lane_point(0.15, 0.0), c = lane_point(0.45, 0.0);
        const double h1 = std::atan2(b.y - a.y, b.x - a.x), h2 = std::atan2(c.y - b.y, c.x - b.x);
        const double kappa = (h2 - h1) / 0.30;
        const double head = std::atan2(b.y - a.y, b.x - a.x);     // rumbo del carril respecto al carro
        // en reversa, girar hacia el lado del carril alinea la parte de atrás
        const double delta = std::atan(P.wheelbase_m * kappa) - 0.6 * head;
        return rad_to_cmd(delta);
    }

    // ── Control ──────────────────────────────────────────────────────────
    void lane_keep(const Inputs& in, bool lane_ok, double& steer, int& speed)
    {
        if (P.lane_keep_mode == 1 && lane_ok) {  // pure pursuit sobre la geometría del carril
            pursue(lane_ok, steer);
            const double e = std::fabs(steer - P.steer_center_cmd);
            speed = -std::max(P.approach_cmd, (int)(P.cruise_cmd - P.kp_speed * 0.5 * e));
        } else if (in.has_dist) {                // tu on_lane()
            const double e = -(double)in.dist_px, ae = std::fabs(e);
            double kp = P.lk_kp[4];
            for (int i = 0; i < 4; ++i) if (ae < P.lk_thr[i]) { kp = P.lk_kp[i]; break; }
            steer = std::clamp(P.steer_center_cmd + kp * e, P.lk_min_cmd, P.lk_max_cmd);
            const int u = (int)(P.kp_speed * ae);
            speed = std::max(-P.cruise_cmd, std::min(0, -P.cruise_cmd + u));
        } else if (lane_ok) {                    // respaldo: pure pursuit
            pursue(lane_ok, steer);
            speed = -P.approach_cmd;
        } else {
            steer = P.steer_center_cmd;
            speed = -P.approach_cmd / 2;
        }
    }

    void pursue(bool lane_ok, double& steer)
    {
        if (!lane_ok && center_w_.empty()) { steer = P.steer_center_cmd; return; }
        // Con la curvatura del último tramo extendida hacia adelante: si las
        // líneas se pierden a media curva, el carro sigue la curva y no se va recto
        const auto pl = extended_polyline();
        // Trayectoria deseada = línea central de casa desplazada (carril
        // objetivo, o la curva suave del cambio de carril). La distancia de
        // mirada se mide sobre ESTA trayectoria, no sobre la línea de casa.
        std::vector<Vec2> path; path.reserve(pl.size());
        for (size_t i = 0; i < pl.size(); ++i) {
            const size_t a = (i == 0) ? 0 : i - 1, b = (i == 0) ? std::min<size_t>(1, pl.size()-1) : i;
            double sx = pl[b].x - pl[a].x, sy = pl[b].y - pl[a].y; const double nn = std::hypot(sx, sy);
            if (nn < 1e-9) { sx = 1; sy = 0; } else { sx /= nn; sy /= nn; }
            const double off = maneuver_offset(std::max(0.0, pl[i].x));
            path.push_back({ pl[i].x - sy * off, pl[i].y + sx * off });
        }
        const Vec2 tgt = point_at(path, P.lookahead_m);
        // re-proyectar el objetivo a la distancia de mirada
        const double L = std::max(0.2, std::hypot(tgt.x, tgt.y));
        const double kappa = 2.0 * tgt.y / (L * L);
        // límite de ángulo en maniobra: si el modelo está mal, un volantazo
        // a tope saca al carro de la pista antes de que se corrija
        const double lim = P.maneuver_max_steer_deg * M_PI / 180.0;
        steer = rad_to_cmd(std::clamp(std::atan(P.wheelbase_m * kappa), -lim, lim));
        last_target_ = tgt;
    }
    Vec2 last_target_;

    int done_frames_ = 0;
    bool changed_lane(bool lane_ok, double /*dt*/)
    {
        if (!lane_ok) { done_frames_ = 0; return false; }
        const auto pl = car_polyline();
        const double lat_err = lateral_of(pl, {0.0, 0.0}) - target_lane_ * width();   // respecto al carril objetivo
        const Vec2 tn = normal_near(pl, {0.0, 0.0});                   // normal del carril junto al carro
        const double head = std::atan2(-tn.x, tn.y);                   // rumbo del carril respecto al carro
        const bool ok = man_progress(0.0) > 0.97 &&
                        std::fabs(lat_err) < P.change_done_lat_m + 0.04 &&
                        std::fabs(head) < P.change_done_head;
        done_frames_ = ok ? done_frames_ + 1 : 0;
        return done_frames_ >= P.change_done_frames;
    }

    void start_change_left(const Inputs& in, const std::vector<Obstacle>& obs, Outputs& out,
                           const char* why = "inicio de rebase")
    {
        start_maneuver(1, +1, P.change_length_m);
        settle_ = false; anchor_hold_ = false; done_frames_ = 0; have_target_ = false;
        side_seen_ = false; side_clear_travel_ = -1;
        const bool lane_ok = lane_valid_ && (in.t - lane_t_) < P.lane_timeout_s;
        remember_obstacle_in_lane(obs, lane_ok, -1);      // el que vamos a rebasar
        go(State::CHANGE_LEFT, in.t, out, why);
    }
    void start_change_right(const Inputs& in, Outputs& out, const char* why)
    {
        start_maneuver(0, -1, P.return_length_m);
        done_frames_ = 0; have_target_ = false;
        go(State::CHANGE_RIGHT, in.t, out, why);
    }

    void go(State s, double t, Outputs& out, const char* why)
    {
        if (s != State::CHANGE_LEFT && s != State::CHANGE_RIGHT) man_dir_ = 0;
        char buf[160];
        std::snprintf(buf, sizeof buf, "%s -> %s (%s)", state_name(st_), state_name(s), why);
        out.event = buf;
        st_ = s; t_enter_ = t;
    }
};

}  // namespace avim
