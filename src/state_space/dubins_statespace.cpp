#include "motion_planning/state_space/dubins_statespace.hpp"
#include "motion_planning/state_space/euclidean_state.hpp"
#include <cmath>
#include <vector>
#include <string>
#include <algorithm>
#include <iostream>
#include <iomanip>  // at top

double normalizeAngle(double a) {
    // Pull into (-PI, +PI]
    while (a <= -M_PI) a += 2.0 * M_PI;
    while (a >   M_PI) a -= 2.0 * M_PI;
    return a;
}

DubinsStateSpace::DubinsStateSpace(double min_turning_radius, int dimension, unsigned int seed)
    : StateSpace(dimension), // Pass the dimension through
      min_turning_radius_(min_turning_radius) {

    std::srand(seed); // TODO: For sampling the same batch every time just for debug and test. --> remove it later.
    weights_.resize(3);
    weights_ << 1.0, 1.0, 0.4; // These weights are still for the 3D case
}

// Approximate distance for the KD-Tree (fast neighbor search)
double DubinsStateSpace::distance(const std::shared_ptr<State>& state1, const std::shared_ptr<State>& state2) const {
    Eigen::VectorXd diff = state1->getValue() - state2->getValue();
    diff(2) = normalizeAngle(diff(2)); // Normalize the angle difference
    return diff.cwiseProduct(weights_).norm();
}



// // Steer with debug info
// Trajectory DubinsStateSpace::steer(const Eigen::VectorXd& from,
//                                      const Eigen::VectorXd& to) const
// {
//     const double r = min_turning_radius_;
//     if (r <= 0) {
//          std::cerr << "[ERR] Turning radius must be positive." << std::endl;
//          return Trajectory{false, 0, 0, {}};
//     }

//     // 1) Extract states and normalize headings
//     double th0 = normalizeAngle(from[2]);
//     double th1 = normalizeAngle(to[2]);
//     Eigen::Vector2d p0(from[0], from[1]);
//     Eigen::Vector2d p1(to[0],   to[1]);

//     std::cout << "\n--- Dubins Steer Calculation ---\n";
//     std::cout << std::fixed << std::setprecision(6);
//     std::cout << "From: (" << p0.x() << ", " << p0.y() << ", " << th0 << ")\n";
//     std::cout << "To:   (" << p1.x() << ", " << p1.y() << ", " << th1 << ")\n";
//     std::cout << "Turning Radius r = " << r << "\n";

//     // 2) Define lambdas for calculating circle centers with deep debugging
//     auto left_center  = [&](const Eigen::Vector2d& p, double th){
//         std::cout << "  [LC DBG] Computing Left Center for p=(" << p.x() << "," << p.y() << "), th=" << th << "\n";
//         double angle = th + M_PI/2.0;
//         double cos_a = cos(angle);
//         double sin_a = sin(angle);
//         Eigen::Vector2d dir_vec(cos_a, sin_a);
//         Eigen::Vector2d offset = r * dir_vec;
//         Eigen::Vector2d result = p + offset;
//         std::cout << "  [LC DBG] -> angle=" << angle << ", dir_vec=(" << dir_vec.x() << "," << dir_vec.y() << ")\n";
//         std::cout << "  [LC DBG] -> r=" << r << ", offset=(" << offset.x() << "," << offset.y() << ")\n";
//         std::cout << "  [LC DBG] -> result = p + offset = (" << result.x() << "," << result.y() << ")\n";
//         return result;
//     };
//     auto right_center = [&](const Eigen::Vector2d& p, double th){
//         std::cout << "  [RC DBG] Computing Right Center for p=(" << p.x() << "," << p.y() << "), th=" << th << "\n";
//         double angle = th - M_PI/2.0;
//         double cos_a = cos(angle);
//         double sin_a = sin(angle);
//         Eigen::Vector2d dir_vec(cos_a, sin_a);
//         Eigen::Vector2d offset = r * dir_vec;
//         Eigen::Vector2d result = p + offset;
//         std::cout << "  [RC DBG] -> angle=" << angle << ", dir_vec=(" << dir_vec.x() << "," << dir_vec.y() << ")\n";
//         std::cout << "  [RC DBG] -> r=" << r << ", offset=(" << offset.x() << "," << offset.y() << ")\n";
//         std::cout << "  [RC DBG] -> result = p + offset = (" << result.x() << "," << result.y() << ")\n";
//         return result;
//     };

//     // 3) Calculate the four possible circle centers
//     Eigen::Vector2d C0L = left_center(p0, th0);
//     Eigen::Vector2d C0R = right_center(p0, th0);
//     Eigen::Vector2d C1L = left_center(p1, th1);
//     Eigen::Vector2d C1R = right_center(p1, th1);

//     // 4) Log the calculated centers for debugging
//     std::cout << "\n--- Circle Center Calculations ---\n";
//     std::cout << "Start Ctrs: C0L=(" << C0L.x() << "," << C0L.y() << "), C0R=(" << C0R.x() << "," << C0R.y() << ")\n";
//     std::cout << "Goal Ctrs:  C1L=(" << C1L.x() << "," << C1L.y() << "), C1R=(" << C1R.x() << "," << C1R.y() << ")\n";
//     std::cout << "Goal Deltas: ΔC1L=(" << (C1L.x() - p1.x()) << "," << (C1L.y() - p1.y()) << "), ΔC1R=(" << (C1R.x() - p1.x()) << "," << (C1R.y() - p1.y()) << ")\n";


//     // 5) arc length helper
//     auto arc_len = [&](const Eigen::Vector2d& A,
//                        const Eigen::Vector2d& B,
//                        const Eigen::Vector2d& C,
//                        bool clockwise)
//     {
//         double alpha = atan2(A.y() - C.y(), A.x() - C.x());
//         double beta  = atan2(B.y() - C.y(), B.x() - C.x());
//         double d_th  = normalizeAngle(beta - alpha);
//         if (clockwise && d_th > 0)    d_th -= 2*M_PI;
//         if (!clockwise && d_th < 0)   d_th += 2*M_PI;
//         return std::abs(d_th) * r;
//     };

//     struct Maneuver { double cost; std::string type; std::vector<Eigen::Vector2d> pts; };
//     std::vector<Maneuver> all_maneuvers;

//     std::cout << "\n--- Evaluating 6 Maneuvers ---\n";

//     // --- RSR ---
//     {
//         double D = (C1R - C0R).norm();
//         Eigen::Vector2d v = (C1R - C0R)/D;
//         Eigen::Vector2d T0 = C0R + Eigen::Vector2d(-v.y(), v.x()) * r;
//         Eigen::Vector2d T1 = C1R + Eigen::Vector2d(-v.y(), v.x()) * r;

//         double arc0     = arc_len(p0, T0, C0R, /*cw=*/true);
//         double straight = (T1 - T0).norm();
//         double arc1     = arc_len(T1, p1, C1R, /*cw=*/true);
//         double cost     = arc0 + straight + arc1;

//         std::cout << "  RSR: D=" << D << " arc0=" << arc0 << " str=" << straight << " arc1=" << arc1 << " total=" << cost << "\n";
//         all_maneuvers.push_back({cost, "RSR", {T0, T1}});
//     }

//     // --- LSL ---
//     {
//         double D = (C1L - C0L).norm();
//         Eigen::Vector2d v = (C1L - C0L)/D;
//         Eigen::Vector2d T0 = C0L + Eigen::Vector2d(v.y(), -v.x()) * r;
//         Eigen::Vector2d T1 = C1L + Eigen::Vector2d(v.y(), -v.x()) * r;

//         double arc0     = arc_len(p0, T0, C0L, /*cw=*/false);
//         double straight = (T1 - T0).norm();
//         double arc1     = arc_len(T1, p1, C1L, /*cw=*/false);
//         double cost     = arc0 + straight + arc1;

//         std::cout << "  LSL: D=" << D << " arc0=" << arc0 << " str=" << straight << " arc1=" << arc1 << " total=" << cost << "\n";
//         all_maneuvers.push_back({cost, "LSL", {T0, T1}});
//     }

//     // --- RSL ---
//     {
//         double D = (C1L - C0R).norm();
//         if (D >= 2*r) {
//             double theta = atan2(C1L.y()-C0R.y(), C1L.x()-C0R.x());
//             double phi   = acos(std::clamp(2*r/D, -1.0, 1.0));
//             Eigen::Vector2d T0 = C0R + Eigen::Vector2d(cos(theta+phi), sin(theta+phi)) * r;
//             Eigen::Vector2d T1 = C1L + Eigen::Vector2d(cos(theta+phi-M_PI), sin(theta+phi-M_PI)) * r;

//             double arc0     = arc_len(p0, T0, C0R, /*cw=*/true);
//             double straight = (T1 - T0).norm();
//             double arc1     = arc_len(T1, p1, C1L, /*cw=*/false);
//             double cost     = arc0 + straight + arc1;

//             std::cout << "  RSL: D=" << D << " arc0=" << arc0 << " str=" << straight << " arc1=" << arc1 << " total=" << cost << "\n";
//             all_maneuvers.push_back({cost, "RSL", {T0, T1}});
//         } else {
//             std::cout << "  RSL: Not possible (D=" << D << " < 2r)\n";
//         }
//     }

//     // --- LSR ---
//     {
//         double D = (C1R - C0L).norm();
//         if (D >= 2*r) {
//             double theta = atan2(C1R.y()-C0L.y(), C1R.x()-C0L.x());
//             double phi   = acos(std::clamp(2*r/D, -1.0, 1.0));
//             Eigen::Vector2d T0 = C0L + Eigen::Vector2d(cos(theta-phi), sin(theta-phi)) * r;
//             Eigen::Vector2d T1 = C1R + Eigen::Vector2d(cos(theta-phi+M_PI), sin(theta-phi+M_PI)) * r;

//             double arc0     = arc_len(p0, T0, C0L, /*cw=*/false);
//             double straight = (T1 - T0).norm();
//             double arc1     = arc_len(T1, p1, C1R, /*cw=*/true);
//             double cost     = arc0 + straight + arc1;

//             std::cout << "  LSR: D=" << D << " arc0=" << arc0 << " str=" << straight << " arc1=" << arc1 << " total=" << cost << "\n";
//             all_maneuvers.push_back({cost, "LSR", {T0, T1}});
//         } else {
//             std::cout << "  LSR: Not possible (D=" << D << " < 2r)\n";
//         }
//     }

//     // --- RLR ---
//     {
//         double D = (C1R - C0R).norm();
//         if (D < 4*r) {
//             double theta = atan2(C1R.y()-C0R.y(), C1R.x()-C0R.x());
//             double phi   = acos(std::clamp(D/(4*r), -1.0, 1.0));
//             Eigen::Vector2d C_aux = C0R + Eigen::Vector2d(cos(theta+phi), sin(theta+phi)) * 2*r;
//             Eigen::Vector2d T0    = (C0R + C_aux)*0.5;
//             Eigen::Vector2d T1    = (C1R + C_aux)*0.5;

//             double arc0   = arc_len(p0,  T0,   C0R,   /*cw=*/true);
//             double arcMid = arc_len(T0,  T1,   C_aux, /*cw=*/false);
//             double arc1   = arc_len(T1,  p1,   C1R,   /*cw=*/true);
//             double cost   = arc0 + arcMid + arc1;

//             std::cout << "  RLR: D=" << D << " total=" << cost << "\n";
//             all_maneuvers.push_back({cost, "RLR", {T0, T1, C_aux}});
//         } else {
//             std::cout << "  RLR: Not possible (D=" << D << " >= 4r)\n";
//         }
//     }

//     // --- LRL ---
//     {
//         double D = (C1L - C0L).norm();
//         if (D < 4*r) {
//             double theta = atan2(C1L.y()-C0L.y(), C1L.x()-C0L.x());
//             double phi   = acos(std::clamp(D/(4*r), -1.0, 1.0));
//             Eigen::Vector2d C_aux = C0L + Eigen::Vector2d(cos(theta-phi), sin(theta-phi)) * 2*r;
//             Eigen::Vector2d T0    = (C0L + C_aux)*0.5;
//             Eigen::Vector2d T1    = (C1L + C_aux)*0.5;

//             double arc0   = arc_len(p0,  T0,   C0L,   /*cw=*/false);
//             double arcMid = arc_len(T0,  T1,   C_aux, /*cw=*/true);
//             double arc1   = arc_len(T1,  p1,   C1L,   /*cw=*/false);
//             double cost   = arc0 + arcMid + arc1;
            
//             std::cout << "  LRL: D=" << D << " total=" << cost << "\n";
//             all_maneuvers.push_back({cost, "LRL", {T0, T1, C_aux}});
//         } else {
//             std::cout << "  LRL: Not possible (D=" << D << " >= 4r)\n";
//         }
//     }

//     // 6) Pick the best maneuver
//     if (all_maneuvers.empty()) {
//         std::cout << "[WARN] No valid Dubins maneuvers found.\n";
//         return Trajectory{false, std::numeric_limits<double>::infinity(), std::numeric_limits<double>::infinity(), {}};
//     }
//     auto best = std::min_element(
//         all_maneuvers.begin(), all_maneuvers.end(),
//         [](const auto& a, const auto& b){ return a.cost < b.cost; }
//     );
//     if (std::isinf(best->cost)) {
//         std::cout << "[WARN] Best Dubins maneuver has infinite cost.\n";
//         return Trajectory{false, std::numeric_limits<double>::infinity(), std::numeric_limits<double>::infinity(), {}};
//     }

//     // 7) Log chosen maneuver and discretize path
//     std::cout << "\n--- CHOSEN MANEUVER ---\n"
//               << "  Type: " << best->type
//               << "   Cost: " << best->cost << "\n";
    
//     Trajectory out;
//     out.is_valid = true;
//     out.cost     = best->cost;
//     out.geometric_distance = out.cost
    
//     const double discretization_step = 0.5; // meters

//     auto sample_arc = [&](const Eigen::Vector2d& A, const Eigen::Vector2d& B, const Eigen::Vector2d& C, bool clockwise) {
//         double len = arc_len(A, B, C, clockwise);
//         int N = std::max(1, static_cast<int>(ceil(len / discretization_step)));
//         double alpha = atan2(A.y()-C.y(), A.x()-C.x());
//         double d_th_total = normalizeAngle(atan2(B.y()-C.y(), B.x()-C.x()) - alpha);
//         if (clockwise && d_th_total > 0)  d_th_total -= 2 * M_PI;
//         if (!clockwise && d_th_total < 0) d_th_total += 2 * M_PI;

//         for(int i = 1; i <= N; i++){
//             double phi = alpha + d_th_total * (static_cast<double>(i) / N);
//             Eigen::VectorXd pt(3);
//             pt << C.x() + r * cos(phi),
//                   C.y() + r * sin(phi),
//                   normalizeAngle(phi + (clockwise ? -M_PI/2.0 : +M_PI/2.0));
//             out.path_points.push_back(pt);
//         }
//     };

//     auto sample_straight = [&](const Eigen::Vector2d& A, const Eigen::Vector2d& B) {
//         double L = (B - A).norm();
//         int N = std::max(1, static_cast<int>(ceil(L / discretization_step)));
//         double heading = atan2(B.y() - A.y(), B.x() - A.x());
//         for(int i = 1; i <= N; i++){
//             Eigen::Vector2d P_interp = A + (B-A) * (static_cast<double>(i) / N);
//             Eigen::VectorXd pt(3);
//             pt << P_interp.x(), P_interp.y(), normalizeAngle(heading);
//             out.path_points.push_back(pt);
//         }
//     };

//     out.path_points.push_back(from);
//     const auto& P = best->pts;
//     if      (best->type == "RSR") { sample_arc(p0,P[0],C0R,true);  sample_straight(P[0],P[1]); sample_arc(P[1],p1,C1R,true);  }
//     else if (best->type == "LSL") { sample_arc(p0,P[0],C0L,false); sample_straight(P[0],P[1]); sample_arc(P[1],p1,C1L,false); }
//     else if (best->type == "RSL") { sample_arc(p0,P[0],C0R,true);  sample_straight(P[0],P[1]); sample_arc(P[1],p1,C1L,false); }
//     else if (best->type == "LSR") { sample_arc(p0,P[0],C0L,false); sample_straight(P[0],P[1]); sample_arc(P[1],p1,C1R,true);  }
//     else if (best->type == "RLR") { sample_arc(p0,P[0],C0R,true);  sample_arc(P[0],P[1],P[2],false); sample_arc(P[1],p1,C1R,true);  }
//     else if (best->type == "LRL") { sample_arc(p0,P[0],C0L,false); sample_arc(P[0],P[1],P[2],true);  sample_arc(P[1],p1,C1L,false); }
    
//     return out;
// }

// // Steer without debug info --> i think this is redundant
// Trajectory DubinsStateSpace::steer(const Eigen::VectorXd& from,
//                                      const Eigen::VectorXd& to) const
// {
//     const double r = min_turning_radius_;
//     if (r <= 0) {
//          std::cerr << "[ERR] Turning radius must be positive." << std::endl;
//          return Trajectory{false, 0, 0, {}};
//     }

//     // 1) Extract states and normalize headings
//     double th0 = normalizeAngle(from[2]);
//     double th1 = normalizeAngle(to[2]);
//     Eigen::Vector2d p0(from[0], from[1]);
//     Eigen::Vector2d p1(to[0],   to[1]);

//     // 2) Define lambdas for calculating circle centers
//     auto left_center  = [&](const Eigen::Vector2d& p, double th){
//         double angle = th + M_PI/2.0;
//         return p + Eigen::Vector2d(cos(angle), sin(angle)) * r;
//     };
//     auto right_center = [&](const Eigen::Vector2d& p, double th){
//         double angle = th - M_PI/2.0;
//         return p + Eigen::Vector2d(cos(angle), sin(angle)) * r;
//     };

//     // 3) Calculate the four possible circle centers
//     Eigen::Vector2d C0L = left_center(p0, th0);
//     Eigen::Vector2d C0R = right_center(p0, th0);
//     Eigen::Vector2d C1L = left_center(p1, th1);
//     Eigen::Vector2d C1R = right_center(p1, th1);

//     // 4) Arc length helper
//     auto arc_len = [&](const Eigen::Vector2d& A,
//                        const Eigen::Vector2d& B,
//                        const Eigen::Vector2d& C,
//                        bool clockwise)
//     {
//         double alpha = atan2(A.y() - C.y(), A.x() - C.x());
//         double beta  = atan2(B.y() - C.y(), B.x() - C.x());
//         double d_th  = normalizeAngle(beta - alpha);
//         if (clockwise && d_th > 0)    d_th -= 2*M_PI;
//         if (!clockwise && d_th < 0)   d_th += 2*M_PI;
//         return std::abs(d_th) * r;
//     };

//     struct Maneuver { double cost; std::string type; std::vector<Eigen::Vector2d> pts; };
//     std::vector<Maneuver> all_maneuvers;

//     // --- RSR ---
//     {
//         double D = (C1R - C0R).norm();
//         Eigen::Vector2d v = (C1R - C0R)/D;
//         Eigen::Vector2d T0 = C0R + Eigen::Vector2d(-v.y(), v.x()) * r;
//         Eigen::Vector2d T1 = C1R + Eigen::Vector2d(-v.y(), v.x()) * r;

//         double arc0     = arc_len(p0, T0, C0R, true);
//         double straight = (T1 - T0).norm();
//         double arc1     = arc_len(T1, p1, C1R, true);
//         all_maneuvers.push_back({arc0 + straight + arc1, "RSR", {T0, T1}});
//     }

//     // --- LSL ---
//     {
//         double D = (C1L - C0L).norm();
//         Eigen::Vector2d v = (C1L - C0L)/D;
//         Eigen::Vector2d T0 = C0L + Eigen::Vector2d(v.y(), -v.x()) * r;
//         Eigen::Vector2d T1 = C1L + Eigen::Vector2d(v.y(), -v.x()) * r;

//         double arc0     = arc_len(p0, T0, C0L, false);
//         double straight = (T1 - T0).norm();
//         double arc1     = arc_len(T1, p1, C1L, false);
//         all_maneuvers.push_back({arc0 + straight + arc1, "LSL", {T0, T1}});
//     }

//     // --- RSL ---
//     {
//         double D = (C1L - C0R).norm();
//         if (D >= 2*r) {
//             double theta = atan2(C1L.y()-C0R.y(), C1L.x()-C0R.x());
//             double phi   = acos(std::clamp(2*r/D, -1.0, 1.0));
//             Eigen::Vector2d T0 = C0R + Eigen::Vector2d(cos(theta+phi), sin(theta+phi)) * r;
//             Eigen::Vector2d T1 = C1L + Eigen::Vector2d(cos(theta+phi-M_PI), sin(theta+phi-M_PI)) * r;

//             double arc0     = arc_len(p0, T0, C0R, true);
//             double straight = (T1 - T0).norm();
//             double arc1     = arc_len(T1, p1, C1L, false);
//             all_maneuvers.push_back({arc0 + straight + arc1, "RSL", {T0, T1}});
//         }
//     }

//     // --- LSR ---
//     {
//         double D = (C1R - C0L).norm();
//         if (D >= 2*r) {
//             double theta = atan2(C1R.y()-C0L.y(), C1R.x()-C0L.x());
//             double phi   = acos(std::clamp(2*r/D, -1.0, 1.0));
//             Eigen::Vector2d T0 = C0L + Eigen::Vector2d(cos(theta-phi), sin(theta-phi)) * r;
//             Eigen::Vector2d T1 = C1R + Eigen::Vector2d(cos(theta-phi+M_PI), sin(theta-phi+M_PI)) * r;

//             double arc0     = arc_len(p0, T0, C0L, false);
//             double straight = (T1 - T0).norm();
//             double arc1     = arc_len(T1, p1, C1R, true);
//             all_maneuvers.push_back({arc0 + straight + arc1, "LSR", {T0, T1}});
//         }
//     }

//     // --- RLR ---
//     {
//         double D = (C1R - C0R).norm();
//         if (D < 4*r) {
//             double theta = atan2(C1R.y()-C0R.y(), C1R.x()-C0R.x());
//             double phi   = acos(std::clamp(D/(4*r), -1.0, 1.0));
//             Eigen::Vector2d C_aux = C0R + Eigen::Vector2d(cos(theta+phi), sin(theta+phi)) * 2*r;
//             Eigen::Vector2d T0    = (C0R + C_aux)*0.5;
//             Eigen::Vector2d T1    = (C1R + C_aux)*0.5;

//             double arc0   = arc_len(p0, T0, C0R, true);
//             double arcMid = arc_len(T0, T1, C_aux, false);
//             double arc1   = arc_len(T1, p1, C1R, true);
//             all_maneuvers.push_back({arc0 + arcMid + arc1, "RLR", {T0, T1, C_aux}});
//         }
//     }

//     // --- LRL ---
//     {
//         double D = (C1L - C0L).norm();
//         if (D < 4*r) {
//             double theta = atan2(C1L.y()-C0L.y(), C1L.x()-C0L.x());
//             double phi   = acos(std::clamp(D/(4*r), -1.0, 1.0));
//             Eigen::Vector2d C_aux = C0L + Eigen::Vector2d(cos(theta-phi), sin(theta-phi)) * 2*r;
//             Eigen::Vector2d T0    = (C0L + C_aux)*0.5;
//             Eigen::Vector2d T1    = (C1L + C_aux)*0.5;

//             double arc0   = arc_len(p0, T0, C0L, false);
//             double arcMid = arc_len(T0, T1, C_aux, true);
//             double arc1   = arc_len(T1, p1, C1L, false);
//             all_maneuvers.push_back({arc0 + arcMid + arc1, "LRL", {T0, T1, C_aux}});
//         }
//     }

//     // 5) Pick the best maneuver
//     if (all_maneuvers.empty()) {
//         return Trajectory{false, std::numeric_limits<double>::infinity(), std::numeric_limits<double>::infinity(), {}};
//     }
//     auto best = std::min_element(
//         all_maneuvers.begin(), all_maneuvers.end(),
//         [](const auto& a, const auto& b){ return a.cost < b.cost; }
//     );

//     // 6) Discretize path
//     Trajectory out;
//     out.is_valid = true;
//     out.cost     = best->cost;
    
//     const double discretization_step = 0.5; // meters

//     auto sample_arc = [&](const Eigen::Vector2d& A, const Eigen::Vector2d& B, const Eigen::Vector2d& C, bool clockwise) {
//         double len = arc_len(A, B, C, clockwise);
//         int N = std::max(1, static_cast<int>(ceil(len / discretization_step)));
//         double alpha = atan2(A.y()-C.y(), A.x()-C.x());
//         double d_th_total = normalizeAngle(atan2(B.y()-C.y(), B.x()-C.x()) - alpha);
//         if (clockwise && d_th_total > 0)  d_th_total -= 2 * M_PI;
//         if (!clockwise && d_th_total < 0) d_th_total += 2 * M_PI;

//         for(int i = 1; i <= N; i++){
//             double phi = alpha + d_th_total * (static_cast<double>(i) / N);
//             Eigen::VectorXd pt(3);
//             pt << C.x() + r * cos(phi),
//                   C.y() + r * sin(phi),
//                   normalizeAngle(phi + (clockwise ? -M_PI/2.0 : +M_PI/2.0));
//             out.path_points.push_back(pt);
//         }
//     };

//     auto sample_straight = [&](const Eigen::Vector2d& A, const Eigen::Vector2d& B) {
//         double L = (B - A).norm();
//         int N = std::max(1, static_cast<int>(ceil(L / discretization_step)));
//         double heading = atan2(B.y() - A.y(), B.x() - A.x());
//         for(int i = 1; i <= N; i++){
//             Eigen::Vector2d P_interp = A + (B-A) * (static_cast<double>(i) / N);
//             Eigen::VectorXd pt(3);
//             pt << P_interp.x(), P_interp.y(), heading;
//             out.path_points.push_back(pt);
//         }
//     };

//     out.path_points.push_back(from);
//     const auto& P = best->pts;
//     if      (best->type == "RSR") { sample_arc(p0,P[0],C0R,true);  sample_straight(P[0],P[1]); sample_arc(P[1],p1,C1R,true);  }
//     else if (best->type == "LSL") { sample_arc(p0,P[0],C0L,false); sample_straight(P[0],P[1]); sample_arc(P[1],p1,C1L,false); }
//     else if (best->type == "RSL") { sample_arc(p0,P[0],C0R,true);  sample_straight(P[0],P[1]); sample_arc(P[1],p1,C1L,false); }
//     else if (best->type == "LSR") { sample_arc(p0,P[0],C0L,false); sample_straight(P[0],P[1]); sample_arc(P[1],p1,C1R,true);  }
//     else if (best->type == "RLR") { sample_arc(p0,P[0],C0R,true);  sample_arc(P[0],P[1],P[2],false); sample_arc(P[1],p1,C1R,true);  }
//     else if (best->type == "LRL") { sample_arc(p0,P[0],C0L,false); sample_arc(P[0],P[1],P[2],true);  sample_arc(P[1],p1,C1L,false); }
    
//     return out;
// }


// Steer (not optimized)
// Trajectory DubinsStateSpace::steer(const Eigen::VectorXd& from,
//                                      const Eigen::VectorXd& to) const
// {
//     const double r = min_turning_radius_;
//     if (r <= 0) {
//          std::cerr << "[ERR] Turning radius must be positive." << std::endl;
//          return Trajectory{false, 0, 0, {}};
//     }

//     // 1) Extract states and normalize headings
//     double th0 = normalizeAngle(from[2]); //Initial theta
//     double th1 = normalizeAngle(to[2]); // Goal theta
//     Eigen::Vector2d p0(from[0], from[1]); // Initial loc (for example where the robot is, since we are moving backward in the tree)
//     Eigen::Vector2d p1(to[0],   to[1]); // Goal loc




//     // 2) Define lambdas for calculating circle centers with deep debugging
//     auto left_center  = [&](const Eigen::Vector2d& p, double th){
//         double angle = th + M_PI/2.0;
//         double cos_a = cos(angle);
//         double sin_a = sin(angle);
//         Eigen::Vector2d dir_vec(cos_a, sin_a);
//         Eigen::Vector2d offset = r * dir_vec;
//         Eigen::Vector2d result = p + offset;

//         return result;
//     };
//     auto right_center = [&](const Eigen::Vector2d& p, double th){
//         double angle = th - M_PI/2.0;
//         double cos_a = cos(angle);
//         double sin_a = sin(angle);
//         Eigen::Vector2d dir_vec(cos_a, sin_a);
//         Eigen::Vector2d offset = r * dir_vec;
//         Eigen::Vector2d result = p + offset;

//         return result;
//     };

//     // 3) Calculate the four possible circle centers
//     Eigen::Vector2d C0L = left_center(p0, th0);
//     Eigen::Vector2d C0R = right_center(p0, th0);
//     Eigen::Vector2d C1L = left_center(p1, th1);
//     Eigen::Vector2d C1R = right_center(p1, th1);


//     // 5) arc length helper
//     auto arc_len = [&](const Eigen::Vector2d& A,
//                        const Eigen::Vector2d& B,
//                        const Eigen::Vector2d& C,
//                        bool clockwise)
//     {
//         double alpha = atan2(A.y() - C.y(), A.x() - C.x());
//         double beta  = atan2(B.y() - C.y(), B.x() - C.x());
//         double d_th  = normalizeAngle(beta - alpha);
//         if (clockwise && d_th > 0)    d_th -= 2*M_PI;
//         if (!clockwise && d_th < 0)   d_th += 2*M_PI;
//         return std::abs(d_th) * r;
//     };

//     struct Maneuver { double cost; std::string type; std::vector<Eigen::Vector2d> pts; };
//     std::vector<Maneuver> all_maneuvers;


//     // --- RSR ---
//     {
//         double D = (C1R - C0R).norm();
//         Eigen::Vector2d v = (C1R - C0R)/D;
//         Eigen::Vector2d T0 = C0R + Eigen::Vector2d(-v.y(), v.x()) * r;
//         Eigen::Vector2d T1 = C1R + Eigen::Vector2d(-v.y(), v.x()) * r;

//         double arc0     = arc_len(p0, T0, C0R, /*cw=*/true);
//         double straight = (T1 - T0).norm();
//         double arc1     = arc_len(T1, p1, C1R, /*cw=*/true);
//         double cost     = arc0 + straight + arc1;

//         all_maneuvers.push_back({cost, "RSR", {T0, T1}});
//     }

//     // --- LSL ---
//     {
//         double D = (C1L - C0L).norm();
//         Eigen::Vector2d v = (C1L - C0L)/D;
//         Eigen::Vector2d T0 = C0L + Eigen::Vector2d(v.y(), -v.x()) * r;
//         Eigen::Vector2d T1 = C1L + Eigen::Vector2d(v.y(), -v.x()) * r;

//         double arc0     = arc_len(p0, T0, C0L, /*cw=*/false);
//         double straight = (T1 - T0).norm();
//         double arc1     = arc_len(T1, p1, C1L, /*cw=*/false);
//         double cost     = arc0 + straight + arc1;

//         all_maneuvers.push_back({cost, "LSL", {T0, T1}});
//     }

//     // --- RSL ---
//     {
//         double D = (C1L - C0R).norm();
//         if (D >= 2*r) {
//             double theta = atan2(C1L.y()-C0R.y(), C1L.x()-C0R.x());
//             double phi   = acos(std::clamp(2*r/D, -1.0, 1.0));
//             Eigen::Vector2d T0 = C0R + Eigen::Vector2d(cos(theta+phi), sin(theta+phi)) * r;
//             Eigen::Vector2d T1 = C1L + Eigen::Vector2d(cos(theta+phi-M_PI), sin(theta+phi-M_PI)) * r;

//             double arc0     = arc_len(p0, T0, C0R, /*cw=*/true);
//             double straight = (T1 - T0).norm();
//             double arc1     = arc_len(T1, p1, C1L, /*cw=*/false);
//             double cost     = arc0 + straight + arc1;

//             all_maneuvers.push_back({cost, "RSL", {T0, T1}});
//         } 
//     }

//     // --- LSR ---
//     {
//         double D = (C1R - C0L).norm();
//         if (D >= 2*r) {
//             double theta = atan2(C1R.y()-C0L.y(), C1R.x()-C0L.x());
//             double phi   = acos(std::clamp(2*r/D, -1.0, 1.0));
//             Eigen::Vector2d T0 = C0L + Eigen::Vector2d(cos(theta-phi), sin(theta-phi)) * r;
//             Eigen::Vector2d T1 = C1R + Eigen::Vector2d(cos(theta-phi+M_PI), sin(theta-phi+M_PI)) * r;

//             double arc0     = arc_len(p0, T0, C0L, /*cw=*/false);
//             double straight = (T1 - T0).norm();
//             double arc1     = arc_len(T1, p1, C1R, /*cw=*/true);
//             double cost     = arc0 + straight + arc1;

//             all_maneuvers.push_back({cost, "LSR", {T0, T1}});
//         }
//     }

//     // --- RLR ---
//     {
//         double D = (C1R - C0R).norm();
//         if (D < 4*r) {
//             double theta = atan2(C1R.y()-C0R.y(), C1R.x()-C0R.x());
//             double phi   = acos(std::clamp(D/(4*r), -1.0, 1.0));
//             Eigen::Vector2d C_aux = C0R + Eigen::Vector2d(cos(theta+phi), sin(theta+phi)) * 2*r;
//             Eigen::Vector2d T0    = (C0R + C_aux)*0.5;
//             Eigen::Vector2d T1    = (C1R + C_aux)*0.5;

//             double arc0   = arc_len(p0,  T0,   C0R,   /*cw=*/true);
//             double arcMid = arc_len(T0,  T1,   C_aux, /*cw=*/false);
//             double arc1   = arc_len(T1,  p1,   C1R,   /*cw=*/true);
//             double cost   = arc0 + arcMid + arc1;

//             all_maneuvers.push_back({cost, "RLR", {T0, T1, C_aux}});
//         }
//     }

//     // --- LRL ---
//     {
//         double D = (C1L - C0L).norm();
//         if (D < 4*r) {
//             double theta = atan2(C1L.y()-C0L.y(), C1L.x()-C0L.x());
//             double phi   = acos(std::clamp(D/(4*r), -1.0, 1.0));
//             Eigen::Vector2d C_aux = C0L + Eigen::Vector2d(cos(theta-phi), sin(theta-phi)) * 2*r;
//             Eigen::Vector2d T0    = (C0L + C_aux)*0.5;
//             Eigen::Vector2d T1    = (C1L + C_aux)*0.5;

//             double arc0   = arc_len(p0,  T0,   C0L,   /*cw=*/false);
//             double arcMid = arc_len(T0,  T1,   C_aux, /*cw=*/true);
//             double arc1   = arc_len(T1,  p1,   C1L,   /*cw=*/false);
//             double cost   = arc0 + arcMid + arc1;
            
//             all_maneuvers.push_back({cost, "LRL", {T0, T1, C_aux}});
//         }
//     }

//     // 6) Pick the best maneuver
//     if (all_maneuvers.empty()) {
//         return Trajectory{false, std::numeric_limits<double>::infinity(), std::numeric_limits<double>::infinity(), {}};
//     }
//     auto best = std::min_element(
//         all_maneuvers.begin(), all_maneuvers.end(),
//         [](const auto& a, const auto& b){ return a.cost < b.cost; }
//     );
//     if (std::isinf(best->cost)) {
//         return Trajectory{false, std::numeric_limits<double>::infinity(), std::numeric_limits<double>::infinity() , {}};
//     }


//     Trajectory out;
//     out.is_valid = true;
//     out.cost     = best->cost;
//     out.geometric_distance = out.cost; // in Dubin in 3D case (x,y,theta) without (t) geometric and cost is the same!
//     // --- NEW: Store maneuver info for the time-aware class to use ---
//     out.maneuver_type = best->type;
//     out.maneuver_pts = best->pts;

//     // --- NEW: Populate GEOMETRIC part of Analytical Segments ---
//     out.analytical_segments.clear();
//     const auto& P = best->pts;
//     //////////////////////
//     if (best->type == "RSR") {
//         out.maneuver_centers = {C0R, C1R}; // Store centers for later use
//         out.analytical_segments.push_back({SegmentType::ARC, 0.0, p0, P[0], C0R, r, true});
//         out.analytical_segments.push_back({SegmentType::LINE, 0.0, P[0], P[1]});
//         out.analytical_segments.push_back({SegmentType::ARC, 0.0, P[1], p1, C1R, r, true});
//     } else if (best->type == "LSL") {
//         out.maneuver_centers = {C0L, C1L};
//         out.analytical_segments.push_back({SegmentType::ARC, 0.0, p0, P[0], C0L, r, false});
//         out.analytical_segments.push_back({SegmentType::LINE, 0.0, P[0], P[1]});
//         out.analytical_segments.push_back({SegmentType::ARC, 0.0, P[1], p1, C1L, r, false});
//     } else if (best->type == "RSL") {
//         out.maneuver_centers = {C0R, C1L};
//         out.analytical_segments.push_back({SegmentType::ARC, 0.0, p0, P[0], C0R, r, true});
//         out.analytical_segments.push_back({SegmentType::LINE, 0.0, P[0], P[1]});
//         out.analytical_segments.push_back({SegmentType::ARC, 0.0, P[1], p1, C1L, r, false});
//     } else if (best->type == "LSR") {
//         out.maneuver_centers = {C0L, C1R};
//         out.analytical_segments.push_back({SegmentType::ARC, 0.0, p0, P[0], C0L, r, false});
//         out.analytical_segments.push_back({SegmentType::LINE, 0.0, P[0], P[1]});
//         out.analytical_segments.push_back({SegmentType::ARC, 0.0, P[1], p1, C1R, r, true});
//     } else if (best->type == "RLR") {
//         out.maneuver_centers = {C0R, P[2], C1R}; // P[2] is C_aux
//         out.analytical_segments.push_back({SegmentType::ARC, 0.0, p0, P[0], C0R, r, true});
//         out.analytical_segments.push_back({SegmentType::ARC, 0.0, P[0], P[1], P[2], r, false});
//         out.analytical_segments.push_back({SegmentType::ARC, 0.0, P[1], p1, C1R, r, true});
//     } else if (best->type == "LRL") {
//         out.maneuver_centers = {C0L, P[2], C1L}; // P[2] is C_aux
//         out.analytical_segments.push_back({SegmentType::ARC, 0.0, p0, P[0], C0L, r, false});
//         out.analytical_segments.push_back({SegmentType::ARC, 0.0, P[0], P[1], P[2], r, true});
//         out.analytical_segments.push_back({SegmentType::ARC, 0.0, P[1], p1, C1L, r, false});
//     }



//     ///////////////////////


//     const double discretization_step = 1.5; // meters

//     // auto sample_arc = [&](const Eigen::Vector2d& A, const Eigen::Vector2d& B, const Eigen::Vector2d& C, bool clockwise) {
//     //     double len = arc_len(A, B, C, clockwise);
//     //     int N = std::max(1, static_cast<int>(ceil(len / discretization_step)));
//     //     double alpha = atan2(A.y()-C.y(), A.x()-C.x());
//     //     double d_th_total = normalizeAngle(atan2(B.y()-C.y(), B.x()-C.x()) - alpha);
//     //     if (clockwise && d_th_total > 0)  d_th_total -= 2 * M_PI;
//     //     if (!clockwise && d_th_total < 0) d_th_total += 2 * M_PI;

//     //     for(int i = 1; i <= N; i++){
//     //         double phi = alpha + d_th_total * (static_cast<double>(i) / N);
//     //         Eigen::VectorXd pt(3);
//     //         pt << C.x() + r * cos(phi),
//     //               C.y() + r * sin(phi),
//     //               // HEADING: The heading is tangent to the circle, which is
//     //               // offset by +/- 90 degrees from the angle to the center (phi).
//     //               normalizeAngle(phi + (clockwise ? -M_PI/2.0 : +M_PI/2.0));
//     //         out.path_points.push_back(pt);
//     //     }
//     // };

//     // Define the angular step in radians.
//     const double delta_phi = 0.1; 

//     auto sample_arc = [&](const Eigen::Vector2d& A, const Eigen::Vector2d& B, const Eigen::Vector2d& C, bool clockwise) {
//         // Calculate the start and end angles of the arc relative to the circle's center.
//         double start_angle = atan2(A.y() - C.y(), A.x() - C.x());
//         double end_angle = atan2(B.y() - C.y(), B.x() - C.x());

//         // Calculate the total angle the arc spans, handling direction and wrapping.
//         double total_angle_change = normalizeAngle(end_angle - start_angle);
//         if (clockwise && total_angle_change > 0) {
//             total_angle_change -= 2.0 * M_PI;
//         }
//         if (!clockwise && total_angle_change < 0) {
//             total_angle_change += 2.0 * M_PI;
//         }

//         // Determine the number of steps needed based on the desired angular resolution.
//         int num_steps = static_cast<int>(ceil(std::abs(total_angle_change) / delta_phi));
        
//         // If the arc is too small, just add the endpoint to ensure connection.
//         if (num_steps == 0) {
//             Eigen::VectorXd pt(3);
//             pt << B.x(), B.y(), normalizeAngle(end_angle + (clockwise ? -M_PI/2.0 : +M_PI/2.0));
//             out.path_points.push_back(pt);
//             return;
//         }

//         // Calculate the exact angular step to ensure the last point lands perfectly.
//         double angle_step = total_angle_change / num_steps;

//         // Generate points along the arc.
//         for (int i = 1; i <= num_steps; ++i) {
//             double phi = start_angle + i * angle_step;
//             // FIX: Create a vector of the same size as the input 'from' state
//             Eigen::VectorXd pt(from.size()); 
//             pt.setZero(); // Initialize to zero

//             // Fill the geometric part (x, y, theta)
//             pt.head<3>() << C.x() + r * cos(phi),
//                             C.y() + r * sin(phi),
//                             normalizeAngle(phi + (clockwise ? -M_PI/2.0 : +M_PI/2.0));
            
//             // The time component (pt[3]) remains 0, as it will be correctly calculated
//             // by the derived DubinsTimeStateSpace class.
//             out.path_points.push_back(pt);
//         }
//     };


//     auto sample_straight = [&](const Eigen::Vector2d& A, const Eigen::Vector2d& B) {
//         double L = (B - A).norm();
//         // int N = std::max(1, static_cast<int>(ceil(L / discretization_step)));
//         int N = 1; // Treating Line as Only one Segment
//         double heading = atan2(B.y() - A.y(), B.x() - A.x());

//         for(int i = 1; i <= N; i++){
//             Eigen::Vector2d P_interp = A + (B-A) * (static_cast<double>(i) / N);
//             // FIX: Create a vector of the same size as the input 'from' state
//             Eigen::VectorXd pt(from.size());
//             pt.setZero();

//             // Fill the geometric part
//             pt.head<3>() << P_interp.x(), P_interp.y(), normalizeAngle(heading);

//             out.path_points.push_back(pt);
//         }

//     };

//     out.path_points.push_back(from);

//     if      (best->type == "RSR") { sample_arc(p0,P[0],C0R,true);  sample_straight(P[0],P[1]); sample_arc(P[1],p1,C1R,true);  }
//     else if (best->type == "LSL") { sample_arc(p0,P[0],C0L,false); sample_straight(P[0],P[1]); sample_arc(P[1],p1,C1L,false); }
//     else if (best->type == "RSL") { sample_arc(p0,P[0],C0R,true);  sample_straight(P[0],P[1]); sample_arc(P[1],p1,C1L,false); }
//     else if (best->type == "LSR") { sample_arc(p0,P[0],C0L,false); sample_straight(P[0],P[1]); sample_arc(P[1],p1,C1R,true);  }
//     else if (best->type == "RLR") { sample_arc(p0,P[0],C0R,true);  sample_arc(P[0],P[1],P[2],false); sample_arc(P[1],p1,C1R,true);  }
//     else if (best->type == "LRL") { sample_arc(p0,P[0],C0L,false); sample_arc(P[0],P[1],P[2],true);  sample_arc(P[1],p1,C1L,false); }
    
//     return out;
// }




static inline double arc_len_optimized(const Eigen::Vector2d& A, const Eigen::Vector2d& B, const Eigen::Vector2d& C, double r, bool clockwise) {
    double alpha = atan2(A.y() - C.y(), A.x() - C.x());
    double beta  = atan2(B.y() - C.y(), B.x() - C.x());
    double d_th  = normalizeAngle(beta - alpha);
    if (clockwise && d_th > 0) d_th -= 2.0 * M_PI;
    if (!clockwise && d_th < 0) d_th += 2.0 * M_PI;
    return std::abs(d_th) * r;
}

static inline void sample_arc_optimized(
    std::vector<Eigen::VectorXd>& path_points,
    const Eigen::VectorXd& from_state,
    const Eigen::Vector2d& start_point_2d,
    const Eigen::Vector2d& end_point_2d,
    const Eigen::Vector2d& center_2d,
    double radius,
    bool is_clockwise,
    double angular_resolution_rad)
{
    double start_angle = atan2(start_point_2d.y() - center_2d.y(), start_point_2d.x() - center_2d.x());
    double end_angle = atan2(end_point_2d.y() - center_2d.y(), end_point_2d.x() - center_2d.x());
    double total_angle_change = normalizeAngle(end_angle - start_angle);

    if (is_clockwise && total_angle_change > 0) total_angle_change -= 2.0 * M_PI;
    if (!is_clockwise && total_angle_change < 0) total_angle_change += 2.0 * M_PI;

    if (std::abs(total_angle_change) < 1e-9) return;

    int num_steps = static_cast<int>(std::ceil(std::abs(total_angle_change) / angular_resolution_rad));
    if (num_steps == 0) num_steps = 1;
    
    double angle_step = total_angle_change / num_steps;

    double s_step, c_step;
    sincos(angle_step, &s_step, &c_step);

    double s_phi, c_phi;
    sincos(start_angle, &s_phi, &c_phi);

    double current_heading = normalizeAngle(start_angle + (is_clockwise ? -M_PI / 2.0 : M_PI / 2.0));

    path_points.reserve(path_points.size() + num_steps);
    for (int i = 0; i < num_steps; ++i) {
      double c_new = c_phi * c_step - s_phi * s_step;
      double s_new = c_phi * s_step + s_phi * c_step;
      c_phi = c_new;
      s_phi = s_new;

      current_heading = normalizeAngle(current_heading + angle_step);

      Eigen::VectorXd pt(from_state.size());
      pt.setZero();
      pt.head<3>() << center_2d.x() + radius * c_phi,
                      center_2d.y() + radius * s_phi,
                      current_heading;
      
      if (from_state.size() > 3) {
          pt.tail(from_state.size() - 3) = from_state.tail(from_state.size() - 3);
      }
      path_points.push_back(pt);
    }
}




Trajectory DubinsStateSpace::steer(const Eigen::VectorXd& from, const Eigen::VectorXd& to) const {
    const double r = min_turning_radius_;
    if (r <= 1e-6) {
        return Trajectory{false, 0, 0, {}};
    }

    const double th0 = normalizeAngle(from[2]);
    const double th1 = normalizeAngle(to[2]);
    const Eigen::Vector2d p0(from[0], from[1]);
    const Eigen::Vector2d p1(to[0], to[1]);

    double s0, c0, s1, c1;
    sincos(th0, &s0, &c0);
    sincos(th1, &s1, &c1);

    const Eigen::Vector2d C0L = p0 + r * Eigen::Vector2d(-s0, c0);
    const Eigen::Vector2d C0R = p0 + r * Eigen::Vector2d(s0, -c0);
    const Eigen::Vector2d C1L = p1 + r * Eigen::Vector2d(-s1, c1);
    const Eigen::Vector2d C1R = p1 + r * Eigen::Vector2d(s1, -c1);

    struct Maneuver { double cost; std::string type; std::vector<Eigen::Vector2d> pts; };
    Maneuver best_maneuver = { std::numeric_limits<double>::infinity(), "", {} };

    // --- RSR ---
    {
        Eigen::Vector2d delta = C1R - C0R;
        double D = delta.norm();
        Eigen::Vector2d v = delta / D;
        Eigen::Vector2d T0 = C0R + r * Eigen::Vector2d(-v.y(), v.x());
        Eigen::Vector2d T1 = C1R + r * Eigen::Vector2d(-v.y(), v.x());
        double cost = arc_len_optimized(p0, T0, C0R, r, true) + D + arc_len_optimized(T1, p1, C1R, r, true);
        if (cost < best_maneuver.cost) best_maneuver = {cost, "RSR", {T0, T1}};
    }
    // --- LSL ---
    {
        Eigen::Vector2d delta = C1L - C0L;
        double D = delta.norm();
        Eigen::Vector2d v = delta / D;
        Eigen::Vector2d T0 = C0L + r * Eigen::Vector2d(v.y(), -v.x());
        Eigen::Vector2d T1 = C1L + r * Eigen::Vector2d(v.y(), -v.x());
        double cost = arc_len_optimized(p0, T0, C0L, r, false) + D + arc_len_optimized(T1, p1, C1L, r, false);
        if (cost < best_maneuver.cost) best_maneuver = {cost, "LSL", {T0, T1}};
    }
    // --- RSL ---
    {
        Eigen::Vector2d delta = C1L - C0R;
        double D_sq = delta.squaredNorm();
        if (D_sq >= 4 * r * r) {
            double D = std::sqrt(D_sq);
            double theta = atan2(delta.y(), delta.x());
            double phi = acos(2 * r / D);
            double s_tp, c_tp, s_tpm, c_tpm;
            sincos(theta + phi, &s_tp, &c_tp);
            sincos(theta + phi - M_PI, &s_tpm, &c_tpm);
            Eigen::Vector2d T0 = C0R + r * Eigen::Vector2d(c_tp, s_tp);
            Eigen::Vector2d T1 = C1L + r * Eigen::Vector2d(c_tpm, s_tpm);
            double cost = arc_len_optimized(p0, T0, C0R, r, true) + (T1 - T0).norm() + arc_len_optimized(T1, p1, C1L, r, false);
            if (cost < best_maneuver.cost) best_maneuver = {cost, "RSL", {T0, T1}};
        }
    }
    // --- LSR ---
    {
        Eigen::Vector2d delta = C1R - C0L;
        double D_sq = delta.squaredNorm();
        if (D_sq >= 4 * r * r) {
            double D = std::sqrt(D_sq);
            double theta = atan2(delta.y(), delta.x());
            double phi = acos(2 * r / D);
            double s_tm, c_tm, s_tmp, c_tmp;
            sincos(theta - phi, &s_tm, &c_tm);
            sincos(theta - phi + M_PI, &s_tmp, &c_tmp);
            Eigen::Vector2d T0 = C0L + r * Eigen::Vector2d(c_tm, s_tm);
            Eigen::Vector2d T1 = C1R + r * Eigen::Vector2d(c_tmp, s_tmp);
            double cost = arc_len_optimized(p0, T0, C0L, r, false) + (T1 - T0).norm() + arc_len_optimized(T1, p1, C1R, r, true);
            if (cost < best_maneuver.cost) best_maneuver = {cost, "LSR", {T0, T1}};
        }
    }
    // --- RLR ---
    {
        Eigen::Vector2d delta = C1R - C0R;
        double D_sq = delta.squaredNorm();
        if (D_sq < 16 * r * r) {
            double D = std::sqrt(D_sq);
            double theta = atan2(delta.y(), delta.x());
            double phi = acos(D / (4 * r));
            double s_tp, c_tp;
            sincos(theta + phi, &s_tp, &c_tp);
            Eigen::Vector2d C_aux = C0R + 2 * r * Eigen::Vector2d(c_tp, s_tp);
            Eigen::Vector2d T0 = (C0R + C_aux) * 0.5;
            Eigen::Vector2d T1 = (C1R + C_aux) * 0.5;
            double cost = arc_len_optimized(p0, T0, C0R, r, true) + arc_len_optimized(T0, T1, C_aux, r, false) + arc_len_optimized(T1, p1, C1R, r, true);
            if (cost < best_maneuver.cost) best_maneuver = {cost, "RLR", {T0, T1, C_aux}};
        }
    }
    // --- LRL ---
    {
        Eigen::Vector2d delta = C1L - C0L;
        double D_sq = delta.squaredNorm();
        if (D_sq < 16 * r * r) {
            double D = std::sqrt(D_sq);
            double theta = atan2(delta.y(), delta.x());
            double phi = acos(D / (4 * r));
            double s_tm, c_tm;
            sincos(theta - phi, &s_tm, &c_tm);
            Eigen::Vector2d C_aux = C0L + 2 * r * Eigen::Vector2d(c_tm, s_tm);
            Eigen::Vector2d T0 = (C0L + C_aux) * 0.5;
            Eigen::Vector2d T1 = (C1L + C_aux) * 0.5;
            double cost = arc_len_optimized(p0, T0, C0L, r, false) + arc_len_optimized(T0, T1, C_aux, r, true) + arc_len_optimized(T1, p1, C1L, r, false);
            if (cost < best_maneuver.cost) best_maneuver = {cost, "LRL", {T0, T1, C_aux}};
        }
    }

    if (std::isinf(best_maneuver.cost)) {
        return Trajectory{false, std::numeric_limits<double>::infinity(), std::numeric_limits<double>::infinity(), {}};
    }

    Trajectory out;
    out.is_valid = true;
    out.cost = best_maneuver.cost;
    out.geometric_distance = best_maneuver.cost;
    out.maneuver_type = best_maneuver.type;
    out.maneuver_pts = best_maneuver.pts;

    // out.analytical_segments.clear();
    // const auto& P = best_maneuver.pts;
    // if (best_maneuver.type == "RSR") {
    //     out.maneuver_centers = {C0R, C1R};
    //     out.analytical_segments.push_back({SegmentType::ARC, 0.0, p0, P[0], C0R, r, true});
    //     out.analytical_segments.push_back({SegmentType::LINE, 0.0, P[0], P[1]});
    //     out.analytical_segments.push_back({SegmentType::ARC, 0.0, P[1], p1, C1R, r, true});
    // } else if (best_maneuver.type == "LSL") {
    //     out.maneuver_centers = {C0L, C1L};
    //     out.analytical_segments.push_back({SegmentType::ARC, 0.0, p0, P[0], C0L, r, false});
    //     out.analytical_segments.push_back({SegmentType::LINE, 0.0, P[0], P[1]});
    //     out.analytical_segments.push_back({SegmentType::ARC, 0.0, P[1], p1, C1L, r, false});
    // } else if (best_maneuver.type == "RSL") {
    //     out.maneuver_centers = {C0R, C1L};
    //     out.analytical_segments.push_back({SegmentType::ARC, 0.0, p0, P[0], C0R, r, true});
    //     out.analytical_segments.push_back({SegmentType::LINE, 0.0, P[0], P[1]});
    //     out.analytical_segments.push_back({SegmentType::ARC, 0.0, P[1], p1, C1L, r, false});
    // } else if (best_maneuver.type == "LSR") {
    //     out.maneuver_centers = {C0L, C1R};
    //     out.analytical_segments.push_back({SegmentType::ARC, 0.0, p0, P[0], C0L, r, false});
    //     out.analytical_segments.push_back({SegmentType::LINE, 0.0, P[0], P[1]});
    //     out.analytical_segments.push_back({SegmentType::ARC, 0.0, P[1], p1, C1R, r, true});
    // } else if (best_maneuver.type == "RLR") {
    //     out.maneuver_centers = {C0R, P[2], C1R};
    //     out.analytical_segments.push_back({SegmentType::ARC, 0.0, p0, P[0], C0R, r, true});
    //     out.analytical_segments.push_back({SegmentType::ARC, 0.0, P[0], P[1], P[2], r, false});
    //     out.analytical_segments.push_back({SegmentType::ARC, 0.0, P[1], p1, C1R, r, true});
    // } else if (best_maneuver.type == "LRL") {
    //     out.maneuver_centers = {C0L, P[2], C1L};
    //     out.analytical_segments.push_back({SegmentType::ARC, 0.0, p0, P[0], C0L, r, false});
    //     out.analytical_segments.push_back({SegmentType::ARC, 0.0, P[0], P[1], P[2], r, true});
    //     out.analytical_segments.push_back({SegmentType::ARC, 0.0, P[1], p1, C1L, r, false});
    // }

    const double delta_phi = 0.1;
    out.path_points.push_back(from);

    // FIX: Define P before the if/else-if block
    const auto& P_for_discretize = best_maneuver.pts;

    auto push_waypoint = [&](const Eigen::Vector2d& point_2d, const Eigen::Vector2d& center, bool is_clockwise) {
        Eigen::VectorXd waypoint(from.size());
        waypoint.setZero();
        double heading = normalizeAngle(atan2(point_2d.y() - center.y(), point_2d.x() - center.x()) + (is_clockwise ? -M_PI / 2.0 : M_PI / 2.0));
        waypoint.head<3>() << point_2d, heading;
        if (from.size() > 3) waypoint.tail(from.size() - 3) = from.tail(from.size() - 3);
        out.path_points.push_back(waypoint);
    };

    if (best_maneuver.type == "RSR") {
        sample_arc_optimized(out.path_points, from, p0, P_for_discretize[0], C0R, r, true, delta_phi);
        push_waypoint(P_for_discretize[1], C1R, true);
        sample_arc_optimized(out.path_points, from, P_for_discretize[1], p1, C1R, r, true, delta_phi);
    } else if (best_maneuver.type == "LSL") {
        sample_arc_optimized(out.path_points, from, p0, P_for_discretize[0], C0L, r, false, delta_phi);
        push_waypoint(P_for_discretize[1], C1L, false);
        sample_arc_optimized(out.path_points, from, P_for_discretize[1], p1, C1L, r, false, delta_phi);
    } else if (best_maneuver.type == "RSL") {
        sample_arc_optimized(out.path_points, from, p0, P_for_discretize[0], C0R, r, true, delta_phi);
        push_waypoint(P_for_discretize[1], C1L, false);
        sample_arc_optimized(out.path_points, from, P_for_discretize[1], p1, C1L, r, false, delta_phi);
    } else if (best_maneuver.type == "LSR") {
        sample_arc_optimized(out.path_points, from, p0, P_for_discretize[0], C0L, r, false, delta_phi);
        push_waypoint(P_for_discretize[1], C1R, true);
        sample_arc_optimized(out.path_points, from, P_for_discretize[1], p1, C1R, r, true, delta_phi);
    } else if (best_maneuver.type == "RLR") {
        sample_arc_optimized(out.path_points, from, p0, P_for_discretize[0], C0R, r, true, delta_phi);
        sample_arc_optimized(out.path_points, from, P_for_discretize[0], P_for_discretize[1], P_for_discretize[2], r, false, delta_phi);
        sample_arc_optimized(out.path_points, from, P_for_discretize[1], p1, C1R, r, true, delta_phi);
    } else if (best_maneuver.type == "LRL") {
        sample_arc_optimized(out.path_points, from, p0, P_for_discretize[0], C0L, r, false, delta_phi);
        sample_arc_optimized(out.path_points, from, P_for_discretize[0], P_for_discretize[1], P_for_discretize[2], r, true, delta_phi);
        sample_arc_optimized(out.path_points, from, P_for_discretize[1], p1, C1L, r, false, delta_phi);
    }

    return out;
}

// --- Implement other overridden functions as placeholders ---
std::shared_ptr<State> DubinsStateSpace::addState(const Eigen::VectorXd& value) {
    return StateSpace::addState(std::make_shared<EuclideanState>(value));
}

std::shared_ptr<State> DubinsStateSpace::sampleUniform(double min, double max) {
    Eigen::VectorXd values(3);
    values[0] = min + (max - min) * (static_cast<double>(rand()) / RAND_MAX);
    values[1] = min + (max - min) * (static_cast<double>(rand()) / RAND_MAX);
    values[2] = 2.0 * M_PI * (static_cast<double>(rand()) / RAND_MAX);
    return this->addState(values);
}

void DubinsStateSpace::sampleUniform(double min, double max, int k) { /* ... */ }

std::shared_ptr<State> DubinsStateSpace::sampleUniform(const Eigen::VectorXd& min_bounds, const Eigen::VectorXd& max_bounds) {
    // 1. Check that the input bounds are 3-dimensional.
    // This class specifically handles the 3D Dubins car model.
    if (min_bounds.size() != 3 || max_bounds.size() != 3) {
        throw std::invalid_argument("DubinsStateSpace requires 3D bounds for vector-based sampling.");
    }

    // 2. Create a 3D vector for the new sample.
    Eigen::VectorXd values(3);

    // 3. Sample x, y, and theta from their respective bounds.
    for (int i = 0; i < 3; ++i) {
        // Generate a random double between 0.0 and 1.0
        double random_coeff = static_cast<double>(rand()) / RAND_MAX;
        
        // Scale the random value to the range for the current dimension
        values[i] = min_bounds[i] + (max_bounds[i] - min_bounds[i]) * random_coeff;
    }
    
    // 4. Use the class's own addState method to create and return the new state.
    return this->addState(values);
}



std::shared_ptr<State> DubinsStateSpace::interpolate(const std::shared_ptr<State>& s1, const std::shared_ptr<State>& s2, double t) const { return nullptr; }
bool DubinsStateSpace::isValid(const std::shared_ptr<State>& state) const { return true; }