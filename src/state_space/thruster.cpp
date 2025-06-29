// ThrusterSteerStateSpace.hpp
#ifndef THRUSTER_STEER_STATE_SPACE_HPP
#define THRUSTER_STEER_STATE_SPACE_HPP

#include "motion_planning/state_space/state_space.hpp"
#include "motion_planning/state_space/euclidean_state.hpp"  // Reuse EuclideanState for state values
#include <Eigen/Dense>
#include <vector>
#include <cmath>
#include <algorithm>
#include <limits>
#include <set>

// Trajectory struct definition
struct Trajectory {
    bool is_valid;
    double cost;                                // Total cost (time for time-optimal)
    std::vector<Eigen::VectorXd> path_points;  // Discretized states along the path
};

class ThrusterSteerStateSpace : public StateSpace {
public:
    // dimension = 2*D_spatial + 1 (e.g., 7 for 3D pos/vel/time)
    ThrusterSteerStateSpace(int dimension, double max_acceleration);

    // StateSpace interface
    std::shared_ptr<State> addState(const Eigen::VectorXd& value) override;
    std::shared_ptr<State> sampleUniform(double min = 0.0, double max = 1.0) override;
    void sampleUniform(double min, double max, int k) override;
    std::shared_ptr<State> sampleUniform(const Eigen::VectorXd& min_bounds,
                                         const Eigen::VectorXd& max_bounds) override;
    double distance(const std::shared_ptr<State>& s1,
                    const std::shared_ptr<State>& s2) const override;
    std::shared_ptr<State> interpolate(const std::shared_ptr<State>& s1,
                                       const std::shared_ptr<State>& s2,
                                       double t) const override;
    bool isValid(const std::shared_ptr<State>& state) const override;

    // Main steering function: from [x,v,t] to [x,v,t]
    Trajectory steer(const Eigen::VectorXd& from,
                     const Eigen::VectorXd& to) const override;

private:
    double max_acceleration_;

    // 1D steering: returns [t1, t2, a1, a2, v_coast] or NaN-vector on failure
    Eigen::VectorXd steering1D(double x0, double x1,
                               double v0, double v1,
                               double t0, double t1,
                               double a_max) const;

    // N-D steering result
    struct NDSteeringResult {
        bool success;
        Eigen::VectorXd Time;  // size N
        Eigen::MatrixXd X;     // N x D
        Eigen::MatrixXd V;     // N x D
        Eigen::MatrixXd A;     // (N-1) x D
    };

    // N-D steering
    NDSteeringResult steeringND(const Eigen::VectorXd& x0,
                                const Eigen::VectorXd& x1,
                                const Eigen::VectorXd& v0,
                                const Eigen::VectorXd& v1,
                                double t0, double t1,
                                const Eigen::VectorXd& a_max) const;

    // Fine-grain interpolation at resolution dt
    std::tuple<Eigen::VectorXd, Eigen::MatrixXd,
               Eigen::MatrixXd, Eigen::MatrixXd>
    fineGrain(const Eigen::VectorXd& Time_raw,
              const Eigen::MatrixXd& A_raw,
              const Eigen::MatrixXd& V_raw,
              const Eigen::MatrixXd& X_raw,
              double dt) const;

    Eigen::VectorXd getSpatialPosition(const Eigen::VectorXd& state) const;
    Eigen::VectorXd getSpatialVelocity(const Eigen::VectorXd& state) const;
};

#endif // THRUSTER_STEER_STATE_SPACE_HPP


// ThrusterSteerStateSpace.cpp
#include "motion_planning/state_space/thruster_steer_state_space.hpp"
#include <stdexcept>
#include <iostream>

ThrusterSteerStateSpace::ThrusterSteerStateSpace(int dimension,
                                                 double max_acceleration)
    : StateSpace(dimension), max_acceleration_(max_acceleration) {
    if (dimension <= 1 || (dimension - 1) % 2 != 0) {
        throw std::invalid_argument(
            "Dimension must be 2*D_spatial+1 (e.g., 7 for 3D pos/vel/time)");
    }
}

std::shared_ptr<State> ThrusterSteerStateSpace::addState(const Eigen::VectorXd& v) {
    return StateSpace::addState(std::make_shared<EuclideanState>(v));
}

std::shared_ptr<State> ThrusterSteerStateSpace::sampleUniform(double lo, double hi) {
    Eigen::VectorXd r = Eigen::VectorXd::Random(dimension_);
    r = lo + (hi - lo) * (r.array() + 1.0) / 2.0;
    return addState(r);
}

void ThrusterSteerStateSpace::sampleUniform(double lo, double hi, int k) {
    for (int i = 0; i < k; ++i) sampleUniform(lo, hi);
}

std::shared_ptr<State> ThrusterSteerStateSpace::sampleUniform(const Eigen::VectorXd& lo,
                                                                const Eigen::VectorXd& hi) {
    Eigen::VectorXd v(dimension_);
    for (int i = 0; i < dimension_; ++i) {
        double f = double(std::rand())/RAND_MAX;
        v[i] = lo[i] + f*(hi[i]-lo[i]);
    }
    return addState(v);
}

double ThrusterSteerStateSpace::distance(const std::shared_ptr<State>& s1,
                                         const std::shared_ptr<State>& s2) const {
    return (s1->getValue() - s2->getValue()).norm();
}

std::shared_ptr<State> ThrusterSteerStateSpace::interpolate(const std::shared_ptr<State>& s1,
                                                             const std::shared_ptr<State>& s2,
                                                             double t) const {
    Eigen::VectorXd v = s1->getValue() + t*(s2->getValue()-s1->getValue());
    return std::make_shared<EuclideanState>(v);
}

bool ThrusterSteerStateSpace::isValid(const std::shared_ptr<State>&) const {
    return true;
}

Eigen::VectorXd ThrusterSteerStateSpace::steering1D(double x0, double x1,
                                                   double v0, double v1,
                                                   double t0, double t1,
                                                   double a_max) const {
    // Ported directly from Julia steering_1D
    Eigen::VectorXd sol = Eigen::VectorXd::Constant(5, std::numeric_limits<double>::quiet_NaN());
    double dt = t1 - t0;
    if (dt <= 0) return sol;
    // flip if v0<0 omitted here for brevity—add symmetrical logic as needed
    double dx = x1 - x0;
    double t_hat = std::abs(v1 - v0)/a_max;
    if (t_hat > dt) return sol;
    double vmax = (dt*a_max + v0 + v1)/2.0;
    double vmin = (v0 + v1 - dt*a_max)/2.0;
    double tau1 = v0*v0/(2*a_max);
    double tau2 = v1*v1/(2*a_max);
    double ta = v0/a_max;
    double tb = dt - v1/a_max;
    double t1_rel, t2_rel, v_co;
    // Choose subcase per Julia. Here we implement the common four cases when v0,v1>=0
    if (dx < tau1+tau2) {
        double z = std::sqrt((dx - (dt*vmin + (v0-vmin)*(v0-vmin)/(2*a_max)
                                      + (v1-vmin)*(v1-vmin)/(2*a_max))) * a_max);
        double tmid = (ta+tb)/2;
        v_co = vmin + z;
        t1_rel = tmid - z/a_max;
        t2_rel = tmid + z/a_max;
    } else if (dx > dt*vmax - (vmax-v0)*(vmax-v0)/(2*a_max)
                         - (vmax-v1)*(vmax-v1)/(2*a_max)) {
        double z = std::sqrt(((dt*vmax - (vmax-v0)*(vmax-v0)/(2*a_max)
                               - (vmax-v1)*(vmax-v1)/(2*a_max)) - dx) * a_max);
        double tmid = (vmax-v0)/a_max;
        v_co = vmax - z;
        t1_rel = tmid - z/a_max;
        t2_rel = tmid + z/a_max;
    } else {
        // cruise case
        v_co = dx/dt;
        t1_rel = (v_co - v0)/a_max;
        t2_rel = dt - (v_co - v1)/a_max;
    }
    double a1 = (v_co-v0)/t1_rel;
    double a2 = (v1-v_co)/(dt-t2_rel);
    sol << t0+t1_rel, t0+t2_rel, a1, a2, v_co;
    return sol;
}

ThrusterSteerStateSpace::NDSteeringResult
ThrusterSteerStateSpace::steeringND(const Eigen::VectorXd& x0,
                                     const Eigen::VectorXd& x1,
                                     const Eigen::VectorXd& v0,
                                     const Eigen::VectorXd& v1,
                                     double t0, double t1,
                                     const Eigen::VectorXd& a_max) const {
    int D = x0.size();
    NDSteeringResult R; R.success=false;
    Eigen::MatrixXd raw_t(2,D), raw_a(3,D);
    for (int d=0; d<D; ++d) {
        auto s1 = steering1D(x0[d], x1[d], v0[d], v1[d], t0, t1, a_max[d]);
        if (std::isnan(s1[0])) return R;
        raw_t.col(d) = s1.head<2>();
        raw_a.col(d) << s1[2], 0.0, s1[3];
    }
    std::set<double> times = {t0, t1};
    for (int d=0; d<D; ++d) {
        times.insert(raw_t(0,d)); times.insert(raw_t(1,d));
    }
    std::vector<double> Tvec(times.begin(), times.end());
    int N = Tvec.size();
    R.Time = Eigen::Map<Eigen::VectorXd>(Tvec.data(),N);
    R.X.resize(N,D); R.V.resize(N,D); R.A.resize(N-1,D);
    // integrate each segment per dimension
    for (int d=0; d<D; ++d) {
        double cx=x0[d], cv=v0[d], ct=t0;
        for (int i=0;i<N;i++){
            double nt=Tvec[i], dt=nt-ct;
            double a_cur = (ct<raw_t(0,d)? raw_a(0,d)
                            : (ct<raw_t(1,d)? 0.0: raw_a(2,d)));
            if (i>0) R.A(i-1,d)=a_cur;
            cx += cv*dt + 0.5*a_cur*dt*dt;
            cv += a_cur*dt;
            R.X(i,d)=cx; R.V(i,d)=cv;
            ct=nt;
        }
    }
    R.success=true;
    return R;
}

std::tuple<Eigen::VectorXd,Eigen::MatrixXd,Eigen::MatrixXd,Eigen::MatrixXd>
ThrusterSteerStateSpace::fineGrain(const Eigen::VectorXd& T,
                                    const Eigen::MatrixXd& A,
                                    const Eigen::MatrixXd& V,
                                    const Eigen::MatrixXd& X,
                                    double dt) const {
    int D=X.cols(), M=T.size();
    double total=T[M-1]-T[0];
    int estimate=int(std::ceil(total/dt))+M+1;
    Eigen::VectorXd Tf(estimate);
    Eigen::MatrixXd Xf(estimate,D), Vf(estimate,D), Af(estimate,D);
    int idx=0;
    for(int i=0;i<M-1;++i){
        Tf[idx]=T[i]; Xf.row(idx)=X.row(i); Vf.row(idx)=V.row(i); Af.row(idx)=A.row(i); idx++;
        double start=T[i], end=T[i+1];
        for(double t=start+dt; t<end; t+=dt){
            double dft=t-start;
            Tf[idx]=t;
            Xf.row(idx)=X.row(i) + V.row(i)*dft + A.row(i)*0.5*dft*dft;
            Vf.row(idx)=V.row(i)+A.row(i)*dft;
            Af.row(idx)=A.row(i);
            idx++;
        }
    }
    Tf[idx]=T[M-1]; Xf.row(idx)=X.row(M-1); Vf.row(idx)=V.row(M-1); Af.row(idx).setZero(); idx++;
    Tf.conservativeResize(idx); Xf.conservativeResize(idx,D); Vf.conservativeResize(idx,D); Af.conservativeResize(idx,D);
    return {Tf,Af,Vf,Xf};
}

Eigen::VectorXd ThrusterSteerStateSpace::getSpatialPosition(const Eigen::VectorXd& s) const {
    int D=(dimension_-1)/2;
    return s.head(D);
}

Eigen::VectorXd ThrusterSteerStateSpace::getSpatialVelocity(const Eigen::VectorXd& s) const {
    int D=(dimension_-1)/2;
    return s.segment(D,D);
}

Trajectory ThrusterSteerStateSpace::steer(const Eigen::VectorXd& from,
                                           const Eigen::VectorXd& to) const {
    Trajectory out; out.is_valid=false; out.cost=std::numeric_limits<double>::infinity();
    int D=(dimension_-1)/2;
    double t_from=from[dimension_-1], t_to=to[dimension_-1];
    double dt=t_from - t_to;
    if(dt<=0) return out;
    Eigen::VectorXd x0=to.head(D), x1=from.head(D);
    Eigen::VectorXd v0=to.segment(D,D), v1=from.segment(D,D);
    Eigen::VectorXd amax=Eigen::VectorXd::Constant(D,max_acceleration_);
    auto R=steeringND(x0,x1,v0,v1,t_to,t_from,amax);
    if(!R.success) return out;
    auto [Tf,Af,Vf,Xf]=fineGrain(R.Time,R.A,R.V,R.X, /*dt=*/0.5);
    out.cost=dt; out.is_valid=true;
    out.path_points.reserve(Tf.size());
    for(int i=Tf.size()-1;i>=0;--i){ // reverse
        Eigen::VectorXd st(dimension_);
        st.head(D)=Xf.row(i).transpose();
        st.segment(D,D)=Vf.row(i).transpose();
        st[dimension_-1]=Tf[i];
        out.path_points.push_back(st);
    }
    return out;
}