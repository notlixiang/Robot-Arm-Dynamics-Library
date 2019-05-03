#include <RobotDynamics.h>

RobotDynamics::RobotDynamics(unsigned int JOINT_NUM_, vector<Eigen::Vector3d> mass_center_,
                             vector<Eigen::Matrix3d> inertia_, vector<double> mass_,
                             vector<double> damping_, vector<double> friction_) :
        JOINT_NUM(JOINT_NUM_) {
    PC.assign(mass_center_.begin(), mass_center_.end());
    I.assign(inertia_.begin(), inertia_.end());
    m.assign(mass_.begin(), mass_.end());
    damping.assign(damping_.begin(), damping_.end());
    friction.assign(friction_.begin(), friction_.end());
    assert(JOINT_NUM == PC.size());
    assert(JOINT_NUM == I.size());
    assert(JOINT_NUM == m.size());
    assert(JOINT_NUM == damping.size());
    assert(JOINT_NUM == friction.size());
}

RobotDynamics::~RobotDynamics() {}

vector<double> RobotDynamics::
getTau(double G, vector<Eigen::Matrix3d> R, vector<Eigen::Vector3d> P,
       vector<double> theta_d, vector<double> theta_dd) {
    assert(JOINT_NUM == R.size());
    assert(JOINT_NUM == P.size());
    assert(JOINT_NUM == theta_d.size());
    assert(JOINT_NUM == theta_dd.size());

    vector<Eigen::Vector3d> w(JOINT_NUM + 1);
    vector<Eigen::Vector3d> w_d(JOINT_NUM + 1);
    vector<Eigen::Vector3d> v_d(JOINT_NUM + 1);
    vector<Eigen::Vector3d> vC_d(JOINT_NUM + 1);
    vector<Eigen::Vector3d> F(JOINT_NUM + 1);
    vector<Eigen::Vector3d> N(JOINT_NUM + 1);
    vector<Eigen::Vector3d> f(JOINT_NUM + 2);
    vector<Eigen::Vector3d> n(JOINT_NUM + 2);
    for (int i = 0; i < JOINT_NUM + 1; i++) {
        w[i].setZero();
        w_d[i].setZero();
        v_d[i].setZero();
        vC_d[i].setZero();
        F[i].setZero();
        N[i].setZero();
    }
    for (int i = 0; i < JOINT_NUM + 2; i++) {
        f[i].setZero();
        n[i].setZero();
    }
    vector<double> tau(JOINT_NUM);
    Eigen::Vector3d Z(0, 0, 1);
//    printf("cal tau\n");
//        std::cout << v_d[0].transpose() << " ";

    v_d[0] = Eigen::Vector3d(0, 0, G);
    for (int i = 0; i < JOINT_NUM; i++) {
        w[i + 1] = R[i].inverse() * w[i] + theta_d[i] * Z;
        w_d[i + 1] = R[i].inverse() * w_d[i] + (R[i].inverse() * w[i]).cross(theta_d[i] * Z) + theta_dd[i] * Z;
        v_d[i + 1] = R[i].inverse() * (w_d[i].cross(P[i]) + w[i].cross(w[i].cross(P[i])) + v_d[i]);
        vC_d[i + 1] = w_d[i + 1].cross(PC[i]) + w[i + 1].cross(w[i + 1].cross(PC[i])) + v_d[i + 1];
        F[i + 1] = m[i] * vC_d[i + 1];
        N[i + 1] = I[i] * w_d[i + 1] + w[i + 1].cross(I[i] * w[i + 1]);
//        std::cout << w[i + 1].transpose() << " ";
//        std::cout << w_d[i + 1].transpose() << " ";
//        std::cout << v_d[i + 1].transpose() << " ";
//        std::cout << vC_d[i + 1].transpose() << " ";
//        std::cout << F[i + 1].transpose() << " ";
//        std::cout << N[i + 1].transpose() << " ";
//        std::cout << P[i].transpose() << " ";
//        std::cout << PC[i].transpose() << " ";
//        std::cout << R[i](0,0) << " "<< R[i](0,1) << " "<< R[i](0,2) << " ";
//        std::cout << R[i](1,0) << " "<< R[i](1,1) << " "<< R[i](1,2) << " ";
//        std::cout << R[i](2,0) << " "<< R[i](2,1) << " "<< R[i](2,2) << " ";
    }
    Eigen::Matrix3d identity_mat;
    identity_mat.setIdentity();
    R.push_back(identity_mat);
    P.push_back(Eigen::Vector3d(0, 0, 0));
    for (int i = JOINT_NUM; i >= 1; i--) {
        f[i] = R[i] * f[i + 1] + F[i];
        n[i] = N[i] + R[i] * n[i + 1] + PC[i - 1].cross(F[i]) + P[i].cross(R[i] * f[i + 1]);
        tau[i - 1] = n[i].transpose() * Z + damping[i - 1] * theta_d[i - 1];
//        std::cout << f[i].transpose() << " ";
//        std::cout << n[i].transpose() << " ";
//        std::cout << tau[i - 1].transpose() << " ";
//        std::cout << ((R[i] * f[i + 1])).transpose() << " ";
    }
//    std::cout << "\n";
    return tau;
}


vector<double> RobotDynamics::
getTau(double G, vector<Eigen::Quaterniond> q, vector<Eigen::Vector3d> P,
       vector<double> theta_d, vector<double> theta_dd) {
    vector<Eigen::Matrix3d> R;
    unsigned int len = q.size();
    R.resize(len);
    for (int i = 0; i < len; i++) {
        R[i] = q[i];
    }
    return getTau(G, R, P, theta_d, theta_dd);
}