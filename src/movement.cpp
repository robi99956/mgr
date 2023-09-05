#include <iostream>
#include <nlopt.hpp>
#include "movement.h"

#define UNUSED(__x) (void)__x

static inline double rotation(const Vector &x) {return x[0];}
static inline double delta_x(const Vector &x) {return x[1];}
static inline double delta_y(const Vector &x) {return x[2];}

class MovementSolution {
    public:
        Vector solution;
        double loss;

        MovementSolution(Vector solution, double loss) {
            this->solution = solution;
            this->loss = loss;
        }
};

MovementModel::MovementModel(Vector &x) {
    this->dx = delta_x(x);
    this->dy = delta_y(x);
    this->xo = 0;
    this->yo = 0;
    this->dangle = rotation(x);
}

std::string MovementModel::to_string() {
    char buf[128];
    sprintf(buf, "dx = %f, dy = %f, dangle = %f",
            this->dx, this->dy, this->dangle);
    return std::string(buf);
}

MovementDetector::MovementDetector() {
    this->last_known_position = VehiclePosition();
}

static Matrix generate_transformation_matrix(double alpha, double t_x, double t_y) {
    Matrix m(3, 3);
    m << cos(alpha), -sin(alpha), t_x,
         sin(alpha),  cos(alpha), t_y,
         0.0,          0.0,       1;  
    return m;
}

MovementSolution ap_movement_solver(Vector init_theta, std::vector<MatchingPoints> &points) {
    const int epochs = 100;
    Vector eta(3);
    eta << 1e-6, 1e-2, 1e-2;
    const int log_period = 10;

    Matrix x = Matrix::Ones(3, points.size());
    Matrix y = Matrix::Ones(3, points.size());

    for(size_t i=0; i<points.size(); i++) {
        x.coeffRef(0, i) = points[i].previous.x;
        x.coeffRef(1, i) = points[i].previous.y;
        y.coeffRef(0, i) = points[i].current.x;
        y.coeffRef(1, i) = points[i].current.y;
    }

//    std::cout<< x << std::endl << y << std::endl;

    Vector theta = init_theta;
    double loss = INFINITY;

    for(int i=0; i<epochs; i++) {
        double alpha = rotation(theta), t_x = delta_x(theta), t_y = delta_y(theta);

        // forward pass
        Matrix A = generate_transformation_matrix(alpha, t_x, t_y);
        Matrix y_hat = A * x;
        Matrix L = 0.5*(y_hat-y).cwiseAbs2();

        loss = L.sum();
        if( i % log_period == 0 ) {
            printf("epoch: %d - loss = %f, theta = %f,%f,%f\n", i, loss, alpha, t_x, t_y);
        }

        // backward pass
        Matrix dy_hat = y_hat - y;
        Matrix dA = dy_hat * x.transpose();
        double dalpha = dA.coeff(0, 0)*(-sin(alpha)) + dA.coeff(0, 1)*(-cos(alpha))
            + dA.coeff(1, 0) * cos(alpha) + dA.coeff(1, 1)*(-sin(alpha));
        double dt_x = dA.coeff(0, 2);
        double dt_y = dA.coeff(1, 2);

        Vector dtheta = Vector(3);
        dtheta << dalpha, dt_x, dt_y;

        for(int i=0; i<3; i++) {
            theta.coeffRef(i) -= dtheta.coeff(i)*eta.coeff(i);
        }
    }

    std::cout << "dtheta = \n" << theta << std::endl;
    return MovementSolution(theta, loss);
}

#define MAX_ALLOWED_LOSS 30
#define MIN_ALLOWED_POINT_PAIRS 3
VehiclePositionSolution MovementDetector::process(std::vector<MatchingPoints> points) {
    if( points.size() < MIN_ALLOWED_POINT_PAIRS ) {
        return VehiclePositionSolution(this->last_known_position, false);
    }
//    Vector init = Vector::Random(3);
    Vector init = Vector::Zero(3);
    MovementSolution solution = ap_movement_solver(init, points);

    if( solution.loss > MAX_ALLOWED_LOSS ) {
        return VehiclePositionSolution(this->last_known_position, false);
    }
    MovementModel model(solution.solution);

    VehiclePosition new_position(
            this->last_known_position.x + model.dx,
            this->last_known_position.y + model.dy,
            this->last_known_position.angle + model.dangle);

    this->last_known_position = new_position;
    return VehiclePositionSolution(new_position, true);
}
