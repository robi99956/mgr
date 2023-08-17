#include <iostream>
#include <nlopt.hpp>
#include "movement.h"

static inline double delta_x(const Vector &x) {return x[0];}
static inline double delta_y(const Vector &x) {return x[1];}
static inline double center_x(const Vector &x) {return x[2];}
static inline double center_y(const Vector &x) {return x[3];}
static inline double rotation(const Vector &x) {return x[4];}

MovementModel::MovementModel(Vector &x) {
    this->dx = delta_x(x);
    this->dy = delta_y(x);
    this->xo = center_x(x);
    this->yo = center_y(x);
    this->dangle = rotation(x);
}

std::string MovementModel::to_string() {
    char buf[128];
    sprintf(buf, "dx = %f, dy = %f, dangle = %f",
            this->dx, this->dy, this->dangle);
    return std::string(buf);
}

double optimization_problem(const Vector &x, Vector &grad, void * arg) {
    (void)grad;
    std::vector<MatchingPoints> * points = static_cast<std::vector<MatchingPoints>*>(arg);
    double error_sum = 0;

    for(size_t i=0; i<points->size(); i++) {
        MatchingPoints p(points->at(i));
        double x_error = p.current.x - p.previous.x
                - (delta_x(x) + (p.current.x - center_x(x)) * cos(rotation(x)));
        double y_error = p.current.y - p.previous.y
                - (delta_y(x) + (p.current.y - center_y(x)) * sin(rotation(x)));
        error_sum += x_error*x_error + y_error*y_error;
    }

    return error_sum;
}

MovementDetector::MovementDetector() {
    this->last_known_position = VehiclePosition();
}

VehiclePosition MovementDetector::process(std::vector<MatchingPoints> points) {
//    nlopt::opt solver(nlopt::algorithm::LN_BOBYQA, 5);
//    nlopt::opt solver(nlopt::algorithm::LN_COBYLA, 5);
    nlopt::opt solver(nlopt::algorithm::LN_NEWUOA_BOUND, 5);
    solver.set_min_objective(optimization_problem, (void*)&points);
    solver.set_stopval(20);
    solver.set_xtol_rel(1e-4);

    Vector solution = Vector(5);
    double final_value = 0;
    nlopt::result result = solver.optimize(solution, final_value);
    printf("result = %d, fun = %f, evals = %d\n", result, final_value, solver.get_numevals());
    for( size_t i=0; i<solution.size(); i++ ) {
        printf("%f, ", solution[i]);
    }
    printf("\n");

    MovementModel model(solution);

    VehiclePosition new_position(
            this->last_known_position.x + model.dx,
            this->last_known_position.y + model.dy,
            this->last_known_position.angle + model.dangle);

    this->last_known_position = new_position;
    return new_position;
}
