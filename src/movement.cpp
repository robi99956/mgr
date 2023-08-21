#include <iostream>
#include <nlopt.hpp>
#include "movement.h"

#define UNUSED(__x) (void)__x

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
    std::vector<MatchingPoints> * points = static_cast<std::vector<MatchingPoints>*>(arg);
    double error_sum = 0;

    for(size_t i=0; i<grad.size(); i++) {
        grad[i] = 0;
    }

    for(size_t i=0; i<points->size(); i++) {
        MatchingPoints p(points->at(i));
        double x_error = p.current.x - p.previous.x
                - (delta_x(x) + (p.current.x - center_x(x)) * cos(rotation(x)));
        double y_error = p.current.y - p.previous.y
                - (delta_y(x) + (p.current.y - center_y(x)) * sin(rotation(x)));
        error_sum += x_error*x_error + y_error*y_error;
                
        if( grad.empty() == false ) {
            grad[0] += -2*(-(p.current.x-center_x(x))*cos(rotation(x))
                            +p.current.x-p.previous.x-delta_x(x));
            grad[1] += -2*(-(p.current.y-center_y(x))*sin(rotation(x))
                            +p.current.y-p.previous.y-delta_y(x));
            grad[2] += 2*cos(rotation(x))*(-(p.current.x-center_x(x))
                            *cos(rotation(x))+p.current.x-p.previous.x-delta_x(x));
            grad[3] += 2*sin(rotation(x))*(-(p.current.y-center_y(x))
                            *sin(rotation(x))+p.current.y-p.previous.y-delta_y(x));
            grad[4] += 2*(p.current.x-center_x(x))
                    *sin(rotation(x))
                    *(-(p.current.x-center_x(x))
                    *cos(rotation(x))+p.current.x-p.previous.x-delta_x(x))

                    -2*(p.current.y-center_y(x))
                    *cos(rotation(x))
                    *((p.current.y-center_y(x))
                    *-1*sin(rotation(x))+p.current.y-p.previous.y-delta_y(x));
        }
    }

    return error_sum / points->size();
}

double f(const Vector &x, std::vector<MatchingPoints> * points) {
    double error_sum = 0;
    for(size_t i=0; i<points->size(); i++) {
        MatchingPoints p(points->at(i));
        double x_error = p.current.x - p.previous.x
                - (delta_x(x) + (p.current.x - center_x(x)) * cos(rotation(x)));
        double y_error = p.current.y - p.previous.y
                - (delta_y(x) + (p.current.y - center_y(x)) * sin(rotation(x)));
        error_sum += x_error*x_error + y_error*y_error;
    }
    return sqrt(error_sum);
}

double optimization_problem_sqrt(const Vector &x, Vector &grad, void * arg) {
    std::vector<MatchingPoints> * points = static_cast<std::vector<MatchingPoints>*>(arg);
    double f_value = f(x, points);

    const double dx = 1e-2;

    for( size_t i=0; i<grad.size(); i++ ) {
        Vector x_plus(x);
        x_plus[i] += dx;
        grad[i] = (f(x_plus, points) - f(x, points)) / dx;
        printf("grad[%lu] = %f\n", i, grad[i]);
    }

    return f_value;
}

MovementDetector::MovementDetector() {
    this->last_known_position = VehiclePosition();
}

VehiclePosition MovementDetector::process(std::vector<MatchingPoints> points) {
    nlopt::opt solver(nlopt::algorithm::LN_BOBYQA, 5);
//    nlopt::opt solver(nlopt::algorithm::LN_COBYLA, 5);
//    nlopt::opt solver(nlopt::algorithm::LN_NEWUOA_BOUND, 5);
//    nlopt::opt solver(nlopt::algorithm::LD_TNEWTON, 5);
//    nlopt::opt solver(nlopt::algorithm::LD_MMA, 5);
//    nlopt::opt solver(nlopt::algorithm::GN_ESCH, 5);
    solver.set_min_objective(optimization_problem_sqrt, (void*)&points);
//    solver.set_stopval(20);
    solver.set_ftol_rel(1e-5);
//    solver.set_xtol_rel(1e-4);
//    solver.set_xtol_abs((Vector){2, 2, 2, 2, 1e2});
    solver.set_lower_bounds((Vector){-700, -900, -2000, -2000, -5});
    solver.set_upper_bounds((Vector){700, 900, 2000, 2000, 5});

    Vector solution = (Vector){0, 0, 0, 0, 0};
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
