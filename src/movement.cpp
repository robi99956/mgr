#include "movement.h"
#include "optimization.h"

using namespace std;


static inline double delta_x(Vector &x) {return x[0];}
static inline double delta_y(Vector &x) {return x[1];}
static inline double center_x(Vector &x) {return x[2];}
static inline double center_y(Vector &x) {return x[3];}
static inline double rotation(Vector &x) {return x[4];}

MovementModel::MovementModel(Vector &x) {
    this->dx = delta_x(x);
    this->dy = delta_y(x);
    this->xo = center_x(x);
    this->yo = center_y(x);
    this->dangle = rotation(x);
}

class MovementProblem: public OptimizationProblem {
private:
    vector<MatchingPoints> matching_points;

public:
    MovementProblem( vector<MatchingPoints> &matching_points ) {
        this->matching_points = vector<MatchingPoints>(matching_points); 
    }

    double f(int index, Vector &x) {
        MatchingPoints points = this->matching_points[index/2];
        if( (index % 2) == 0 ) {
            return points.current.x - points.previous.x 
                - (delta_x(x) + (points.current.x - center_x(x)) * cos(rotation(x)));
        } else {
            return points.current.y - points.previous.y 
                - (delta_y(x) + (points.current.y - center_y(x)) * sin(rotation(x)));
        }
    }

    double df(int index, int derivative, Vector &x) {
        MatchingPoints points = this->matching_points[index/2];
        if( (index % 2) == 0 ) {
            switch( derivative ) {
                case 0: return -1;
                case 1: return 0;
                case 2: return -1*cos(rotation(x));
                case 3: return 0;
                case 4: return -1*(points.current.x - center_x(x))*sin(rotation(x));
                default: return 0;
            }
        } else {
            switch( derivative ) {
                case 0: return 0;
                case 1: return -1;
                case 2: return 0;
                case 3: return -1*sin(rotation(x));
                case 4: return (points.current.y - center_y(x))*cos(rotation(x));
                default: return 0;
            }
        }
    }

    Vector weights( void ) {
        return Vector::Ones(5);
    }

    int variable_count( void ) {
        return 5;
    }

    int row_count( void ) {
        return this->matching_points.size() * 2;
    }
};

VehiclePosition MovementDetector::process(vector<MatchingPoints> points) {
    MovementProblem problem(points);
    Vector solution = newton_raphson_nonsquare(
                &problem, 
                Vector::Zero(problem.variable_count()), 
                1e-4);
    MovementModel model(solution);

    VehiclePosition new_position(
            this->last_known_position.x + model.dx,
            this->last_known_position.y + model.dy,
            this->last_known_position.angle + model.dangle);

    this->last_known_position = new_position;
    return new_position;
}
