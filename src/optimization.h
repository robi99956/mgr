#ifndef OPTIMIZATION_H
#define OPTIMIZATION_H

#include "matrix_math.h"

class OptimizationProblem {
public:
    virtual double f(int index, Vector &x) = 0;
    virtual double df(int index, int derivative, Vector &x) = 0;
    virtual Vector weights( void ) = 0;
    virtual int variable_count( void ) = 0;
    virtual int row_count( void ) = 0;
};

extern Vector newton_raphson(int size, 
        OptimizationProblem * problem,
        Vector start,
        double eps);

Vector newton_raphson_nonsquare(
        OptimizationProblem * problem,
        Vector start,
        double eps);

#endif
