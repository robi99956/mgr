#ifndef OPTIMIZATION_H
#define OPTIMIZATION_H

#include "matrix_math.h"

class OptimizationProblem {
public:
    virtual double f(int index, Vector &x);
    virtual double df(int index, int derivative, Vector &x);
    virtual std::vector<double> weights( void );
};

extern Vector newton_raphson(int size, 
        OptimizationProblem &problem,
        Vector start,
        double eps);

#endif
