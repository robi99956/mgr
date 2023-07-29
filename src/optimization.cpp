#include "optimization.h"

Vector newton_raphson(int size, 
        OptimizationProblem &problem,
        Vector start,
        double eps) {
    
    Vector X = start.clone();
    Vector F = Vector(size);
    Matrix J = Matrix(size, size);

    do {
        for( int i=0; i<size; i++ ) {
            F[i] = problem.f(i, X);
        }

        for( int i=0; i<size; i++ ) {
            for( int j=0; j<size; j++ ) {
                J.at(i, j) = problem.df(i, j, X);
            }
        }
        J.elim_partial(F);
        X = X - F;
    } while( F.norm(problem.weights()) > eps );

    return X;
}
