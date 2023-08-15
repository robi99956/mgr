#include <iostream>
#include "optimization.h"

/*
Vector newton_raphson(int size, 
        OptimizationProblem * problem,
        Vector start,
        double eps) {
    
    Vector X = start.clone();
    Vector F = Vector(size);
    Matrix J = Matrix(size, size);

    do {
        for( int i=0; i<size; i++ ) {
            F[i] = problem->f(i, X);
        }

        for( int i=0; i<size; i++ ) {
            for( int j=0; j<size; j++ ) {
                J.at(i, j) = problem->df(i, j, X);
            }
        }
        J.elim_partial(F);
        X = X - F;
    } while( F.norm(problem->weights()) > eps );

    return X;
}
*/

double vector_norm(Vector &x, Vector weights) {
    double sum = 0;
    for( int i=0; i<x.rows(); i++ ) {
        double d = x[i] * weights[i];
        sum += d*d;
    }

    return sqrt(sum);
}

Vector newton_raphson_nonsquare(
        OptimizationProblem * problem,
        Vector start,
        double eps) {

    int equations = problem->row_count();
    int vars = problem->variable_count();

    Matrix J = Matrix::Zero(equations, vars);
    Vector F = Vector::Zero(equations);
    Vector X = start;

    for( int i=0; i<10; i++ ) {
        for( int i=0; i<equations; i++ ) {
            F[i] = problem->f(i, X);
        }

        for( int var=0; var<vars; var++ ) {
            for( int equation=0; equation < equations; equation++ ) {
                J.coeffRef(equation, var) = problem->df(equation, var, X);
            }
        }

        std::cout << "F = \n" << F << std::endl << std::endl;
        std::cout << "X = \n" << X << std::endl << std::endl;
        std::cout << "J = \n" << J << std::endl << std::endl;

        X = X - J.completeOrthogonalDecomposition().pseudoInverse() * F;
        if( vector_norm(F, problem->weights()) <= eps ) {
            break;
        }
    }

    return X;
}
