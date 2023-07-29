#ifndef MATRIX_MATH_H
#define MATRIX_MATH_H

#include "vector"

class Vector {
private:
    std::vector<double> storage;

public:
    Vector(int rows);
    Vector(std::vector<double> init_array);

    int rows( void );

    Vector clone( void );
    void swap_rows(int r1, int r2);
    double norm(std::vector<double> weights);
    void print( void );

    double& operator[](int index);
    Vector operator-(Vector other);
};

class Matrix {
private:
    std::vector<double> storage;
    int rows, cols;

public:
    Matrix( int rows, int cols );
    Matrix(int size);
    Matrix(int rows, int cols, std::vector<double> &storage);

    void swap_rows(int r1, int r2);
    void elim_partial(Vector &b);
    void print( void );
    double& at(int row, int col);
    
    Vector operator*(Vector &other);
};

#endif
