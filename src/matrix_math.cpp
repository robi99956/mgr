#include <math.h>
#include <stdio.h>
#include <stdexcept>

#include "matrix_math.h"

Vector::Vector(int rows) {
    this->storage = std::vector<double>(rows, 0);
}

Vector::Vector(std::vector<double> init_array) {
    this->storage = init_array;
}

int Vector::rows( void ) {
    return this->storage.size();
}

Vector Vector::clone( void ) {
    std::vector<double> array_copy = std::vector<double>(this->storage);
    return Vector(array_copy);
}

void Vector::swap_rows(int r1, int r2) {
    if( r1 == r2 ) {
        return;
    }

    double tmp = this->storage[r1];
    this->storage[r1] = this->storage[r2];
    this->storage[r2] = tmp;
}

double Vector::norm(std::vector<double> weights) {
    double sum = 0;
    for( int i=0; i<this->rows(); i++ ) {
        double d = this->storage[i] * weights[i];
        sum += d*d;
    }

    return sqrt(sum);
}

void Vector::print( void ) {
    for( int i=0; i<this->rows(); i++ ) {
        printf("%f | ", this->storage[i]);
        printf("\n");
    }
}

double& Vector::operator[](int index) {
    return this->storage.at(index);
}

Vector Vector::operator-(Vector other) {
    Vector v = Vector(this->rows());
    for( int i=0; i<this->rows(); i++) {
        v[i] = this->storage[i] - other[i];
    }

    return v;
}

Matrix::Matrix( int rows, int cols ) {
    this->rows = rows;
    this->cols = cols;
    this->storage = std::vector<double>(rows*cols, 0);
}

Matrix::Matrix(int size) {
    this->rows = size;
    this->cols = size;
    this->storage = std::vector<double>(size*size, 0);
    for( int i=0; i<size; i++ ) {
        this->at(i, i) = 1;
    }
}

Matrix::Matrix(int rows, int cols, std::vector<double> &storage) {
    this->rows = rows;
    this->cols = cols;
    this->storage = std::vector<double>(storage);
}

void Matrix::swap_rows(int r1, int r2) {
    if( r1 == r2 ) {
        return;
    }

    int first_r1 = r1*this->cols;
    int first_r2 = r2*this->cols;

    for( int i=0; i<this->cols; i++ ) {
        double tmp = this->storage[first_r1+i];
        this->storage[first_r1+i] = this->storage[first_r2+i];
        this->storage[first_r2+i] = tmp;
    }
}

void Matrix::elim_partial(Vector &b) {
    for( int diag = 0; diag < rows; diag++ ) {
        int max_row = diag;
        double max_val = abs(this->at(diag, diag));
        double d = 0;

        for( int row = diag+1; row<this->rows; row++ ) {
            if( (d=abs(this->at(row, diag))) > max_val ) {
                max_row = row;
                max_val = d;
            }
        }

        this->swap_rows(diag, max_row);
        b.swap_rows(diag, max_row);

        double invd = 1.0 / this->at(diag, diag);

        for( int col = diag; col < this->cols; col++ ) {
            this->at(diag, col) *= invd;
        }
        b[diag] *= invd;

        for( int row = 0; row<rows; row++ ) {
            d = this->at(row, diag);
            if( row != diag ) {
                for( int col = diag; col < cols; col++ ) {
                    this->at(row, col) -= d*this->at(diag, col);
                }
                b[row] -= d*b[diag];
            }
        }
    }
}

void Matrix::print( void ) {
    for( int i=0; i<this->rows; i++ ) {
        for( int j=0; j<this->cols; j++ ) {
            printf("%f | ", this->at(i, j));
        }
        printf("\n");
    }
}

Vector Matrix::operator*(Vector &other) {
    if( this->cols != other.rows() ) {
        throw std::invalid_argument("Matrix dimensions invalid");
    }

    Vector v = Vector(other.rows());

    for( int i=0; i<this->rows; i++ ) {
        double sum = 0;
        for( int j=0; j<other.rows(); j++ ) {
            sum += this->at(i, j) * other[j];
        }
        v[i] = sum;
    }

    return v;
}

double& Matrix::at(int row, int col) {
    return this->storage.at(row*this->cols + col);
}
