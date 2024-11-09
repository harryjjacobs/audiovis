/*
 * complex.h
 *
 *  Created on: Nov 24, 2023
 *      Author: harryjjacobs
 */

#ifndef INC_COMPLEX_H_
#define INC_COMPLEX_H_

// represents a complex number
typedef struct {
  double real;
  double imag;
} Complex;

Complex complex_mult(Complex lhs, Complex rhs);

Complex complex_add(Complex lhs, Complex rhs);

Complex complex_subtract(Complex lhs, Complex rhs);

Complex complex_negate(Complex c);

#endif /* INC_COMPLEX_H_ */
