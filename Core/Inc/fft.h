/*
 * fft.h
 *
 *  Created on: Nov 24, 2023
 *      Author: harryjjacobs
 */

#ifndef INC_FFT_H_
#define INC_FFT_H_

#include "complex.h"

double calc_amplitude(const Complex *complex, double n);

double calc_phase(const Complex *complex);

void fft(Complex *in, Complex *out, unsigned int n);

#endif /* INC_FFT_H_ */
