/*
 * fft.c
 *
 *  Created on: Nov 24, 2023
 *      Author: harryjjacobs
 */


#include <math.h>

#include "complex.h"

#define PI 3.14159265

double calc_amplitude(const Complex *complex, double n) {
  // the magnitude of the complex number, normalised by the number of sample
  // points
  double magnitude =
      sqrt(complex->real * complex->real + complex->imag * complex->imag);
  return magnitude / n;
}

double calc_phase(const Complex *complex) {
  return atan2(complex->imag, complex->real);
}

// https://en.wikipedia.org/wiki/Cooley%E2%80%93Tukey_FFT_algorithm#Pseudocode
// Note to self as this seems to have just clicked in my head:
// The time-saving in the FFT comes from the fact that we only need to calculate
// DFT for N/2 each time, so at each recursion of the function we have a time
// saving of a half, because of the periodic nature of sin and cos. We know
// X_k+N/2 without iteration of k further because a jump by a multiple of PI
// means we are in the negative part of the "wave" that is symmetric to to the
// positive part in 0..N/2
void fftrec(Complex *in, Complex *out, unsigned int n, unsigned int stride) {
  if (n == 1) {
    // at the lowest level division of N
    *out = *in;
    return;
  }
  // recursively split in two
  fftrec(in, out, n / 2, 2 * stride);                   // even: 0..N/2-1
  fftrec(in + stride, out + n / 2, n / 2, 2 * stride);  // odd: (N/2..N-1)
  // combine even and odd 'half' DFTs into the full DFT
  for (int k = 0; k < n / 2; k++) {
    double angle = -2.0 * PI * (double)k / n;
    Complex exp_factor = {.real = cos(angle),
                          .imag = sin(angle)};  // e^-(2pi/n)k
    Complex twiddle = complex_mult(exp_factor, out[k + n / 2]);
    // out[k] is the 'even' half, out[k + n / 2] is the 'odd' half
    out[k + n / 2] = complex_subtract(out[k], twiddle);
    out[k] = complex_add(out[k], twiddle);
  }
}

void fft(Complex *in, Complex *out, unsigned int n) {
  fftrec(in, out, n, 1);
}
