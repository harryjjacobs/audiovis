/*
 * utils.h
 *
 *  Created on: 2 May 2024
 *      Author: harryjjacobs
 */

#ifndef INC_UTILS_H_
#define INC_UTILS_H_

#define PI 3.14159265

double min(double a, double b) {
  if (a <= b) {
    return a;
  }
  return b;
}

double max(double a, double b) {
  if (a >= b) {
    return a;
  }
  return b;
}

#endif /* INC_UTILS_H_ */
