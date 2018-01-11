#ifndef WRAPPER_H
#define WRAPPER_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef void* fcl_model_t;

fcl_model_t fcl_model_new();
void fcl_model_free(fcl_model_t model_ptr);

void fcl_model_begin(fcl_model_t model_ptr);
void fcl_model_add_triangle(fcl_model_t model_ptr, const float* p0,
                            const float* p1, const float* p2);
void fcl_model_end(fcl_model_t model_ptr);

int fcl_collide(const fcl_model_t model_ptr_1, const float* rotate_1,
                const float* translate_1, const fcl_model_t model_ptr_2,
                const float* rotate_2, const float* translate_2);
void fcl_distance(const fcl_model_t model_ptr_1, const float* rotate_1,
                  const float* translate_1, const fcl_model_t model_ptr_2,
                  const float* rotate_2, const float* translate_2,
                  double rel_error, double abs_error, int* success,
                  double* distance);

#ifdef __cplusplus
}
#endif

#endif  // WRAPPER_H
