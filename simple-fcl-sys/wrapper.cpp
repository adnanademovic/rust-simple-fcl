#include <fcl/BVH/BVH_model.h>
#include <fcl/collision.h>
#include <fcl/distance.h>

#include "wrapper.h"

typedef fcl::BVHModel<fcl::OBBRSS> ModelData;
typedef boost::shared_ptr<ModelData> Model;

fcl_model_t fcl_model_new() {
  return new Model(new ModelData);
}

void fcl_model_free(fcl_model_t model_ptr) {
  Model* index = (Model*)model_ptr;
  delete index;
}

void fcl_model_begin(fcl_model_t model_ptr) {
  (*((Model*)model_ptr))->beginModel();
}

void fcl_model_add_triangle(fcl_model_t model_ptr, const float* p0,
                            const float* p1, const float* p2) {
  (*((Model*)model_ptr))
      ->addTriangle(fcl::Vec3f(p0[0], p0[1], p0[2]),
                    fcl::Vec3f(p1[0], p1[1], p1[2]),
                    fcl::Vec3f(p2[0], p2[1], p2[2]));
}

void fcl_model_end(fcl_model_t model_ptr) {
  (*((Model*)model_ptr))->endModel();
}

int fcl_collide(const fcl_model_t model_ptr_1, const float* rotate_1,
                const float* translate_1, const fcl_model_t model_ptr_2,
                const float* rotate_2, const float* translate_2) {
  fcl::Matrix3f r_a(rotate_1[0], rotate_1[3], rotate_1[6], rotate_1[1],
                    rotate_1[4], rotate_1[7], rotate_1[2], rotate_1[5],
                    rotate_1[8]);
  fcl::Vec3f t_a(translate_1[0], translate_1[1], translate_1[2]);
  fcl::CollisionObject a(*((Model*)model_ptr_1), fcl::Transform3f(r_a, t_a));
  fcl::Matrix3f r_b(rotate_2[0], rotate_2[3], rotate_2[6], rotate_2[1],
                    rotate_2[4], rotate_2[7], rotate_2[2], rotate_2[5],
                    rotate_2[8]);
  fcl::Vec3f t_b(translate_2[0], translate_2[1], translate_2[2]);
  fcl::CollisionObject b(*((Model*)model_ptr_2), fcl::Transform3f(r_b, t_b));

  fcl::CollisionRequest request;
  fcl::CollisionResult result;
  fcl::collide(&a, &b, request, result);
  return result.isCollision();
}

void fcl_distance(const fcl_model_t model_ptr_1, const float* rotate_1,
                  const float* translate_1, const fcl_model_t model_ptr_2,
                  const float* rotate_2, const float* translate_2,
                  int enable_nearest_points, double rel_error, double abs_error,
                  int* success, double* distance, float* p1, float* p2) {
  fcl::Matrix3f r_a(rotate_1[0], rotate_1[1], rotate_1[2], rotate_1[3],
                    rotate_1[4], rotate_1[5], rotate_1[6], rotate_1[7],
                    rotate_1[8]);
  fcl::Vec3f t_a(translate_1[0], translate_1[1], translate_1[2]);
  fcl::CollisionObject a(*((Model*)model_ptr_1), fcl::Transform3f(r_a, t_a));
  fcl::Matrix3f r_b(rotate_2[0], rotate_2[1], rotate_2[2], rotate_2[3],
                    rotate_2[4], rotate_2[5], rotate_2[6], rotate_2[7],
                    rotate_2[8]);
  fcl::Vec3f t_b(translate_2[0], translate_2[1], translate_2[2]);
  fcl::CollisionObject b(*((Model*)model_ptr_2), fcl::Transform3f(r_b, t_b));

  fcl::DistanceRequest request(enable_nearest_points, rel_error, abs_error);
  fcl::DistanceResult result;
  fcl::distance(&a, &b, request, result);

  *distance = result.min_distance;
  *success = result.min_distance > 0.0;
  p1[0] = result.nearest_points[0][0];
  p1[1] = result.nearest_points[0][1];
  p1[2] = result.nearest_points[0][2];
  p2[0] = result.nearest_points[1][0];
  p2[1] = result.nearest_points[1][1];
  p2[2] = result.nearest_points[1][2];
}
