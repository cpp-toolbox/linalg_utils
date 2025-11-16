#ifndef LINALG_UTILS_HPP
#define LINALG_UTILS_HPP

#include <cmath>
#include <array>
#include <glm/glm.hpp>
#include <functional>

#include <glm/glm.hpp>
#include <glm/gtc/epsilon.hpp>
#include <cmath>

#include "sbpt_generated_includes.hpp"

namespace linalg_utils {

/* RIGHT HAND RULE FOR CROSS PRODUCT
 *
 *                 (A x B)
 *
 *                    |
 *                    |
 *                    |
 *                    |
 *                    |
 *                    o----------   B
 *                   /
 *                  /
 *                 /
 *                /
 *
 *             A
 *
 */

std::function<glm::vec4(const glm::vec4 &)> make_matrix_multiplier(const glm::mat4 &m);

// NOTE: a plane can be specified by the equation (n . x) = 0 where n is the normal vector of the plane, this can
// represent any plane going through the origin. If we want to allow the plane to instead pass through some arbitrary
// point p, then the equation becomes (n . (p - x)) = 0 and now p is guarenteed to be on the plane, so we're offseting
// the plane by p the equation can then become n.p - n.x = 0 which is equivalent to n.x + d = 0 fs d in R, so the most
// compact way to express a plane is by n, d which is 4d vector

class Plane {
  public:
    Plane(const glm::vec3 &n, const glm::vec3 &point) : normal(glm::normalize(n)), point_on_plane(point) {}

    Plane(const glm::vec3 &p0, const glm::vec3 &p1, const glm::vec3 &p2) : point_on_plane(p0) {
        glm::vec3 edge1 = p1 - p0;
        glm::vec3 edge2 = p2 - p0;
        normal = glm::normalize(glm::cross(edge1, edge2));
    }

    glm::vec3 get_normal() const { return normal; }
    glm::vec3 get_point() const { return point_on_plane; }

    float signed_distance(const glm::vec3 &point) const { return glm::dot(normal, point - point_on_plane); }

    enum class Side { OnPlane, NormalSide, OppositeSide };

    Side classify_point(const glm::vec3 &point, float eps = 1e-6f) const {
        float dist = signed_distance(point);
        if (glm::epsilonEqual(dist, 0.0f, eps))
            return Side::OnPlane;
        return (dist > 0.0f) ? Side::NormalSide : Side::OppositeSide;
    }

  private:
    glm::vec3 normal;         // normalized normal
    glm::vec3 point_on_plane; // reference point on plane
};

// a billboard matrix is one such that it takes the basis and transforms it to be the basis that is passed in
// this allows you to take quads and make them face the camera very easily which is the main use case
// NOTE: look doesn't always have to be the direction that the camera is facing, sometimes you might want to instead
// just get the vector that goes from the cameras position to the object as the look vector.

/**
 * @brief Creates a transformation matrix from a position and a look direction.
 *
 * This function constructs a 4x4 transformation matrix that represents a translation
 * to a given position and an orientation where the local forward direction points
 * along the specified look vector. The up direction is adjusted to maintain
 * orthogonality with the forward and right vectors.
 *
 * @param position The position to translate to in world space.
 * @param look_vector The forward direction to look towards. This vector will be normalized.
 * @param up_hint A hint for the up direction, used to compute a right vector and
 * orthogonal up vector.
 *
 * @return glm::mat4 The resulting 4x4 transformation matrix where:
 * - Column 0: Right vector
 * - Column 1: Up vector
 * - Column 2: Forward vector
 * - Column 3: Position
 *
 * @note The function ensures that the resulting axes are orthogonal and normalized.
 * The input `look_vector` must not be zero. The `up_hint` should not be parallel
 * to `look_vector` to avoid a degenerate right vector.
 */
glm::mat4 create_translation_and_look_transform(const glm::vec3 &position, const glm::vec3 &look_vector,
                                                const glm::vec3 &up_hint = glm::vec3(0.0f, 1.0f, 0.0f));

// NOTE: I don't think this is used anymore
glm::mat4 change_of_basis_move_y_to_look_dir(const glm::vec3 &position, const glm::vec3 &look_vector,
                                             const glm::vec3 &up_hint = glm::vec3(0.0f, 1.0f, 0.0f));

// NOTE: this is actually one that takes in a look vector and returns back
// a matrix that makes an object look in that direction, this is useful if you just want everything to appear flat
// towards you instead of having that effect where the thing moves along a cylinder around you.

glm::mat4 get_translation_transform_matrix(const glm::vec3 &position);

/**
 * @brief Creates a rotation matrix for a billboard transform.
 *
 * This function generates a 4x4 rotation matrix that orients an object
 * to face a particular direction, typically used for camera-facing billboards.
 * The orientation is defined by the given right, up, and look vectors.
 *
 * @param right The right vector of the billboard's local coordinate system.
 * @param up The up vector of the billboard's local coordinate system.
 * @param look The look (forward) vector of the billboard.
 *             The matrix inverts this vector internally to ensure proper facing.
 * @return A glm::mat4 representing the billboard's rotation transform.
 *
 * @note The returned matrix contains only rotation. Translation should be applied separately.
 */
glm::mat4 create_billboard_transform(const glm::vec3 &right, const glm::vec3 &up, const glm::vec3 &look);
/**
 * @brief Creates a rotation matrix to orient an object to face a given direction.
 *
 * This function generates a 4x4 rotation matrix that rotates an object so that its
 * forward direction aligns with the specified `look` vector. It automatically
 * computes the right and up vectors to form an orthonormal basis, using the world
 * Y-axis as a reference for up. This is commonly used for billboards or camera-facing objects.
 *
 * @param look The desired forward (look) direction of the object.
 *             The object will be rotated to face this direction.
 * @return A glm::mat4 representing the rotation transform that aligns the object
 *         with the `look` vector. The matrix contains only rotation; translation
 *         should be applied separately.
 *
 * @note The look vector is inverted in the matrix (-look) so that the object
 *       faces the given direction correctly.
 *
 * @example
 * glm::vec3 target_dir = glm::normalize(camera_position - object_position);
 * glm::mat4 rotation = create_billboard_transform(target_dir);
 * glm::mat4 transform = glm::translate(glm::mat4(1.0f), object_position) * rotation;
 */
glm::mat4 create_billboard_transform(const glm::vec3 &look);

/**
 * @brief Creates a rotation matrix for a billboard that locks one axis.
 *
 * This function generates a 4x4 rotation matrix that rotates an object to face
 * the given `look` direction while keeping the specified `lock_axis` fixed.
 * This is useful for billboards that should rotate freely around one axis
 * (e.g., a Y-axis lock for upright sprites) but always face a target.
 *
 * @param lock_axis The axis to remain fixed in world space (e.g., Y-axis for upright objects).
 *                  This axis is used as the up vector in the rotation matrix.
 * @param look The desired forward (look) direction of the object.
 *             The object will be rotated to face this direction while respecting the lock axis.
 * @return A glm::mat4 representing the rotation transform that aligns the object
 *         with the `look` vector while keeping the `lock_axis` fixed. Contains only rotation.
 *
 * @note The look vector is inverted in the matrix (-look) so the object faces the given direction.
 *
 * @example
 * glm::vec3 lock_axis(0.0f, 1.0f, 0.0f); // Keep upright along Y
 * glm::vec3 target_dir = glm::normalize(camera_position - object_position);
 * glm::mat4 rotation = create_billboard_transform_with_lock_axis(lock_axis, target_dir);
 * glm::mat4 transform = glm::translate(glm::mat4(1.0f), object_position) * rotation;
 */
enum struct LockAxisBinding { UP, RIGHT, FORWARD };
glm::mat4 create_billboard_transform_with_lock_axis(const glm::vec3 &lock_axis, const glm::vec3 &look,
                                                    LockAxisBinding lock_axis_binding = LockAxisBinding::UP);

/*
                   ooo OOO OOO ooo
               oOO        X        OOo
           oOO   \        X turns  /   OOo
        oOO       \       X  |    /       OOo
      oOO          \      X  |   /          OOo
    oOO             \     X  v  /             OOo
   oOO               \    X----/               OOo
  oOO                 \   X   /                 OOo
 oOO                   \  X  /                   OOo
 oOO                    \ X /                    OOo
 oOO                     \X/                     OOo
 NOTE: note that the allowable vectors span 2 * turns because you can go clockwise or counter clockwise
*/
bool angle_between_vectors_is_within(glm::vec3 v, glm::vec3 w, double turns);

/*
                   ooo OOO OOO ooo
               oOO        X        OOo
           oOO   \        X turns  /   OOo
        oOO       \       C  |    /       OOo
      oOO          \      E  |   /          OOo
    oOO             \     N  v  /             OOo
   oOO               \----T----/               OOo
  oOO                 \   E   /                 OOo
 oOO                   \  R  /                   OOo
 oOO                    \ X /                    OOo
 oOO                     \X/                     OOo
*/
bool vector_is_within_centered_sector(glm::vec3 center, glm::vec3 other, double sector_angle_turns);

} // namespace linalg_utils

#endif // LINALG_UTILS_HPP
