#ifndef LINALG_UTILS_HPP
#define LINALG_UTILS_HPP

#include <cmath>
#include <array>
#include <glm/glm.hpp>
#include <functional>

#include <glm/glm.hpp>
#include <glm/gtc/epsilon.hpp>
#include <cmath>

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

} // namespace linalg_utils

#endif // LINALG_UTILS_HPP
