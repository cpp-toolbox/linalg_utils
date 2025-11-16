#include "linalg_utils.hpp"
#define GLM_ENABLE_EXPERIMENTAL
#include <glm/gtx/matrix_decompose.hpp>
#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtc/quaternion.hpp>
#include <glm/gtx/quaternion.hpp>
#include <glm/gtx/vector_angle.hpp>

namespace linalg_utils {

std::function<glm::vec4(const glm::vec4 &)> make_matrix_multiplier(const glm::mat4 &m) {
    return [m](const glm::vec4 &v) -> glm::vec4 { return m * v; };
}

glm::mat4 create_translation_and_look_transform(const glm::vec3 &position, const glm::vec3 &look_vector,
                                                const glm::vec3 &up_hint) {
    // normalize the look vector (forward direction)
    glm::vec3 z_axis = glm::normalize(look_vector);
    // compute the right vector using cross product
    glm::vec3 x_axis = glm::normalize(glm::cross(up_hint, z_axis));
    // compute the up vector again to ensure orthogonality
    glm::vec3 y_axis = glm::cross(z_axis, x_axis);

    glm::mat4 transform = glm::mat4(1.0f);
    transform[0] = glm::vec4(x_axis, 0.0f);   // Right vector
    transform[1] = glm::vec4(y_axis, 0.0f);   // Up vector
    transform[2] = glm::vec4(z_axis, 0.0f);   // Forward vector
    transform[3] = glm::vec4(position, 1.0f); // Position

    return transform;
}

// TODO: in future re-use the above function, only make this change when we can asses if it works
glm::mat4 change_of_basis_move_y_to_look_dir(const glm::vec3 &position, const glm::vec3 &look_vector,
                                             const glm::vec3 &up_hint) {
    // normalize the look vector (forward direction)
    glm::vec3 y_axis = glm::normalize(look_vector);
    // compute the right vector using cross product
    glm::vec3 x_axis = glm::normalize(glm::cross(up_hint, y_axis));
    // compute the up vector again to ensure orthogonality
    glm::vec3 z_axis = glm::cross(y_axis, x_axis);

    glm::mat4 transform = glm::mat4(1.0f);
    transform[0] = glm::vec4(x_axis, 0.0f);   // Right vector
    transform[1] = glm::vec4(y_axis, 0.0f);   // Up vector
    transform[2] = glm::vec4(z_axis, 0.0f);   // Forward vector
    transform[3] = glm::vec4(position, 1.0f); // Position

    return transform;
}

glm::mat4 get_translation_transform_matrix(const glm::vec3 &position) {
    return glm::translate(glm::mat4(1.0f), position);
}

glm::mat4 create_billboard_transform(const glm::vec3 &right, const glm::vec3 &up, const glm::vec3 &look) {
    // Create the rotation matrix to orient the object
    glm::mat4 billboard_mat(1.0f);

    // the matrix has its columns sideways
    billboard_mat[0] = glm::vec4(right, 0.0f); // right vector
    billboard_mat[1] = glm::vec4(up, 0.0f);    // up vector
    billboard_mat[2] = glm::vec4(-look, 0.0f); // look vector (inverted for proper facing)

    return billboard_mat;
}

glm::mat4 create_billboard_transform(const glm::vec3 &look) {
    glm::vec3 up(0.0f, 1.0f, 0.0f);
    glm::vec3 right = glm::normalize(glm::cross(look, up));
    glm::vec3 new_up = glm::normalize(glm::cross(right, look));
    glm::mat4 billboard_mat(1.0f);
    // set the right, up, and look vectors as columns for the rotation matrix
    billboard_mat[0] = glm::vec4(right, 0.0f);  // right vector
    billboard_mat[1] = glm::vec4(new_up, 0.0f); // new up vector
    billboard_mat[2] = glm::vec4(-look, 0.0f);  // look vector (inverted for proper facing)

    return billboard_mat;
}

glm::mat4 create_billboard_transform_with_lock_axis(const glm::vec3 &lock_axis, const glm::vec3 &look,
                                                    LockAxisBinding lock_axis_binding) {
    glm::vec3 right, up, forward;
    forward = glm::normalize(look);

    switch (lock_axis_binding) {
    case LockAxisBinding::UP:
        up = glm::normalize(lock_axis);
        right = glm::normalize(glm::cross(forward, up));
        forward = glm::cross(up, right); // ensure orthogonality
        break;

    case LockAxisBinding::RIGHT:
        right = glm::normalize(lock_axis);
        up = glm::normalize(glm::cross(right, forward));
        forward = glm::cross(up, right); // ensure orthogonality
        break;

    case LockAxisBinding::FORWARD:
        forward = glm::normalize(lock_axis);
        right = glm::normalize(glm::cross(forward, glm::vec3(0.0f, 1.0f, 0.0f)));
        // handle degenerate case where lock_axis is vertical
        if (glm::length2(right) < 0.0001f) {
            right = glm::normalize(glm::cross(forward, glm::vec3(1.0f, 0.0f, 0.0f)));
        }
        up = glm::cross(forward, right);
        break;
    }

    glm::mat4 billboard_mat(1.0f);
    billboard_mat[0] = glm::vec4(right, 0.0f);
    billboard_mat[1] = glm::vec4(up, 0.0f);
    billboard_mat[2] = glm::vec4(-forward, 0.0f); // invert forward to face the look direction

    return billboard_mat;
}

bool is_rotation_translation_scale_matrix(const glm::mat4 &matrix) {
    // check if the last row is [0, 0, 0, 1]
    if (!glm::all(glm::equal(matrix[3], glm::vec4(0.0f, 0.0f, 0.0f, 1.0f), glm::epsilon<float>()))) {
        return false;
    }

    glm::vec3 scale, translation, skew;
    glm::quat rotation;
    glm::vec4 perspective;

    if (!glm::decompose(matrix, scale, rotation, translation, skew, perspective)) {
        return false;
    }

    // ensure perspective is negligible
    if (glm::length(perspective) > glm::epsilon<float>()) {
        return false;
    }

    return true;
}

bool angle_between_vectors_is_within(glm::vec3 v, glm::vec3 w, double turns) {
    if (turns < 0.0 || turns > 1.0) {
        return false; // invalid turn fraction
    }

    float angle_radians = glm::angle(glm::normalize(v), glm::normalize(w));
    return angle_radians <= turns::turns_to_radians(turns);
}

bool vector_is_within_centered_sector(glm::vec3 center, glm::vec3 other, double sector_angle_turns) {
    return angle_between_vectors_is_within(center, other, sector_angle_turns / 2.0);
}

} // namespace linalg_utils
