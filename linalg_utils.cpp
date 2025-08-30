#include "linalg_utils.hpp"

namespace linalg_utils {

std::function<glm::vec4(const glm::vec4 &)> make_matrix_multiplier(const glm::mat4 &m) {
    return [m](const glm::vec4 &v) -> glm::vec4 { return m * v; };
}
} // namespace linalg_utils
