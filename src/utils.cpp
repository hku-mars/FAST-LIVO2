// utils.cpp
#include <vector>
#include <cstdint> // for int64_t
#include <limits>  // for std::numeric_limits
#include <stdexcept> // for std::out_of_range

std::vector<int> convertToIntVectorSafe(const std::vector<int64_t>& int64_vector) {
    std::vector<int> int_vector;
    int_vector.reserve(int64_vector.size()); // 预留空间以提高效率

    for (int64_t value : int64_vector) {
        if (value < std::numeric_limits<int>::min() || value > std::numeric_limits<int>::max()) {
            throw std::out_of_range("Value is out of range for int");
        }
        int_vector.push_back(static_cast<int>(value));
    }

    return int_vector;
}
