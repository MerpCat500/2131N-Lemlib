/**
 * @file math.hpp
 * @author Andrew Hilton (2131N)
 * @brief Helper Math functions not supplied by other libraries
 * @version 0.1
 * @date 2025-06-04
 *
 * @copyright Copyright (c) 2025
 *
 */

#pragma once

#include <numeric>
#include <vector>



/**
 * @brief Returns the sign of a number.
 * @details Returns 1 if the number is positive, -1 if negative, else it returns the value (in cases
 * of -nan, +nan, -0, +0).
 *
 * @tparam T Return Type and Type of the input
 * @param x Value to check sign of
 * @return T Returns 1 if positive, -1 if negative, or the input value itself for cases like -0 or
 * +0
 */
template <typename T>
T sign(T x)
{
  if (x > 0)  // if Value is positive
    return 1;
  else if (x < 0)  // if Value is negative
    return -1;
  else         // Value is NaN, inf, or other edgecase
    return x;  // return edgecase
}

/**
 * @brief Averages a vector
 *
 * @tparam T Type of vector elements and return type
 * @param vector Vector to be averaged
 * @return T Returns the average of the vector elements
 */
template <typename T>
T average(const std::vector<T>& vector)
{
  // Add up all values in vector and divide by the # (size) of elements in the vector
  return std::accumulate(vector.begin(), vector.end(), T{}) / static_cast<T>(vector.size());
}