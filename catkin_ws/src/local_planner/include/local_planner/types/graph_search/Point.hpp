#ifndef PLANNING_POINT_HPP
#define PLANNING_POINT_HPP

// Libraries
#include <boost/cstdfloat.hpp>

namespace local_planner
{

using float64_t = boost::float64_t; ///< Alias for 64 bit float

/// @brief Class to store information about a point in space
class Point
{
public:
    /// @brief Default constructor
    Point(){}

    /// @brief Overloaded constructor to take in x and y
    /// @param x X-value
    /// @param y Y-value
    Point(const float64_t x, const float64_t y) :
        m_x{x},
        m_y{y}
    {}

    /// @brief Default destructor
    ~Point() = default;

    /// @brief Accessor
    /// @return Val
    /// @{
    float64_t getX() const noexcept {return m_x;}
    float64_t getY() const noexcept {return m_y;}
    /// @}

    /// @brief Mutator
    /// @param Val val
    /// @{
    void setX(const float64_t val) noexcept {m_x = val;}
    void setY(const float64_t val) noexcept {m_y = val;}
    /// @}

private:
    float64_t m_x{0.0}; ///< X value
    float64_t m_y{0.0}; ///< Y value
};

} // namespace local_planner

#endif // PLANNING_POINT_HPP
