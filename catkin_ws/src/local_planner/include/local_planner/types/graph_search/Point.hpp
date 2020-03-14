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

namespace std
{
/// @brief Template specialization of hash for Point
template <>
class hash<local_planner::Point>
{
public:
    /// @brief Default operator for hashing points
    /// @param point The point to be hashed
    /// @return hash
    size_t operator()(const local_planner::Point& point) const noexcept
    {
        const std::int64_t x_m    = static_cast<std::int64_t>(std::floor(point.getX()));
        const std::int64_t y_m    = static_cast<std::int64_t>(std::floor(point.getY()));
        const std::size_t  x_hash = hash<std::int64_t>()(x_m);
        const std::size_t  y_hash = hash<std::int64_t>()(y_m);
        
        std::size_t seed = 0U;
        boost::hash_combine(seed, x_hash);
        boost::hash_combine(seed, y_hash);
        
        return hash<std::size_t>()(seed);
    }
};

} // namespace std

#endif // PLANNING_POINT_HPP
