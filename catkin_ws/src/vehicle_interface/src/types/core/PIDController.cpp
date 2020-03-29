// Component
#include "PIDController.hpp"

// Standard
#include <utility>

namespace vi
{

PIDController::PIDController(const PIDConfig& cfg) :
    m_cfg{cfg},
    m_last_time_s{ros::Time(0.0)}
{}

PIDController::~PIDController() = default;

void PIDController::update(const ros::Time& now_s)
{
    updateDtS(now_s);
    updateError();
    m_output = m_setpoint + calculateP() + calculateI() + calculateD();
    // std::cout << "error: " << m_error << " setpoint: " << m_setpoint << " p: " << calculateP() << " i: " << calculateI() << " d: " << calculateD() << " dt: " << m_dt_s.toSec() << std::endl;
    m_last_error = m_error;
    m_last_time_s = now_s;
}

void PIDController::updateDtS(const ros::Time& now_s) noexcept
{
    m_dt_s = now_s - m_last_time_s;

    if (m_dt_s.toSec() > m_cfg.getMaxDtS())
    {
        m_dt_s = ros::Duration(m_cfg.getMaxDtS());
    }
}

void PIDController::updateError() noexcept
{
    m_error = m_setpoint - m_feedback;
}

float64_t PIDController::calculateP() const noexcept
{
    return m_error*m_cfg.getPGain();
}

float64_t PIDController::calculateI() noexcept
{
    m_integral += m_error*m_dt_s.toSec()*m_cfg.getIGain();

    return m_integral;
}

float64_t PIDController::calculateD() const noexcept
{
    float64_t d = (m_error - m_last_error)/m_dt_s.toSec()*m_cfg.getDGain();
    if ((std::isnan(d)) || (std::isinf(d)))
    {
        d = 0.0;
    }

    return d;
}

} // namespace vi