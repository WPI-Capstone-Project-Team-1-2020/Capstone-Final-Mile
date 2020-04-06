// Component
#include "PIDController.hpp"

// Standard
#include <utility>

namespace vi
{

PIDController::PIDController(const PIDConfig& cfg) :
    m_cfg{cfg},
    m_last_time_s{ros::Time(0.0)},
    m_ctl_buffer{boost::circular_buffer<float64_t>(cfg.getControlBufferSize())}
{}

PIDController::~PIDController() = default;

void PIDController::update(const ros::Time& now_s)
{
    updateDtS(now_s);
    updateError();
    m_output = filterOutput(m_setpoint + calculateP() + calculateI() + calculateD());
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

float64_t PIDController::filterOutput(const float64_t input)
{
    m_ctl_buffer.push_back(input);
    return std::accumulate(m_ctl_buffer.begin(), m_ctl_buffer.end(), 0.0)/m_ctl_buffer.size();
}

} // namespace vi