#ifndef FAST_LIO_STARTUP_FRESHNESS_GUARD_HPP_
#define FAST_LIO_STARTUP_FRESHNESS_GUARD_HPP_

#include <cmath>
#include <cstdint>
#include <limits>
#include <stdexcept>

/**
 * Startup-only admission policy for synchronized FAST-LIO measurements.
 *
 * The policy deliberately does not inspect point data or estimator state.  It
 * consumes synchronized lidar_end_time values and returns whether the caller
 * may enter the existing estimator path.  Once admitted, every subsequent
 * group follows the legacy path, including stale groups; runtime freshness is
 * intentionally monitored outside FAST-LIO.
 */
class StartupFreshnessGuard
{
public:
    enum class IngressDecision
    {
        ACCEPT,
        DROP_STALE,
    };

    enum class Decision
    {
        PROCESS,
        DROP_STALE,
        WAIT_FRESH_WINDOW,
        ADMIT,
    };

    enum class RejectionReason
    {
        NONE,
        STALE,
        NONFINITE_TIMESTAMP,
        FUTURE_TIMESTAMP,
        TIMESTAMP_REVERSAL,
    };

    StartupFreshnessGuard(const bool enabled, const double max_age_sec, const int required_consecutive_groups)
    : enabled_(enabled), max_age_sec_(max_age_sec), required_consecutive_groups_(required_consecutive_groups)
    {
        if (!std::isfinite(max_age_sec_) || max_age_sec_ <= 0.0)
        {
            throw std::invalid_argument("startup_max_lidar_age_sec must be finite and > 0");
        }
        if (required_consecutive_groups_ <= 0)
        {
            throw std::invalid_argument("startup_fresh_consecutive_groups must be > 0");
        }
    }

    Decision observe(const double now_sec, const double lidar_end_time_sec)
    {
        last_rejection_reason_ = RejectionReason::NONE;

        if (!enabled_ || admitted_)
        {
            return Decision::PROCESS;
        }

        if (!std::isfinite(now_sec) || !std::isfinite(lidar_end_time_sec))
        {
            fresh_group_count_ = 0;
            ++invalid_timestamp_groups_;
            last_rejection_reason_ = RejectionReason::NONFINITE_TIMESTAMP;
            return Decision::DROP_STALE;
        }

        last_age_sec_ = now_sec - lidar_end_time_sec;
        if (!std::isfinite(last_age_sec_))
        {
            fresh_group_count_ = 0;
            ++invalid_timestamp_groups_;
            last_rejection_reason_ = RejectionReason::NONFINITE_TIMESTAMP;
            return Decision::DROP_STALE;
        }

        if (has_last_lidar_end_time_ && lidar_end_time_sec < last_lidar_end_time_sec_)
        {
            fresh_group_count_ = 0;
            ++invalid_timestamp_groups_;
            last_rejection_reason_ = RejectionReason::TIMESTAMP_REVERSAL;
            return Decision::DROP_STALE;
        }
        last_lidar_end_time_sec_ = lidar_end_time_sec;
        has_last_lidar_end_time_ = true;

        // Future sensor timestamps are not treated as fresh.  This keeps the
        // admission rule conservative and explicit across ROS clock domains.
        if (last_age_sec_ < 0.0)
        {
            fresh_group_count_ = 0;
            ++invalid_timestamp_groups_;
            last_rejection_reason_ = RejectionReason::FUTURE_TIMESTAMP;
            return Decision::DROP_STALE;
        }

        // The boundary is inclusive: age == max_age_sec is fresh, while any
        // greater age is stale.
        if (last_age_sec_ > max_age_sec_)
        {
            fresh_group_count_ = 0;
            ++stale_groups_dropped_;
            last_rejection_reason_ = RejectionReason::STALE;
            return Decision::DROP_STALE;
        }

        ++fresh_group_count_;
        if (fresh_group_count_ >= required_consecutive_groups_)
        {
            admitted_ = true;
            return Decision::ADMIT;
        }
        return Decision::WAIT_FRESH_WINDOW;
    }

    /**
     * Cheap startup-only filter for the raw Livox message header.
     *
     * This intentionally does not increment the synchronized fresh-group
     * count: only a synchronized lidar_end_time may establish admission.
     * A rejected raw message does, however, reset that count so a hidden
     * stale callback cannot be bridged across a fresh window.
     */
    IngressDecision observe_ingress(const double now_sec, const double header_time_sec)
    {
        if (!enabled_ || admitted_)
        {
            return IngressDecision::ACCEPT;
        }

        if (!std::isfinite(now_sec) || !std::isfinite(header_time_sec))
        {
            reset_fresh_window();
            ++ingress_invalid_timestamp_groups_;
            last_ingress_rejection_reason_ = RejectionReason::NONFINITE_TIMESTAMP;
            return IngressDecision::DROP_STALE;
        }

        const double age_sec = now_sec - header_time_sec;
        last_ingress_age_sec_ = age_sec;
        if (!std::isfinite(age_sec))
        {
            reset_fresh_window();
            ++ingress_invalid_timestamp_groups_;
            last_ingress_rejection_reason_ = RejectionReason::NONFINITE_TIMESTAMP;
            return IngressDecision::DROP_STALE;
        }

        if (has_last_ingress_header_time_ && header_time_sec < last_ingress_header_time_sec_)
        {
            reset_fresh_window();
            ++ingress_invalid_timestamp_groups_;
            last_ingress_rejection_reason_ = RejectionReason::TIMESTAMP_REVERSAL;
            return IngressDecision::DROP_STALE;
        }
        last_ingress_header_time_sec_ = header_time_sec;
        has_last_ingress_header_time_ = true;

        // Header time is the scan/base timestamp, so this is deliberately a
        // conservative startup filter.  The synchronized lidar_end_time
        // check in observe() remains the final admission authority.
        if (age_sec < 0.0)
        {
            reset_fresh_window();
            ++ingress_invalid_timestamp_groups_;
            last_ingress_rejection_reason_ = RejectionReason::FUTURE_TIMESTAMP;
            return IngressDecision::DROP_STALE;
        }
        if (age_sec > max_age_sec_)
        {
            reset_fresh_window();
            ++ingress_stale_dropped_;
            last_ingress_rejection_reason_ = RejectionReason::STALE;
            return IngressDecision::DROP_STALE;
        }

        last_ingress_rejection_reason_ = RejectionReason::NONE;
        return IngressDecision::ACCEPT;
    }

    bool enabled() const { return enabled_; }
    bool admitted() const { return admitted_; }
    int fresh_group_count() const { return fresh_group_count_; }
    std::uint64_t stale_groups_dropped() const { return stale_groups_dropped_; }
    std::uint64_t invalid_timestamp_groups() const { return invalid_timestamp_groups_; }
    std::uint64_t ingress_stale_dropped() const { return ingress_stale_dropped_; }
    std::uint64_t ingress_invalid_timestamp_groups() const { return ingress_invalid_timestamp_groups_; }
    double last_age_sec() const { return last_age_sec_; }
    double last_ingress_age_sec() const { return last_ingress_age_sec_; }
    RejectionReason last_rejection_reason() const { return last_rejection_reason_; }
    RejectionReason last_ingress_rejection_reason() const { return last_ingress_rejection_reason_; }

private:
    void reset_fresh_window() { fresh_group_count_ = 0; }

    bool enabled_ = false;
    double max_age_sec_ = 0.5;
    int required_consecutive_groups_ = 10;
    bool admitted_ = false;
    bool has_last_lidar_end_time_ = false;
    int fresh_group_count_ = 0;
    std::uint64_t stale_groups_dropped_ = 0;
    std::uint64_t invalid_timestamp_groups_ = 0;
    std::uint64_t ingress_stale_dropped_ = 0;
    std::uint64_t ingress_invalid_timestamp_groups_ = 0;
    double last_lidar_end_time_sec_ = std::numeric_limits<double>::quiet_NaN();
    double last_age_sec_ = std::numeric_limits<double>::quiet_NaN();
    bool has_last_ingress_header_time_ = false;
    double last_ingress_header_time_sec_ = std::numeric_limits<double>::quiet_NaN();
    double last_ingress_age_sec_ = std::numeric_limits<double>::quiet_NaN();
    RejectionReason last_rejection_reason_ = RejectionReason::NONE;
    RejectionReason last_ingress_rejection_reason_ = RejectionReason::NONE;
};

#endif  // FAST_LIO_STARTUP_FRESHNESS_GUARD_HPP_
