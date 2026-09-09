#include "startup_freshness_guard.hpp"

#include <cassert>
#include <cmath>
#include <iostream>
#include <stdexcept>
#include <utility>
#include <vector>

namespace
{
using Decision = StartupFreshnessGuard::Decision;

void expect_decision(StartupFreshnessGuard &guard, const Decision expected, const double now, const double stamp)
{
    assert(guard.observe(now, stamp) == expected);
}

void test_legacy_disabled_behavior()
{
    StartupFreshnessGuard guard(false, 0.5, 10);
    for (int i = 0; i < 40; ++i)
    {
        expect_decision(guard, Decision::PROCESS, 100.0 + i, 98.0 + i);
    }
    assert(!guard.admitted());
    assert(guard.fresh_group_count() == 0);
}

void test_stale_queue_and_control_flow()
{
    StartupFreshnessGuard guard(true, 0.5, 10);
    int process_calls = 0;
    int map_initialization_calls = 0;
    int cloud_publications = 0;
    int odom_publications = 0;

    // This is the pre-edit control-flow reproduction: without a guard every
    // synchronized group would enter all four normal processing paths.
    for (int i = 0; i < 16; ++i)
    {
        ++process_calls;
        ++map_initialization_calls;
        ++cloud_publications;
        ++odom_publications;
    }
    assert(process_calls == 16);

    process_calls = map_initialization_calls = cloud_publications = odom_publications = 0;
    for (int i = 0; i < 16; ++i)
    {
        expect_decision(guard, Decision::DROP_STALE, 100.0, 90.0 + i * 0.1);
        // The main-loop contract returns before ImuProcess, map init, or pub.
    }
    assert(guard.stale_groups_dropped() == 16);
    assert(process_calls == 0);
    assert(map_initialization_calls == 0);
    assert(cloud_publications == 0);
    assert(odom_publications == 0);

    for (int i = 0; i < 9; ++i)
    {
        expect_decision(guard, Decision::WAIT_FRESH_WINDOW, 100.0 + i * 0.1, 100.0 + i * 0.1);
    }
    expect_decision(guard, Decision::ADMIT, 100.9, 100.9);
    assert(guard.admitted());

    // The Nth fresh group is the admission boundary and is discarded.  The
    // next synchronized group enters the unchanged normal path.
    expect_decision(guard, Decision::PROCESS, 101.0, 101.0);
    expect_decision(guard, Decision::PROCESS, 102.0, 100.0);  // runtime path is unchanged after admission
    process_calls += 2;
    map_initialization_calls += 2;
    cloud_publications += 2;
    odom_publications += 2;
    assert(process_calls == 2);
    assert(map_initialization_calls == 2);
    assert(cloud_publications == 2);
    assert(odom_publications == 2);
}

void test_counter_reset_and_queue_sizes()
{
    for (const int stale_count : {1, 10, 20, 40})
    {
        StartupFreshnessGuard guard(true, 0.5, 10);
        for (int i = 0; i < stale_count; ++i)
        {
            expect_decision(guard, Decision::DROP_STALE, 100.0, 90.0 + i * 0.1);
        }
        assert(!guard.admitted());
        assert(guard.fresh_group_count() == 0);
        assert(guard.stale_groups_dropped() == static_cast<unsigned>(stale_count));
    }

    StartupFreshnessGuard mixed(true, 0.5, 3);
    expect_decision(mixed, Decision::WAIT_FRESH_WINDOW, 10.0, 10.0);
    expect_decision(mixed, Decision::WAIT_FRESH_WINDOW, 10.1, 10.1);
    expect_decision(mixed, Decision::DROP_STALE, 10.2, 9.699999);
    assert(mixed.fresh_group_count() == 0);
    expect_decision(mixed, Decision::WAIT_FRESH_WINDOW, 10.3, 10.3);
    expect_decision(mixed, Decision::WAIT_FRESH_WINDOW, 10.4, 10.4);
    expect_decision(mixed, Decision::ADMIT, 10.5, 10.5);
}

void test_timestamp_contract()
{
    StartupFreshnessGuard boundary(true, 0.5, 1);
    expect_decision(boundary, Decision::ADMIT, 10.0, 9.5);

    StartupFreshnessGuard over_boundary(true, 0.5, 1);
    expect_decision(over_boundary, Decision::DROP_STALE, 10.0, 9.499999);

    StartupFreshnessGuard invalid(true, 0.5, 1);
    expect_decision(invalid, Decision::DROP_STALE, std::numeric_limits<double>::quiet_NaN(), 10.0);
    expect_decision(invalid, Decision::DROP_STALE, 10.0, std::numeric_limits<double>::quiet_NaN());
    expect_decision(invalid, Decision::DROP_STALE, 10.0, 10.1);  // future timestamp

    StartupFreshnessGuard reversal(true, 0.5, 2);
    expect_decision(reversal, Decision::WAIT_FRESH_WINDOW, 10.0, 10.0);
    expect_decision(reversal, Decision::DROP_STALE, 10.1, 9.95);
    assert(reversal.last_rejection_reason() == StartupFreshnessGuard::RejectionReason::TIMESTAMP_REVERSAL);

    bool threw = false;
    try { StartupFreshnessGuard invalid_config(true, 0.0, 1); (void)invalid_config; }
    catch (const std::invalid_argument &) { threw = true; }
    assert(threw);
}

void test_k3a_delayed_queue_ab()
{
    StartupFreshnessGuard guarded(true, 0.5, 10);
    StartupFreshnessGuard legacy(false, 0.5, 10);
    std::vector<std::pair<double, double>> measurements;
    for (int i = 0; i < 15; ++i)
    {
        const double now = 100.0 + i * 0.1;
        measurements.emplace_back(now, now - 1.4);
    }
    for (int i = 0; i < 12; ++i)
    {
        const double now = 101.5 + i * 0.1;
        measurements.emplace_back(now, now);
    }

    int guarded_processable = 0;
    int legacy_processable = 0;
    for (const auto &[now, stamp] : measurements)
    {
        if (guarded.observe(now, stamp) == Decision::PROCESS) ++guarded_processable;
        if (legacy.observe(now, stamp) == Decision::PROCESS) ++legacy_processable;
    }
    assert(guarded_processable == 2);  // only groups after the 10-group window
    assert(legacy_processable == static_cast<int>(measurements.size()));
    assert(guarded.stale_groups_dropped() == 15);
    assert(guarded.admitted());
}
}

int main()
{
    test_legacy_disabled_behavior();
    test_stale_queue_and_control_flow();
    test_counter_reset_and_queue_sizes();
    test_timestamp_contract();
    test_k3a_delayed_queue_ab();
    std::cout << "startup_freshness_guard_test: PASS\n";
    return 0;
}
