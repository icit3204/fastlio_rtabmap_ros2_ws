#include "startup_freshness_guard.hpp"

#include <cassert>
#include <cmath>
#include <iostream>
#include <limits>
#include <vector>

namespace
{
using Decision = StartupFreshnessGuard::Decision;
using IngressDecision = StartupFreshnessGuard::IngressDecision;

struct PipelineCounters
{
    int preprocess_calls = 0;
    int lidar_buffer_pushes = 0;
    int sync_calls = 0;
    int estimator_calls = 0;
    int publications = 0;
};

struct MockPipeline
{
    StartupFreshnessGuard guard;
    bool ingress_enabled = false;
    PipelineCounters counters;

    MockPipeline(const bool ingress_enabled_in, const bool final_guard_enabled = true, const int required = 10)
    : guard(final_guard_enabled, 0.5, required), ingress_enabled(ingress_enabled_in)
    {
    }

    void raw_message(const double now, const double raw_header, const double synchronized_end)
    {
        if (ingress_enabled && guard.observe_ingress(now, raw_header) == IngressDecision::DROP_STALE)
        {
            return;
        }

        ++counters.preprocess_calls;
        ++counters.lidar_buffer_pushes;
        ++counters.sync_calls;
        const Decision decision = guard.observe(now, synchronized_end);
        if (decision == Decision::PROCESS)
        {
            ++counters.estimator_calls;
            ++counters.publications;
        }
    }
};

void test_disabled_legacy_path()
{
    MockPipeline pipeline(false, false);
    pipeline.raw_message(100.0, 98.6, 98.6);
    assert(pipeline.counters.preprocess_calls == 1);
    assert(pipeline.counters.lidar_buffer_pushes == 1);
    assert(pipeline.counters.sync_calls == 1);
    assert(pipeline.counters.estimator_calls == 1);
    assert(pipeline.counters.publications == 1);
}

void test_stale_ingress_is_cheaply_rejected()
{
    for (const int count : {1, 15, 16, 40})
    {
        MockPipeline pipeline(true);
        for (int i = 0; i < count; ++i)
        {
            pipeline.raw_message(100.0 + i * 0.1, 98.6 + i * 0.1, 98.6 + i * 0.1);
        }
        assert(pipeline.guard.ingress_stale_dropped() == static_cast<unsigned>(count));
        assert(pipeline.counters.preprocess_calls == 0);
        assert(pipeline.counters.lidar_buffer_pushes == 0);
        assert(pipeline.counters.sync_calls == 0);
        assert(pipeline.counters.estimator_calls == 0);
        assert(pipeline.counters.publications == 0);
    }
}

void test_fresh_tail_reaches_final_guard()
{
    MockPipeline pipeline(true);
    for (int i = 0; i < 16; ++i)
    {
        pipeline.raw_message(100.0 + i * 0.1, 98.6 + i * 0.1, 98.6 + i * 0.1);
    }
    for (int i = 0; i < 9; ++i)
    {
        const double now = 102.0 + i * 0.1;
        pipeline.raw_message(now, now, now);
    }
    assert(!pipeline.guard.admitted());
    assert(pipeline.counters.preprocess_calls == 9);
    assert(pipeline.counters.estimator_calls == 0);

    pipeline.raw_message(102.9, 102.9, 102.9);
    assert(pipeline.guard.admitted());
    assert(pipeline.counters.preprocess_calls == 10);
    assert(pipeline.counters.estimator_calls == 0);  // admission group is discarded

    pipeline.raw_message(103.0, 103.0, 103.0);
    assert(pipeline.counters.preprocess_calls == 11);
    assert(pipeline.counters.estimator_calls == 1);
    assert(pipeline.counters.publications == 1);
}

void test_current_raw_can_fail_synchronized_guard()
{
    MockPipeline pipeline(true);
    pipeline.raw_message(100.0, 100.0, 98.0);
    assert(pipeline.counters.preprocess_calls == 1);
    assert(pipeline.counters.lidar_buffer_pushes == 1);
    assert(pipeline.guard.stale_groups_dropped() == 1U);
    assert(pipeline.counters.estimator_calls == 0);
}

void test_ingress_rejection_resets_one_shared_fresh_window()
{
    StartupFreshnessGuard guard(true, 0.5, 10);
    for (int i = 0; i < 5; ++i)
    {
        assert(guard.observe(100.0 + i * 0.1, 100.0 + i * 0.1) == Decision::WAIT_FRESH_WINDOW);
    }
    assert(guard.fresh_group_count() == 5);

    assert(guard.observe_ingress(100.6, 99.2) == IngressDecision::DROP_STALE);
    assert(guard.fresh_group_count() == 0);

    for (int i = 0; i < 9; ++i)
    {
        assert(guard.observe(101.0 + i * 0.1, 101.0 + i * 0.1) == Decision::WAIT_FRESH_WINDOW);
    }
    assert(!guard.admitted());
    assert(guard.observe(101.9, 101.9) == Decision::ADMIT);
}

void test_ingress_timestamp_contract()
{
    StartupFreshnessGuard guard(true, 0.5, 1);
    assert(guard.observe_ingress(10.0, 9.5) == IngressDecision::ACCEPT);

    StartupFreshnessGuard over(true, 0.5, 1);
    assert(over.observe_ingress(10.0, 9.499999) == IngressDecision::DROP_STALE);

    StartupFreshnessGuard invalid(true, 0.5, 1);
    assert(invalid.observe_ingress(10.0, std::numeric_limits<double>::quiet_NaN()) == IngressDecision::DROP_STALE);
    assert(invalid.observe_ingress(10.0, std::numeric_limits<double>::infinity()) == IngressDecision::DROP_STALE);

    StartupFreshnessGuard future(true, 0.5, 1);
    assert(future.observe_ingress(10.0, 10.000001) == IngressDecision::DROP_STALE);

    StartupFreshnessGuard reversal(true, 0.5, 2);
    assert(reversal.observe_ingress(10.0, 10.0) == IngressDecision::ACCEPT);
    assert(reversal.observe_ingress(10.1, 9.99) == IngressDecision::DROP_STALE);
    assert(reversal.fresh_group_count() == 0);
}

void test_post_admission_bypass_restores_runtime_path()
{
    StartupFreshnessGuard guard(true, 0.5, 1);
    assert(guard.observe(10.0, 10.0) == Decision::ADMIT);
    assert(guard.observe_ingress(100.0, 98.0) == IngressDecision::ACCEPT);
    assert(guard.observe(100.0, 98.0) == Decision::PROCESS);
}

void test_k3e_treadmill_pre_and_post()
{
    MockPipeline legacy(false, true);
    MockPipeline guarded(true);

    for (int i = 0; i < 1190; ++i)
    {
        const double now = 100.0 + i * 0.1;
        const double old_stamp = now - 1.4;
        legacy.raw_message(now, old_stamp, old_stamp);
        guarded.raw_message(now, old_stamp, old_stamp);
    }
    assert(legacy.counters.preprocess_calls == 1190);
    assert(legacy.counters.lidar_buffer_pushes == 1190);
    assert(legacy.counters.sync_calls == 1190);
    assert(legacy.counters.estimator_calls == 0);

    assert(guarded.counters.preprocess_calls == 0);
    assert(guarded.counters.lidar_buffer_pushes == 0);
    assert(guarded.counters.sync_calls == 0);
    assert(guarded.counters.estimator_calls == 0);
    assert(guarded.guard.ingress_stale_dropped() == 1190U);

    for (int i = 0; i < 11; ++i)
    {
        const double now = 220.0 + i * 0.1;
        guarded.raw_message(now, now, now);
    }
    assert(guarded.guard.admitted());
    assert(guarded.counters.preprocess_calls == 11);
    assert(guarded.counters.estimator_calls == 1);
}
}

int main()
{
    test_disabled_legacy_path();
    test_stale_ingress_is_cheaply_rejected();
    test_fresh_tail_reaches_final_guard();
    test_current_raw_can_fail_synchronized_guard();
    test_ingress_rejection_resets_one_shared_fresh_window();
    test_ingress_timestamp_contract();
    test_post_admission_bypass_restores_runtime_path();
    test_k3e_treadmill_pre_and_post();
    std::cout << "startup_ingress_guard_test: PASS\n";
    return 0;
}
