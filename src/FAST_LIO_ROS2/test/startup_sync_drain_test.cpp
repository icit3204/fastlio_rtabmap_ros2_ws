#include "startup_freshness_guard.hpp"

#include <cassert>
#include <cstddef>
#include <deque>
#include <iostream>
#include <limits>
#include <optional>

namespace
{
using Decision = StartupFreshnessGuard::Decision;

struct Group
{
    double now;
    double lidar_end;
};

struct Counters
{
    std::size_t sync_calls = 0;
    std::size_t successful_syncs = 0;
    std::size_t estimator_calls = 0;
    std::size_t map_initialization_calls = 0;
    std::size_t cloud_publications = 0;
    std::size_t odom_publications = 0;
    std::size_t last_drain_batch = 0;
    std::size_t max_drain_batch = 0;
};

class MockTimerPipeline
{
public:
    MockTimerPipeline(const bool enabled, const int required = 10)
    : guard(enabled, 0.5, required)
    {
    }

    void push(const Group group) { queue.push_back(group); }

    void make_admitted(const double now)
    {
        assert(guard.observe(now, now) == Decision::ADMIT);
    }

    void return_false_after(const std::size_t successful_groups)
    {
        false_after_successes = successful_groups;
    }

    void timer_callback()
    {
        const bool startup_drain_active = guard.enabled() && !guard.admitted();
        if (startup_drain_active)
        {
            std::size_t drain_batch = 0;
            while (true)
            {
                Group group{};
                if (!sync_packages(group))
                {
                    finish_batch(drain_batch);
                    return;
                }

                const Decision decision = guard.observe(group.now, group.lidar_end);
                if (decision == Decision::DROP_STALE)
                {
                    ++drain_batch;
                    continue;
                }

                finish_batch(drain_batch);
                // WAIT_FRESH_WINDOW and ADMIT terminate this callback.  The
                // next callback owns the next progression step.
                return;
            }
        }

        // Disabled and admitted modes retain one-group behavior.
        Group group{};
        if (!sync_packages(group))
        {
            return;
        }
        if (guard.observe(group.now, group.lidar_end) != Decision::PROCESS)
        {
            return;
        }
        process_group();
    }

    StartupFreshnessGuard guard;
    std::deque<Group> queue;
    Counters counters;

private:
    bool sync_packages(Group &group)
    {
        ++counters.sync_calls;
        if (false_after_successes.has_value() &&
            counters.successful_syncs >= *false_after_successes)
        {
            return false;
        }
        if (queue.empty())
        {
            return false;
        }
        group = queue.front();
        queue.pop_front();
        ++counters.successful_syncs;
        return true;
    }

    void finish_batch(const std::size_t batch)
    {
        counters.last_drain_batch = batch;
        if (batch > counters.max_drain_batch)
        {
            counters.max_drain_batch = batch;
        }
    }

    void process_group()
    {
        ++counters.estimator_calls;
        ++counters.map_initialization_calls;
        ++counters.cloud_publications;
        ++counters.odom_publications;
    }

    std::optional<std::size_t> false_after_successes;
};

Group stale(const int index)
{
    const double now = 100.0 + static_cast<double>(index) * 0.1;
    return {now, now - 1.4};
}

Group fresh(const int index)
{
    const double now = 200.0 + static_cast<double>(index) * 0.1;
    return {now, now};
}

void add_stale(MockTimerPipeline &pipeline, const int count)
{
    for (int i = 0; i < count; ++i)
    {
        pipeline.push(stale(i));
    }
}

void test_disabled_one_group_legacy_behavior()
{
    MockTimerPipeline pipeline(false);
    add_stale(pipeline, 2);
    pipeline.timer_callback();
    assert(pipeline.counters.sync_calls == 1);
    assert(pipeline.counters.estimator_calls == 1);
    assert(pipeline.queue.size() == 1);
}

void test_admitted_one_group_legacy_behavior()
{
    MockTimerPipeline pipeline(true, 1);
    pipeline.make_admitted(10.0);
    pipeline.push(stale(0));
    pipeline.push(stale(1));
    pipeline.timer_callback();
    assert(pipeline.counters.sync_calls == 1);
    assert(pipeline.counters.estimator_calls == 1);
    assert(pipeline.queue.size() == 1);
}

void test_stale_prefix_sizes(const int count)
{
    MockTimerPipeline pipeline(true);
    add_stale(pipeline, count);
    pipeline.timer_callback();
    assert(pipeline.counters.last_drain_batch == static_cast<std::size_t>(count));
    assert(pipeline.counters.max_drain_batch == static_cast<std::size_t>(count));
    assert(pipeline.guard.stale_groups_dropped() == static_cast<std::size_t>(count));
    assert(pipeline.counters.estimator_calls == 0);
    assert(pipeline.counters.map_initialization_calls == 0);
    assert(pipeline.counters.cloud_publications == 0);
    assert(pipeline.counters.odom_publications == 0);
    assert(pipeline.queue.empty());
}

void test_stale_prefix_then_fresh_stops_at_first_fresh(const int stale_count)
{
    MockTimerPipeline pipeline(true);
    add_stale(pipeline, stale_count);
    pipeline.push(fresh(0));
    pipeline.push(fresh(1));
    pipeline.timer_callback();
    assert(pipeline.counters.last_drain_batch == static_cast<std::size_t>(stale_count));
    assert(pipeline.guard.fresh_group_count() == 1);
    assert(pipeline.counters.estimator_calls == 0);
    assert(pipeline.queue.size() == 1);
}

void test_sync_false_after_progress_stops_without_fake_drop()
{
    MockTimerPipeline pipeline(true);
    add_stale(pipeline, 5);
    pipeline.return_false_after(5);
    pipeline.timer_callback();
    assert(pipeline.counters.last_drain_batch == 5);
    assert(pipeline.counters.sync_calls == 6);
    assert(pipeline.counters.successful_syncs == 5);
    assert(pipeline.counters.estimator_calls == 0);
    assert(pipeline.guard.stale_groups_dropped() == 5);
}

void test_sync_false_first_action_stops()
{
    MockTimerPipeline pipeline(true);
    pipeline.return_false_after(0);
    pipeline.timer_callback();
    assert(pipeline.counters.sync_calls == 1);
    assert(pipeline.counters.successful_syncs == 0);
    assert(pipeline.counters.last_drain_batch == 0);
    assert(pipeline.counters.estimator_calls == 0);
}

void test_fresh_window_and_admission_semantics()
{
    MockTimerPipeline pipeline(true);
    for (int i = 0; i < 5; ++i)
    {
        pipeline.push(fresh(i));
        pipeline.timer_callback();
    }
    assert(pipeline.guard.fresh_group_count() == 5);
    assert(!pipeline.guard.admitted());

    pipeline.push(stale(0));
    pipeline.timer_callback();
    assert(pipeline.guard.fresh_group_count() == 0);

    for (int i = 0; i < 9; ++i)
    {
        pipeline.push(fresh(10 + i));
        pipeline.timer_callback();
    }
    assert(pipeline.guard.fresh_group_count() == 9);
    assert(!pipeline.guard.admitted());

    pipeline.push(fresh(30));
    pipeline.timer_callback();
    assert(pipeline.guard.admitted());
    assert(pipeline.counters.estimator_calls == 0);  // tenth is discarded

    pipeline.push(fresh(31));
    pipeline.timer_callback();
    assert(pipeline.counters.estimator_calls == 1);
    assert(pipeline.counters.cloud_publications == 1);
    assert(pipeline.counters.odom_publications == 1);
}

void test_finite_large_queue_has_no_infinite_loop()
{
    MockTimerPipeline pipeline(true);
    add_stale(pipeline, 1000);
    pipeline.timer_callback();
    assert(pipeline.counters.max_drain_batch == 1000);
    assert(pipeline.counters.sync_calls == 1001);  // final false sync
    assert(pipeline.counters.estimator_calls == 0);
}
}

int main()
{
    test_disabled_one_group_legacy_behavior();
    test_admitted_one_group_legacy_behavior();
    test_stale_prefix_sizes(1);
    test_stale_prefix_sizes(15);
    test_stale_prefix_sizes(40);
    test_stale_prefix_sizes(210);
    test_stale_prefix_then_fresh_stops_at_first_fresh(15);
    test_stale_prefix_then_fresh_stops_at_first_fresh(40);
    test_sync_false_after_progress_stops_without_fake_drop();
    test_sync_false_first_action_stops();
    test_fresh_window_and_admission_semantics();
    test_finite_large_queue_has_no_infinite_loop();
    std::cout << "startup_sync_drain_test: PASS\n";
    return 0;
}
