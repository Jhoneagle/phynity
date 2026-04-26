#include <catch2/catch_test_macros.hpp>
#include <core/jobs/schedule_recorder.hpp>
#include <core/jobs/schedule_replayer.hpp>

#include <cstdio>
#include <filesystem>
#include <fstream>
#include <string>

using namespace phynity::jobs;

namespace
{

std::string temp_path()
{
    auto tmp = std::filesystem::temp_directory_path() / "phynity_schedule_test.bin";
    return tmp.string();
}

} // namespace

TEST_CASE("ScheduleRecorder records tasks in order", "[jobs][schedule_replay]")
{
    ScheduleRecorder recorder;

    recorder.begin_frame(0);
    recorder.record_task_start(JobId{0, 0}, 0);
    recorder.record_task_start(JobId{1, 0}, 1);
    recorder.record_task_start(JobId{2, 0}, 0);
    recorder.end_frame();

    REQUIRE(recorder.frames().size() == 1);

    const auto &frame = recorder.frames()[0];
    REQUIRE(frame.frame_index == 0);
    REQUIRE(frame.tasks.size() == 3);

    // Sorted by start_order
    REQUIRE(frame.tasks[0].start_order == 0);
    REQUIRE(frame.tasks[1].start_order == 1);
    REQUIRE(frame.tasks[2].start_order == 2);
}

TEST_CASE("ScheduleRecorder multiple frames", "[jobs][schedule_replay]")
{
    ScheduleRecorder recorder;

    recorder.begin_frame(0);
    recorder.record_task_start(JobId{0, 0}, 0);
    recorder.end_frame();

    recorder.begin_frame(1);
    recorder.record_task_start(JobId{0, 0}, 0);
    recorder.record_task_start(JobId{1, 0}, 1);
    recorder.end_frame();

    REQUIRE(recorder.frames().size() == 2);
    REQUIRE(recorder.frames()[0].tasks.size() == 1);
    REQUIRE(recorder.frames()[1].tasks.size() == 2);
}

TEST_CASE("Schedule save/load round-trip", "[jobs][schedule_replay]")
{
    auto path = temp_path();

    // Record
    ScheduleRecorder recorder;

    recorder.begin_frame(0);
    recorder.record_task_start(JobId{0, 0}, 0);
    recorder.record_task_start(JobId{1, 0}, 1);
    recorder.record_task_start(JobId{2, 0}, 0);
    recorder.end_frame();

    recorder.begin_frame(1);
    recorder.record_task_start(JobId{3, 0}, 2);
    recorder.end_frame();

    REQUIRE(recorder.save(path));

    // Replay
    ScheduleReplayer replayer;
    REQUIRE(replayer.load(path));

    REQUIRE(replayer.frame_count() == 2);
    REQUIRE(replayer.has_frame(0));
    REQUIRE(replayer.has_frame(1));
    REQUIRE_FALSE(replayer.has_frame(2));

    // Frame 0: 3 tasks in recorded order
    const auto *tasks0 = replayer.frame_tasks(0);
    REQUIRE(tasks0 != nullptr);
    REQUIRE(tasks0->size() == 3);
    REQUIRE((*tasks0)[0].task_id == 0);
    REQUIRE((*tasks0)[1].task_id == 1);
    REQUIRE((*tasks0)[2].task_id == 2);

    // Frame 1: 1 task
    const auto *tasks1 = replayer.frame_tasks(1);
    REQUIRE(tasks1 != nullptr);
    REQUIRE(tasks1->size() == 1);
    REQUIRE((*tasks1)[0].task_id == 3);

    // Cleanup
    std::filesystem::remove(path);
}

TEST_CASE("ScheduleReplayer rejects invalid file", "[jobs][schedule_replay]")
{
    auto path = temp_path();

    // Write garbage
    {
        std::ofstream out(path, std::ios::binary);
        uint32_t garbage = 0xDEADBEEF;
        out.write(reinterpret_cast<const char *>(&garbage), sizeof(garbage));
    }

    ScheduleReplayer replayer;
    REQUIRE_FALSE(replayer.load(path));

    std::filesystem::remove(path);
}

TEST_CASE("ScheduleReplayer missing frame returns nullptr", "[jobs][schedule_replay]")
{
    ScheduleReplayer replayer;
    REQUIRE(replayer.frame_tasks(99) == nullptr);
}

TEST_CASE("ScheduleRecorder clear resets state", "[jobs][schedule_replay]")
{
    ScheduleRecorder recorder;

    recorder.begin_frame(0);
    recorder.record_task_start(JobId{0, 0}, 0);
    recorder.end_frame();

    REQUIRE(recorder.frames().size() == 1);

    recorder.clear();
    REQUIRE(recorder.frames().empty());
}
