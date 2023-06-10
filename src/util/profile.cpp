#include "profile.h"

#ifdef CPU_MIMXRT1176DVMAA
#include "clock_controller.h"
#include "logger.h"
#else
#include <chrono>
#endif

#include <etl/map.h>
#include <etl/stack.h>
#include <etl/string.h>

#include <stdint.h>

namespace profile {

    typedef etl::string<128> Identifier;

    constexpr size_t MAX_ENTRIES = 30;

    struct TimePoint {
        Identifier identifier;
        uint32_t value;
    };

#ifdef CPU_MIMXRT1176DVMAA

    static uint32_t __attribute__((unused)) get_time() {
        return clock_controller::ms();
    }

    static void __attribute__((unused))
    output_current(const Identifier identifier,
                   const uint32_t duration,
                   const uint32_t average,
                   const uint32_t max,
                   const uint32_t max_index) {
        logger::infof("%-40s: average: %-5d ms, current: %d ms, max: %d ms at "
                      "index %d\r\n",
                      identifier.data(),
                      average,
                      duration,
                      max,
                      max_index);
    }

    static void __attribute__((unused))
    output_report(const Identifier identifier,
                  const float average,
                  const uint32_t samples,
                  const uint32_t max,
                  const uint32_t max_index) {
        logger::infof("%-40s: average: %f ms over %d samples, max: %d ms at "
                      "index %d\r\n",
                      identifier.data(),
                      average,
                      samples,
                      max,
                      max_index);
    }

#else

    static uint32_t __attribute__((unused)) get_time() {

        return std::chrono::high_resolution_clock::now()
                   .time_since_epoch()
                   .count() /
               1000;
    }

    static void __attribute__((unused))
    output_current(const Identifier identifier,
                   const uint32_t duration,
                   const uint32_t average,
                   const uint32_t max,
                   const uint32_t max_index) {
        printf("%-40s: average: %-5d us, current: %d us, max: %d us at index "
               "%d\r\n",
               identifier.data(),
               average,
               duration,
               max,
               max_index);
    }

    static void __attribute__((unused))
    output_report(const Identifier identifier,
                  const float average,
                  const uint32_t samples,
                  const uint32_t max,
                  const uint32_t max_index) {
        printf("%-40s: average: %f us over %d samples, max: %d us at index "
               "%d\r\n",
               identifier.data(),
               average,
               samples,
               max,
               max_index);
    }

#endif

    /**
     * @brief Stack of start time points, used to allow for nested profiling.
     */
    etl::stack<TimePoint, MAX_ENTRIES> time_points;

    /**
     * @brief Map of the maximum for the profile sections.
     */
    etl::map<Identifier, uint32_t, MAX_ENTRIES> max;

    /**
     * @brief Map of at which index the maximum for the profile sections
     * occured.
     */
    etl::map<Identifier, uint32_t, MAX_ENTRIES> max_indices;

    /**
     * @brief Map of the total time for each profile section. Used for
     * calculating the average.
     */
    etl::map<Identifier, uint32_t, MAX_ENTRIES> totals;

    /**
     * @brief Map of the amount of times the profile sections are profiled. Used
     * for calculating the average.
     */
    etl::map<Identifier, uint32_t, MAX_ENTRIES> amount;

    void start(const char* identifier_) {
        Identifier identifier(identifier_);

        if (time_points.full()) {
            printf("Profile stack full, can't add: %s!\r\n", identifier_);
            exit(1);
        }

        TimePoint time_point{identifier, get_time()};
        time_points.push(time_point);
    }

    uint32_t end() {

        TimePoint time_point;
        time_points.pop_into(time_point);

        if (!totals.contains(time_point.identifier)) {
            totals[time_point.identifier] = 0;
            amount[time_point.identifier] = 0;
            max[time_point.identifier]    = 0;
        }

        const uint32_t duration = get_time() - time_point.value;
        totals[time_point.identifier] += duration;
        amount[time_point.identifier] += 1;

        if (duration > max[time_point.identifier]) {
            max_indices[time_point.identifier] = amount[time_point.identifier] -
                                                 1;
            max[time_point.identifier] = duration;
        }

        return duration;
    }

    void report() {

        for (etl::map<Identifier, uint32_t, MAX_ENTRIES>::iterator iterator =
                 totals.begin();
             iterator != totals.end();
             iterator++) {

            const uint32_t total   = iterator->second;
            const uint32_t samples = amount[iterator->first];

            const float average = static_cast<float>(total) /
                                  static_cast<float>(samples);

            output_report(iterator->first,
                          average,
                          samples,
                          max[iterator->first],
                          max_indices[iterator->first]);
        }
    }
}
