#pragma once
#include <cstdint>
#include "FreeRTOS.h"
#include "task.h"
#include "hardware/sync.h"

// 1 turn/sec = 1.492 mph  → scale factor = 14920 / 10000 (integer math)
static constexpr int32_t WIND_SCALE_NUM   = 14920;
static constexpr int32_t WIND_SCALE_DEN   = 10000;

class WindMonitor {
public:
    // N samples for running avg of 1s speeds (e.g., last 10 seconds)
    static constexpr int RUN_AVG_SIZE = 10;

    // 60 entries = max gust per minute over the past hour
    static constexpr int GUST_MIN_BUF = 60;

    WindMonitor()
        : clicks_1s_(0), clicks_gust_(0),
          run_sum_(0), run_idx_(0), run_filled_(0),
          gust_minute_idx_(0)
    {
        for (int i = 0; i < RUN_AVG_SIZE; ++i) run_buf_[i] = 0;
        for (int i = 0; i < GUST_MIN_BUF; ++i) gust_min_buf_[i] = 0;
    }

    // ---- ISR hook: call from the GPIO IRQ (VERY fast) ----
    inline void onPulse() {
        // These are touched from ISR and tasks; keep them very simple
        clicks_1s_++;
        clicks_gust_++;
    }

    // ---- Called every 1s from a FreeRTOS task ----
    // Computes 1s average speed, updates running-average buffer.
    inline int32_t sampleAverage1s() {
        uint32_t clicks = swapAndClear_(clicks_1s_);
        //int32_t mph = toMphFromInterval(clicks, 1000);
        int32_t mph = toMphFromInterval10(clicks, 1000); // returs 10ths, 75 = 7.5
        // update running average circular buffer
        run_sum_ -= run_buf_[run_idx_];
        run_buf_[run_idx_] = mph;
        run_sum_ += mph;
        run_idx_ = (run_idx_ + 1) % RUN_AVG_SIZE;
        if (run_filled_ < RUN_AVG_SIZE) run_filled_++;
        return mph;
    }

    // ---- Called every GUST_MS (e.g. 200 ms) from a FreeRTOS task ----
    // Updates the current-minute max gust using short-interval bursts.
    inline int32_t sampleGustInterval(uint32_t interval_ms) {
        uint32_t clicks = swapAndClear_(clicks_gust_);
        //int32_t mph = toMphFromInterval(clicks, interval_ms);
        int32_t mph = toMphFromInterval10(clicks, interval_ms); // returs 10ths, 75 = 7.5
        if (mph > gust_min_buf_[gust_minute_idx_]) {
            gust_min_buf_[gust_minute_idx_] = mph;
        }
        return mph;
    }

    // ---- Called every 60s from a FreeRTOS task ----
    inline void rotateMinute() {
        gust_minute_idx_ = (gust_minute_idx_ + 1) % GUST_MIN_BUF;
        gust_min_buf_[gust_minute_idx_] = 0; // clear the new current minute slot
    }

    // ---- Getters (can be called anytime from tasks) ----
    inline int32_t getRunningAverageMph() const {
        if (run_filled_ == 0) return 0;
        return static_cast<int32_t>(run_sum_ / run_filled_);
    }

    inline int32_t getHourlyMaxGustMph() const {
        int32_t maxv = 0;
        for (int i = 0; i < GUST_MIN_BUF; ++i) {
            if (gust_min_buf_[i] > maxv) maxv = gust_min_buf_[i];
        }
        return maxv;
    }

    // Utility for conversions if you need them elsewhere
    static inline int32_t toMphFromInterval(uint32_t clicks, uint32_t interval_ms) {
        // (clicks per second) * 1.492 ≈ mph
        // clicks/sec = clicks * 1000 / interval_ms
        // mph = (WIND_SCALE_NUM / WIND_SCALE_DEN) * clicks * 1000 / interval_ms
        //      = (WIND_SCALE_NUM * clicks * 1000) / (WIND_SCALE_DEN * interval_ms)
        int64_t num = static_cast<int64_t>(WIND_SCALE_NUM) * clicks * 1000LL;
        int64_t den = static_cast<int64_t>(WIND_SCALE_DEN) * interval_ms;
        return static_cast<int32_t>(num / den);
    }

    // Returns mph * 10 (fixed-point, 1 decimal place)
    static inline int32_t toMphFromInterval10(uint32_t clicks, uint32_t ms) {
        long long num = 1LL * 14920 * clicks * 1000LL * 10;    // extra ×10 for tenths
        long long den = 1LL * 10000 * ms;
        return (int32_t)((num + den/2) / den); // rounded to nearest tenth
    }


private:
    // Atomically swap & clear a counter that’s touched in ISR + task
    inline uint32_t swapAndClear_(volatile uint32_t& counter) {
        taskENTER_CRITICAL();
        uint32_t val = counter;
        counter = 0;
        taskEXIT_CRITICAL();
        return val;
    }

    // ISR-updated counters
    volatile uint32_t clicks_1s_;
    volatile uint32_t clicks_gust_;

    // Running average state
    int32_t  run_buf_[RUN_AVG_SIZE];
    int64_t  run_sum_;
    int      run_idx_;
    int      run_filled_;

    // Per-minute max gust over past hour
    int32_t  gust_min_buf_[GUST_MIN_BUF];
    int      gust_minute_idx_;
};
