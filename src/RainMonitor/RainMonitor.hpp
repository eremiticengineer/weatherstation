#pragma once
#include <cstdint>
#include "hardware/sync.h"
#include "pico/time.h"   // for time_us_32() if LOCKOUT_US > 0

class RainMonitor {
public:
    // ===== Configuration =====
    static constexpr int   MIN_BUF_SIZE    = 60;        // 60 minutes
    static constexpr int   DAY_BUF_SIZE    = 31;        // store last 31 days
    static constexpr float MM_PER_TIP      = 0.2794f;   // bucket calibration (mm/tip)
    static constexpr uint32_t LOCKOUT_US   = 1500;      // reject pulses <1.5 ms apart; set 0 to disable

    RainMonitor()
        : minute_idx_(0),
          current_minute_tips_(0),
          last_irq_us_(0),
          day_idx_(0),
          day_tip_total_(0)
    {
        for (int i = 0; i < MIN_BUF_SIZE; ++i) tips_per_min_[i] = 0;
        for (int i = 0; i < DAY_BUF_SIZE; ++i) tips_per_day_[i] = 0;
    }

    // ===== ISR hook =====
    inline void onTip() {
        if (LOCKOUT_US) {
            uint32_t now = time_us_32();
            if ((uint32_t)(now - last_irq_us_) < LOCKOUT_US) return;
            last_irq_us_ = now;
        }
        current_minute_tips_ = current_minute_tips_ + 1; // avoid ++ on volatile
    }

    // ===== Call once per minute =====
    inline void rotateMinute() {
        uint32_t tips_this_min = swapAndClear_(current_minute_tips_);

        // Store into current minute slot
        tips_per_min_[minute_idx_] = clampU16(tips_this_min);

        // Accumulate into today's total
        day_tip_total_ += tips_this_min;

        // Advance minute ring; keep new current slot zeroed (optional)
        minute_idx_ = (minute_idx_ + 1) % MIN_BUF_SIZE;
        tips_per_min_[minute_idx_] = 0;
    }

    // ===== Call at local midnight (or when your clock rolls over the date) =====
    inline void rotateDay() {
        // Archive the day’s total into the day ring buffer
        tips_per_day_[day_idx_] = day_tip_total_;
        day_idx_ = (day_idx_ + 1) % DAY_BUF_SIZE;

        // Reset today
        day_tip_total_ = 0;
    }

    // ===== Queries =====
    inline uint32_t getHourlyTips() const {
        uint32_t s = 0;
        for (int i = 0; i < MIN_BUF_SIZE; ++i) s += tips_per_min_[i];
        return s;
    }
    inline float getHourlyRainMm() const { return getHourlyTips() * MM_PER_TIP; }

    inline uint32_t getTodayTips() const { return day_tip_total_; }
    inline float getTodayRainMm() const { return getTodayTips() * MM_PER_TIP; }

    inline uint32_t getYesterdayTips() const {
        int prev = (day_idx_ + DAY_BUF_SIZE - 1) % DAY_BUF_SIZE;
        return tips_per_day_[prev];
    }
    inline float getYesterdayRainMm() const { return getYesterdayTips() * MM_PER_TIP; }

    // Sum of the last N completed days (not including today).
    inline uint32_t getLastNDaysTips(int n) const {
        if (n <= 0) return 0;
        if (n > DAY_BUF_SIZE) n = DAY_BUF_SIZE;
        uint32_t s = 0;
        int idx = day_idx_;
        for (int i = 0; i < n; ++i) {
            idx = (idx + DAY_BUF_SIZE - 1) % DAY_BUF_SIZE;
            s += tips_per_day_[idx];
        }
        return s;
    }
    inline float getLastNDaysRainMm(int n) const { return getLastNDaysTips(n) * MM_PER_TIP; }

private:
    inline uint32_t swapAndClear_(volatile uint32_t& counter) {
        uint32_t save = save_and_disable_interrupts();
        uint32_t val  = counter;
        counter = 0;
        restore_interrupts(save);
        return val;
    }
    static inline uint16_t clampU16(uint32_t v) {
        return (v > 0xFFFFu) ? 0xFFFFu : static_cast<uint16_t>(v);
    }

    // Per-minute ring
    uint16_t tips_per_min_[MIN_BUF_SIZE];
    int      minute_idx_;

    // ISR-updated
    volatile uint32_t current_minute_tips_;
    volatile uint32_t last_irq_us_;

    // Daily accumulation + ring of past days
    int       day_idx_;
    uint32_t  day_tip_total_;                // today
    uint32_t  tips_per_day_[DAY_BUF_SIZE];   // completed days
};
