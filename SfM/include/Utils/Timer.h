//
// Created by lacie on 20/08/2023.
//

#ifndef SFM_TIMER_H
#define SFM_TIMER_H

#include <chrono>

namespace SimpleSfM
{
    class Timer {
    public:
        Timer();
        void Start();
        bool IsStart();

        void Restart();
        void Pause();
        bool IsPause();
        void Resume();
        void Reset();

        double ElapsedMicorSeconds() const;
        double ElapsedSeconds() const;
        double ElapsedMinutes() const;
        double ElapsedHours() const;

        void PrintSeconds() const;
        void PrintMinutes() const;
        void PrintHours() const;

    private:
        bool started_;
        bool paused_;

        std::chrono::high_resolution_clock::time_point start_time_;
        std::chrono::high_resolution_clock::time_point pause_time_;
    };
}

#endif //SFM_TIMER_H
