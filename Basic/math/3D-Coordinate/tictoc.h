//
// Created by lacie-life on 2023/24/07.   
//

#include <chrono>

class TicToc {
public:
    TicToc() {
        tic();
    }

    void tic() {
        start = std::chrono::system_clock::now();
    }

    auto toc_ms() {
        end = std::chrono::system_clock::now();
        return std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count();
    }

    auto toc_us() {
        end = std::chrono::system_clock::now();
        return std::chrono::duration_cast<std::chrono::microseconds>(end - start).count();
    }


    private:
        std::chrono::time_point<std::chrono::system_clock> start, end;

};
