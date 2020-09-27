#ifndef TIMER_H
#define TIMER_H

#include <thread>
#include <chrono>
#include <functional>
#include <future>
#include <cstdio>
#include <iostream>

class Timer
{
    public:
        Timer() {};

    template<class callable>
    void connect(callable &&f) //paso mediante semántica de movimiento
    {
        std::thread([=]()  //como una funcion lambda
                    {
                        while (true) {
                            if (go.load())           // ve is ya está lanzada
                                std::invoke(f);     //forma de llamar a la funcion f
                            std::this_thread::sleep_for(std::chrono::milliseconds(period.load()));
                        }
                    }).detach(); //no detiene la ejecucion de la funcion (como un hilo)
    };

    void start(int p) {
        period.store(p);
        go.store(true);
        begin = std::chrono::steady_clock::now();
    };

    void stop() { go.store(!go); };

    void setPeriod(int p) { period.store(p); };

    auto getElapsedTime()
    {
        auto end = std::chrono::steady_clock::now();
        return std::chrono::duration_cast<std::chrono::milliseconds>(end - begin).count();
    }

    //añadir nuevas funciones

private:
    std::atomic_bool go = false;
    std::atomic_int period = 0;
    std::chrono::steady_clock::time_point begin;
};

#endif // TIMER_H
