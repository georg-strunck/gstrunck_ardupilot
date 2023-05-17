/*
  simple hello world sketch
  Andrew Tridgell September 2011
*/

#include <AP_HAL/AP_HAL.h>

void setup();
void loop();

// Register a hal object...? So we can use all the nice hal functions
const AP_HAL::HAL& hal = AP_HAL::get_HAL();

class my_test_timer_class
{
  public:
    // Write timer process callback function overloaded
    void operator()() 
    {
        int my_time = AP_HAL::millis();
        int my_sec  = my_time/1000;
        int my_msec = my_time - my_sec*1000;

        hal.console->printf("Time since boot: %d [msec]\n \t\t %d [sec] %d [msec]\n", my_time, my_sec, my_msec);
    }
};

// Write timer process callback function overloaded
void my_test_timer_callback()
{
    int my_time = AP_HAL::millis();
    int my_sec  = my_time/1000;
    int my_msec = my_time - my_sec*1000;

    hal.console->printf("Time since boot: %d [msec]\n \t\t %d [sec] %d [msec]\n", my_time, my_sec, my_msec);
}

void setup()
{
    // hal.scheduler->delay(1000);
    hal.console->printf("hello world\n");
    // hal.scheduler->register_timer_process(AP_HAL_MEMBERPROC(&AP_Baro_MS5611::_update));
    my_test_timer_class tmi_class;
    // hal.scheduler->register_timer_process(FUNCTOR(tmi_class, my_test_timer_callback(), void));
}

void loop()
{
    hal.scheduler->delay(1000);
    my_test_timer_callback();
}

AP_HAL_MAIN();
