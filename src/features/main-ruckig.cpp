#include <Arduino.h>
#include <bdb16.h>
#include <ruckig/ruckig.hpp>

constexpr double deg_to_rad = 1.0/180*M_PI;
constexpr double rpm_to_rad_per_s = 2.0*M_PI/60.0;
constexpr double gear_ratio = 21.5*80.0/20.0; 
constexpr double full_throttle = 16.0*1500.0/gear_ratio*rpm_to_rad_per_s;

void setup() {
    BDB16::init();

    printf("Hello World ruckig main\n");

    using namespace ruckig;
    Ruckig<1> otg(1.0/300.0);  // 300Hz control cycle
    InputParameter<1> input;
    OutputParameter<1> output;

    // Set input parameters
    input.current_position = {55*deg_to_rad};
    input.current_velocity = {0.0};
    input.current_acceleration = {0.0};

    input.target_position = {-55.0*deg_to_rad};
    input.target_velocity = {0.0};
    input.target_acceleration = {0.0};

    input.max_velocity = {full_throttle};
    input.max_acceleration = {100.0};
    input.max_jerk = {20000.0};

    // Set minimum duration (equals the trajectory duration when target velocity and acceleration are zero)
    input.minimum_duration = 0.4;

    // Generate the trajectory within the control loop
    printf("t position vel acc jerk\n");
    auto t0 = micros();
    while (otg.update(input, output) == Result::Working) {
        printf("%.4f %.4f %.4f %.4f %.4f %0.4f\n", output.time, output.new_position[0], output.new_velocity[0], output.new_acceleration[0], output.new_jerk[0], output.new_velocity[0]/full_throttle);

        output.pass_to_input(input);
    }

    printf("Computation took %luus\n", (micros()-t0));

    printf("Trajectory duration %f [s].\n", output.trajectory.get_duration());


}

void loop() {
  
}
