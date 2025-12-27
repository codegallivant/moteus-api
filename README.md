# moteus-api

C++ API to read and write data with moteus servo controllers. Layered over moteus drivers.

## Library Usage
See`examples/`.
```cpp
// Example implementing read operations and velocity control on 2 controllers

#include "moteus_api.hpp" // Make sure to have the relevant moteus_api.hpp, moteus_api.cpp and moteus drivers present at appropriate paths
#include <csignal>
#include <chrono>


void signal_handler(int signal) {
    // Cleanly stops controllers and releases memory
    MoteusAPI::Controller::destroyAll();
}

int main() {
    // Important to ensure all servos are stopped upon any termination signal 
    std::signal(SIGINT, signal_handler);

    MoteusAPI::Controller* controller1 = MoteusAPI::Controller::create(1, "can0");
    MoteusAPI::Controller* controller2 = MoteusAPI::Controller::create(2, "can0");

    // Specify which parameters should be read
    MoteusAPI::ReadState rs({
        .d_current = true,
        .q_current = true,
        .torque = true,
        .velocity = true
    });

    // Read operation on controller 1
    controller1->read(rs);
    std::cout << "Read Data:" << std::endl;
    rs.display();

    // Read operation on controller 2
    controller2->read(rs);
    std::cout << "Read Data:" << std::endl;
    rs.display();


    // Sending constant velocity command for 5000ms to both controllers        
    MoteusAPI::CommandState cs({
        .kp_scale = 0.1,
        .kd_scale = 0.1,
        .position = std::numeric_limits<double>::quiet_NaN(),
        .velocity = 2.0,
    });
    
    controller1->writeDuration(cs, 5000);
    controller2->writeDuration(cs, 5000);

    // Can also write and fetch read response
    controller1->write(cs, rs);
    controller2->write(cs, rs);
}
```

## Test your Moteus via CLI
1. Build
```bash
./build.sh
```
2. Run Tests
```bash
./tests.sh --socketcan-iface can0 --moteus-id "1"

# if using multiple moteus on the same bus-
./tests.sh --socketcan-iface can0 --moteus-id "1 6 3 ..."
```
This will run read, velocity control, constant acceleration trajectory and torque control tests.
![example tests output](docs/assets/image.png)
3. Further, you can send custom read/write commands via CLI
```bash
cd build/tests/
./read --help 
./write --help
```