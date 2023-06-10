# Tiny VIO

A bare-metal visual-inertial odometry implementation based on [LARVIO](https://github.com/PetWorm/LARVIO) and MSCKF on the i.MX RT1170 microcontroller. The code in this project represents the work done over a year with a specialisation project and a master's thesis at the Norwegian University of Science and Technology (NTNU).

## Setup for the i.MX RT1170-EVK

The debugger on the evaluation board needs to be flashed with the JLink firmware, more information [here](https://community.nxp.com/t5/i-MX-RT-Knowledge-Base/Using-J-Link-with-MIMXRT1160-EVK-or-MIMXRT1170-EVK/ta-p/1529760). JLink also needs to be installed for flashing.

### Compiling & flashing

The project uses CMake:

1. `mkdir build && cd build`
2. `cmake ..`
3. `make -j`
4. `make flash`

### Debugging

In order to debug, first compile with debug configuration (the default is release): `cmake -DCMAKE_BUILD_TYPE=Debug .. && make -j`. Then do `make gdbcore0`, which will start a GDB server which can be utilised to debug the code on the board. A provided `.gdbinit` file is located in this repo which shows the commands required to link towards the GDB server.

## Testing against EuRoC

With a Ethernet link between a host PC with a static IP address set to `192.168.0.100` and the i.MX RT1170-EVK's 1 GBit port, the VIO pipeline can be tested with a proxy datalink and visualisation app from the `viz` folder. It requires a `rust` installation:

```
RUST_LOG=debug cargo run -- --dataset-path=<path to dataset section in EuRoC>
```

## Compiler version

Newer version of the arm-none-eabi toolchain has proved to have adverse affects for the run time. Using 12.2 has a quite serious impact on performance. The project has been developed with version 11.3.1 of the toolchain.

## Credits

This project utilises mainly Eigen and the Embedded Template Library. Supporting functionality is provided by OpenCV, Raylib and yamlcpp. The respective licenses can be found in the `licenses` folder.
