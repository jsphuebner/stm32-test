This program tests a completed main boards functionality by running the following checks
* Unused pins of MCU must be floating
* Outputs are looped back to inputs (see diagrams test-mainboard-*.png)
    * Low pass time constant is checked
    * Continuity from ouput to input is checked
* Window comparator is tested
* PWM inhibit circuitry (NAND gate) is tested
* Resolver excitation is generated (but not tested)
* LED blinks when all tests are successful

# Connection diagram
Thanks to Dima for this contribution!
Revision 3 main board:

![Revision 3 main board connection diagram](test-mainboard-rev3.png)

# Compiling
You will need the arm-none-eabi toolchain: https://developer.arm.com/open-source/gnu-toolchain/gnu-rm/downloads
The only external depedency is libopencm3 which I forked. You can download and build this dependency by typing

`make get-deps`

Now you can compile stm32-test by typing

`make`

or

`make HWCONFIG=HWCONFIG_REV3`

And upload it to your board using a JTAG/SWD adapter
