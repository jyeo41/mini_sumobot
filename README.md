# Mini Sumobot
The purpose of this project is to deepen my knowledge and understanding of various MCU peripherals as well as communication protocols such as UART and I2C.
Up until now, I have learned the foundations and building blocks through various small projects. This mini sumobot project aims to encompass everything I've learned and more
to challenge myself to build something from the ground up. Everything from writing the code, sourcing the parts, and prototyping to make sure the robot functions
and adheres to the requirements set by the official Sumobot competition rules as outlined in the Robogames website: https://robogames.net/rules/all-sumo.php
Another objective is to learn about industry-standard practices and to incorporate better software developing habits for personal growth.

The following are the goals I aim to achieve with this project:
* Improve quality of writing bare-metal code by implementing abstraction layers
* Familiarize myself with more peripherals such as I2C, ADC, DMA, and NEC protocol
* Learn proper CI/CD pipelines through the combination of Github Actions and Docker
* Go beyond solely step debugging through an IDE by also using custom a Asserthandler and TRACE function over UART communication
* Better understand how the build environment works by using a Makefile + Toolchain instead of an IDE
* Get more comfortable within a Linux environment

## Toolchain
Makefile + ARM Gnu Toolchain Compiler (GCC)
Neovim
Picocom (serial terminal for UART logging)
STM32Programmer (to flash the program)
STM32CubeIDE (solely for step debugging if needed)
