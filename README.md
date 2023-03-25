# cBot

cBot is an autonomous robot system developed for teaching at universities. The goal is to promote programming in C language with exciting applications in the field of autonomous robots using professional development tools. 

cBot has a rich set of equipment that allows a wide range of experiments:
- 32 bit microprocessor with 72 MHz for the execution of more complex algorithms
- Two stepper motors for precise driving maneuvers with simple kinematics
- Three adjustable ultrasonic sensors for detecting objects
- Two light sensors to following lines
- One graphic display with 128x64 pixels for output of text and images
- Four buttons to manual control of the robot
- One speaker for playing melodies
- Expandability with up to three servos
- A serial interface for extension or remote control of the robot

Despite professional features of the system, the required components can be purchased relatively cheap.

## Assemble your own cBot

The following steps are required to assemble the robot:
- Solder mainboard (tutorial will be available soon)
- Assemble robot (tutorial will be available soon)
- Validate assembly (tutorial will be available soon)

## Start Programming with cBot

cBot is programmed with the programming environment STM32CubeIDE. The programming environment can be downloaded and used free of charge from the manufacturer ST after registration. The project files have been created with STM32CubeIDE version 1.11.0. However, with only little effort the project can also be imported into older versions.

Download STM32CubeIDE from <https://www.st.com/en/development-tools/stm32cubeide.html>. Just follow the instructions, while installing the IDE.

A [video tutorial](https://www.youtube.com/watch?v=gb_FOSKnrtg) explains the next steps to run the first application on cBot:
- Clone this repository onto your computer. Alternatively it can be downloaded as a ZIP file and unpacked.
- Import the project as existing projekt into the workspace of STM32CubeIDE. Make sure to not copy the project into the workspace.
- Compile the project
- Connect the debugger with your computer and cBot
- Connect the powerbank to cBot
- Flash the software onto cBot