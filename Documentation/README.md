# Project Overview

This folder contains all the relevant documentation for the module EN2160 - Electronic Design Realization, Department of Electronics and Telecommunication, University of Moratuwa.

## Latex Files

- [Project Proposal](https://www.overleaf.com/read/vdhwpdgcyxzz#c0ae59)
- [Self Evaluation](https://www.overleaf.com/read/htvppdczqtfv#5dd420)
- [Progress Report](https://www.overleaf.com/read/qyvzbrpfnvsf#e779c1)
- [Expectations of Group Formation](https://www.overleaf.com/read/qyvzbrpfnvsf#e779c1)
- [Conceptual Design Report](https://www.overleaf.com/read/qgtddjcsfdcf#2d864c)

# Project Overview

The proposed project, titled ”Pick and Place Robot Arm,” aims to develop an automated robotic arm system for the precise assembly of H-Bridge components. The primary focus is on automating the assembly of rivet, washer, and transistor parts, contributing to enhanced efficiency and accuracy in electronic component manufacturing.



## Introduction

The project aims to modernize the assembly process of electronic components, focusing on enhancing the precision in assembling items like rivets, washers, and transistors. The automation of this intricate assembly process promises a substantial improvement in efficiency and accuracy within the realm of electronic component manufacturing.

The outlined objectives encompass diverse engineering facets, ranging from the design of the robotic arm system to the application of mathematical concepts, implementation of control systems, and evaluation of the project’s viability in Sri Lanka’s electronics manufacturing industry. The incorporation of hands-on skills acquired during university studies, including programming, mechanical design, fabrication, PCB design, soldering, testing, and calibration, emphasizes a comprehensive and practical approach to the project.

## Review Existing Solutions

The investigation extends to solutions showcased in YouTube videos, covering pick and place mechanisms, screw robots, and innovative approaches like bolt and washer insertion.



Notable technologies, such as Yaskawa Motoman’s 3D Vision Picking, are highlighted for advanced manipulation techniques.

This comprehensive review serves as a foundation for the proposed robotic arm system. The project aims to incorporate the best features observed during the research, leveraging the strengths of existing solutions to develop a state-of-the-art robotic arm tailored to the specific requirements of the H-Bridge component assembly process.

## Proposed Device Architecture

The device architecture for the Pick and Place Robot Arm system consists of several interconnected components designed to work seamlessly to achieve precise assembly.

### Horizontal and Vertical Conveyor Systems

The horizontal conveyor system serves as the foundation for the linear movement of the robotic arm along the x-axis.


It comprises:
- **T8 Screw Rod**: A horizontally oriented T8 screw rod is responsible for the linear motion along the x-axis. It is connected to a stepper motor, which drives the rotation of the screw rod.

- **Motor Driver**: The motor driver receives control signals from the controller interface and translates them into appropriate power outputs for the stepper motor. It plays a crucial role in controlling the speed and direction of the horizontal conveyor system.
- **Stepper Motor**: The stepper motor is responsible for converting electrical signals from the motor driver into rotational motion. The rotation of the stepper motor drives the T8 screw rod, enabling precise horizontal movement.


### Gripper Mechanism

The gripper mechanism is connected to the vertical conveyor system and is responsible for picking up and placing H-Bridge components. It includes:
- **Gripper Module**: The gripper module is designed to securely hold and release electronic components during the pick and place operation. It is attached to the vertical conveyor system to enable controlled vertical movement.
- **Actuation System**: The actuation system within the gripper module allows controlled opening and closing of the gripper, ensuring a secure hold on the components during transportation.

### Controller Interface

The controller interface serves as the central hub for user interaction and control over the entire robotic arm system. It consists of:
- **Microcontroller/Processor**: The microcontroller or processor acts as the brain of the system, processing user commands and generating control signals for the motor drivers.
- **User Interface**: The user interface provides a platform for users to input commands and monitor the system’s status. This can include physical buttons, touchscreen displays, or a computer interface.

This device architecture ensures a coherent and integrated system, allowing precise control over the horizontal and vertical movements of the robotic arm, as well as the gripping mechanism. Each component plays a vital role in achieving the overall objective of automating the assembly process of H-Bridge components.
