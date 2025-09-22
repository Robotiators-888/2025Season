# 2025 FRC Season Robot Code

[![CI](https://github.com/Robotiators-888/2025Season/actions/workflows/main.yml/badge.svg)](https://github.com/Robotiators-888/2025Season/actions/workflows/main.yml)

This repository contains the code for FRC Team 888 "The Robotiators" for the 2025 FRC Season.

## Project Overview

This project is the complete robot code for the 2025 FRC Season. The robot is a swerve drive robot designed to compete in the Reefscape game. It is capable of intaking game pieces ("coral" and "algae"), scoring them at different levels, and climbing at the end of the match. The robot uses PhotonVision for AprilTag detection and PathPlanner for autonomous path following.

## Features

- **Swerve Drive**: The robot uses a 4-wheel swerve drive with REV MAXSwerve modules, providing high maneuverability on the field.
- **Elevator**: A multi-stage elevator for lifting game pieces to different scoring heights.
- **Roller Intake**: A roller-based intake system for acquiring "coral" and "algae".
- **Pivot Mechanism**: A pivot mechanism for positioning the intake and scoring mechanism.
- **PhotonVision**: Utilizes two PhotonVision cameras for AprilTag detection, enabling accurate pose estimation and alignment.
- **PathPlanner**: Autonomous routines are built using PathPlanner, allowing for complex and reliable autonomous modes.
- **Custom Dashboard**: The robot sends data to a custom dashboard called Elastic, providing real-time information and notifications.

## Setup

To set up the development environment, you will need to install the following tools:

1.  **Visual Studio Code**: The recommended IDE for FRC Java development.
2.  **WPILib Extension for VS Code**: This extension provides the necessary tools for FRC development, including the WPILib libraries, Gradle, and a simulator.
3.  **Java 17**: The required version of Java for the 2025 FRC season.
4.  **Git**: For version control.

Once you have installed these tools, you can clone this repository and open it in VS Code. The WPILib extension should automatically configure the project.

To build the code, run the following command in the terminal:

```bash
./gradlew build
```

To deploy the code to the robot, run:

```bash
./gradlew deploy
```

## Controls

The robot is controlled by two Xbox controllers.

### Driver 1 (Primary Drive Controller)

-   **Left Stick (Y-Axis)**: Drive forward/backward.
-   **Left Stick (X-Axis)**: Strafe left/right.
-   **Right Stick (X-Axis)**: Rotate left/right.
-   **Right Bumper (hold)**: "Turbo" mode (faster, more sensitive driving with squared inputs).
-   **Left Stick Button (press)**: Zero the gyro heading to the current direction.
-   **POV Up/Down (hold)**: Raise/lower the climber mechanism.
-   **Y Button (hold)**: Automatically align the robot to the nearest "Algae" scoring position.
-   **X Button (hold)**: Automatically align the robot to the left side of the nearest "Reef" scoring position.
-   **B Button (hold)**: Automatically align the robot to the right side of the nearest "Reef" scoring position.

### Driver 2 (Secondary Operator Controller)

-   **A Button (press)**: Move elevator and pivot to the intake position (bottom).
-   **B Button (press)**: Move elevator and pivot to score on Level 2.
-   **X Button (press)**: Move elevator and pivot to score on Level 3.
-   **Y Button (press)**: Move elevator and pivot to score on Level 4.
-   **POV Up (press)**: Move elevator and pivot to the Algae intake position.
-   **POV Down (press)**: Move elevator and pivot to the L2 Algae scoring position.
-   **POV Left (press)**: Move elevator and pivot to the Processor (safe handoff) position.
-   **POV Right (press)**: Initiate the sequence for scoring on the Barge.
-   **Left Bumper (hold)**: Run the intake rollers in reverse to outtake game pieces slowly.
-   **Right Bumper (hold)**: Run the intake rollers to acquire a game piece. The controller will rumble when a piece is detected.
-   **Right Trigger (hold)**: Eject a game piece quickly from the rollers.
-   **Left Trigger (hold)**: Run the rollers at high speed to score Algae.
