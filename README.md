1764 Master Robot
=======

## Table of Contents
---
[Whats in the Base Code](#whats-in-the-base-code)  
[How To's](#how-tos)  
[Resources](#resources)  

## Whats in the Base Code
---
* Swervedrive Subsystem  
* Path following  
* Controller config  

## How To's
---
#### Calibrating Wheels  
Module 0s can be set in DrivetrainConstants
More steps to be added later...  

#### Generating Paths Using Pathviewer  
To run Pathviewer, follow the following steps:  
1. Clone the [Pathviewer repo](https://github.com/FRCTeam2910/PathViewer)  
2. Open up cmd and cd to Pathviewer root directory  
3. Run **git submodule init**  
4. Run **git submodule update**
5. Run **gradlew build**  
6. Run **gradlew run**  
7. Put generated path in **src/main/java/org/1764**  

## Resources
---

FRC Documentation [FRC Documentation](https://wpilib.screenstepslive.com/s/currentCS/m/java/l/1027503-installing-c-and-java-development-tools-for-frc)

CTRE Documentation [CTRE Documentation] (http://www.ctr-electronics.com/control-system/hro.html#product_tabs_technical_resources)

Rev Robotics Documentation [REV Robotics Documentation] (http://www.revrobotics.com/sparkmax-software/)

NavX Documentation [NavX Documentation] (https://pdocs.kauailabs.com/navx-mxp/software/roborio-libraries/)

Limelight Documentation [Limelight Documentation] (http://docs.limelightvision.io/en/latest/)