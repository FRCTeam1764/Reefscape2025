// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.StateManager;
import frc.robot.subsystems.StateManager.States;
import frc.robot.subsystems.IntakeRollers;
import frc.robot.subsystems.IntakeWristRev;

import frc.robot.commands.BasicCommands.ElevatorCommandLimit;
import frc.robot.commands.BasicCommands.IntakeCommand;
import frc.robot.commands.BasicCommands.RequestStateChange;

import frc.robot.commands.DefaultCommands.DefaultElevatorCommand;
import frc.robot.commands.DefaultCommands.DefaultRollerCommand;
import frc.robot.commands.DefaultCommands.DefaultWristCommand;
public class RobotContainer {
    
    /* Setting up bindings for necessary control of the swerve drive platform */
   
    private final CommandXboxController pilot = new CommandXboxController(0);
    private final CommandXboxController copilot = new CommandXboxController(1);

    private final StateManager stateManager = new StateManager();
    // private final Climber climber = new Climber();
    private final Elevator elevator = new Elevator(stateManager);
    private final IntakeRollers rollers = new IntakeRollers();
    private final IntakeWristRev wrist = new IntakeWristRev(stateManager);

    

    
    
    private final CommandFactory commandFactory = new CommandFactory( elevator, rollers, wrist, pilot, stateManager);
    
    public RobotContainer() {
        
        configureBindings();
    }

    private void configureBindings() {
        // Note that X is defined as forward according to WPILib convention,
        // and Y is defined as to the left according to WPILib convention.
        

        elevator.setDefaultCommand(new DefaultElevatorCommand(elevator, stateManager));
        wrist.setDefaultCommand(new DefaultWristCommand(wrist, stateManager));
        rollers.setDefaultCommand(new DefaultRollerCommand(rollers, stateManager));

        //climber.setDefaultCommand(new De+faultClimberCommand(climber, stateManager, copilot)); 

        pilot.start().onTrue(new RequestStateChange(States.IDLE, stateManager));
        configureMainBindings();

        
    }

    private void configureMainBindings() {
        pilot.leftBumper().whileTrue(commandFactory.LevelPosition(2));
        pilot.leftBumper().onFalse(commandFactory.LevelScoreL2());
        pilot.leftTrigger().onTrue(commandFactory.LevelPosition(1));
        pilot.leftTrigger().onFalse(commandFactory.Level1Score());
        pilot.a().onTrue(commandFactory.upDownWrist());

        
        
        pilot.b().whileTrue(new IntakeCommand(rollers, -.2, false));

        pilot.rightTrigger(.7).onTrue(commandFactory.IntakeCoralPosition());
        pilot.rightTrigger(.7).onFalse(commandFactory.IntakeCoralTest());

        pilot.back().whileTrue(new ElevatorCommandLimit(elevator));
        
    }

    
}
