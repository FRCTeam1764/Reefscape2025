// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import java.util.concurrent.CopyOnWriteArrayList;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.StateManager;
import frc.robot.subsystems.StateManager.States;
import frc.robot.subsystems.IntakeRollers;
import frc.robot.subsystems.IntakeWristRev;
import frc.robot.subsystems.LimelightSubsystem;

import frc.robot.commands.BasicCommands.ElevatorCommandLimit;
import frc.robot.commands.BasicCommands.IntakeCommand;
import frc.robot.commands.BasicCommands.RequestStateChange;

import frc.robot.commands.DefaultCommands.DefaultElevatorCommand;
import frc.robot.commands.DefaultCommands.DefaultRollerCommand;
import frc.robot.commands.DefaultCommands.DefaultWristCommand;
import frc.robot.commands.DriveCommands.DriveRobotCentric;
import frc.robot.commands.DriveCommands.DriveToLimeLightVisionOffset;
import frc.robot.commands.DriveCommands.DriveToTargetOffset;
import frc.robot.commands.DriveCommands.DriveToTargetOffsetLL3;
import frc.robot.commands.DriveCommands.LockOnAprilTag;
import frc.robot.commands.DriveCommands.TrackObject;
import frc.robot.commands.DriveCommands.TurnToAngle;
import frc.robot.constants.CommandConstants;
import frc.robot.generated.TunerConstants;
import frc.robot.libraries.external.drivers.Limelight;
import frc.robot.state.IDLE;
import frc.robot.state.INTERPOLATED_STATE;
import frc.robot.subsystems.CommandSwerveDrivetrain;

public class RobotContainer {
    private double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
    private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity

    /* Setting up bindings for necessary control of the swerve drive platform */
    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDeadband(MaxSpeed * 0.06).withRotationalDeadband(MaxAngularRate * 0.05) // Add a 10% deadband
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors

    private final SwerveRequest.RobotCentric robotCentricDrive = new SwerveRequest.RobotCentric().withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.025).withDriveRequestType(DriveRequestType.OpenLoopVoltage).withDriveRequestType(DriveRequestType.OpenLoopVoltage);
    private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
    private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();

    private final Telemetry logger = new Telemetry(MaxSpeed);

    private final CommandXboxController pilot = new CommandXboxController(0);
    private final CommandXboxController copilot = new CommandXboxController(1);

    public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();

    private final StateManager stateManager = new StateManager();
    // private final Climber climber = new Climber();
    private final Elevator elevator = new Elevator(stateManager);
    private final IntakeRollers rollers = new IntakeRollers();
    private final IntakeWristRev wrist = new IntakeWristRev(stateManager);

    

    
    
    private final CommandFactory commandFactory = new CommandFactory( elevator, rollers, wrist, limelight4, limelight3, limelight2, pilot, drivetrain, stateManager);
    private final AutonomousCommandFactory autoFactory = new AutonomousCommandFactory( elevator, rollers, wrist, limelight4, limelight3, limelight2, pilot, drivetrain, stateManager);


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

    public void changePipeline() {
        limelight3.setPipeline(1);
    }


    public Command getAutonomousCommand() {
        return chooser.getSelected();
    }
}
