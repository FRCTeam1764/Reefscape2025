// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
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

    

    
    private final LimelightSubsystem limelight3 = new LimelightSubsystem( drivetrain,"limelight-three",0,0,0);
    private final LimelightSubsystem limelight2 = new LimelightSubsystem(drivetrain,"limelight-two",0,0,0);
    private final LimelightSubsystem limelight4 = new LimelightSubsystem(drivetrain, "limelight-four",0,0,0);

    private final CommandFactory commandFactory = new CommandFactory( elevator, rollers, wrist, limelight4, limelight3, limelight2, pilot, drivetrain, stateManager);
    private final AutonomousCommandFactory autoFactory = new AutonomousCommandFactory( elevator, rollers, wrist, limelight4, limelight3, limelight2, pilot, drivetrain, stateManager);


    private final SendableChooser<Command> chooser ;
    public RobotContainer() {
        stateManager.requestNewState(States.IDLE);
        chooser = AutoBuilder.buildAutoChooser("tests");
        SmartDashboard.putData("Autos",chooser);
        //drivetrain.seedFieldCentric();
        configureBindings();
        //autoFactory.configAutonomousCommands();
    }

    private void configureBindings() {
        // Note that X is defined as forward according to WPILib convention,
        // and Y is defined as to the left according to WPILib convention.
        drivetrain.setDefaultCommand(
            // Drivetrain will execute this command periodically
            drivetrain.applyRequest(() ->
                drive.withVelocityX(-pilot.getLeftY() * MaxSpeed) // Drive forward with negative Y (forward)
                    .withVelocityY(-pilot.getLeftX() * MaxSpeed) // Drive left with negative X (left)
                    .withRotationalRate(-pilot.getRightX() * MaxAngularRate) // Drive counterclockwise with negative X (left)
            )
        );

        elevator.setDefaultCommand(new DefaultElevatorCommand(elevator, stateManager));
        wrist.setDefaultCommand(new DefaultWristCommand(wrist, stateManager));
        rollers.setDefaultCommand(new DefaultRollerCommand(rollers, stateManager));

        //climber.setDefaultCommand(new DefaultClimberCommand(climber, stateManager, copilot)); 

        pilot.y().onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldCentric()));
        pilot.start().onTrue(new RequestStateChange(States.IDLE, stateManager));
        configureMainBindings();

        drivetrain.registerTelemetry(logger::telemeterize);
    }

    private void configureMainBindings() {
        pilot.leftBumper().whileTrue(commandFactory.LevelPosition(2));
        pilot.leftBumper().onFalse(commandFactory.LevelScoreL2());
        pilot.rightTrigger().onTrue(commandFactory.Level4Position());
        pilot.rightTrigger().onFalse(commandFactory.Level4Score());
        pilot.rightBumper().onTrue(commandFactory.LevelPosition(3));
        pilot.rightBumper().onFalse(commandFactory.LevelScore());
        //pilot.back().whileTrue(new TrackObject(drivetrain, limelight3, 2));
        //copilot.start().whileTrue(new ClimberPosition(climber));
        // copilot.start().onTrue(autoFactory.autoCoralPickup());
        // copilot.start().onFalse(autoFactory.autoCoralReturn());
        // copilot.back().onTrue(autoFactory.autoAlignCoral());

        pilot.b().whileTrue(new DriveRobotCentric(drivetrain,pilot));

        pilot.x().whileTrue(new LockOnAprilTag(drivetrain, limelight3, 0, pilot, false));
        pilot.a().whileTrue(new TurnToAngle(drivetrain, limelight3));

        pilot.pov(90).whileTrue(new DriveToTargetOffset(drivetrain, limelight4, 0, 0, 17.3, 9.3));
        pilot.pov(0).whileTrue(new TurnToAngle(drivetrain, limelight3));
        pilot.pov(270).whileTrue(new DriveToTargetOffsetLL3(drivetrain, limelight3, 0, 0, -18, 14.8));//-15.7, 7.4));
        pilot.pov(180).onTrue(new InstantCommand(() -> limelight3.setPipeline(0)));

        copilot.pov(0).onTrue(commandFactory.algaeProcessorPosition());
        copilot.pov(0).onFalse(commandFactory.algaeProcessorScore());
        copilot.b().onTrue(commandFactory.algaeLowPosition());
        copilot.b().onFalse(commandFactory.algaeIdle());
        copilot.y().onTrue(commandFactory.algaeHighPosition());
        copilot.y().onFalse(commandFactory.algaeIdle());
        copilot.a().onTrue(commandFactory.algaeBargePosition());
        //copilot.a().onFalse(commandFactory.algaeBargeBack());
        
        //copilot.a().onFalse(commandFactory.algaeBargeScore());


        copilot.rightTrigger(.7).onTrue(commandFactory.IntakeCoralPosition());
        copilot.rightTrigger(.7).onFalse(commandFactory.IntakeCoralTest());

        // copilot.pov(180).onTrue(commandFactory.algaeGroundPosition());
        // copilot.pov(180).onFalse();
        
        copilot.pov(270).onTrue(commandFactory.algaeGroundPosition());
        copilot.pov(270).onFalse(new RequestStateChange(States.IDLE, stateManager));

        copilot.back().whileTrue(new ElevatorCommandLimit(elevator));
    }


    public Command getAutonomousCommand() {
        return chooser.getSelected();
    }
}
