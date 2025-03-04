// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.StateManager;
import frc.robot.subsystems.StateManager.States;
import swervelib.parser.SwerveParser;
import frc.robot.subsystems.IntakeRollers;
import frc.robot.subsystems.IntakeWristRev;
import frc.robot.subsystems.LimelightSubsystem;
import frc.robot.CommandFactory.desiredAction;
import frc.robot.commands.BasicCommands.ElevatorCommand;
import frc.robot.commands.BasicCommands.IntakeCommand;
import frc.robot.commands.BasicCommands.RequestStateChange;
import frc.robot.commands.BasicCommands.WristCommand;
import frc.robot.commands.ComplexCommands.returnToIdle;
import frc.robot.commands.DefaultCommands.DefaultClimberCommand;
import frc.robot.commands.DefaultCommands.DefaultElevatorCommand;
import frc.robot.commands.DefaultCommands.DefaultRollerCommand;
import frc.robot.commands.DefaultCommands.DefaultWristCommand;
import frc.robot.commands.DriveCommands.DriveToLimeLightVisionOffset;
import frc.robot.commands.DriveCommands.DriveToTargetOffset;
import frc.robot.commands.DriveCommands.LockOnAprilTag;
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

    private final SwerveRequest.RobotCentric robotCentricDrive = new SwerveRequest.RobotCentric().withDeadband(MaxSpeed * 0.025).withRotationalDeadband(MaxAngularRate * 0.05) // Add a 10% deadband
    .withDriveRequestType(DriveRequestType.OpenLoopVoltage);
    private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
    private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();

    private final Telemetry logger = new Telemetry(MaxSpeed);

    private final CommandXboxController pilot = new CommandXboxController(0);
    private final CommandXboxController copilot = new CommandXboxController(1);

    public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();

    private final StateManager stateManager = new StateManager();
    //private final Climber climber = new Climber();
    private final Elevator elevator = new Elevator(stateManager);
    private final IntakeRollers rollers = new IntakeRollers();
    private final IntakeWristRev wrist = new IntakeWristRev(stateManager);

    

    
    private final LimelightSubsystem limelight3 = new LimelightSubsystem( drivetrain,"limelight-three",0,0,0);
    private final LimelightSubsystem limelight2 = new LimelightSubsystem(drivetrain,"limelight-two",0,0,0);
    private final LimelightSubsystem limelight4 = new LimelightSubsystem(drivetrain, "limelight-four",0,0,0);

    private final CommandFactory commandFactory = new CommandFactory( elevator, rollers, wrist, limelight4, limelight3, limelight2, pilot, drivetrain, stateManager);
    private final AutonomousCommandFactory autoFactory = new AutonomousCommandFactory( elevator, rollers, wrist, limelight4, limelight3, limelight2, pilot, drivetrain, stateManager);

    private final cheaterDPAD dpad = new cheaterDPAD(commandFactory, stateManager);

    public RobotContainer() {
        stateManager.requestNewState(States.IDLE);
        //drivetrain.seedFieldCentric();
        configureBindings();
        autoFactory.configAutonomousCommands();
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

        configureOldBindings();

        drivetrain.registerTelemetry(logger::telemeterize);
    }

    private void configureOldBindings() {
        pilot.leftTrigger().onTrue(commandFactory.LevelPosition(1));
        pilot.leftTrigger().onFalse(commandFactory.LevelScore());
        pilot.leftBumper().onTrue(commandFactory.LevelPosition(2));
        pilot.leftBumper().onFalse(commandFactory.LevelScore());
        pilot.rightTrigger().onTrue(commandFactory.Level4Position());
        pilot.rightTrigger().onFalse(commandFactory.Level4Score());
        pilot.rightBumper().onTrue(commandFactory.LevelPosition(3));
        pilot.rightBumper().onFalse(commandFactory.LevelScore());

        

        pilot.b().whileTrue(new InstantCommand( ()-> drivetrain.applyRequest(() ->
            robotCentricDrive.withVelocityX(-pilot.getLeftY() * MaxSpeed) // Drive forward with negative Y (forward)
                .withVelocityY(-pilot.getLeftX() * MaxSpeed) // Drive left with negative X (left)
                .withRotationalRate(-pilot.getRightX() * MaxAngularRate)) // Drive counterclockwise with negative X (left)
        ));

        pilot.x().whileTrue(new LockOnAprilTag(drivetrain, limelight3, 0, pilot, false));
        pilot.a().whileTrue(new TurnToAngle(drivetrain, limelight3));
        pilot.pov(0).whileTrue(new DriveToTargetOffset(drivetrain, limelight3, 0, 0, -15, 15));
        //pilot.pov(90).whileTrue(new DriveToTargetOffset(drivetrain, limelight4, 0, 0, -22, 13));
        pilot.pov(90).whileTrue(new DriveToTargetOffset(drivetrain, limelight4, 0, 0, 17.3, 9.3));
        pilot.pov(180).whileTrue(new TurnToAngle(drivetrain, limelight4));


        copilot.pov(0).onTrue(commandFactory.algaeProcessorPosition());
        copilot.pov(0).onFalse(commandFactory.algaeProcessorScore());
        copilot.b().onTrue(commandFactory.algaeLowPosition());
        copilot.b().onFalse(commandFactory.algaeIdle());
        copilot.y().onTrue(commandFactory.algaeHighPosition());
        copilot.y().onFalse(commandFactory.algaeIdle());


        copilot.rightTrigger(.7).onTrue(new RequestStateChange(States.INTAKE_CORAL, stateManager));
        copilot.rightTrigger(.7).onFalse(new SequentialCommandGroup(
            new ParallelCommandGroup(new WaitCommand(1.5), new ElevatorCommand(elevator, 8.8)),
            new ParallelRaceGroup(new WaitCommand(0.25), new ElevatorCommand(elevator, 9.375)),
            new ParallelDeadlineGroup(new WaitCommand(1), new WristCommand(wrist, 60)), 
            new RequestStateChange(States.IDLE, stateManager)
        ));
    }

    private void configureCueBindings() {
        // pilot.leftTrigger(.7).whileTrue(commandFactory.getDesiredAction(true));
        // pilot.leftTrigger().onFalse(commandFactory.getDesiredAction(false));

        // //pilot.a().onTrue(commandFactory.getLimelightAction());


        // copilot.leftTrigger(.7).onTrue(new InstantCommand(() -> commandFactory.setDesiredAction(desiredAction.SCOREL1)));
        // copilot.leftBumper().onTrue(new InstantCommand(() -> commandFactory.setDesiredAction(desiredAction.SCOREL2)));
        // copilot.rightTrigger(.7).onTrue(new InstantCommand(() -> commandFactory.setDesiredAction(desiredAction.SCOREL4)));
        // copilot.rightBumper().onTrue(new InstantCommand(() -> commandFactory.setDesiredAction(desiredAction.SCOREL3)));

        // copilot.y().onTrue(commandFactory.setDesiredAction(desiredAction.INTAKE_ALGAE_HIGH));
        // copilot.b().onTrue(commandFactory.setDesiredAction(desiredAction.INTAKE_ALGAE_LOW));
        //copilot.a().onTrue(commandFactory.setDesiredAction(desiredAction.INTAKE_ALGAE_GROUND));
        //copilot.x().onTrue(commandFactory.setDesiredAction(desiredAction.CLIMBER));

        //copilot.pov(180).onTrue(commandFactory.setDesiredAction(desiredAction.PROCESSOR));
    }

    public Command getAutonomousCommand() {
        return Commands.print("No autonomous command configured");
    }
}
