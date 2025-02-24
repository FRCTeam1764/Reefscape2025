// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.geometry.Rotation2d;
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
import frc.robot.subsystems.IntakeRollers;
import frc.robot.subsystems.IntakeWristRev;
import frc.robot.subsystems.LimelightSubsystem;
import frc.robot.commands.BasicCommands.ElevatorCommand;
import frc.robot.commands.BasicCommands.IntakeCommand;
import frc.robot.commands.BasicCommands.RequestStateChange;
import frc.robot.commands.BasicCommands.WristCommand;
import frc.robot.commands.ComplexCommands.returnToIdle;
import frc.robot.commands.DefaultCommands.DefaultElevatorCommand;
import frc.robot.commands.DefaultCommands.DefaultRollerCommand;
import frc.robot.commands.DefaultCommands.DefaultWristCommand;
import frc.robot.commands.DriveCommands.DriveToTargetOffset;
import frc.robot.commands.DriveCommands.LockOnAprilTag;
import frc.robot.commands.DriveCommands.TurnToAngle;
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
            .withDeadband(MaxSpeed * 0.025).withRotationalDeadband(MaxAngularRate * 0.05) // Add a 10% deadband
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors
    private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
    private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();

    private final Telemetry logger = new Telemetry(MaxSpeed);

    private final CommandXboxController pilot = new CommandXboxController(0);
    private final CommandXboxController copilot = new CommandXboxController(1);

    public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();

    private final StateManager stateManager = new StateManager();
    private final Climber climber = new Climber();
    private final Elevator elevator = new Elevator(stateManager);
    private final IntakeRollers rollers = new IntakeRollers();
    private final IntakeWristRev wrist = new IntakeWristRev(stateManager);

    
    private final LimelightSubsystem limelight4 = new LimelightSubsystem( drivetrain,"Limelight",0,0,0);
    private final LimelightSubsystem limelight3 = new LimelightSubsystem(drivetrain,"limelight-three",0,0,0);
    private final LimelightSubsystem limelight2 = new LimelightSubsystem(drivetrain, "Limelight-two",0,0,0);

    private final CommandFactory commandFactory = new CommandFactory(climber, elevator, rollers, wrist, limelight4, limelight3, limelight2, copilot, drivetrain, stateManager);

    public RobotContainer() {
        stateManager.requestNewState(States.IDLE);
        configureBindings();
    }

    //code waits for no man
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

        // joystick.a().whileTrue(drivetrain.applyRequest(() -> brake));
        // joystick.b().whileTrue(drivetrain.applyRequest(() ->
        //     point.withModuleDirection(new Rotation2d(-joystick.getLeftY(), -joystick.getLeftX()))
        // ));

        // // Run SysId routines when holding back/start and X/Y.
        // // Note that each routine should be run exactly once in a single log.
        //  joystick.x().whileTrue(drivetrain.sysIdDynamic(Direction.kForward));
        //  joystick.y().whileTrue(drivetrain.sysIdDynamic(Direction.kReverse));
        //  joystick.a().whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward));
        //  joystick.b().whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse));

        // reset the field-centric heading on left bumper press
        elevator.setDefaultCommand(new DefaultElevatorCommand(elevator, stateManager));
        wrist.setDefaultCommand(new DefaultWristCommand(wrist, stateManager));
        rollers.setDefaultCommand(new DefaultRollerCommand(rollers, stateManager));

        pilot.y().onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldCentric()));
        
        pilot.b().onTrue(commandFactory.IntakeCoralTest());
        pilot.x().onTrue(commandFactory.algaeLowPosition());
        pilot.x().onFalse(commandFactory.algaeIdle());
        pilot.a().onTrue(commandFactory.algaeGroundPosition());
        pilot.a().onFalse(commandFactory.algaeIdle());
        pilot.start().onTrue(new RequestStateChange(States.IDLE, stateManager));

        pilot.pov(0).onTrue(commandFactory.Level4Position());
        pilot.pov(0).onFalse(commandFactory.Level4Score());
        pilot.pov(90).onTrue(commandFactory.LevelPosition(3));
        pilot.pov(90).onFalse(commandFactory.LevelScore());
        pilot.pov(180).onTrue(commandFactory.LevelPosition(2));
        pilot.pov(180).onFalse(commandFactory.LevelScore());

        climber.climberOn(Math.abs(pilot.getLeftX())<0.07 ? 0 : pilot.getLeftX()/2);

        //OPTIONAL STUFF TO TEST LATER
        pilot.rightBumper().whileTrue(new LockOnAprilTag(drivetrain, limelight4, 0, pilot, false));
        pilot.leftBumper().whileTrue(new TurnToAngle(drivetrain, 90.0));
        //test if this works or not
        pilot.leftTrigger(.5).whileTrue(new DriveToTargetOffset(drivetrain, limelight4, 0, 0, 2, 2));


//old code 
      //  // pilot.x().whileTrue(new InstantCommand(() -> rollers.wheelsIntake(0.2)));
      //  // joystick.x().onFalse(new InstantCommand(() -> rollers.wheelsIntake(0)));

    // pilot.b().onTrue(new RequestStateChange(States.INTAKE_CORAL, stateManager));
    
    // pilot.b().onFalse(new SequentialCommandGroup(
    //     new ParallelCommandGroup(new WaitCommand(1.5), new ElevatorCommand(elevator, 9),
    //     new ParallelRaceGroup(new WaitCommand(0.25), new ElevatorCommand(elevator, 9.375)),
    //     new WristCommand(wrist, 60), 
    //     new RequestStateChange(States.IDLE, stateManager)
    // )));
    
    // pilot.pov(0).onTrue(new RequestStateChange(States.L4, stateManager));

    
    // pilot.pov(0).onTrue(new RequestStateChange(States.L4, stateManager));
    // pilot.pov(0).onFalse(new SequentialCommandGroup(
    //                                 new WristCommand(wrist, 40), 
    //                                 new ParallelDeadlineGroup(
    //                                     new WaitCommand(.5), 
    //                                     new IntakeCommand(rollers, .2, false), 
    //                                     new WristCommand(wrist, 40)), 
    //                                     new returnToIdle(stateManager)));
    
    // pilot.pov(90).onTrue(new RequestStateChange(States.L3, stateManager));
    // pilot.pov(90).onFalse(new SequentialCommandGroup(
    //                                 new WristCommand(wrist, 40), 
    //                                 new ParallelDeadlineGroup(
    //                                     new WaitCommand(.5), 
    //                                     new IntakeCommand(rollers, .2, false), 
    //                                     new WristCommand(wrist, 40)), 
    //                                 new RequestStateChange(States.IDLE, stateManager)));
    
    // pilot.pov(180).onTrue(new RequestStateChange(States.L2, stateManager));
    // pilot.pov(180).onFalse(new SequentialCommandGroup(
    //                                 new WristCommand(wrist, 40), 
    //                                 new ParallelDeadlineGroup(
    //                                     new WaitCommand(.5), 
    //                                     new IntakeCommand(rollers, .2, false), 
    //                                     new WristCommand(wrist, 40)), 
    //                                 new RequestStateChange(States.IDLE, stateManager)));

    // // pilot.a().whileTrue(new InstantCommand(() -> climber.climberOn(.5))); //TODO: FIND VALUE
    // // pilot.a().onFalse(new InstantCommand(() -> climber.climberOn(0)));
    // // pilot.b().whileTrue(new InstantCommand(() -> climber.climberOn(-.5)));
    // // pilot.b().onFalse(new InstantCommand(() -> climber.climberOn(0)));
    // climber.climberOn(Math.abs(pilot.getLeftX())<0.07 ? 0 : pilot.getLeftX()/2);

    // pilot.a().onTrue(new RequestStateChange(States.INTAKE_ALGAE_GROUND, stateManager));
    // pilot.a().onFalse(new RequestStateChange(States.IDLE_ALGAE,stateManager));

    // pilot.x().onTrue(new RequestStateChange(States.INTAKE_ALGAE_LOW, stateManager));
    // pilot.x().onFalse(new RequestStateChange(States.IDLE_ALGAE,stateManager));


  
        drivetrain.registerTelemetry(logger::telemeterize);
    }

    public Command getAutonomousCommand() {
        return Commands.print("No autonomous command configured");
    }
}
