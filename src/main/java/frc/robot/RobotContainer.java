package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.commands.*;
import frc.robot.subsystems.*;
import frc.robot.libraries.external.control.Path;
import frc.robot.libraries.external.control.Trajectory;
import frc.robot.libraries.external.util.AutonomousChooser;

import frc.robot.state.RobotState;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;


public class RobotContainer {
    private final Joystick driver = new Joystick(0);
    private final Joystick secondaryController = new Joystick(1);

    /* Drive Controls */
    private final int translationAxis = XboxController.Axis.kLeftY.value;
    private final int strafeAxis = XboxController.Axis.kLeftX.value;
    private final int rotationAxis = XboxController.Axis.kRightX.value;

    /* Driver Buttons */
    private final JoystickButton zeroGyro = new JoystickButton(driver, XboxController.Button.kY.value);
    private final JoystickButton robotCentric = new JoystickButton(driver, XboxController.Button.kBack.value);
   // private final JoystickButton blinkinButton = new JoystickButton(driver, XboxController.Button.kB.value);
    // private final JoystickButton highButton = new JoystickButton(driver, XboxController.Button.kY.value);
    // private final JoystickButton lowButton = new JoystickButton(driver, XboxController.Button.kA.value);
    // private final JoystickButton midButton = new JoystickButton(driver, XboxController.Button.kX.value);
    private final JoystickButton limelight1 = new JoystickButton(driver, XboxController.Button.kA.value);
    //private final JoystickButton limelight2 = new JoystickButton(driver, XboxController.Button.kB.value);

    
    /* CoPilot Buttons */

    //    private final JoystickButton midRung = new JoystickButton(secondaryController, XboxController.Button.kX.value);

    private final JoystickButton highRung = new JoystickButton(secondaryController, XboxController.Button.kY.value);

    private final JoystickButton midRung = new JoystickButton(secondaryController, XboxController.Button.kX.value);

    private final JoystickButton lowRung = new JoystickButton(secondaryController, XboxController.Button.kA.value);
    private final JoystickButton Blinkin = new JoystickButton(secondaryController, XboxController.Button.kRightBumper.value);
    private final JoystickButton intakeOut = new JoystickButton(secondaryController, XboxController.Button.kLeftBumper.value);
private final JoystickButton lowPickUp = new JoystickButton(secondaryController, XboxController.Button.kStart.value);
    private final JoystickButton playerStation = new JoystickButton(secondaryController, XboxController.Button.kB.value);

    /* Subsystems */
    public RobotState robotState = new RobotState(driver);
    private final Swerve s_Swerve = new Swerve();
    private final Superstructure superstructure = new Superstructure();



    private final LimelightSubsystem limelight = new LimelightSubsystem(NetworkTableInstance.getDefault().getTable("limelight"));
    // private final DrivetrainSubsystem drivetrainSubsystem = new DrivetrainSubsystem();
    // private final VisionSubsystem visionSubsystem = new VisionSubsystem(drivetrainSubsystem);

    private Trajectory[] trajectories;
     private final AutonomousChooser autonomousChooser;


    public RobotContainer() {
        // secondaryController.getLeftXAxis().setInverted(true);
        // secondaryController.getRightXAxis().setInverted(true);


        // CommandScheduler.getInstance().setDefaultCommand(drivetrainSubsystem, new DriveCommand(drivetrainSubsystem, getDriveForwardAxis(), getDriveStrafeAxis(), getDriveRotationAxis()));
        
        s_Swerve.setDefaultCommand(
            new TeleopSwerve(
                s_Swerve, 
                () -> -driver.getRawAxis(translationAxis), 
                () -> -driver.getRawAxis(strafeAxis), 
                () -> -driver.getRawAxis(rotationAxis), 
                () -> robotCentric.getAsBoolean(),
                robotState
            )
        );

        configurePilotButtonBindings();
        configureCoPilotButtonBindings();
         autonomousChooser = new AutonomousChooser(trajectories, this);
    }


    private void configurePilotButtonBindings() {
        zeroGyro.onTrue(new InstantCommand(() -> s_Swerve.zeroGyro()));

    }
//do new button bindings
   private void configureCoPilotButtonBindings() {

      


       //  limelight1.onTrue(new LimelightCommand(limelight, 1, s_Swerve, robotState.swerveState,robotState.limelightState)); // set it up for a toggleontrue later

     
    }

     public Command getAutonomousCommand() {
    //     return autonomousChooser.getCommand(this);
    // }
    // // public Command getAutonomousCommand() {
         // An ExampleCommand will run in autonomous
         return new AutoBalance(s_Swerve, robotState);
     // return new AutoBalance(s_Swerve, robotState);
// return autonomousChooser.getCommand(this);
     }


     public Swerve getDrivetrainSubsystem() {
         return s_Swerve;
     }


   public Superstructure getSuperstructure() {
       return superstructure;
   }



    public Joystick getsecondaryController() {
        return secondaryController;
    }

    public Joystick getPrimaryController() {
        return driver;
    }

   public Trajectory[] getTrajectories() {
       return trajectories;
   }


 public AutonomousChooser getAutonomousChooser() {
         return autonomousChooser;
     }







}
