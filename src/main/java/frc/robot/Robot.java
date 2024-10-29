package frc.robot;




import com.fasterxml.jackson.databind.util.RootNameLookup;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.RobotController;
// import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
//import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.libraries.NewSwerve.CTREConfigs;
// import edu.wpi.first.wpilibj.util.Color;
// import edu.wpi.first.wpilibj2.command.CommandScheduler;
// import frc.robot.commands.BlinkinCommand;
// import frc.robot.commands.intakeCommand;
// import frc.robot.libraries.external.math.RigidTransform2;
// import frc.robot.libraries.external.math.Rotation2;
import frc.robot.libraries.external.robot.UpdateManager;
// import frc.robot.libraries.external.robot.drivers.Limelight;
import frc.robot.constants.Constants;


public class Robot extends TimedRobot {
   private static Robot instance = null;
   Spark blinkin;
   
  

    public static CTREConfigs ctreConfigs;


    //private Command m_autonomousCommand;
    public RobotContainer robotContainer;
    private UpdateManager updateManager = new UpdateManager(
            // robotContainer.getDrivetrainSubsystem()
    );

   public Robot() {
       instance = this;
   }


   public static Robot getInstance() {
       return instance;
   }


   @Override
   public void robotInit() {

       RobotController.setBrownoutVoltage(7.0);
         robotContainer = new RobotContainer();
  

        robotContainer.getDrivetrainSubsystem().getNavx().calibrate();
        robotContainer.getDrivetrainSubsystem().zeroGyro();

       updateManager.startLoop(5.0e-3);


   }


   @Override
   public void robotPeriodic() {

        CommandScheduler.getInstance().run();

   }


   @Override
   public void autonomousInit() {
    //    robotContainer.getDrivetrainSubsystem().resetPose(RigidTransform2.ZERO);
    //    robotContainer.getDrivetrainSubsystem().resetGyroAngle(Rotation2.ZERO);


        robotContainer.getAutonomousCommand().schedule();//fix later
   }


    @Override
    public void disabledPeriodic() {
        // robotContainer.getVisionSubsystem().setLedMode(Limelight.LedMode.OFF);
    }

   @Override
   public void teleopInit() {
    CommandScheduler.getInstance().cancelAll();
   }


}
