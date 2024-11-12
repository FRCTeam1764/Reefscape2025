package frc.robot;

import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.libraries.external.robot.UpdateManager;


public class Robot extends TimedRobot {
    private static Robot instance = null;
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
    updateManager.startLoop(5.0e-3);

   }


   @Override
   public void robotPeriodic() {
    CommandScheduler.getInstance().run();
   }

   @Override
   public void autonomousInit() {
    robotContainer.getAutonomousCommand().schedule();

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
