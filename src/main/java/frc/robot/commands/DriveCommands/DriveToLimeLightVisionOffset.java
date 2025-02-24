// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.DriveCommands;

import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.libraries.external.drivers.Limelight;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.LimelightSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html




public class DriveToLimeLightVisionOffset extends SequentialCommandGroup {
  /** Creates a new DriveToLimeLightVisionOffset. */



  public double getRotation(int tagID) {
    switch (tagID) {
        // BLUE ALLIANCE ONES - GYRO MUST BE ZEROD CORRECTLY
        case 18:
            return 0.0;
        case 19:
            return 60.0;
        case 20:
            return 120;
        case 21:
            return 180;
        case 22:
            return 240;
        case 17:
            return 300;
        // RED ALLIANCE ONES
        case 7:
            return 0.0;
        case 8:
            return 60.0;
        case 9:
            return 120;
        case 10:
            return 180;
        case 11:
            return 240;
        case 6:
            return 300;
        default:
            return 0.0;
    }
}

public double getTXValues( boolean right){
  return  right ? -.1 : 0.24 ; //tune alter arbtirary values
}

public double getTYValues(boolean right){
  return  right ? -.1 : 0.24; //tune later arbitary values
}

  public DriveToLimeLightVisionOffset(CommandSwerveDrivetrain swerve, LimelightSubsystem limelight, int pipeline, boolean right) {
    
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());


    
    addCommands(
      new TurnToAngle(swerve,getRotation((int) limelight.getTargetAprilTagID())),
      new DriveToTargetOffset(swerve, limelight, 0, pipeline, getTXValues(right), getTYValues(right))
    );
  }
}
