// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.DriveCommands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.libraries.external.drivers.Limelight;
import frc.robot.subsystems.CommandSwerveDrivetrain;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class DriveToLimeLightVisionOffset extends SequentialCommandGroup {
  /** Creates a new DriveToLimeLightVisionOffset. */
  public DriveToLimeLightVisionOffset(CommandSwerveDrivetrain swerve, Limelight limelight) {
    
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands();
  }
}
