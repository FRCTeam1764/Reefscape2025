// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.constants;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Seconds;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Time;

/** Add your docs here. */
public class AprilTagLocalizationConstants {
  public static class LimelightDetails {
    public String name;
    public Matrix<N3, N1> closeStdDevs;
    public Matrix<N3, N1> farStdDevs;
    public Matrix<N3, N1> inverseOffset;

    /*
     * 
     */
    public LimelightDetails(String name, Matrix<N3, N1> closeStdDevs, Matrix<N3, N1> farStdDevs, Matrix<N3,N1> inverseOffset) {
      this.name = name;
      this.closeStdDevs = closeStdDevs;
      this.farStdDevs = farStdDevs;
      this.inverseOffset = inverseOffset;
    }
  }

  public static final String LIMELIGHT_NAME = "limelight-back";
  public static final Matrix<N3, N1> LIMELIGHT_CLOSE_STDDEV =  VecBuilder.fill(0.05, 0.05, 999999999.9);
  public static final Matrix<N3, N1> LIMELIGHT_FAR_STDDEV =  VecBuilder.fill(0.05, 0.05, 999999999.9);
  public static final Matrix<N3, N1> LIMELIGHT_INVERSE_OFFSET =  VecBuilder.fill(0.38, 0, 0);
  public static final LimelightDetails LIMELIGHT_DETAILS = new LimelightDetails(LIMELIGHT_NAME, LIMELIGHT_CLOSE_STDDEV, LIMELIGHT_CLOSE_STDDEV,LIMELIGHT_INVERSE_OFFSET);
  public static final AprilTagFieldLayout FIELD_LAYOUT = AprilTagFieldLayout.loadField(AprilTagFields.k2024Crescendo); //TODO MAKE 2025
  public static final Distance MAX_TAG_DISTANCE = Meters.of(3.0);
  public static final Time LOCALIZATION_PERIOD = Seconds.of(0.02);
}