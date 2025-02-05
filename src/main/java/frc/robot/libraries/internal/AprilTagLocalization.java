// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.libraries.internal;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.DegreesPerSecond;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Seconds;
import static frc.robot.constants.AprilTagLocalizationConstants.FIELD_LAYOUT;
import static frc.robot.constants.AprilTagLocalizationConstants.LOCALIZATION_PERIOD;
import static frc.robot.constants.AprilTagLocalizationConstants.MAX_TAG_DISTANCE;

import java.util.function.Supplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.MutAngle;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.constants.AprilTagLocalizationConstants.LimelightDetails;
import frc.robot.libraries.internal.LimelightHelpers.PoseEstimate;


/**
 * A class that uses the limelight to localize the robot using AprilTags.
 * it runs in a background thread instead of the main robot loop.
 */
public class AprilTagLocalization {
  private Notifier m_notifier = new Notifier(this::poseEstimate);  //calls pose estimate on the the period
  private LimelightDetails[] m_LimelightDetails;  // list of limelights that can provide updates
  private Supplier<Pose2d> m_robotPoseSupplier;  // supplies the pose of the robot
  private boolean m_FullTrust;  //to allow for button trust the tag estimate over all else.
  private MutAngle m_yaw = Degrees.mutable(0); ;
  private MutAngle m_OldYaw = Degrees.mutable(0);  // the previous yaw 
  private VisionConsumer m_VisionConsumer;
  private ResetPose m_poseReset;

  /**
   * Creates a new AprilTagLocalization.
   * @param poseSupplier supplies the current robot pose
   * @param visionConsumer // a consumer that accepts the vision pose, timestamp, and std deviations
   * @param details // the details of the limelight, more than one can be passed to allow for multipe on the bot.
   */
  public AprilTagLocalization(Supplier<Pose2d> poseSupplier, ResetPose resetPose, VisionConsumer visionConsumer,  LimelightDetails ... details) {
    m_notifier.startPeriodic(LOCALIZATION_PERIOD.in(Seconds)); // set up a pose estimation loop with a 0.02 second period.
    m_LimelightDetails = details;
    m_robotPoseSupplier = poseSupplier;
    m_poseReset = resetPose;
    m_VisionConsumer = visionConsumer;
  }

  /**
   * Sets the full trust of the vision system.  The robot will trust the vision system over all other sensors.
   * @param fullTrust
   */
  public void setFullTrust(boolean fullTrust) {
    m_FullTrust = fullTrust;
  }

  public Command setTrust(boolean trust) {
    return Commands.runOnce(
        () -> {
          setFullTrust(trust);
        });
  }

  /**
   * Checks if the pose is off the field.
   * @param observation
   * @return
   */
  private boolean isPoseOffField(Pose2d observation) { 
    // Coordinates for where Pose is on the field
    return observation.getX() < 0.0 // What X position robot is on the field.
        || observation.getY() < 0.0 // What Y position robot is on the field.
        || observation.getX() > FIELD_LAYOUT.getFieldLength()// Whether the robot X position is on the field or not
        || observation.getY() > FIELD_LAYOUT.getFieldWidth(); // Whether the robot X position is on the field or not
  }

  /**
   * Interpolates between two std deviations.
   * @param closeStdDevs
   * @param farStdDevs
   * @param scale
   * @return
   */
  private Matrix<N3, N1> interpolate(Matrix<N3, N1> closeStdDevs, Matrix<N3, N1> farStdDevs, double scale){
    return VecBuilder.fill(MathUtil.interpolate(closeStdDevs.get(0,0), farStdDevs.get(0,0), scale),
                    MathUtil.interpolate(closeStdDevs.get(1,0), farStdDevs.get(1,0), scale),
                    MathUtil.interpolate(closeStdDevs.get(2,0), farStdDevs.get(2,0), scale));
  }

  /**
   * Estimates the pose of the robot using the limelight.
   * This function will run in a background thread once per AprilTagLocalizationConstants.LOCALIZATION_PERIOD.
   */
  public void poseEstimate() {
    for (LimelightDetails limelightDetail : m_LimelightDetails) {
      m_yaw.mut_replace(Degrees.of(m_robotPoseSupplier.get().getRotation().getDegrees()));
      AngularVelocity yawRate = (m_yaw.minus(m_OldYaw).div(LOCALIZATION_PERIOD));
      // Set Orientation using LimelightHelpers.SetRobotOrientation and the m_robotPoseSupplier
      LimelightHelpers.SetRobotOrientation(limelightDetail.name, m_yaw.in(Degrees), yawRate.in(DegreesPerSecond),0,0,0,0 );  // Set Orientation using LimelightHelpers.SetRobotOrientation and the m_robotPoseSupplier
      // Get the pose from the Limelight
      PoseEstimate poseEstimate = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(limelightDetail.name);  // Get the pose from the Limelight 

      if (poseEstimate != null && poseEstimate.pose.getX() != 0.0 && poseEstimate.pose.getY() != 0.0) {
        // remove the offset of the camera
        poseEstimate.pose = poseEstimate.pose.transformBy(new Transform2d(limelightDetail.inverseOffset.get(0, 0),limelightDetail.inverseOffset.get(1, 0),Rotation2d.fromDegrees(limelightDetail.inverseOffset.get(2, 0)))); 

        double scale = poseEstimate.avgTagDist / MAX_TAG_DISTANCE.in(Meters); // scale the std deviation by the distance
        // Validate the pose for sanity reject bad poses  if fullTrust is true accept regarless of sanity

        if (m_FullTrust) {
          // set the pose in the pose consumer
          m_poseReset.accept(new Pose2d(poseEstimate.pose.getX(),poseEstimate.pose.getY(),Rotation2d.fromDegrees(m_yaw.in(Degrees))));
        } else if (!(isPoseOffField(poseEstimate.pose)) && poseEstimate.avgTagDist < MAX_TAG_DISTANCE.in(Meters)) { // reject poses that are more than max tag distance we trust
          // scale std deviation by distance if fullTrust is true set the stdDevs super low.
          Matrix<N3,N1> interpolated = interpolate(limelightDetail.closeStdDevs, limelightDetail.farStdDevs, scale);
          
          // set the pose in the pose consumer
          m_VisionConsumer.accept(poseEstimate.pose, poseEstimate.timestampSeconds, interpolated);
        }
        m_OldYaw.mut_replace(m_yaw);
      }
    }
  }
  
  /** Defines a function pointer to a function with a signature
   * ( Pose2d visionRobotPoseMeters, double timestampSeconds, Matrix<N3, N1> visionMeasurementStdDevs)
   * to accept the vision pose estimate
   * */
  @FunctionalInterface
  public interface VisionConsumer {
    void accept(
      Pose2d visionRobotPoseMeters,
      double timestampSeconds,
      Matrix<N3, N1> visionMeasurementStdDevs);
  }

  @FunctionalInterface
  public interface ResetPose {
    void accept( 
      Pose2d pose2d
    );    
  }
}