package frc.robot.subsystems;

import java.util.Optional;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.LimelightHelpers;
import swervelib.telemetry.SwerveDriveTelemetry;

public class LimelightSubsystem extends SubsystemBase {
  /** Creates a new LimeLight. */
  private NetworkTable table;
  private NetworkTableEntry tx;
  private NetworkTableEntry ty;
  private NetworkTableEntry ta;
  private NetworkTableEntry tv;
  private NetworkTableEntry tid;
  private double horizontal_offset = 0;
  private String name;
  private Pose2d botpose;
  private SwerveSubsystem driveTrain;
  boolean trust = false;




  public LimelightSubsystem(String name,SwerveSubsystem swerve) {
    /**
     * tx - Horizontal Offset
     * ty - Vertical Offset 
     * ta - Area of target 
     * tv - Target Visible
     */

    this.table = NetworkTableInstance.getDefault().getTable(name);
    this.tx = table.getEntry("tx");
    this.ty = table.getEntry("ty");
    this.ta = table.getEntry("ta");
    this.tv = table.getEntry("tv");
        this.tid = table.getEntry("tid");

this.name = name;
this.driveTrain = swerve;
  }

  public LimelightSubsystem(String name,double offset,SwerveSubsystem drivetrain) {
    /**
     * tx - Horizontal Offset
     * ty - Vertical Offset 
     * ta - Area of target 
     * tv - Target Visible
     */

    this.table = NetworkTableInstance.getDefault().getTable(name);
    this.tx = table.getEntry("tx");
    this.ty = table.getEntry("ty");
    this.ta = table.getEntry("ta");
    this.tv = table.getEntry("tv");
    this.tid = table.getEntry("tid");
    this.horizontal_offset = offset;
    this.name = name;
    this.driveTrain = drivetrain;
  }
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public double getHorizontalAngleOfErrorDegrees(){
    return getTx().getDouble(0.0) +horizontal_offset;

  }

  public double getVerticalAngleOfErrorDegrees(){
    return getTy().getDouble(0.0) +0;
  }


 public NetworkTableEntry getTx() {
    return tx;
  }

  public NetworkTableEntry getTy() {
    return ty;
  }

  public NetworkTableEntry getTa() {
    return ta;
  }

  public void setPipeline(int pipe){
    table.getEntry("pipeline").setNumber(pipe);
  }

  public int getID(){
    return (int) this.tid.getInteger(0);
  }

  public double getTxAngleRadians() {
    return Units.degreesToRadians(tx.getDouble(0));
  }

  public double getTargetAngleRadians() {
    return getTxAngleRadians();
  }
  public boolean hasTarget(){
    return tv.getDouble(0) != 0;
  }
}

public void updatePoseEstimation(SwerveSubsystem swerveDrive)
{
  if (SwerveDriveTelemetry.isSimulation){
    //TODO: DO VISION STUFF
  }
 
    Optional<EstimatedRobotPose> poseEst = getEstimatedGlobalPose(camera);
    if (poseEst.isPresent())
    {
      var pose = poseEst.get();
      swerveDrive.addVisionMeasurement(pose.estimatedPose.toPose2d(),
                                       pose.timestampSeconds,
                                       camera.curStdDevs);
    }
  }

private int longDistangePoseEstimationCount = 0;
private double maximumAmbiguity = .3;

    private Optional<EstimatedRobotPose> filterPose(Optional<EstimatedRobotPose> pose)
  {
    if (pose.isPresent())
    {
      double bestTargetAmbiguity = 1; // 1 is max ambiguity
      for (LimelightHelpers. target : pose.get().targetsUsed)
      {
        double ambiguity = target.getPoseAmbiguity();
        if (ambiguity != -1 && ambiguity < bestTargetAmbiguity)
        {
          bestTargetAmbiguity = ambiguity;
        }
      }
      //ambiguity to high dont use estimate
      if (bestTargetAmbiguity > maximumAmbiguity)
      {
        return Optional.empty();
      }

      //est pose is very far from recorded robot pose
      if (PhotonUtils.getDistanceToPose(currentPose.get(), pose.get().estimatedPose.toPose2d()) > 1)
      {
        longDistangePoseEstimationCount++;

        //if it calculates that were 10 meter away for more than 10 times in a row its probably right
        if (longDistangePoseEstimationCount < 10)
        {
          return Optional.empty();
        }
      } else
      {
        longDistangePoseEstimationCount = 0;
      }
      return pose;
    }
    return Optional.empty();
  }


}