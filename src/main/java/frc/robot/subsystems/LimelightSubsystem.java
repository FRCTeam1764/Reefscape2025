package frc.robot.subsystems;

import java.util.Optional;

import org.dyn4j.geometry.Rotation;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.LimelightHelpers;
import frc.robot.LimelightHelpers.PoseEstimate;
import swervelib.telemetry.SwerveDriveTelemetry;

public class LimelightSubsystem extends SubsystemBase {
  /** Creates a new LimeLight. */

  private double horizontal_offset = 0;
  private Limelight[] Limelights;
  private Pose2d botpose;
  private SwerveSubsystem driveTrain;
  boolean trust = false;


   

  public LimelightSubsystem(SwerveSubsystem swerve, int numOfLimelights) {
    /**
     * tx - Horizontal Offset
     * ty - Vertical Offset 
     * ta - Area of target 
     * tv - Target Visible
     */


this.driveTrain = swerve;
Limelights = new Limelight[numOfLimelights];

  }


  


public void addCamera(String camera, Rotation rot, double yoffset, double xoffset){
Limelights[0] = new Limelight(camera, rot, yoffset, xoffset);

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
    //TODO: DO VISION STUFF FOR SIM 
  }
 
    Optional<PoseEstimate> poseEst = getEstimatedGlobalPose(camera);
    if (poseEst.isPresent())
    {
      var pose = poseEst.get();
      swerveDrive.addVisionMeasurement(pose.estimatedPose.toPose2d(),
                                       pose.timestampSeconds,
                                       camera.curStdDevs);
    }
  }
