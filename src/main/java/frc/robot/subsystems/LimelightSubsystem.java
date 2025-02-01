package frc.robot.subsystems;


import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.LimelightHelpers;
import frc.robot.LimelightHelpers.PoseEstimate;
import swervelib.SwerveDrive;
import swervelib.telemetry.SwerveDriveTelemetry;

public class LimelightSubsystem extends SubsystemBase {
  /** Creates a new LimeLight. */

  private double horizontal_offset = 0;
  private String Limelight;
  private Pose2d botpose;
  private SwerveSubsystem driveTrain;

  private double txoffset;
  private double tyoffset;
  private double rotyaw;
  boolean trust = false;


   
//note: we may in the future need to hvae rotpitch/roll but for now im im not implimenting that. it sucks! 3dvectors are awful

  public LimelightSubsystem(SwerveSubsystem swerve, String LimelightName,double txoffset, double tyoffset, double rotyaw) {
    /**
     * tx - Horizontal Offset
     * ty - Vertical Offset 
     * ta - Area of target 
     * tv - Target Visible
     */


this.driveTrain = swerve;
this.txoffset = txoffset;
this.tyoffset = tyoffset;
this.rotyaw = rotyaw;
Limelight = LimelightName;

  }


  



  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    updatePoseEstimation();
  }

  public double getHorizontalAngleOfErrorDegrees(){
    return getTx();

  }

  public double getVerticalAngleOfErrorDegrees(){
    return getTy() +0;
  }


 public double getTx() {
    return LimelightHelpers.getTX(Limelight);
  }

  public double getTy() {
    return LimelightHelpers.getTY(Limelight);
  }

  public double getTa() {
    return LimelightHelpers.getTA(Limelight);
  }

  public void setPipeline(int pipe){
LimelightHelpers.setPipelineIndex(Limelight, pipe);
  }

  public int getID(){
    return (int) LimelightHelpers.getFiducialID(Limelight);
  }


  public boolean hasTarget(){
    return LimelightHelpers.getTV(Limelight);
  }


  public double getTxAngleRadians(){
    return   Units.degreesToRadians(getTx());
  }
public void updatePoseEstimation()
{
  if (SwerveDriveTelemetry.isSimulation){
    //TODO: DO VISION STUFF FOR SIM 
  }
 
  //TODO: REPLACE AND IMPLIMENT BRAD CODE LATER
  

  LimelightHelpers.SetRobotOrientation(Limelight, driveTrain.getPose().getRotation().getDegrees(),0,0,0,0,0);

  PoseEstimate poseEstimate = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(Limelight);
  if (poseEstimate != null && poseEstimate.pose.getX() != 0.0 && poseEstimate.pose.getY() != 0.0) {

  poseEstimate.pose.transformBy(new Transform2d(txoffset, tyoffset, new Rotation2d(rotyaw)));

  

  driveTrain.addVisionMeasurement(poseEstimate.pose,poseEstimate.timestampSeconds);
  }
}


}