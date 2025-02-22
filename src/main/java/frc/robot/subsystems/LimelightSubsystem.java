package frc.robot.subsystems;


import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.util.datalog.DataLog;
import edu.wpi.first.util.datalog.DoubleLogEntry;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.libraries.internal.LimelightHelpers;
import frc.robot.libraries.internal.LimelightHelpers.PoseEstimate;
import swervelib.telemetry.SwerveDriveTelemetry;

public class LimelightSubsystem extends SubsystemBase {
  /** Creates a new LimeLight. */

 // private double horizontal_offset = 0;
  private String Limelight;
//  private Pose2d botpose;
  private CommandSwerveDrivetrain driveTrain;

  private double txoffset;
  private double tyoffset;
  private double rotyaw;
  boolean trust = false;


   
//note: we may in the future need to hvae rotpitch/roll but for now im im not implimenting that. it sucks! 3dvectors are awful

  public LimelightSubsystem(CommandSwerveDrivetrain swerve, String LimelightName,double txoffset, double tyoffset, double rotyaw) {
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

  public double getXDistance(){
    return LimelightHelpers.getCameraPose3d_RobotSpace(Limelight).getX();

  }
  public double getZDistance(){
    return LimelightHelpers.getCameraPose3d_RobotSpace(Limelight).getZ();
    
  }
  public void setPipeline(int pipe){
LimelightHelpers.setPipelineIndex(Limelight, pipe);
  }

  public int getID(){
    return (int) LimelightHelpers.getFiducialID(Limelight);
  }

  public double getDistanceToTarget(){
    double x = LimelightHelpers.getCameraPose3d_TargetSpace(Limelight).getX();
    double y = LimelightHelpers.getCameraPose3d_TargetSpace(Limelight).getZ();
    return Math.sqrt(Math.pow(x, 2) + Math.pow(y, 2));
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

  
//TODO use advantagescope to double check this is functional
  driveTrain.addVisionMeasurement(poseEstimate.pose,poseEstimate.timestampSeconds);
  }
}


}