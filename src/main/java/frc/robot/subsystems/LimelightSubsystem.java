package frc.robot.subsystems;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.LimelightHelpers;
import frc.robot.RobotContainer;
import frc.robot.constants.Constants;
//import webblib.util.RectanglePoseArea;

public class LimelightSubsystem extends SubsystemBase {
  /** Creates a new LimeLight. */
  // public static final RectanglePoseArea field =
  // new RectanglePoseArea(new Translation2d(0.0, 0.0), new Translation2d(16.54, 8.02));
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

    
//TODO: test or somtehin, ya 
/* 
      Double targetDistance = LimelightHelpers.getTargetPose3d_CameraSpace(name).getTranslation().getDistance(new Translation3d());
      // Tune this for your robot around how much variance you see in the pose at a given distance
    //  Double confidence = 1 - ((targetDistance - 1) / 6);
      LimelightHelpers.Results result =
          LimelightHelpers.getLatestResults(name).targetingResults;
      if (result.valid) {
        botpose = LimelightHelpers.getBotPose2d_wpiBlue(name);
          if (driveTrain.getPose().getTranslation().getDistance(botpose.getTranslation()) < 0.5
              || trust
              || result.targets_Fiducials.length > 1) {
             driveTrain.addVisionMeasurement(
                botpose,
                Timer.getFPGATimestamp()
                    - (result.latency_capture / 1000.0)
                    - (result.latency_pipeline / 1000.0)
             );
          }
          
        }
        */
      
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