package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

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