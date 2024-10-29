package frc.robot.commands;

import frc.robot.constants.SwerveConstants;
import frc.robot.state.LimelightState;
import frc.robot.state.RobotState;
import frc.robot.subsystems.Swerve;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;


import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.hal.ThreadsJNI;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.LimelightSubsystem;


public class TeleopSwerve extends CommandBase {    
    static final double kOffBalanceAngleThresholdDegrees = 10;
    static final double kOonBalanceAngleThresholdDegrees  = 5;
    double offsetToleranceProprtion;

    private Swerve s_Swerve;    
    private DoubleSupplier translationSup;
    private DoubleSupplier strafeSup;
    private DoubleSupplier rotationSup;
    private BooleanSupplier robotCentricSup;
    private LimelightState limelightState;
    private LimelightSubsystem limelightSubsystem;
    private RobotState robotState;
    private double strafeVal;

public double square(double num){
    return num * Math.abs(num);
}

    public TeleopSwerve(Swerve s_Swerve, DoubleSupplier translationSup, DoubleSupplier strafeSup, DoubleSupplier rotationSup, BooleanSupplier robotCentricSup,RobotState robotState) {
        this.s_Swerve = s_Swerve;
        addRequirements(s_Swerve);
        offsetToleranceProprtion = 0.1;

        this.translationSup = translationSup;
        this.strafeSup = strafeSup;
        this.rotationSup = rotationSup;
        this.robotCentricSup = robotCentricSup;
        this.robotState = robotState;
        this.limelightState = robotState.limelightState;
    }

    /*
     * checks if limelight is on + target is in sight
     * moves to target
     */
    public double moveLeftOrRight() {
        if(robotState.swerveState.getSlowMode()){
            strafeVal = MathUtil.applyDeadband(square(strafeSup.getAsDouble())*.3, SwerveConstants.stickDeadband);
        }
        else {
            strafeVal = MathUtil.applyDeadband(square(strafeSup.getAsDouble()), SwerveConstants.stickDeadband);
        }
        return strafeVal;
    }


    
    public void execute() {
        /* Get Values, Deadband*/

        




        double translationVal = MathUtil.applyDeadband(getTranslation(), SwerveConstants.stickDeadband);
        double rotationVal = MathUtil.applyDeadband(square(rotationSup.getAsDouble()), SwerveConstants.stickDeadband);
        // double strafeVal = MathUtil.applyDeadband(strafeSup.getAsDouble()*.75, SwerveConstants.stickDeadband);
       
       //auto balance if autobalance has been toggled
        if(robotState.swerveState.getStartButton()){
            robotState.swerveState.swerveAutoBalance();
        }
        if(robotState.swerveState.getSlowButton()){
            robotState.swerveState.ToggleSlowMode();
        }
        // System.out.println(translationVal);
        /* Drive */
        s_Swerve.drive(
            new Translation2d(translationVal, moveLeftOrRight()).times(SwerveConstants.Swerve.maxSpeed), 
            rotationVal * SwerveConstants.Swerve.maxAngularVelocity, 
            !robotCentricSup.getAsBoolean(), 
            true
        );
    }
    double transValue;
    public double getTranslation(){
        if(robotState.swerveState.getSwerveState()){
            transValue = getAutoLevel();
        }
        else if(robotState.swerveState.getSlowMode()){
            transValue = translationSup.getAsDouble()*.3;
        }
        else{
            transValue =square(translationSup.getAsDouble());
        }
        return transValue;
    }
    double error;
    double autoLevelPwr;
    public double getAutoLevel(){
       error = -s_Swerve.getNavx().getPitch();
       if(Math.abs(error)<1){
           robotState.swerveState.noSwerveAutoBalance();;
       }
       autoLevelPwr = -Math.min(error*.023, 1);
        // System.out.println(error+ " " +autoLevelPwr);
       return autoLevelPwr;
   }
}