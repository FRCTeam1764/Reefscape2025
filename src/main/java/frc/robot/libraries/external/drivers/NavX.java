package frc.robot.libraries.external.drivers;

import com.studica.frc.AHRS;
import com.studica.frc.AHRS.NavXComType;
import com.studica.frc.AHRS.NavXUpdateRate;

import com.studica.frc.AHRS;
import edu.wpi.first.wpilibj.SPI;
import frc.robot.libraries.external.drivers.Gyroscope;
import frc.robot.libraries.external.math.Rotation2;

public final class NavX extends Gyroscope {
    private final AHRS navX;

    public NavX() {
      navX = new AHRS(NavXComType.kMXP_SPI, NavXUpdateRate.k50Hz);
    }

    @Override
    public void calibrate() {
        navX.reset();
    }

    @Override
    public Rotation2 getUnadjustedAngle() {
        return Rotation2.fromRadians(getAxis(Axis.YAW));
    }

    @Override
    public double getUnadjustedRate() {
        return Math.toRadians(navX.getRate());
    }

    public double getAxis(Axis axis) {
        switch (axis) {
            case PITCH:
                return Math.toRadians(navX.getPitch());
            case ROLL:
                return Math.toRadians(navX.getRoll());
            case YAW:
                return Math.toRadians(navX.getYaw());
            default:
                return 0.0;
        }
    }

    public enum Axis {
        PITCH,
        ROLL,
        YAW
    }
}