package frc.robot.libraries.internal;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix6.hardware.TalonFX;



/**
 * This class is a thin wrapper around the CANTalon that reduces CAN bus / CPU overhead by skipping duplicate set
 * commands. (By default the Talon flushes the Tx buffer on every set call).
 */
public class LazyTalonFX extends TalonFX {
    protected double mLastSet = Double.NaN;
    protected ControlMode mLastControlMode = null;

    public LazyTalonFX(int deviceNumber,String canbus) {
        super(deviceNumber, canbus);
    }

    public double getLastSet() {
        return mLastSet;
    }

    @Override
    public void set(double value) {
        if (value != mLastSet) {
            mLastSet = value;
            super.set(value);
        }
    }
}