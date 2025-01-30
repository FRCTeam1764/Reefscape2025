package frc.robot.constants;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import frc.robot.common.CanPort;

public final class Constants {
    
    public static final String CANIVORE_NAME = "1764 canivore";
    public static final int PRESSURE_SENSOR_PORT =   0;

//TODO: FIND PORTS
    public static final CanPort ELEVATOR_MOTOR1 =     new CanPort(0);
    public static final CanPort ELEVATOR_MOTOR2 =     new CanPort(0); 

    public static final CanPort CLIMBER_MOTOR =     new CanPort(0);

    public static final CanPort INTAKE_MOTOR =     new CanPort(0);
    public static final CanPort WRIST_MOTOR1 =     new CanPort(0);

    public static final int BLINKIN_SPARKPORT = 0;
    public static final int INTAKE_BREAK_BEAM = 0;
    public static final int ELEVATOR_SWITCH1 = 0; 

    

    
}

