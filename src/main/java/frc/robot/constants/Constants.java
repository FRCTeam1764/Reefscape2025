package frc.robot.constants;

import frc.robot.common.CanPort;

public final class Constants {
    
    public static final String CANIVORE_NAME = "1764 canivore";
    public static final int PRESSURE_SENSOR_PORT =   0;

//TODO: FIND PORTS
    public static final CanPort ELEVATOR_MOTOR1 =     new CanPort(61); // links
    public static final CanPort ELEVATOR_MOTOR2 =     new CanPort(62); // rechts

    public static final CanPort CLIMBER_MOTOR =     new CanPort(60);

    public static final CanPort INTAKE_MOTOR =     new CanPort(0);
    public static final CanPort WRIST_MOTOR1 =     new CanPort(54);
    public static final CanPort WRIST_CAN_PORT =   new CanPort(0);

    public static final CanPort INTAKE_CANCODER = new CanPort(0);

    public static final int BLINKIN_SPARKPORT = 0;
    public static final int INTAKE_LIMITSWITCH = 0;
    public static final int INTAKE_BREAKBEAM = 0;

    public static final int ELEVATOR_SWITCHUP1 = 0; 
    public static final int ELEVATOR_SWITCHUP2 = 0;

    public static final int ELEVATOR_SWITCHDOWN1 = 0;
    public static final int ELEVATOR_SWITCHDOWN2 = 0;
    

    
}

