package frc.robot.constants;

import frc.robot.common.CanPort;

public final class Constants {
    
    public static final String CANIVORE_NAME = "1764 canivore";

//TODO: FIND PORTS
    public static final CanPort ELEVATOR_MOTOR1 =     new CanPort(61); // links
    public static final CanPort ELEVATOR_MOTOR2 =     new CanPort(62); // rechts

    //public static final CanPort CLIMBER_MOTOR =     new CanPort(60);

    public static final CanPort INTAKE_MOTOR =     new CanPort(13);
    public static final CanPort WRIST_MOTOR1 =     new CanPort(54);
   // public static final CanPort WRIST_CAN_PORT =   new CanPort(0);

    public static final CanPort INTAKE_CANCODER = new CanPort(0);

    public static final int BLINKIN_SPARKPORT = 2;
    public static final int INTAKE_LIMITSWITCH = 7;
    public static final int INTAKE_BREAKBEAM = 9;

    public static final int ELEVATOR_SWITCHUP1 = 0; 
    public static final int ELEVATOR_SWITCHUP2 = 2;

    public static final int ELEVATOR_SWITCHDOWN1 = 1;
    public static final int ELEVATOR_SWITCHDOWN2 = 5;
    

    
}

