package frc.robot.constants;

public final class CommandConstants {
    // for talon fx, 0.916 inches per rotation\

    //reef levels elevator
    public static final int ELEVATOR_LEVEL_ONE = 0; //TODO: FIX
    public static final int ELEVATOR_LEVEL_TWO = 24; //23.833 repeating
    public static final int ELEVATOR_LEVEL_THREE = 37; //36.66 repeating
    public static final int ELEVATOR_LEVEL_FOUR = 49; //48.5833 repeating

    public static final int ELEVATOR_STOP_SAFE = 5; //TODO: FIX 

    //reef levels wrist angles
    //TODO: fix, currently in degrees
    public static final int WRIST_LEVEL_ONE = 0; //TODO - level one
    public static final int WRIST_LEVEL_TWO = 140; 
    public static final int WRIST_LEVEL_THREE = 140;
    public static final int WRIST_LEVEL_FOUR = 170;

    public static final int WRIST_GRAB_ANGLE = 0; //TODO
    public static final int WRIST_PROCESSOR = 0; //TODO: fix

    public static final int WRIST_DOWN = 0; //TODO: fix
    public static final int WRIST_HALF = 90; //TODO: fix
    public static final int WRIST_UP = 180; //TODO: fix
    public static final int WRIST_ALGAE = 135; //TODO: fix

    //speed
    public static final double INTAKE_CORAL_PICKUP_SPEED = 0.6;
    public static final double INTAKE_CORAL_OUT_SPEED = 0.6;

    public static final int INTAKE_BOUNCE_ALGAE_OUT = 0;
    public static final int INTAKE_GRAB_ALGAE = 0;

    public static final int ELEVATOR_DOWN = 0;
    public static final double CLIMBER_SPEED = 0.5;

    //climber
    public static final int CLIMBER_DOWN = 0; //TODO: FIX
    
    
    //keys
    public static final String INTAKE_KEY = "WristEncoderPosition";
    public static final String ELEVATOR_KEY = "ElevatorPosition";
    public static final String ROLLER_KEY = "RollerSpeed";



}