package frc.robot.constants;

import frc.robot.common.CanPort;

public final class Constants {
    
    public static final String CANIVORE_NAME = "1764 canivore";

    // public static final CanPort BACK_INTAKE_MOTOR =     new CanPort(21);
    // public static final CanPort SIDE_INTAKE_MOTOR =     new CanPort(22);
    // public static final CanPort INTAKE_OPENER_MOTOR =   new CanPort(23);



    public static final CanPort PIVOTY_MOTOR =          new CanPort(36, CANIVORE_NAME);
    public static final CanPort PIVOTY_MOTOR_2 =        new CanPort(20, CANIVORE_NAME);

    public static final CanPort ELEVATOR_MOTOR =        new CanPort(32, CANIVORE_NAME);
    public static final CanPort ELAVATOR_MOTOR_2 =      new CanPort(33, CANIVORE_NAME);

    public static final CanPort WRIST_MOTOR =           new CanPort(23);
    public static final CanPort INTAKE_MOTOR =          new CanPort(22); // may need to be swapped


    // public static final int MIN_EXTEND_BREAK_BEAM =  1;
    // public static final int MAX_EXTEND_BREAK_BEAM =  3;
    // public static final int MID_EXTEND_BREAK_BEAM =  2;
    public static final int ELEVATOR_LIMIT_SWITCH = 3;//TO DO: FIND PORT

    public static final int ELEVATOR_LIMIT_SWITCH2 = 4;//TO DO: FIND PORT

    public static final int PIVOTY_BREAK_BEAM =      9;

    public static final int INTAKE_BREAK_BEAM1 = 1;//TO DO: FIND PORT
    public static final int INTAKE_BREAK_BEAM2 = 2;

    public static final int BLINKIN_SPARK =          9;
    public static final int PRESSURE_SENSOR_PORT =   0;

    public static final int COLOR_SENSOR_2 =         5;
    public static final int COLOR_SENSOR_1 =         7;




    public static final int WRIST_ANGLE_ENCODER = 1;
    public static final int ARM_ANGLE_ENCODER = 0;

    public static final int DROP_INTAKE_SOLENOID_CHANNEL = 1;
    public static final int RAISE_INTAKE_SOLENOID_CHANNEL = 0;


    
   




    // unused

        /**
     * The left-to-right distance between the drivetrain wheels
     *    //  public static final double DRIVETRAIN_TRACKWIDTH_METERS = 1.0; 
     * Should be measured from center to center.
     */
    /**
     * The front-to-back distance between the drivetrain wheels.
          //  public static final double DRIVETRAIN_WHEELBASE_METERS = 1.0; 
     * Should be measured from center to center.
     */


    // public static final int FRONT_LEFT_MODULE_DRIVE_MOTOR =   (6);
    // public static final int FRONT_RIGHT_MODULE_DRIVE_MOTOR =   (7);
    // public static final int BACK_LEFT_MODULE_DRIVE_MOTOR =   (9);
    // public static final int BACK_RIGHT_MODULE_DRIVE_MOTOR =   (8);

    // public static final int FRONT_LEFT_MODULE_STEER_MOTOR =   (10);
    // public static final int FRONT_RIGHT_MODULE_STEER_MOTOR =   (11);
    // public static final int BACK_LEFT_MODULE_STEER_MOTOR =   (13);
    // public static final int BACK_RIGHT_MODULE_STEER_MOTOR =   (12);

    // public static final int FRONT_LEFT_MODULE_STEER_ENCODER =   (15);
    // public static final int FRONT_RIGHT_MODULE_STEER_ENCODER =   (16);
    // public static final int BACK_LEFT_MODULE_STEER_ENCODER =   (18);
    // public static final int BACK_RIGHT_MODULE_STEER_ENCODER =   (17);

    // In degrees
    // increasing turns clockwise
    // public static final double FRONT_LEFT_MODULE_STEER_OFFSET = Math.toRadians(-11.46);
    // public static final double FRONT_RIGHT_MODULE_STEER_OFFSET = Math.toRadians(-300.42);
    // public static final double BACK_LEFT_MODULE_STEER_OFFSET = Math.toRadians(-194.77);
    // public static final double BACK_RIGHT_MODULE_STEER_OFFSET = Math.toRadians(-332.06);


}