package frc.robot;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean
 * constants. This class should not be used for any other purpose. All constants
 * should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {

  // Xbox Controllers Port Indexes
  public static final int DRIVE_CONTROLLER_PORT = 0;
  public static final int OPERATOR_CONTROLLER_PORT = 1;
  public static final int TEST_CONTROLLER_PORT = 2;

  // CAN IDs
  // CAN ID 1 - 8 are reserved for the swerve drivetrain motors
  // CAN ID 9-12 are reserved for the swerve angle encoders
  public static final int INTAKE_MOTOR_ID = 15;

  public static final int FEEDER_MOTOR_CAN_ID = 20;

  public static final int SHOOTER_MOTOR_1_CAN_ID = 25;
  public static final int SHOOTER_MOTOR_2_CAN_ID = 26;
  public static final int HOOD_MOTOR_CAN_ID = 27;

  // Solenoid Channels
  public static final int INTAKE_RETRACT_SOLENOID_CHAN = 0;
  public static final int INTAKE_EXTEND_SOLENOID_CHAN = 1;

  // Digital IO ports
  public static final int FEEDER_ENTRY_SENSOR_DIO = 1;
  public static final int FEEDER_EXIT_SENSOR_DIO = 2;
  public static final int HOOD_LIMITSWITCH_DIO = 3;

  // Analog IO ports

  // SPI addresses

}
