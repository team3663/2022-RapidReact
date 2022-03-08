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

  // Swerve drivetrain physical constants
  public static final double DRIVETRAIN_TRACKWIDTH_METERS = .6604; // 0.5461 for both
  public static final double DRIVETRAIN_WHEELBASE_METERS = 0.6604;
  public static final double DRIVE_TRAIN_WHEEL_DIAMETER_METERS = 0.106325;
  public static final double FRONT_LEFT_MODULE_STEER_OFFSET = -Math.toRadians(64.764404296875);//297.7734375
  public static final double FRONT_RIGHT_MODULE_STEER_OFFSET = -Math.toRadians(261.749267578125);//23.291015625
  public static final double BACK_LEFT_MODULE_STEER_OFFSET = -Math.toRadians(11.865234375);//95.27343750000001
  public static final double BACK_RIGHT_MODULE_STEER_OFFSET = -Math.toRadians(94.482421875);//310.60546875

  // Xbox Controllers Port Indexes
  public static final int DRIVE_CONTROLLER_PORT = 0;
  public static final int OPERATOR_CONTROLLER_PORT = 1;
  public static final int TEST_CONTROLLER_PORT = 2;

  /*
   * Front Left to Front Right
   * Back Left to Front Left
   * Front Right to Back Right
   * Back Right to Back Left
   */

  // CAN IDs
  public static final int FRONT_LEFT_MODULE_DRIVE_MOTOR = 1; // 1
  public static final int FRONT_RIGHT_MODULE_DRIVE_MOTOR = 2; // 2
  public static final int BACK_LEFT_MODULE_DRIVE_MOTOR = 3; // 3
  public static final int BACK_RIGHT_MODULE_DRIVE_MOTOR = 4; // 4

  public static final int FRONT_LEFT_MODULE_STEER_MOTOR = 5; // 5
  public static final int FRONT_RIGHT_MODULE_STEER_MOTOR = 6; // 6
  public static final int BACK_LEFT_MODULE_STEER_MOTOR = 7; // 7
  public static final int BACK_RIGHT_MODULE_STEER_MOTOR = 8; // 8

  public static final int FRONT_LEFT_MODULE_STEER_ENCODER = 9;// 9;
  public static final int FRONT_RIGHT_MODULE_STEER_ENCODER = 10;// 10;
  public static final int BACK_LEFT_MODULE_STEER_ENCODER = 11;// 11;
  public static final int BACK_RIGHT_MODULE_STEER_ENCODER = 12;// 12

  public static final int DRIVETRAIN_PIGEON_ID = 13;

  public static final int INTAKE_MOTOR_CAN_ID = 15;

  public static final int FEEDER_MOTOR_CAN_ID = 20;

  public static final int SHOOTER_MOTOR_1_CAN_ID = 25;
  public static final int SHOOTER_MOTOR_2_CAN_ID = 26;
  public static final int HOOD_MOTOR_CAN_ID = 27;

  // Solenoid Channels
  public static final int ARM_RETRACT_SOLENOID_CHAN = 0;
  public static final int ARM_EXTEND_SOLENOID_CHAN = 1;
  public static final int BOOM_RETRACT_SOLENOID_CHAN = 2;
  public static final int BOOM_EXTEND_SOLENOID_CHAN = 3;

  // Digital IO ports
  public static final int FEEDER_ENTRY_SENSOR_DIO = 1;
  public static final int FEEDER_EXIT_SENSOR_DIO = 2;
  public static final int HOOD_LIMITSWITCH_DIO = 3;

  // Analog IO ports

  // SPI addresses

}
