package frc.robot;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.XboxController.Button;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

import frc.robot.drivers.Limelight;
import frc.robot.subsystems.FeederSubsystem;
import frc.robot.subsystems.FeederSubsystem.FeedMode;
import frc.robot.utils.Ranger;
import frc.robot.utils.SimpleRanger;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import static frc.robot.Constants.*;

/**
 * This class is where the bulk of the robot should be declared.
 */
public class RobotContainer {

  private Command autoCommand = null;
  private final XboxController driveController = new XboxController(Constants.DRIVE_CONTROLLER_PORT);
  @SuppressWarnings("unused")
  private final XboxController operatorController = new XboxController(Constants.OPERATOR_CONTROLLER_PORT);

  private static final Limelight vision = new Limelight();
  private final Ranger ranger = new SimpleRanger();

  // Subsystems
  private FeederSubsystem feeder;
  private ShooterSubsystem shooter;
  private IntakeSubsystem intake;

  // Commands

  public RobotContainer() {

    createSubsystems(); // Create our subsystems.
    createCommands(); // Create our commands
    configureButtonBindings(); // Setup our button bindings
  }

  /**
   * Create all of our robot's subsystem objects here.
   */
  void createSubsystems() {

    intake = new IntakeSubsystem(INTAKE_MOTOR_ID, SOLONOID_INWARD_CAN_ID, SOLONOID_OUTWARD_CAN_ID);

    feeder = new FeederSubsystem(FEEDER_MOTOR_CAN_ID, FEEDER_ENTRY_SENSOR_DIO, FEEDER_EXIT_SENSOR_DIO);

    shooter = new ShooterSubsystem(SHOOTER_MOTOR_1_CAN_ID, SHOOTER_MOTOR_2_CAN_ID, HOOD_MOTOR_CAN_ID,
        HOOD_LIMITSWITCH_DIO, ranger);
  }

  /**
   * Create all of our robot's command objects here.
   */
  void createCommands() {

  }

  /**
   * Use this method to define your button->command mappings.
   */
  private void configureButtonBindings() {

    // Button commands to test intake subsystem
    new JoystickButton(driveController, Button.kA.value).whenHeld(new SequentialCommandGroup(
        new InstantCommand(() -> intake.extend(), intake), new InstantCommand(() -> intake.start(), intake)));
    new JoystickButton(driveController, Button.kA.value).whenReleased(new SequentialCommandGroup(
        new InstantCommand(() -> intake.retract(), intake), new InstantCommand(() -> intake.stop(), intake)));

    // Button commands to help test the feeder subsystem.
    new JoystickButton(driveController, Button.kX.value)
        .whenPressed(new InstantCommand(() -> feeder.setFeedMode(FeedMode.STOPPED), feeder));
    new JoystickButton(driveController, Button.kY.value)
        .whenPressed(new InstantCommand(() -> feeder.setFeedMode(FeedMode.CONTINUOUS), feeder));

    // Button commands to test shooter subsystem.
    new JoystickButton(driveController, Button.kStart.value)
        .whenPressed(new InstantCommand(() -> shooter.start(), shooter));
    new JoystickButton(driveController, Button.kBack.value)
        .whenPressed(new InstantCommand(() -> shooter.stop(), shooter));
    new JoystickButton(driveController, Button.kLeftBumper.value)
        .whenPressed(new InstantCommand(() -> shooter.decreasePower(), shooter));
    new JoystickButton(driveController, Button.kRightBumper.value)
        .whenPressed(new InstantCommand(() -> shooter.increasePower(), shooter));

  }

  /**
   * The main {@link Robot} class calls this to get the command to run during
   * autonomous.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return autoCommand;
  }

  public static Limelight getVision() {
    return vision;
  }
}
