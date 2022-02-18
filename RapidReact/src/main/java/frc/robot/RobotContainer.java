package frc.robot;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.XboxController.Button;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.subsystems.FeederSubsystem;
import frc.robot.subsystems.FeederSubsystem.FeedMode;

import static frc.robot.Constants.*;

/**
 * This class is where the bulk of the robot should be declared.
 */
public class RobotContainer {

  private Command autoCommand = null;
  private final XboxController driveController = new XboxController(DRIVE_CONTROLLER_PORT);
  @SuppressWarnings("unused")
  private final XboxController operatorController = new XboxController(OPERATOR_CONTROLLER_PORT);

  // Subsystems
  FeederSubsystem feeder;

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

    feeder = new FeederSubsystem(FEEDER_MOTOR_CAN_ID, FEEDER_ENTRY_SENSOR_DIO, FEEDER_EXIT_SENSOR_DIO);
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

    // Button commands to help test the feeder subsystem.
    new JoystickButton(driveController, Button.kX.value)
        .whenPressed(new InstantCommand(() -> feeder.setFeedMode(FeedMode.STOPPED), feeder));
    new JoystickButton(driveController, Button.kY.value)
        .whenPressed(new InstantCommand(() -> feeder.setFeedMode(FeedMode.CONTINUOUS), feeder));
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
}
