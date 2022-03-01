package frc.robot;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.XboxController.Button;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;

import static frc.robot.Constants.*;

import frc.robot.subsystems.FeederSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

public class RobotContainer {

  private final XboxController controller = new XboxController(DRIVE_CONTROLLER_ID);

  // Subsystems
  private final ShooterSubsystem shooter;
  private final FeederSubsystem feeder;

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {

    shooter = new ShooterSubsystem(SHOOTER_MOTOR_1_CAN_ID, SHOOTER_MOTOR_1_CAN_ID, HOOD_MOTOR_CAN_ID, HOOD_LIMITSWITCH_DIO);

    feeder = new FeederSubsystem(FEEDER_MOTOR_CAN_ID, FEEDER_ENTRY_SENSOR_DIO, FEEDER_EXIT_SENSOR_DIO);

    // Configure the button bindings
    configureButtonBindings();
  }

  /**
   * Use this method to define your button->command mappings.
   */
  private void configureButtonBindings() {

    // Start and Back buttons start and stop the shooter motors
    new JoystickButton(controller, Button.kStart.value).whenPressed(new InstantCommand(() -> shooter.start(), shooter));
    new JoystickButton(controller, Button.kBack.value).whenPressed(new InstantCommand(() -> shooter.stop(), shooter));

    // POV left and right decrease and increase the motor speed
    new POVButton(controller, 90).whenPressed(new InstantCommand(() -> shooter.increaseSpeed(), shooter));
    new POVButton(controller, 270).whenPressed(new InstantCommand(() -> shooter.decreaseSpeed(), shooter));

    // The A button reads the updated PID values from shuffle board and updates the
    // motor controller with them.
    new JoystickButton(controller, Button.kA.value)
        .whenPressed(new InstantCommand(() -> shooter.updatePIDCoefficients(), shooter));

    // The Y button dumps the updated coefficients to the Rio log as Java declarations
    // that can be copied back into the source code.
    new JoystickButton(controller, Button.kY.value)
        .whenPressed(new InstantCommand(() -> shooter.dumpPIDCoefficients(), shooter));

    // POV up and down raise and lower hood.
    new POVButton(controller, 0).whenPressed(new InstantCommand(() -> shooter.raiseHood(), shooter));
    new POVButton(controller, 180).whenPressed(new InstantCommand(() -> shooter.lowerHood(), shooter));

    // Holding the left bumper runs the feeder to shoot cargo
    new JoystickButton(controller, Button.kLeftBumper.value).whenPressed(new InstantCommand(() -> feeder.start(), feeder));
    new JoystickButton(controller, Button.kLeftBumper.value).whenReleased(new InstantCommand(() -> feeder.stop(), feeder));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An ExampleCommand will run in autonomous
    return null;
  }
}