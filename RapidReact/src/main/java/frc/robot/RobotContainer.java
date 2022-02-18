// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static frc.robot.Constants.DRIVE_CONTROLLER_PORT;
import static frc.robot.Constants.OPERATOR_CONTROLLER_PORT;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.XboxController.Button;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.subsystems.IntakeSubsystem;

/**
 * This class is where the bulk of the robot should be declared.
 */
public class RobotContainer {

  private Command autoCommand = null;
  @SuppressWarnings("unused")
  private final XboxController driveController = new XboxController(DRIVE_CONTROLLER_PORT);
  @SuppressWarnings("unused")
  private final XboxController operatorController = new XboxController(OPERATOR_CONTROLLER_PORT);

  // Subsystems
  private IntakeSubsystem intake;

  // Commands

  public RobotContainer() {

    createSubsystems();         // Create our subsystems.
    createCommands();           // Create our commands
    configureButtonBindings();  // Setup our button bindings
  }

  /**
   * Create all of our robot's subsystem objects here.
   */
  void createSubsystems() {
    final IntakeSubsystem intake = new IntakeSubsystem(Constants.INTAKE_MOTOR_ID, Constants.SOLONOID_INWARD_CAN_ID, Constants.SOLONOID_OUTWARD_CAN_ID);

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
      //new JoystickButton(driveController, Button.kA.value).whenPressed(new InstantCommand(() -> intake.extend(), intake));

      new JoystickButton(driveController, Button.kA.value).whenHeld(new SequentialCommandGroup(
        new InstantCommand(() -> intake.extend(), intake), new InstantCommand(() -> intake.start(), intake)));

      new JoystickButton(driveController, Button.kA.value).whenReleased(new SequentialCommandGroup(
        new InstantCommand(() -> intake.retract(), intake), new InstantCommand(() -> intake.stop(), intake)));



      //new JoystickButton(driveController, Button.kA.value).whenReleased(new InstantCommand(() -> intake.retract(), intake));   

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
