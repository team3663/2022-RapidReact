// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.drivers.Limelight;
import frc.robot.subsystems.ShooterSubsystem;

import static frc.robot.Constants.*;

/**
 * This class is where the bulk of the robot should be declared.
 */
public class RobotContainer {

  private Command autoCommand = null;
  @SuppressWarnings("unused")
  private final XboxController driveController = new XboxController(Constants.DRIVE_CONTROLLER_ID);
  @SuppressWarnings("unused")
  private final XboxController operatorController = new XboxController(Constants.OPERATOR_CONTROLLER_ID);

  private static final Limelight vision = new Limelight();

  // Subsystems
  

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
    @SuppressWarnings("unused")
    final ShooterSubsystem shooter = new ShooterSubsystem(SHOOTER_MOTOR_1_CAN_ID, SHOOTER_MOTOR_2_CAN_ID, HOOD_MOTOR_1_CAN_ID, HOOD_LIMITSWITCH_CAN_ID);
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
