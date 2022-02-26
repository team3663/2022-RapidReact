// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.IntakeSubsystem;

public class C_Intake extends CommandBase {
  private final IntakeSubsystem intake;

  /** Creates a new IntakeCommand. */
  public C_Intake(IntakeSubsystem subsystem) {
    intake = subsystem;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(intake);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    System.out.println("IntakeCommand initialized");
  }

  /**
   * lower mechanism and spin ball IN
   */
  public void intakeBall() {
    System.out.println("<<< intakeBall() >>>");
  }

  /**
   * lower mechanism and spin ball OUT
   */
  public void ejectBall() {
    System.out.println("<<< ejectBall() >>>");
  }

  /**
   * lower mechanism BUT motor not spinning
   */
  public void lowerIntake() {
    System.out.println("<<< lowerIntake() >>>");
  }

  /**
   * raise mechanism
   */
  public void raiseIntake() {
    System.out.println("<<< raiseIntake() >>>");
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    System.out.println("IntakeCommand ended");
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
