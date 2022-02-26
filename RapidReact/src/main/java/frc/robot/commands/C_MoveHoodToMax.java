// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ShooterSubsystem;

public class C_MoveHoodToMax extends CommandBase {
  ShooterSubsystem shooterSubsystem;
  private String mode = "";
  /** Creates a new C_MoveHoodToMax. */
  public C_MoveHoodToMax(ShooterSubsystem shooterSubsystem) {
    this.shooterSubsystem = shooterSubsystem;
    addRequirements(shooterSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(shooterSubsystem.getAngle() < 285){
      System.out.println(shooterSubsystem.getAngle());
      shooterSubsystem.raiseAngle(mode);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    //
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return shooterSubsystem.getAngle() >= 285;
  }
}
