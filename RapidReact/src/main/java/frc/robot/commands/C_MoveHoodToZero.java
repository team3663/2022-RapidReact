// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ShooterSubsystem;

public class C_MoveHoodToZero extends CommandBase {
  private ShooterSubsystem shooter;
  /** Creates a new C_MoveHoodToZero. */
  public C_MoveHoodToZero(ShooterSubsystem shooter) {
    this.shooter = shooter;
    addRequirements(shooter);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    System.out.print(shooter.getHoodLimitswitch().get());
    if(!shooter.getHoodLimitswitch().get()){
      shooter.lowerHood();
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    shooter.setAngle(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return shooter.getHoodLimitswitch().get();
  }
}
