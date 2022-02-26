// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.FeederSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.FeederSubsystem.FeedMode;

public class C_LobShot extends CommandBase {
  private FeederSubsystem feederSubsystem;
  private ShooterSubsystem shooterSubsystem;

  private int lobRPM = 1000;
  /** Creates a new C_LobShot. */
  public C_LobShot(ShooterSubsystem shooterSubsystem, FeederSubsystem feederSubsystem) {
    this.shooterSubsystem = shooterSubsystem;
    this.feederSubsystem = feederSubsystem;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(shooterSubsystem);
    addRequirements(feederSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    shooterSubsystem.setAngle(shooterSubsystem.getMinAngle());
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(shooterSubsystem.getAngle() == shooterSubsystem.getMinAngle()){
      shooterSubsystem.setSpeed(lobRPM);
      // add code to check if shooter is at correct speed and then feed balls into shooter
      //if(shooterSubsystem.g)
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    shooterSubsystem.setSpeed(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
