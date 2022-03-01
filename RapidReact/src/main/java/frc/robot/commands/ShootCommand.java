// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.drivers.Limelight;
import frc.robot.subsystems.FeederSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.FeederSubsystem.FeedMode;

public class ShootCommand extends CommandBase {
  //put feeder in preshoot mode check isIdle
  /** Creates a new ShootCommand. */
  private ShooterSubsystem shooterSubsystem;
  private FeederSubsystem feederSubsystem;
  private Limelight limelight;

  public ShootCommand(ShooterSubsystem shooterSubsystem, FeederSubsystem feederSubsystem, Limelight limelight) {
    this.shooterSubsystem = shooterSubsystem;
    this.feederSubsystem = feederSubsystem;
    this.limelight = limelight;
    
    addRequirements(shooterSubsystem, feederSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    feederSubsystem.setFeedMode(FeedMode.PRESHOOT);
    limelight.setLEDMode(limelight.LED_ON);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    limelight.updateTelemetry();
    System.out.println(limelight.getDistance());
    // shooterSubsystem.setRange(limelight.getDistance());
    // if(shooterSubsystem.readyToShoot() && feederSubsystem.isIdle()){
    //   feederSubsystem.setFeedMode(FeedMode.CONTINUOUS);
    // }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    // feederSubsystem.setFeedMode(FeedMode.STOPPED);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
