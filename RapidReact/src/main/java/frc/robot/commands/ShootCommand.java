// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.FeederSubsystem;
import frc.robot.subsystems.LimelightSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.FeederSubsystem.FeedMode;

public class ShootCommand extends CommandBase {
  //put feeder in preshoot mode check isIdle
  /** Creates a new ShootCommand. */
  private ShooterSubsystem shooter;
  private FeederSubsystem feeder;
  private LimelightSubsystem limelight;
  private BooleanSupplier trigger;


  public ShootCommand(ShooterSubsystem shooter, FeederSubsystem feeder, LimelightSubsystem limelight, BooleanSupplier trigger) {
    this.shooter = shooter;
    this.feeder = feeder;
    this.limelight = limelight;
    this.trigger = trigger;
    
    addRequirements(shooter, feeder,limelight);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    feeder.setFeedMode(FeedMode.PRESHOOT);
    limelight.setLEDMode(limelight.LED_ON);
    // shooter.start();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() { 
     shooter.setRange(limelight.getDistance());

    //  if(shooter.readyToShoot() && feeder.isIdle()){
      if (trigger.getAsBoolean()){
        feeder.setFeedMode(FeedMode.CONTINUOUS);
      }else{
       feeder.setFeedMode(FeedMode.STOPPED);
     }
    // }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    feeder.setFeedMode(FeedMode.STOPPED);
    limelight.setLEDMode(limelight.LED_OFF);
    shooter.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
