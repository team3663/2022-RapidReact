package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.FeederSubsystem;
import frc.robot.subsystems.LimelightSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.FeederSubsystem.FeedMode;

public class AutoShootCommand extends CommandBase {
  private ShooterSubsystem shooter;
  private FeederSubsystem feeder;
  private LimelightSubsystem limelight;

  private boolean finished;

  public AutoShootCommand(ShooterSubsystem shooter, FeederSubsystem feeder, LimelightSubsystem limelight) {
    this.shooter = shooter;
    this.feeder = feeder;
    this.limelight = limelight;
    
    addRequirements(shooter, feeder,limelight);
  }

  @Override
  public void initialize() {
    feeder.setFeedMode(FeedMode.PRESHOOT);
    limelight.setLEDMode(limelight.LED_ON);
    shooter.start();
  }

  @Override
  public void execute() { 
    shooter.setRange(limelight.getDistance());
    
    if (shooter.readyToShoot()) {
      feeder.setFeedMode(FeedMode.SHOOT_ONE);
      finished = true;
    }
  }

  @Override
  public void end(boolean interrupted) {
    feeder.setFeedMode(FeedMode.STOPPED);
    limelight.setLEDMode(limelight.LED_OFF);
    shooter.stop();
  }

  @Override
  public boolean isFinished() {
    return finished;
  }
}
