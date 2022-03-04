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

  public AutoShootCommand(ShooterSubsystem shooter, FeederSubsystem feeder, LimelightSubsystem limelight) {
    this.shooter = shooter;
    this.feeder = feeder;
    this.limelight = limelight;
    
    addRequirements(shooter, feeder, limelight);
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
    
    if (shooter.readyToShoot()) { // TODO change this to timer & continuous
      feeder.setFeedMode(FeedMode.SHOOT_ONE);
    }
  }

  @Override
  public void end(boolean interrupted) {
    limelight.setLEDMode(limelight.LED_OFF);
    shooter.stop();
  }

  @Override
  public boolean isFinished() {
    return (feeder.isIdle());
  }
}
