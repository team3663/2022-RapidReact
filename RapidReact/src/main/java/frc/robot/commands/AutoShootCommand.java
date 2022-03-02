package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.drivers.Limelight;
import frc.robot.subsystems.FeederSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.FeederSubsystem.FeedMode;

public class AutoShootCommand extends CommandBase {
  private ShooterSubsystem shooter;
  private FeederSubsystem feeder;
  private Limelight limelight;

  private boolean finished = false;
  
  public AutoShootCommand(ShooterSubsystem shooter, FeederSubsystem feeder, Limelight limelight) {
    this.shooter = shooter;
    this.limelight = limelight;
    addRequirements(shooter);
  }

  @Override
  public void initialize() {
    feeder.setFeedMode(FeedMode.PRESHOOT);
    limelight.setLEDMode(limelight.LED_ON);
  }

  @Override
  public void execute() {
    double distance = limelight.getDistance();
    shooter.setRange(distance);
    if (shooter.readyToShoot() && feeder.isIdle()){
      feeder.setFeedMode(FeedMode.CONTINUOUS);
      shooter.start();
      finished = true;
    }
  }

  @Override
  public void end(boolean interrupted) {
    shooter.stop();
    feeder.setFeedMode(FeedMode.STOPPED);
    limelight.setLEDMode(limelight.LED_OFF);
  }

  @Override
  public boolean isFinished() {
    return finished;
  }
}


