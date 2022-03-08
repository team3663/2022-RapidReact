package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.FeederSubsystem;
import frc.robot.subsystems.LimelightSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.FeederSubsystem.FeedMode;

public class AutoShootCommand extends CommandBase {

    private ShooterSubsystem shooter;
    private FeederSubsystem feeder;
    private LimelightSubsystem limelight = null;
    private double currentRange;
    private boolean stagingCargo;
    private Timer timer = new Timer();

    // Fixed range version, take the range to target as a parameter
    public AutoShootCommand(ShooterSubsystem shooter, FeederSubsystem feeder, double range) {
        this.shooter = shooter;
        this.feeder = feeder;
        this.currentRange = range;

        addRequirements(shooter, feeder);
    }

    // Variable range version, takes a limelight object that is used to determine
    // the range
    public AutoShootCommand(ShooterSubsystem shooter, FeederSubsystem feeder, LimelightSubsystem limelight) {
        this(shooter, feeder, 0);

        this.limelight = limelight;
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        feeder.setFeedMode(FeedMode.PRESHOOT);
        stagingCargo = true;

        if (limelight != null) {
            limelight.setLEDMode(limelight.LED_ON);
        }

        // Initialze the shooter range, if we have a limelight it will get updated each
        // time through periodic.
        shooter.setRange(currentRange);
        
        timer.start();
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        // If we have a limelight then use it to update the current range to target
        if (limelight != null) {
            currentRange = limelight.getAverageDistance();
            shooter.setRange(currentRange);
        }

        // TODO: Temporary work around for bug in fixed range mode.
        shooter.setAngle(71); //70
        shooter.setSpeed(2350); // 2600 TODO tune

        // We bail out here if we are staging cargo and the feeder has not stopped yet.
        if (stagingCargo) {
            if (feeder.isIdle()) {
                stagingCargo = false;
            } else {
                return;
            }
        }

        // We only get here if cargo staging has completed.
        if (timer.hasElapsed(3.0)) { // TODO fix shooter.ready() && shooter.available()
            feeder.setFeedMode(FeedMode.CONTINUOUS);
        }
      
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        feeder.setFeedMode(FeedMode.STOPPED);
        shooter.stop();

        if (limelight != null) {
            limelight.setLEDMode(limelight.LED_OFF);
        }

    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return timer.hasElapsed(6.0); // TODO find a shorter & more precise time
    }
}
