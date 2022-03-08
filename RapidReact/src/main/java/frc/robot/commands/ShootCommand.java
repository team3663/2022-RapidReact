package frc.robot.commands;

import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.FeederSubsystem;
import frc.robot.subsystems.LimelightSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.FeederSubsystem.FeedMode;

public class ShootCommand extends CommandBase {

    private ShooterSubsystem shooter;
    private FeederSubsystem feeder;
    private LimelightSubsystem limelight = null;
    private BooleanSupplier trigger;
    private double currentRange;
    private boolean stagingCargo;

    // Fixed range version, take the range to target as a parameter
    public ShootCommand(ShooterSubsystem shooter, FeederSubsystem feeder, BooleanSupplier trigger, double range) {
        this.shooter = shooter;
        this.feeder = feeder;
        this.trigger = trigger;
        this.currentRange = range;

        addRequirements(shooter, feeder);
    }

    // Variable range version, takes a limelight object that is used to determine
    // the range
    public ShootCommand(ShooterSubsystem shooter, FeederSubsystem feeder, BooleanSupplier trigger,
            LimelightSubsystem limelight) {
        this(shooter, feeder, trigger, 0);

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
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        // If we have a limelight then use it to update the current range to target
        if (limelight != null) {
            currentRange = limelight.getAverageDistance();
            shooter.setRange(currentRange);
        }

        // We bail out here if we are staging cargo and the feeder has not stopped yet.
        if (stagingCargo) {
            if (feeder.isIdle()) {
                stagingCargo = false;
            } else {
                return;
            }
        }

        // We only get here if cargo staging has completed.
        // Use the state of the trigger to decided whether to run or stop the feeder.
        feeder.setFeedMode(trigger.getAsBoolean() ? FeedMode.CONTINUOUS : FeedMode.STOPPED);
      
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
        return false;
    }
}
