package frc.robot.commands;

import java.util.function.BooleanSupplier;
import java.util.function.Consumer;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.FeederSubsystem;
import frc.robot.subsystems.LimelightSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.FeederSubsystem.FeedMode;

public class ShootCommand extends CommandBase {

    private ShooterSubsystem shooter;
    private FeederSubsystem feeder;
    private LimelightSubsystem limelight;

    private Consumer<Boolean> shootReadyNotifier;
    private BooleanSupplier trigger;

    private double currentRange;
    private boolean stagingCargo;

    private boolean fixedRange;

    // Fixed range version, take the range to target as a parameter
    public ShootCommand(ShooterSubsystem shooter, FeederSubsystem feeder, LimelightSubsystem limelight,
                        Consumer<Boolean> shootReadyNotifier, BooleanSupplier trigger,
                        double range) {
        this.shooter = shooter;
        this.feeder = feeder;
        this.limelight = limelight;

        this.shootReadyNotifier = shootReadyNotifier;
        this.trigger = trigger;

        this.currentRange = range;
        this.fixedRange = true;

        addRequirements(shooter, feeder, limelight);
    }

    // Variable range version, takes a limelight object that is used to determine
    // the range
    public ShootCommand(ShooterSubsystem shooter, FeederSubsystem feeder, LimelightSubsystem limelight,
                        Consumer<Boolean> shootReadyNotifier, BooleanSupplier trigger) {
        this(shooter, feeder, limelight, shootReadyNotifier, trigger, 0);

        this.fixedRange = false;
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        shooter.shoot();
        feeder.setFeedMode(FeedMode.PRESHOOT);
        
        stagingCargo = true;
        shootReadyNotifier.accept(false);

        if (!fixedRange) {
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
        if (!fixedRange) {
            currentRange = limelight.getDistance();
            shooter.setRange(currentRange);
        }

        // shuffleboard aligned
        shooter.aligned = limelight.aligned();

        // We bail out here if we are staging cargo and the feeder has not stopped yet.
        if (stagingCargo) {
            if (feeder.isIdle()) {
                stagingCargo = false;
            } else {
                return;
            }
        }

        // Call our shoot ready notifier to let it know whether or not the shooter subsystem is ready to fire.
        shootReadyNotifier.accept(shooter.ready());

        // We only get here if cargo staging has completed.
        // Use the state of the trigger to decided whether to run or stop the feeder.
        if (shooter.ready() && trigger.getAsBoolean() && limelight.aligned()) { 
            feeder.setFeedMode(FeedMode.CONTINUOUS);
        }
        else {
            feeder.setFeedMode(FeedMode.STOPPED);
        }
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        feeder.setFeedMode(FeedMode.STOPPED);
        shooter.idle();

        shootReadyNotifier.accept(false);

        if (!fixedRange) {
            limelight.setLEDMode(limelight.LED_OFF);
        }

    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false;
    }
}
