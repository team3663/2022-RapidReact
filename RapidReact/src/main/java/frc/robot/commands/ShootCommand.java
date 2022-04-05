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
    private BooleanSupplier shootTrigger;
    private BooleanSupplier forceShootTrigger;

    private String shootingPose;
    private boolean varyingRange;
    private boolean stagingCargo;

    // Fixed range version, take the range to target as a parameter
    public ShootCommand(ShooterSubsystem shooter, FeederSubsystem feeder, 
                        Consumer<Boolean> shootReadyNotifier, BooleanSupplier shootTrigger, BooleanSupplier forceShootTrigger,
                        String shootingPose) {
        this.shooter = shooter;
        this.feeder = feeder;

        this.shootReadyNotifier = shootReadyNotifier;
        this.shootTrigger = shootTrigger;
        this.forceShootTrigger = forceShootTrigger;

        this.shootingPose = shootingPose;
        this.varyingRange = false;

        addRequirements(shooter, feeder, limelight);
    }

    // Variable range version, takes a limelight object that is used to determine
    // the range
    public ShootCommand(ShooterSubsystem shooter, FeederSubsystem feeder, LimelightSubsystem limelight,
                        Consumer<Boolean> shootReadyNotifier, BooleanSupplier shootTrigger, BooleanSupplier forceShootTrigger) {
        this(shooter, feeder, shootReadyNotifier, shootTrigger, forceShootTrigger, "");

        this.limelight = limelight;
        this.varyingRange = true;
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        feeder.setFeedMode(FeedMode.PRESHOOT);
        
        stagingCargo = true;
        shootReadyNotifier.accept(false);

        if (varyingRange) {
            limelight.setLEDMode(limelight.LED_ON);
        }
        else {
            shooter.setRange(shootingPose);
        }
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {

        // If we have a limelight then use it to update the current range to target
        if (varyingRange) {
            shooter.setRange(limelight.getDistance());
        }

        // We bail out here if we are staging cargo and the feeder has not stopped yet.
        if (stagingCargo) {
            if (feeder.isIdle()) {
                stagingCargo = false;
            } else {
                return;
            }
        }

        // shooter checks
        boolean aligned = true;
        if (varyingRange) {
            aligned = limelight.aligned();
        }
        boolean atSpeed = shooter.ready();

        // Call our shoot ready notifier to let it know whether or not the shooter subsystem is ready to fire.
        shootReadyNotifier.accept(atSpeed);

        // shuffleboard aligned
        shooter.aligned = aligned;

        // We only get here if cargo staging has completed.
        // Use the state of the trigger to decided whether to run or stop the feeder.
        if (shootTrigger.getAsBoolean() && atSpeed && aligned) { 
            feeder.setFeedMode(FeedMode.CONTINUOUS);
        }
        else if (forceShootTrigger.getAsBoolean()) {
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

        shootReadyNotifier.accept(false);

        if (varyingRange) {
            limelight.setLEDMode(limelight.LED_OFF);
        }
    }
}
