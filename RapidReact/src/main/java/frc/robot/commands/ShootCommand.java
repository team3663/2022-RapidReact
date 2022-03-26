package frc.robot.commands;

import java.util.function.BooleanSupplier;
import java.util.function.Consumer;
import java.util.function.DoubleSupplier;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.FeederSubsystem;
import frc.robot.subsystems.LimelightSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.FeederSubsystem.FeedMode;

public class ShootCommand extends CommandBase {

    private ShooterSubsystem shooter;
    private FeederSubsystem feeder;
    private LimelightSubsystem limelight;
    private DrivetrainSubsystem drivetrain;

    private Consumer<Boolean> shootReadyNotifier;
    private BooleanSupplier trigger;
    private DoubleSupplier translationXSupplier;
    private DoubleSupplier translationYSupplier;

    private PIDController tController = new PIDController(0.12, 0, 0);

    private double currentRange;
    private boolean stagingCargo;

    private boolean fixedRange;
    private boolean auto;
    
    private Timer timer = new Timer();

    // Fixed range version, take the range to target as a parameter
    public ShootCommand(ShooterSubsystem shooter, FeederSubsystem feeder, DrivetrainSubsystem drivetrain, LimelightSubsystem limelight,
                        Consumer<Boolean> shootReadyNotifier, BooleanSupplier trigger,
                        DoubleSupplier translationXSupplier, DoubleSupplier translationYSupplier,
                        double range) {
        this.shooter = shooter;
        this.feeder = feeder;
        this.limelight = limelight;
        this.drivetrain = drivetrain;

        this.shootReadyNotifier = shootReadyNotifier;
        this.trigger = trigger;

        this.translationXSupplier = translationXSupplier;
        this.translationYSupplier = translationYSupplier;
        
        this.tController.setSetpoint(0);
        this.tController.setTolerance(3);

        this.currentRange = range;
        this.fixedRange = true;
        this.auto = false;

        addRequirements(shooter, feeder, drivetrain, limelight);
    }

    // Variable range version, takes a limelight object that is used to determine
    // the range
    public ShootCommand(ShooterSubsystem shooter, FeederSubsystem feeder, DrivetrainSubsystem drivetrain, LimelightSubsystem limelight,
                        Consumer<Boolean> shootReadyNotifier, BooleanSupplier trigger,
                        DoubleSupplier translationXSupplier, DoubleSupplier translationYSupplier) {
        this(shooter, feeder, drivetrain, limelight, shootReadyNotifier, trigger, translationXSupplier, translationYSupplier, 0);

        this.fixedRange = false;
        this.auto = false;
    }

    public ShootCommand(ShooterSubsystem shooter, FeederSubsystem feeder, DrivetrainSubsystem drivetrain, LimelightSubsystem limelight) {
        this.shooter = shooter;
        this.feeder = feeder;
        this.drivetrain = drivetrain;
        this.limelight = limelight;

        this.shootReadyNotifier = null;
        this.trigger = () -> true;

        this.translationXSupplier = () -> 0;
        this.translationYSupplier = () -> 0;
        
        this.tController.setSetpoint(0);

        this.currentRange = 0;
        this.fixedRange = false;
        this.auto = true;
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

        timer.reset();
        
        if (!auto) {
            drivetrain.invertRotation();
        }
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {

        // align with hub
        double currentOffset = limelight.getXOffset();
        double speed = tController.calculate(currentOffset);
        drivetrain.drive(ChassisSpeeds.fromFieldRelativeSpeeds(translationXSupplier.getAsDouble(),
                                                            translationYSupplier.getAsDouble(),
                                                            speed, drivetrain.getPose().getRotation()));
        
        if (tController.atSetpoint()) {
            shooter.aligned = true;
        }

        // If we have a limelight then use it to update the current range to target
        if (!fixedRange) {
            currentRange = limelight.getDistance();
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

        // Call our shoot ready notifier to let it know whether or not the shooter subsystem is ready to fire.
        if (!auto) {
            shootReadyNotifier.accept(shooter.ready());
        }

        // We only get here if cargo staging has completed.
        // Use the state of the trigger to decided whether to run or stop the feeder.
        if (shooter.ready() && tController.atSetpoint()) { 
            if (auto) {
                feeder.setFeedMode(FeedMode.CONTINUOUS);
            }
            else if (trigger.getAsBoolean()) {
                feeder.setFeedMode(FeedMode.CONTINUOUS);
            }
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

        if (!auto) {
            shootReadyNotifier.accept(false);
        }

        if (!fixedRange) {
            limelight.setLEDMode(limelight.LED_OFF);
        }

    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        if (auto && timer.hasElapsed(2)) {
            return true;
        }
        return false;
    }
}