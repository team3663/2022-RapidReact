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
    private LimelightSubsystem limelight;
    private BooleanSupplier trigger;
    private boolean stagingCargo;

    public ShootCommand(ShooterSubsystem shooter, FeederSubsystem feeder, LimelightSubsystem limelight,
            BooleanSupplier trigger) {
        this.shooter = shooter;
        this.feeder = feeder;
        this.limelight = limelight;
        this.trigger = trigger;

        addRequirements(shooter, feeder, limelight);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        feeder.setFeedMode(FeedMode.PRESHOOT);
        stagingCargo = true;

        limelight.setLEDMode(limelight.LED_ON);
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        shooter.setRange(limelight.getDistance());

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
        limelight.setLEDMode(limelight.LED_OFF);
        shooter.stop();
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false;
    }
}
