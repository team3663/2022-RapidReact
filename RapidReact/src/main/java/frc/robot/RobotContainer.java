package frc.robot;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.XboxController.Axis;
import edu.wpi.first.wpilibj.XboxController.Button;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.AutoFollowCargoCommand;
import frc.robot.commands.TeleOpDriveCommand;
import frc.robot.drivers.Limelight;
import frc.robot.drivers.Pigeon;
import frc.robot.drivers.Pixy;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.FeederSubsystem;
import frc.robot.subsystems.FeederSubsystem.FeedMode;
import frc.robot.utils.ControllerUtils;
import frc.robot.utils.Ranger;
import frc.robot.utils.SimpleRanger;
import frc.robot.utils.SwerveDriveConfig;
import frc.robot.utils.SwerveModuleConfig;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import static frc.robot.Constants.*;

/**
 * This class is where the bulk of the robot should be declared.
 */
public class RobotContainer {

   
    private final XboxController driveController = new XboxController(Constants.DRIVE_CONTROLLER_PORT);
    @SuppressWarnings("unused")
    private final XboxController operatorController = new XboxController(Constants.OPERATOR_CONTROLLER_PORT);

    Pigeon pigeon = new Pigeon(DRIVETRAIN_PIGEON_ID);
    private static final Limelight limelight = new Limelight();
    private final Pixy pixy = new Pixy(Pixy.TEAM_RED);
    private final Ranger ranger = new SimpleRanger();

    private Command autoCommand;

    // Subsystems
    private FeederSubsystem feeder;
    private ShooterSubsystem shooter;
    private IntakeSubsystem intake;
    private DrivetrainSubsystem drivetrain;

    // Commands
    private TeleOpDriveCommand teleOpDrive;

    public RobotContainer() {

        createSubsystems(); // Create our subsystems.
        createCommands(); // Create our commands
        configureButtonBindings(); // Setup our button bindings

        drivetrain.setDefaultCommand(teleOpDrive);
    }

    /**
     * Create all of our robot's subsystem objects here.
     */
    void createSubsystems() {
        intake = new IntakeSubsystem(INTAKE_MOTOR_CAN_ID, INTAKE_RETRACT_SOLENOID_CHAN, INTAKE_EXTEND_SOLENOID_CHAN);
        feeder = new FeederSubsystem(FEEDER_MOTOR_CAN_ID, FEEDER_ENTRY_SENSOR_DIO, FEEDER_EXIT_SENSOR_DIO);
        shooter = new ShooterSubsystem(SHOOTER_MOTOR_1_CAN_ID, SHOOTER_MOTOR_2_CAN_ID, HOOD_MOTOR_CAN_ID,
                HOOD_LIMITSWITCH_DIO, ranger, limelight);

        // Setup our server drivetrain subsystem
        SwerveModuleConfig fl = new SwerveModuleConfig(FRONT_LEFT_MODULE_DRIVE_MOTOR, FRONT_LEFT_MODULE_STEER_MOTOR,
                FRONT_LEFT_MODULE_STEER_ENCODER, FRONT_LEFT_MODULE_STEER_OFFSET);
        SwerveModuleConfig fr = new SwerveModuleConfig(FRONT_RIGHT_MODULE_DRIVE_MOTOR, FRONT_RIGHT_MODULE_STEER_MOTOR,
                FRONT_RIGHT_MODULE_STEER_ENCODER, FRONT_RIGHT_MODULE_STEER_OFFSET);
        SwerveModuleConfig bl = new SwerveModuleConfig(BACK_LEFT_MODULE_DRIVE_MOTOR, BACK_LEFT_MODULE_STEER_MOTOR,
                BACK_LEFT_MODULE_STEER_ENCODER, BACK_LEFT_MODULE_STEER_OFFSET);
        SwerveModuleConfig br = new SwerveModuleConfig(BACK_RIGHT_MODULE_DRIVE_MOTOR, BACK_RIGHT_MODULE_STEER_MOTOR,
                BACK_RIGHT_MODULE_STEER_ENCODER, BACK_RIGHT_MODULE_STEER_OFFSET);
        SwerveDriveConfig swerveConfig = new SwerveDriveConfig(fl, fr, bl, br, DRIVETRAIN_TRACKWIDTH_METERS,
                DRIVETRAIN_WHEELBASE_METERS, DRIVE_TRAIN_WHEEL_DIAMETER_METERS);
        drivetrain = new DrivetrainSubsystem(swerveConfig, pigeon, pixy);
    }

    /**
     * Create all of our robot's command objects here.
     */
    void createCommands() {
        teleOpDrive = new TeleOpDriveCommand(
                drivetrain,
                () -> -ControllerUtils.modifyAxis(driveController.getLeftY()) * drivetrain.maxVelocity,
                () -> -ControllerUtils.modifyAxis(driveController.getLeftX()) * drivetrain.maxVelocity,
                () -> -ControllerUtils.modifyAxis(driveController.getRightX()) * drivetrain.maxAngularVelocity);
    }

    /**
     * Use this method to define your button->command mappings.
     */
    private void configureButtonBindings() {

        // Button commands to test intake subsystem
        new JoystickButton(driveController, Button.kA.value).whenHeld(new SequentialCommandGroup(
                new InstantCommand(() -> intake.extend(), intake), new InstantCommand(() -> intake.start(), intake)));
        new JoystickButton(driveController, Button.kA.value).whenReleased(new SequentialCommandGroup(
                new InstantCommand(() -> intake.retract(), intake), new InstantCommand(() -> intake.stop(), intake)));

        // Button commands to help test the feeder subsystem.
        new JoystickButton(driveController, Button.kX.value)
                .whenPressed(new InstantCommand(() -> feeder.setFeedMode(FeedMode.STOPPED), feeder));
        new JoystickButton(driveController, Button.kY.value)
                .whenPressed(new InstantCommand(() -> feeder.setFeedMode(FeedMode.CONTINUOUS), feeder));

        // new JoystickButton(driveController,
        // Button.kBack.value).whenPressed(drivetrain::resetGyroscope);
        // new JoystickButton(driveController,
        // Button.kStart.value).whenPressed(drivetrain::resetPosition);

        // Button commands to test shooter subsystem.
        new JoystickButton(driveController, Button.kStart.value)
                .whenPressed(new InstantCommand(() -> shooter.start(), shooter));
        new JoystickButton(driveController, Button.kBack.value)
                .whenPressed(new InstantCommand(() -> shooter.stop(), shooter));
        new JoystickButton(driveController, Button.kLeftBumper.value)
                .whenPressed(new InstantCommand(() -> shooter.decreasePower(), shooter));
        new JoystickButton(driveController, Button.kRightBumper.value)
                .whenPressed(new InstantCommand(() -> shooter.increasePower(), shooter));
        new JoystickButton(driveController, Axis.kLeftTrigger.value)
                .whenPressed(new InstantCommand(() -> shooter.decreasePower(), shooter));
        new JoystickButton(driveController, Axis.kRightTrigger.value)
                .whenPressed(new InstantCommand(() -> shooter.increasePower(), shooter));
    }

    /**
     * The main {@link Robot} class calls this to get the command to run during
     * autonomous.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() {
            autoCommand = new AutoFollowCargoCommand(drivetrain, pixy);
        return autoCommand;
    }
}
