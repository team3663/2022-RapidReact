package frc.robot;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.XboxController.Button;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.AutoAlignWithHubCommand;
import frc.robot.commands.AutoDriveCommand;
import frc.robot.commands.AutoIntakeCommand;
import frc.robot.commands.AutoShootCommand;
import frc.robot.commands.DriveCommand;
import frc.robot.commands.IntakeCommand;
import frc.robot.commands.ShootCommand;
import frc.robot.drivers.Pigeon;
import frc.robot.subsystems.DriverVisionSubsystem;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.FeederSubsystem;
import frc.robot.utils.ControllerUtils;
import frc.robot.utils.Ranger;
import frc.robot.utils.SimpleRanger;
import frc.robot.utils.SwerveDriveConfig;
import frc.robot.utils.SwerveModuleConfig;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.LimelightSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

import static frc.robot.Constants.*;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.function.Supplier;

/**
 * This class is where the bulk of the robot should be declared.
 */
public class RobotContainer {

    private final XboxController driveController = new XboxController(Constants.DRIVE_CONTROLLER_PORT);
    @SuppressWarnings("unused")
    private final XboxController operatorController = new XboxController(Constants.OPERATOR_CONTROLLER_PORT);

    Pigeon pigeon = new Pigeon(DRIVETRAIN_PIGEON_ID);
    // private final Pixy pixy = new Pixy(Pixy.TEAM_RED);
    private final Ranger ranger = new SimpleRanger();

    // Subsystems
    private FeederSubsystem feeder;
    private ShooterSubsystem shooter;
    private IntakeSubsystem intake;
    private DrivetrainSubsystem drivetrain;
    private DriverVisionSubsystem driver;
    private LimelightSubsystem limelight;

    // Commands
    private DriveCommand drive;

    // Autonomous command creation
    private final HashMap<String, Supplier<Command>> commandCreators = new HashMap<String, Supplier<Command>>();
    private SendableChooser<Supplier<Command>> chooser = new SendableChooser<Supplier<Command>>();

    public RobotContainer() {

        createSubsystems(); // Create our subsystems.
        createCommands(); // Create our commands
        configureButtonBindings(); // Setup our button bindings
        setupCommandChooser();
    }

    /**
     * Create all of our robot's subsystem objects here.
     */
    void createSubsystems() {
        // intake = new IntakeSubsystem(INTAKE_MOTOR_CAN_ID,
        // INTAKE_RETRACT_SOLENOID_CHAN, INTAKE_EXTEND_SOLENOID_CHAN);
        feeder = new FeederSubsystem(FEEDER_MOTOR_CAN_ID, FEEDER_ENTRY_SENSOR_DIO, FEEDER_EXIT_SENSOR_DIO);
        shooter = new ShooterSubsystem(SHOOTER_MOTOR_1_CAN_ID, SHOOTER_MOTOR_2_CAN_ID, HOOD_MOTOR_CAN_ID,
                HOOD_LIMITSWITCH_DIO, ranger);
        intake = new IntakeSubsystem(INTAKE_MOTOR_CAN_ID, BOOM_RETRACT_SOLENOID_CHAN, BOOM_EXTEND_SOLENOID_CHAN,
                ARM_RETRACT_SOLENOID_CHAN, ARM_EXTEND_SOLENOID_CHAN);

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
        drivetrain = new DrivetrainSubsystem(swerveConfig, pigeon); // pixy

        driver = new DriverVisionSubsystem(feeder);
        limelight = new LimelightSubsystem(36, 0.5842, 2.6414);
    }

    /**
     * Create all of our robot's command objects here.
     */
    void createCommands() {

        // Register a creators for our autonomous commands
        registerAutoCommand("Do Nothing", this::createNullCommand);
        registerAutoCommand("Shoot Only", this::createShootOnlyCommand);
        registerAutoCommand("Taxi Only", this::createTaxiOnlyCommand);
        registerAutoCommand("One Ball", this::createOneBallCommand);
        registerAutoCommand("Two Ball", this::createTwoBallCommand);

        // create tele drive command
        drive = new DriveCommand(
                drivetrain,
                () -> -ControllerUtils.modifyAxis(driveController.getLeftY()) * drivetrain.maxVelocity,
                () -> -ControllerUtils.modifyAxis(driveController.getLeftX()) * drivetrain.maxVelocity,
                () -> -ControllerUtils.modifyAxis(driveController.getRightX())
                        * drivetrain.maxAngularVelocity);
        drivetrain.setDefaultCommand(drive);
    }

    /**
     * Use this method to define your button->command mappings.
     */
    private void configureButtonBindings() {

        // Reset the gyroscope on the Pigeon.
        new JoystickButton(driveController, Button.kStart.value)
                .whenPressed(new InstantCommand(() -> drivetrain.resetGyroscope()));

        // Schedule the Shoot command to fire a cargo
        new JoystickButton(driveController, Button.kY.value).whenHeld(
                new ShootCommand(shooter, feeder, limelight, (() -> driveController.getRightTriggerAxis() > 0.8)));

        // Schedule the Intake command to pick-up cargo
        new JoystickButton(driveController, Button.kRightBumper.value)
                .whenHeld(new IntakeCommand(intake, feeder, (() -> driveController.getLeftBumper())));


        // Temporary test commands to be removed before competition
        new JoystickButton(driveController, Button.kA.value).whenHeld(new AutoAlignWithHubCommand(limelight, drivetrain, () -> 0, () -> 0));

        // new POVButton(driveController, 0).whenPressed(new InstantCommand(() ->
        // shooter.setAngle(67.0), shooter));
    }

    /**
     * The main {@link Robot} class calls this to get the command to run during
     * autonomous.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() {
        Supplier<Command> creator = chooser.getSelected();
        return creator.get();
    }

    private void registerAutoCommand(String name, Supplier<Command> creator) {
        commandCreators.put(name, creator);
    }

    /**
     * Setup our autonomous command chooser in the Shuffleboard
     */
    private void setupCommandChooser() {
        List<String> keys = new ArrayList<String>(commandCreators.keySet());
        keys.sort((a, b) -> a.compareTo(b));

        for (String key : keys) {
            chooser.addOption(key, commandCreators.get(key));
        }

        Shuffleboard.getTab("Main")
                .add("Auto Command", chooser)
                .withPosition(0, 1)
                .withSize(2, 1)
                .withWidget(BuiltInWidgets.kComboBoxChooser);
    }

    // ---------------------------------------------------------------------------
    // Autonomous command creators
    // ---------------------------------------------------------------------------

    private Command createNullCommand() {
        return null;
    }

    private Command createTaxiOnlyCommand() {
        return new AutoDriveCommand(drivetrain, new Translation2d(3, 0), Rotation2d.fromDegrees(0));
    }

    private Command createShootOnlyCommand() {
        return new AutoShootCommand(shooter, feeder, limelight);
    }

    private Command createOneBallCommand() {
        return new SequentialCommandGroup(createShootOnlyCommand(), createTaxiOnlyCommand());
    }

    private Command createTwoBallCommand() {
        // don't use this because pixy hardware is not working
        return new SequentialCommandGroup(
                new AutoAlignWithHubCommand(limelight, drivetrain, () -> 0, () -> 0),
                createShootOnlyCommand(),
                new AutoDriveCommand(drivetrain, new Translation2d(3, 0), Rotation2d.fromDegrees(90)),
                // new AutoFollowCargoCommand(drivetrain, pixy),
                new AutoIntakeCommand(intake, feeder),
                new AutoAlignWithHubCommand(limelight, drivetrain, () -> 0, () -> 0),
                createShootOnlyCommand());
    }

    /*
     * private Command createTrajectoryCommand() {
     * Path path = new Path(PATH.backOutOfTarmac);
     * followTrajectory = new SwerveControllerCommand(path.getTrajectory(),
     * drivetrain::getPose,
     * drivetrain.getKinematics(),
     * path.getPidController(),
     * path.getPidController(),
     * path.getAnglePidController(),
     * drivetrain::setModuleStates,
     * drivetrain);
     * return followTrajectory;
     * }
     */
}
