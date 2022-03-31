package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.XboxController.Button;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.AutoAlignWithHubCommand;
import frc.robot.commands.AutoIntakeCommand;
import frc.robot.commands.AutoShootCommand;
import frc.robot.commands.DefaultDriveCommand;
import frc.robot.commands.DefaultShooterCommand;
import frc.robot.commands.FollowerCommand;
import frc.robot.commands.IntakeCommand;
import frc.robot.commands.ShootCommand;
import frc.robot.commands.AutoIntakeCommand.IntakeMode;
import frc.robot.drivers.Pigeon;
import frc.robot.subsystems.DriverVisionSubsystem;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.FeederSubsystem;
import frc.robot.utils.XboxControllerHelper;
import frc.robot.utils.trajectory.TrajectoryFactory;
import frc.robot.utils.Ranger;
import frc.robot.utils.SimpleRanger;
import frc.robot.utils.SwerveDriveConfig;
import frc.robot.utils.SwerveModuleConfig;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.LimelightSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.FeederSubsystem.FeedMode;

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
    private final XboxControllerHelper driveControllerHelper = new XboxControllerHelper(driveController);   
    private final XboxController operatorController = new XboxController(Constants.OPERATOR_CONTROLLER_PORT);

    Pigeon pigeon = new Pigeon(DRIVETRAIN_PIGEON_ID);
    // private final Pixy pixy = new Pixy(Pixy.TEAM_RED);
    private final Ranger ranger = new SimpleRanger();

    // Subsystems
    private FeederSubsystem feeder;
    private ShooterSubsystem shooter;
    private IntakeSubsystem intake;
    private DrivetrainSubsystem drivetrain;
    private LimelightSubsystem limelight;

    // Commands
    private Command fiveBallAutoCommand;

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

        // We don't ever call the DriverVision subsystem, we just create it and let it do its thing.
        new DriverVisionSubsystem();

        limelight = new LimelightSubsystem(CAMERA_ANGLE, CAMERA_HEIGHT, TARGET_HEIGHT);
    }

    /**
     * Create all of our robot's command objects here.
     */
    void createCommands() {

        // Since the path generation is slow we pre-create the autonomous commands that use them to speed things up
        // and pass a lambda to the command chooser that just returns the pre-created command.
        fiveBallAutoCommand = createFiveBallCommand();

        // Register a creators for our autonomous commands
        registerAutoCommand("Do Nothing", this::createNullCommand);
        registerAutoCommand("Shoot Only", this::createShootOnlyCommand);
        registerAutoCommand("Taxi Only", this::createTaxiOnlyCommand);
        registerAutoCommand("One Ball", this::createOneBallCommand);
        registerAutoCommand("Two Ball", this::createTwoBallCommand);
        registerAutoCommand("Three Ball", this::createThreeBallCommand);
        registerAutoCommand("Five Ball", () -> fiveBallAutoCommand);
        registerAutoCommand("TUNE", this::createTuneAutoCommand);

        // Create commands used during teleop
        drivetrain.setDefaultCommand(new DefaultDriveCommand(
                drivetrain,
                () -> -driveControllerHelper.scaleAxis(driveController.getLeftY()) * drivetrain.maxVelocity,
                () -> -driveControllerHelper.scaleAxis(driveController.getLeftX()) * drivetrain.maxVelocity,
                () -> -driveControllerHelper.scaleAxis(driveController.getRightX()) * drivetrain.maxAngularVelocity * 0.9));

        shooter.setDefaultCommand(new DefaultShooterCommand(shooter));
    }

    /**
     * Use this method to define your button->command mappings.
     */
    private void configureButtonBindings() {
        
        // Reset the gyroscope on the Pigeon.
        new JoystickButton(driveController, Button.kStart.value)
                .whenPressed(new InstantCommand(() -> drivetrain.resetPosition()));
        new JoystickButton(driveController, Button.kStart.value)
                .whenPressed(new InstantCommand(() -> drivetrain.setAutoInitPose(new Pose2d(-0.5, -2, Rotation2d.fromDegrees(-90)))));

        // Schedule the Shoot command to fire a cargo
        /*
        new Trigger(() -> driveController.getLeftTriggerAxis() > 0.8).whileActiveOnce(
                new ParallelCommandGroup(new ShootCommand(shooter, feeder, driveControllerHelper::rumble, () -> driveController.getRightTriggerAxis() > 0.8, limelight),
                new AutoAlignWithHubCommand(limelight, drivetrain, 
                () -> -driveControllerHelper.scaleAxis(driveController.getLeftY()) * drivetrain.maxVelocity,
                () -> -driveControllerHelper.scaleAxis(driveController.getLeftX()) * drivetrain.maxVelocity)));

        new JoystickButton(driveController, Button.kA.value).whenHeld(
                new ShootCommand(shooter, feeder, driveControllerHelper::rumble, () -> driveController.getRightTriggerAxis() > 0.8, 0));
        
        new JoystickButton(driveController, Button.kB.value).whenHeld(
            new ParallelCommandGroup(new ShootCommand(shooter, feeder, driveControllerHelper::rumble, () -> true, limelight),
                new AutoAlignWithHubCommand(limelight, drivetrain, 
                () -> -driveControllerHelper.scaleAxis(driveController.getLeftY()) * drivetrain.maxVelocity,
                () -> -driveControllerHelper.scaleAxis(driveController.getLeftX()) * drivetrain.maxVelocity)));
        */

        new Trigger(() -> driveController.getLeftTriggerAxis() > 0.8).whileActiveOnce(
            new ShootCommand(shooter, feeder, drivetrain, limelight,
                            driveControllerHelper::rumble, () -> driveController.getRightTriggerAxis() > 0.8,
                            () -> -driveControllerHelper.scaleAxis(driveController.getLeftY()) * drivetrain.maxVelocity,
                            () -> -driveControllerHelper.scaleAxis(driveController.getLeftX()) * drivetrain.maxVelocity));

        new JoystickButton(driveController, Button.kA.value).whenHeld(
            new ParallelCommandGroup(
                new AutoAlignWithHubCommand(limelight, drivetrain),
                new ShootCommand(shooter, feeder, limelight,
                                driveControllerHelper::rumble, () -> driveController.getRightTriggerAxis() > 0.8,
                                0)));
        
        new JoystickButton(driveController, Button.kB.value).whenHeld(
            new ShootCommand(shooter, feeder, drivetrain, limelight,
                            driveControllerHelper::rumble, () -> true, 
                            () -> -driveControllerHelper.scaleAxis(driveController.getLeftY()) * drivetrain.maxVelocity,
                            () -> -driveControllerHelper.scaleAxis(driveController.getLeftX()) * drivetrain.maxVelocity));

        // Schedule the Intake command to pick-up cargo
        new JoystickButton(driveController, Button.kRightBumper.value)
                .whenHeld(new IntakeCommand(intake, feeder, (() -> driveController.getLeftBumper())));

        // operator controls
        new JoystickButton(operatorController, Button.kA.value).whenPressed(
                    new InstantCommand(() -> feeder.setFeedMode(FeedMode.REVERSE_CONTINUOUS)));
                
        new JoystickButton(operatorController, Button.kA.value).whenReleased(
                    new InstantCommand(() -> feeder.setFeedMode(FeedMode.STOPPED)));

        new JoystickButton(operatorController, Button.kB.value).whenPressed(
            new InstantCommand(() -> feeder.setFeedMode(FeedMode.PRESHOOT)));
        
        new JoystickButton(operatorController, Button.kB.value).whenReleased(
            new InstantCommand(() -> feeder.setFeedMode(FeedMode.STOPPED)));
        
        new JoystickButton(operatorController, Button.kRightBumper.value).whenPressed(
            new InstantCommand(() -> intake.operatorBallIn()));
        
        new JoystickButton(operatorController, Button.kRightBumper.value).whenReleased(
            new InstantCommand(() -> intake.stopMotor()));
        
        new JoystickButton(operatorController, Button.kLeftBumper.value).whenPressed(
            new InstantCommand(() -> intake.operatorBallOut()));

        new JoystickButton(operatorController, Button.kLeftBumper.value).whenReleased(
            new InstantCommand(() -> intake.stopMotor()));
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

    /**
     * Register an autonomous command so it appears in the chooser in Shuffleboard
     * @param name Name of command as it appears in the chooser
     * @param creator Reference to method that creates the command
     */
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

        Shuffleboard.getTab("Driver")
                .add("Auto Command", chooser)
                .withPosition(5, 0)
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
        return new SequentialCommandGroup(
            new InstantCommand(() -> drivetrain.resetPosition()),
            new FollowerCommand(drivetrain, TrajectoryFactory.twoMetersForward)
        );
    }

    private Command createShootOnlyCommand() {
        return new SequentialCommandGroup(
            new InstantCommand(() -> shooter.idle()),
            new ParallelCommandGroup(
                new AutoAlignWithHubCommand(limelight, drivetrain),
                new AutoShootCommand(shooter, feeder, limelight, 1)));
    }

    private Command createOneBallCommand() {
        return new SequentialCommandGroup(createShootOnlyCommand(), createTaxiOnlyCommand());
    }

    private Command createTwoBallCommand() {
        return new SequentialCommandGroup(
            new InstantCommand(() -> drivetrain.resetPosition()),
            new InstantCommand(() -> shooter.idle()),
            new AutoIntakeCommand(intake, feeder, IntakeMode.extended),
            new FollowerCommand(drivetrain, TrajectoryFactory.twoMetersForward),
            new ParallelCommandGroup(
                new AutoAlignWithHubCommand(limelight, drivetrain),
                new AutoShootCommand(shooter, feeder, limelight, 2)),
            new AutoIntakeCommand(intake, feeder, IntakeMode.retracted));
    }

    /**
     * Create our 3-ball autonomous command.
     * 
     * @return Command to perform 5 ball autonomous
     */
    private Command createThreeBallCommand() {
        return new SequentialCommandGroup(new InstantCommand(() -> shooter.idle()),
            new InstantCommand(() -> drivetrain.setAutoInitPose(new Pose2d(-0.5, -2, Rotation2d.fromDegrees(-90)))),
            new AutoIntakeCommand(intake, feeder, IntakeMode.extended),
            //new ShootCommand(shooter, feeder, drivetrain, limelight),
            //new ParallelCommandGroup(
            new FollowerCommand(drivetrain, TrajectoryFactory.start_ball2),
            new AutoIntakeCommand(intake, feeder, IntakeMode.retracted),
                //new InstantCommand(() -> feeder.setFeedMode(FeedMode.PRESHOOT))),
            //new AutoIntakeCommand(intake, feeder, IntakeMode.retracted),

            // new ShootCommand(shooter, feeder, drivetrain, limelight),
            new AutoIntakeCommand(intake, feeder, IntakeMode.extended),
            new FollowerCommand(drivetrain, TrajectoryFactory.start_ball3_test),
            new AutoIntakeCommand(intake, feeder, IntakeMode.retracted)
            // new ShootCommand(shooter, feeder, drivetrain, limelight)
          );
    }

    /**
     * Create our 5-ball autonomous command.
     * 
     * @return Command to perform 5 ball autonomous
     */
    private Command createFiveBallCommand() {
        return new SequentialCommandGroup(new InstantCommand(() -> shooter.idle()),
            new InstantCommand(() -> drivetrain.setAutoInitPose(new Pose2d(-0.5, -2, Rotation2d.fromDegrees(-90)))),
            new AutoIntakeCommand(intake, feeder, IntakeMode.extended),
            //new ShootCommand(shooter, feeder, drivetrain, limelight),
            //new ParallelCommandGroup(
            new FollowerCommand(drivetrain, TrajectoryFactory.start_ball2),
            new AutoIntakeCommand(intake, feeder, IntakeMode.retracted),
                //new InstantCommand(() -> feeder.setFeedMode(FeedMode.PRESHOOT))),
            //new AutoIntakeCommand(intake, feeder, IntakeMode.retracted),

            // new ShootCommand(shooter, feeder, drivetrain, limelight),
            new AutoIntakeCommand(intake, feeder, IntakeMode.extended),
            new FollowerCommand(drivetrain, TrajectoryFactory.start_ball3_test),
            new AutoIntakeCommand(intake, feeder, IntakeMode.retracted),
            // new ShootCommand(shooter, feeder, drivetrain, limelight),
            new AutoIntakeCommand(intake, feeder, IntakeMode.extended),
            new FollowerCommand(drivetrain, TrajectoryFactory.ball3_station_shoot),
            new FollowerCommand(drivetrain, TrajectoryFactory.ball3_shoot_pos),
            new AutoIntakeCommand(intake,feeder, IntakeMode.retracted)
            // new AutoShootCommand(shooter, feeder, limelight)
          );
    }

    public Command createTuneAutoCommand() {
        return new SequentialCommandGroup(
            new InstantCommand(() -> drivetrain.resetPosition()),
            new FollowerCommand(drivetrain, TrajectoryFactory.tune));
    }
}
