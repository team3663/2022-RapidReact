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
import edu.wpi.first.wpilibj2.command.button.POVButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.AutoAlignWithHubCommand;
import frc.robot.commands.AutoIntakeCommand;
import frc.robot.commands.AutoShootCommand;
import frc.robot.commands.DefaultDriveCommand;
import frc.robot.commands.DefaultShooterCommand;
import frc.robot.commands.ExtendElevatorCommand;
import frc.robot.commands.HomeElevatorCommand;
import frc.robot.commands.IncrementWindmillAngle;
import frc.robot.commands.FollowerCommand;
import frc.robot.commands.IntakeCommand;
import frc.robot.commands.MoveWindmillCommand;
import frc.robot.commands.RotateWindmillCommand;
import frc.robot.commands.ShootCommand;
import frc.robot.commands.SwitchBlueHookCommand;
import frc.robot.commands.SwitchRedHookCommand;
import frc.robot.commands.WaitForSecondsCommand;
import frc.robot.commands.WaitShooterAvailableCommand;
import frc.robot.commands.AutoIntakeCommand.IntakeMode;
import frc.robot.drivers.Pigeon;
import frc.robot.subsystems.ClimberSubsystem;
import frc.robot.subsystems.DriverVisionSubsystem;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.FeederSubsystem;
import frc.robot.utils.XboxControllerHelper;
import frc.robot.utils.Ranger;
import frc.robot.utils.SimpleRanger;
import frc.robot.utils.SwerveDriveConfig;
import frc.robot.utils.SwerveModuleConfig;
import frc.robot.utils.trajectory.TrajectoryFactory;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.LimelightSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.ClimberSubsystem.HookPosition;
import frc.robot.subsystems.ClimberSubsystem.WindmillState;
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
    private ClimberSubsystem climber;

    // Commands
    private DefaultDriveCommand drive;
    private Command deployClimberCommand;
    private Command climbCommand;
    private Command climbSecondToThirdCommmand;
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
        climber = new ClimberSubsystem(ELEVATOR_CAN_ID, WINDMILL_1_CAN_ID, WINDMILL_2_CAN_ID, 
                RED_HOOK_CAN_ID, BLUE_HOOK_CAN_ID, WINDMILL_SENSOR_DIO);

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
                () -> -driveControllerHelper.scaleAxis(driveController.getRightX()) * drivetrain.maxAngularVelocity * 0.8));

        // create idle shoot command
        shooter.setDefaultCommand(new DefaultShooterCommand(shooter));

        // Create the command to deploy the climber

        deployClimberCommand = new ParallelCommandGroup(
            new HomeElevatorCommand(climber),
            new SwitchRedHookCommand(climber, HookPosition.Grab),
            new SwitchBlueHookCommand(climber, HookPosition.Grab)
        );

        climbSecondToThirdCommmand = new ParallelCommandGroup(
            new RotateWindmillCommand(climber, WindmillState.SecondToThird),
            new SwitchRedHookCommand(climber, HookPosition.Grab)
        );

        climbCommand = new SequentialCommandGroup(
            new RotateWindmillCommand(climber, WindmillState.FirstToSecond),
            new WaitForSecondsCommand(0.25),
            new SwitchBlueHookCommand(climber, HookPosition.Lock),
            new WaitForSecondsCommand(0.5),
            new RotateWindmillCommand(climber, WindmillState.ShiftWeightOffFirst),
            new WaitForSecondsCommand(0.25),
            new SwitchRedHookCommand(climber, HookPosition.Release),
            new WaitForSecondsCommand(0.25),
            climbSecondToThirdCommmand,
            new WaitForSecondsCommand(0.5),
            new SwitchRedHookCommand(climber, HookPosition.Lock),
            new WaitForSecondsCommand(0.25),
            new RotateWindmillCommand(climber, WindmillState.ShiftWeightOffSecond),
            new WaitForSecondsCommand(0.25),
            new SwitchBlueHookCommand(climber, HookPosition.Release),
            new WaitForSecondsCommand(0.25),
            new RotateWindmillCommand(climber, WindmillState.HangFromThird)
        );
    }

    /**
     * Use this method to define your button->command mappings.
     */
    private void configureButtonBindings() {
        
        // Reset the gyroscope on the Pigeon.
        new JoystickButton(driveController, Button.kStart.value)
                .whenPressed(new InstantCommand(() -> drivetrain.resetPosition()));
        
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
            new ParallelCommandGroup(
                new AutoAlignWithHubCommand(limelight, drivetrain,
                                            () -> -driveControllerHelper.scaleAxis(driveController.getLeftY()) * drivetrain.maxVelocity,
                                            () -> -driveControllerHelper.scaleAxis(driveController.getLeftX()) * drivetrain.maxVelocity),
                new ShootCommand(shooter, feeder, limelight,
                                driveControllerHelper::rumble,
                                () -> driveController.getRightTriggerAxis() > 0.8,
                                () -> driveController.getBButton())));

        new JoystickButton(driveController, Button.kA.value).whenHeld(
            new ParallelCommandGroup(
                new AutoAlignWithHubCommand(limelight, drivetrain),
                new ShootCommand(shooter, feeder, limelight,
                                driveControllerHelper::rumble,
                                () -> driveController.getRightTriggerAxis() > 0.8,
                                () -> driveController.getBButton(),
                                0)));
        
        /*
        new JoystickButton(driveController, Button.kB.value).whenHeld(
            new ParallelCommandGroup(
                new AutoAlignWithHubCommand(limelight, drivetrain,
                                            () -> -driveControllerHelper.scaleAxis(driveController.getLeftY()) * drivetrain.maxVelocity,
                                            () -> -driveControllerHelper.scaleAxis(driveController.getLeftX()) * drivetrain.maxVelocity),
                new ShootCommand(shooter, feeder, limelight,
                                driveControllerHelper::rumble,
                                () -> true)));
        */
            
        // Schedule the Intake command to pick-up cargo
        new JoystickButton(driveController, Button.kRightBumper.value)
                .whenHeld(new IntakeCommand(intake, feeder, (() -> driveController.getLeftBumper())));

        // climb
        new JoystickButton(driveController, Button.kBack.value)
            .whenHeld(deployClimberCommand);

        
        // operator controls
        new JoystickButton(operatorController, Button.kA.value).whenPressed(
                    new InstantCommand(() -> feeder.setFeedMode(FeedMode.REVERSE_CONTINUOUS)));
                
        new JoystickButton(operatorController, Button.kA.value).whenReleased(
                    new InstantCommand(() -> feeder.setFeedMode(FeedMode.STOPPED)));

        new JoystickButton(operatorController, Button.kB.value).whenPressed(
            new InstantCommand(() -> feeder.setFeedMode(FeedMode.PRESHOOT)));
        
        new JoystickButton(operatorController, Button.kB.value).whenReleased(
            new InstantCommand(() -> feeder.setFeedMode(FeedMode.STOPPED)));

        new JoystickButton(operatorController, Button.kLeftBumper.value)
            .whileHeld(new ExtendElevatorCommand(climber, -0.1));

        new JoystickButton(operatorController, Button.kBack.value).whenPressed(deployClimberCommand);

        new JoystickButton(operatorController, Button.kStart.value).whenPressed(climbCommand);

        new JoystickButton(operatorController, Button.kX.value).whenPressed(new RotateWindmillCommand(climber, WindmillState.Home));


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
            new ParallelCommandGroup(new AutoShootCommand(shooter, feeder, limelight, 1.5), new AutoAlignWithHubCommand(limelight, drivetrain)),
                //new InstantCommand(() -> feeder.setFeedMode(FeedMode.PRESHOOT))),
            //new AutoIntakeCommand(intake, feeder, IntakeMode.retracted),
            
            //new AutoShootCommand(shooter, feeder, limelight, 1.5),
            new ParallelCommandGroup(new AutoShootCommand(shooter, feeder, limelight, 1.5), new AutoAlignWithHubCommand(limelight, drivetrain)),
            new AutoIntakeCommand(intake, feeder, IntakeMode.extended),
            new FollowerCommand(drivetrain, TrajectoryFactory.start_ball3_test),
            new AutoIntakeCommand(intake, feeder, IntakeMode.retracted),
            new AutoShootCommand(shooter, feeder, limelight, 1),

            new AutoIntakeCommand(intake, feeder, IntakeMode.extended),
            new FollowerCommand(drivetrain, TrajectoryFactory.ball3_station_shoot),
            new FollowerCommand(drivetrain, TrajectoryFactory.ball3_shoot_pos),
            new AutoIntakeCommand(intake,feeder, IntakeMode.retracted),
            new ParallelCommandGroup(new AutoShootCommand(shooter, feeder, limelight, 1.5), new AutoAlignWithHubCommand(limelight, drivetrain))
          );
    }

    public Command createTuneAutoCommand() {
        return new SequentialCommandGroup(
            new InstantCommand(() -> drivetrain.resetPosition()),
            new FollowerCommand(drivetrain, TrajectoryFactory.tune));
    }
}
