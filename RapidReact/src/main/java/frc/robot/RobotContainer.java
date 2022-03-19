package frc.robot;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
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
import frc.robot.commands.AutoDriveCommand;
import frc.robot.commands.AutoIntakeCommand;
import frc.robot.commands.AutoShootCommand;
import frc.robot.commands.DriveCommand;
import frc.robot.commands.FollowerCommand;
import frc.robot.commands.IntakeCommand;
import frc.robot.commands.ShootCommand;
import frc.robot.commands.WaitShooterAvailableCommand;
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
import frc.robot.subsystems.FeederSubsystem.FeedMode;

import static frc.robot.Constants.*;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.function.Supplier;

import org.frcteam2910.common.control.CentripetalAccelerationConstraint;
import org.frcteam2910.common.control.MaxAccelerationConstraint;
import org.frcteam2910.common.control.MaxVelocityConstraint;
import org.frcteam2910.common.control.Path;
import org.frcteam2910.common.control.SplinePathBuilder;
import org.frcteam2910.common.control.Trajectory;
import org.frcteam2910.common.control.TrajectoryConstraint;
import org.frcteam2910.common.math.Rotation2;
import org.frcteam2910.common.math.Vector2;

/**
 * This class is where the bulk of the robot should be declared.
 */
public class RobotContainer {

    private final XboxController driveController = new XboxController(Constants.DRIVE_CONTROLLER_PORT);
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
    private DriveCommand drive;

    //Auto Commands
    private Command FiveBallAuto;

    // Autonomous command creation
    private final HashMap<String, Supplier<Command>> commandCreators = new HashMap<String, Supplier<Command>>();
    private SendableChooser<Supplier<Command>> chooser = new SendableChooser<Supplier<Command>>();

    public RobotContainer() {

        createSubsystems(); // Create our subsystems.
        createAutoCommands();
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

        limelight = new LimelightSubsystem(36, 0.5842, 2.6414);
    }

    private Command createTunePathCommand() {
        Path path
    }

    private void createAutoCommands(){
        Path ball2 = new SplinePathBuilder(new Vector2(-.5, -2), new Rotation2(-.6, -3.5, true), Rotation2.fromDegrees(90))
        .hermite(new Vector2(-.6, -3.5), new Rotation2(-.6, -3.5, true), Rotation2.fromDegrees(90))
        .build();
      Path ball3 = new SplinePathBuilder(new Vector2(-.6, -3.5), new Rotation2(-2, -2, true), Rotation2.fromDegrees(90)) // heading
        .hermite(new Vector2(-2, -2), new Rotation2(-3.5, -2.2, true), Rotation2.fromDegrees(-32)) // heading, rotation
        .hermite(new Vector2(-3.5, -2.2), new Rotation2(-2, -2, true), Rotation2.fromDegrees(-32)) // heading, rotation
        .build();
      Path ball4 = new SplinePathBuilder(new Vector2(-3.5, -2.2), new Rotation2(-3.5, -2.2, true), Rotation2.fromDegrees(32))
      .hermite(new Vector2(-7, -3), new Rotation2(-7, -3, true),Rotation2.fromDegrees(32))
      .hermite(new Vector2(-4.5, -3), new Rotation2(-7, -3, true), Rotation2.fromDegrees(75))
      .build();
  
      Path ball5 = new SplinePathBuilder(new Vector2(-4.5, -3), new Rotation2(-7, -3, true), Rotation2.fromDegrees(75))
      .hermite(new Vector2(-7, -3), new Rotation2(-7, -3, true),Rotation2.fromDegrees(32))
      .hermite(new Vector2(-3, -3), new Rotation2(-7, -3, true), Rotation2.fromDegrees(75))
      .build();
  
      Path aim3 = new SplinePathBuilder(new Vector2(0, 0), Rotation2.ZERO, Rotation2.ZERO)
        .hermite(new Vector2(-1.184, 0.475), Rotation2.ZERO, Rotation2.ZERO) // rotation
        .build();
  
        TrajectoryConstraint[] constraints = {
          new MaxAccelerationConstraint(3),
          new MaxVelocityConstraint(7),
          new CentripetalAccelerationConstraint(5.0)
        };
    
        Trajectory t1 = new Trajectory(ball2, constraints, Units.inchesToMeters(0.1));
        Trajectory t2 = new Trajectory(ball3, constraints, Units.inchesToMeters(0.1));
        Trajectory t3 = new Trajectory(ball4, constraints, Units.inchesToMeters(0.1));
        //Trajectory t4 = new Trajectory(ball5, constraints, Units.inchesToMeters(0.1));
        //same paths is t3 but with a different end point
  
        FiveBallAuto = new SequentialCommandGroup(
          new InstantCommand(() -> drivetrain.setAutoInitCommand(-.5,-2, Rotation2d.fromDegrees(90))),
          //new AutoShootCommand(shooter, feeder, 3),
          new ParallelCommandGroup(new FollowerCommand(drivetrain, t1), new AutoIntakeCommand(intake, feeder)), 
          new ParallelCommandGroup(new FollowerCommand(drivetrain, t2), new AutoIntakeCommand(intake, feeder)),
          //new AutoShootCommand(shooter, feeder, 3),
          new ParallelCommandGroup(new FollowerCommand(drivetrain, t3), new AutoIntakeCommand(intake, feeder)),
          new AutoShootCommand(shooter, feeder, 3)
          );
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
        // registerAutoCommand("Two Ball", this::createTwoBallCommand);

        // create tele drive command
        drive = new DriveCommand(
                drivetrain,
                () -> -ControllerUtils.modifyAxis(driveController.getLeftY()) * drivetrain.maxVelocity,
                () -> -ControllerUtils.modifyAxis(driveController.getLeftX()) * drivetrain.maxVelocity,
                () -> -ControllerUtils.modifyAxis(driveController.getRightX()) * drivetrain.maxAngularVelocity * 0.9);
                drivetrain.setDefaultCommand(drive);
    }

    /**
     * Use this method to define your button->command mappings.
     */
    private void configureButtonBindings() {

        // Reset the gyroscope on the Pigeon.
        new JoystickButton(driveController, Button.kStart.value)
                .whenPressed(new InstantCommand(() -> drivetrain.resetPosition()));

        // Schedule the Shoot command to fire a cargo
        new Trigger(() -> driveController.getLeftTriggerAxis() > 0.8).whileActiveOnce(
                new ParallelCommandGroup(new ShootCommand(shooter, feeder, () -> driveController.getRightTriggerAxis() > 0.8, limelight),
                new AutoAlignWithHubCommand(limelight, drivetrain, 
                () -> -ControllerUtils.modifyAxis(driveController.getLeftY()) * drivetrain.maxVelocity,
                () -> -ControllerUtils.modifyAxis(driveController.getLeftX()) * drivetrain.maxVelocity)));

        new JoystickButton(driveController, Button.kA.value).whenHeld(
                new ShootCommand(shooter, feeder, () -> driveController.getRightTriggerAxis() > 0.8, 0));

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
        
        // Schedule the Intake command to pick-up cargo
        new JoystickButton(driveController, Button.kRightBumper.value)
                .whenHeld(new IntakeCommand(intake, feeder, (() -> driveController.getLeftBumper())));
    }


    /**
     * The main {@link Robot} class calls this to get the command to run during
     * autonomous.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() {
        //Supplier<Command> creator = chooser.getSelected();
        return FiveBallAuto;
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
        return new AutoDriveCommand(drivetrain, new Translation2d(2, 0), Rotation2d.fromDegrees(0));
    }

    private Command createShootOnlyCommand() {
        return new SequentialCommandGroup(new WaitShooterAvailableCommand(shooter), new AutoShootCommand(shooter, feeder, 2.0));
    }

    private Command createOneBallCommand() {
        return new SequentialCommandGroup(createShootOnlyCommand(), createTaxiOnlyCommand());
    }

    @SuppressWarnings("unused")
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
}
