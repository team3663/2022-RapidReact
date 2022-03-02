package frc.robot;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.XboxController.Axis;
import edu.wpi.first.wpilibj.XboxController.Button;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import frc.robot.commands.AutoDriveCommand;
import frc.robot.commands.AutoFollowCargoCommand;
import frc.robot.commands.AutoShootCommand;
import frc.robot.commands.TeleOpDriveCommand;
import frc.robot.drivers.Limelight;
import frc.robot.drivers.Pigeon;
import frc.robot.drivers.Pixy;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.FeederSubsystem;
import frc.robot.subsystems.FeederSubsystem.FeedMode;
import frc.robot.utils.ControllerUtils;
import frc.robot.utils.Path;
import frc.robot.utils.Ranger;
import frc.robot.utils.SimpleRanger;
import frc.robot.utils.SwerveDriveConfig;
import frc.robot.utils.SwerveModuleConfig;
import frc.robot.utils.Path.PATH;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import static frc.robot.Constants.*;

import java.util.List;

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

    // Subsystems
    private FeederSubsystem feeder;
    private ShooterSubsystem shooter;
    private IntakeSubsystem intake;
    private DrivetrainSubsystem drivetrain;

    // Commands
    private TeleOpDriveCommand teleOpDrive;
    private AutoShootCommand shoot;
    private SwerveControllerCommand followTrajectory;
    private AutoFollowCargoCommand followCargo;

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

        new JoystickButton(driveController,
        Button.kBack.value).whenPressed(drivetrain::resetGyroscope);
        // new JoystickButton(driveController,
        // Button.kStart.value).whenPressed(drivetrain::resetPosition);

        // Button commands to test shooter subsystem.
        /*
        new JoystickButton(driveController, Button.kStart.value)
                .whenPressed(new InstantCommand(() -> shooter.start(), shooter));
        new JoystickButton(driveController, Button.kBack.value)
                .whenPressed(new InstantCommand(() -> shooter.stop(), shooter));
        new JoystickButton(driveController, Button.kLeftBumper.value)
                .whenPressed(new InstantCommand(() -> shooter.decreaseSpeed(), shooter));
        new JoystickButton(driveController, Button.kRightBumper.value)
                .whenPressed(new InstantCommand(() -> shooter.increaseSpeed(), shooter));
        new JoystickButton(driveController, Axis.kLeftTrigger.value)
                .whenPressed(new InstantCommand(() -> shooter.raiseHood(), shooter));
        new JoystickButton(driveController, Axis.kRightTrigger.value)
                .whenPressed(new InstantCommand(() -> shooter.lowerHood(), shooter));
        */


        // new JoystickButton(driveController, Button.kStart.value)
        //        .whenPressed(getFollowTrajectoryCommand());

        new JoystickButton(driveController, Button.kStart.value)
               .whenPressed(new AutoDriveCommand(drivetrain, new Translation2d(2, 0), new Rotation2d()));
        
        //new JoystickButton(driveController, Button.kBack.value)
        //        .whenPressed(getFollowCargoCommand());
    }

    /**
     * The main {@link Robot} class calls this to get the command to run during
     * autonomous.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() {
            return null;
    }

    public Command getFollowTrajectoryCommand() {
        TrajectoryConfig config = new TrajectoryConfig(1.5, 1);
    config.setReversed(true);
    
    Trajectory trajectory = TrajectoryGenerator.generateTrajectory(
        new Pose2d(0, 0, new Rotation2d(0)),                        // Start at the origin facing the +X direction       
        List.of(new Translation2d(1, 0.3)), // Pass through these two interior waypoints, making an 's' curve path
        new Pose2d(2, 0, new Rotation2d(0)),                        // End 3 meters straight ahead of where we started, facing forward
        config);

    double totalTimeSeconds = trajectory.getTotalTimeSeconds();
    System.out.println("-----------------------------> totalTimeSeconds: " + totalTimeSeconds);

    double kpX = 0.00001;  //.01 for both X and Y
    double kpY = 0.00001;
    double kiX = 0.0;
    double kiY = 0.0;
    double kdX = 0.0;
    double kdY = 0.0;
    PIDController xController = new PIDController(kpX, kiX, kdX); 
    PIDController yController = new PIDController(kpY, kiY, kdY); 


    double kPhysicalMaxAngularSpeedRadiansPerSecond = 2 * 2 * Math.PI;
    double kMaxAngularSpeedRadiansPerSecond = kPhysicalMaxAngularSpeedRadiansPerSecond / 10;
    double kMaxAngularAccelerationRadiansPerSecondSquared = Math.PI / 4;

    TrapezoidProfile.Constraints thetaControllerConstraints = new TrapezoidProfile.Constraints(
        kMaxAngularSpeedRadiansPerSecond,
        kMaxAngularAccelerationRadiansPerSecondSquared);

    double kpTheta = 0.0000000000001;   // .04
    double kiTheta = 0.0;
    double kdTheta = 0.0;
    ProfiledPIDController thetaController = new ProfiledPIDController(kpTheta, kiTheta, kdTheta, thetaControllerConstraints); 

    followTrajectory = new SwerveControllerCommand(
      trajectory,
      drivetrain::getPose,
      drivetrain.getKinematics(),
      xController,
      yController,
      thetaController,
      drivetrain::setModuleStates,
      drivetrain);

      return followTrajectory;
    }

    public Command getFollowCargoCommand() {
        followCargo = new AutoFollowCargoCommand(drivetrain, pixy);
        return followCargo;
    }
}
