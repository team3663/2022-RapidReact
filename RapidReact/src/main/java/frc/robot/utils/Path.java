package frc.robot.utils;

import java.util.List;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.trajectory.TrapezoidProfile;

/** Add your docs here. */
public class Path {
    private double MAX_VELOCITY = 1.5;
    private double MAX_ACCELERATION = 1;
    private TrajectoryConfig config = new TrajectoryConfig(MAX_VELOCITY, MAX_ACCELERATION).setReversed(true);

    private PIDController xyPidController = new PIDController(0.00001, 0, 0);
    
    private final static double MAX_PHYSICAL_ANGULAR_VELOCITY = 2 * 2 * Math.PI; // radians per second
    private final static double MAX_ANGULAR_VELOCITY = MAX_PHYSICAL_ANGULAR_VELOCITY / 10;
    private final static double MAX_ANGULAR_ACCELERATION = Math.PI / 4;
    private static TrapezoidProfile.Constraints angleControllerConstraints = new TrapezoidProfile.Constraints(MAX_ANGULAR_VELOCITY, MAX_ANGULAR_ACCELERATION);
    private static ProfiledPIDController anglePidController = new ProfiledPIDController(0.0000000000001, 0, 0, angleControllerConstraints);
    
    private Trajectory trajectory;

    private Pose2d start = new Pose2d(0, 0, Rotation2d.fromDegrees(0));
    private List<Translation2d> waypoints = List.of(new Translation2d(0, 0));
    private Pose2d end = new Pose2d(0, 0, Rotation2d.fromDegrees(0));

    public Path(PATH path) {
         if (path.equals(PATH.backOutOfTarmac)) {
            waypoints = List.of(new Translation2d(1, 0));
            end = new Pose2d(2, 0, Rotation2d.fromDegrees(0));
         }
        trajectory = TrajectoryGenerator.generateTrajectory(start, waypoints, end, config);
    }

    public enum PATH {
        backOutOfTarmac
    }

    public Trajectory getTrajectory() {
        return trajectory;
    }

    public PIDController getPidController() {
        return xyPidController;
    }

    public ProfiledPIDController getAnglePidController() {
        return anglePidController;
    }

    

}
