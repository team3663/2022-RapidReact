// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.utils;

import org.frcteam2910.common.control.SplinePathBuilder;
import org.frcteam2910.common.control.Trajectory;
import org.frcteam2910.common.control.TrajectoryConstraint;
import org.frcteam2910.common.control.CentripetalAccelerationConstraint;
import org.frcteam2910.common.control.MaxAccelerationConstraint;
import org.frcteam2910.common.control.MaxVelocityConstraint;
import org.frcteam2910.common.control.SimplePathBuilder;
import org.frcteam2910.common.math.Rotation2;
import org.frcteam2910.common.math.Vector2;

/** Add your docs here. */
public class TrajectoryFactory {

    private static double sampleDistance = 0.00254;

    public static TrajectoryConstraint[] fast = {
        new MaxAccelerationConstraint(3.5),
        new MaxVelocityConstraint(7),
        new CentripetalAccelerationConstraint(5.0)
      };

    public static TrajectoryConstraint[] slow = {
          new MaxAccelerationConstraint(3.5),
          new MaxVelocityConstraint(7),
          new CentripetalAccelerationConstraint(5.0)
        };

    public static Trajectory tune = new Trajectory(
        new SimplePathBuilder(new Vector2(0, 0), Rotation2.ZERO).lineTo(new Vector2(1, 0), Rotation2.fromDegrees(180)).build(),
        fast,
        sampleDistance);

    public static Trajectory start_ball2_ball3 = new Trajectory(
            new SplinePathBuilder(new Vector2(-.5, -2), new Rotation2(-.6, -3.5, true), Rotation2.fromDegrees(-90))
                .hermite(new Vector2(-.6, -3.5), new Rotation2(-.6, -3.5, true), Rotation2.fromDegrees(-90))
                .hermite(new Vector2(-2, -2), new Rotation2(-3.5, -2.2, true), Rotation2.fromDegrees(163.8720703125)) 
                .hermite(new Vector2(-3.5, -2.2), new Rotation2(-2, -2, true), Rotation2.fromDegrees(-114.96093749999997)) 
                .build(),
            fast,
            sampleDistance);

    public static Trajectory ball2_ball3 = new Trajectory(
        new SplinePathBuilder(new Vector2(-.6, -3.5), new Rotation2(-2, -2, true), Rotation2.fromDegrees(-90)) 
            .hermite(new Vector2(-2, -2), new Rotation2(-3.5, -2.2, true), Rotation2.fromDegrees(163.8720703125)) 
            .hermite(new Vector2(-3.5, -2.2), new Rotation2(-2, -2, true), Rotation2.fromDegrees(-114.96093749999997)) 
            .build(),
        fast,
        sampleDistance);

    public static Trajectory ball3_station_shoot = new Trajectory(
        new SplinePathBuilder(new Vector2(-3.5, -2.2), new Rotation2(-3.5, -2.2, true), Rotation2.fromDegrees(-114.96093749999997))
            .hermite(new Vector2(-7, -3), new Rotation2(-7, -3, true),Rotation2.fromDegrees(-147.216796875))
            .hermite(new Vector2(-4.5, -3), Rotation2.ZERO, Rotation2.fromDegrees(-114.96093749999997))
            .build(),
        slow,
        sampleDistance);
}
