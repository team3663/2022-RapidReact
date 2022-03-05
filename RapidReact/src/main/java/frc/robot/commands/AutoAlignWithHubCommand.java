// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.LimelightSubsystem;

public class AutoAlignWithHubCommand extends CommandBase {
  private DrivetrainSubsystem drivetrain;
  private LimelightSubsystem limelight;

  // private PIDController rotationPidController = new PIDController(0.05, 0, 0); // original
  private PIDController rotationPidController = new PIDController(0.12, 0, 0);

  private double currentOffset;
  private double speed;

  private DoubleSupplier translationXSupplier;
  private DoubleSupplier translationYSupplier;

  // during tele
  public AutoAlignWithHubCommand(LimelightSubsystem limelight, DrivetrainSubsystem drivetrain,
                                  DoubleSupplier translationXSupplier,
                                  DoubleSupplier translationYSupplier) {
    this.limelight = limelight;
    this.drivetrain = drivetrain;

    this.translationXSupplier = translationXSupplier;
    this.translationYSupplier = translationYSupplier;

    rotationPidController.setSetpoint(0);

    addRequirements(drivetrain);
  }

  @Override
  public void initialize() {
  }

  @Override
  public void execute() {
    currentOffset = limelight.getXOffset();
    speed = rotationPidController.calculate(currentOffset);

    drivetrain.drive(ChassisSpeeds.fromFieldRelativeSpeeds(translationXSupplier.getAsDouble(),
                                                            translationYSupplier.getAsDouble(),
                                                            speed, drivetrain.getGyroscopeRotation()));
  }

  @Override
  public void end(boolean interrupted) {}

  @Override
  public boolean isFinished() {
      return false;

    // return (Math.abs(speed) < 0.01) || (Math.abs(currentOffset) < 0.01); // TODO fix value
  }
}
