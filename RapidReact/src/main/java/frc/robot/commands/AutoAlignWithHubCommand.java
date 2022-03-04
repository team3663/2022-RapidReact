// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.LimelightSubsystem;

public class AutoAlignWithHubCommand extends CommandBase {
  private DrivetrainSubsystem drivetrain;
  private LimelightSubsystem limelight;

  private PIDController rotationPidController = new PIDController(1.2, 0, 0); // TODO tune pid

  private double currentOffset;
  private double speed;

  public AutoAlignWithHubCommand(LimelightSubsystem limelight, DrivetrainSubsystem drivetrain) {
    this.limelight = limelight;
    this.drivetrain = drivetrain;

    rotationPidController.setSetpoint(0); // TODO double check if this is zero

    addRequirements(drivetrain, limelight);
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    currentOffset = limelight.getXOffset();
    speed = rotationPidController.calculate(currentOffset);

    drivetrain.drive(new ChassisSpeeds(0, 0, speed));
  }

  @Override
  public void end(boolean interrupted) {
    drivetrain.drive(new ChassisSpeeds(0, 0, 0));
  }

  @Override
  public boolean isFinished() {
    return (Math.abs(speed) < 0.01) || (Math.abs(currentOffset) < 0.01); // TODO fix value
  }
}
