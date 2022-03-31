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

	private PIDController tController = new PIDController(0.055, 0, 0.005);

	private double currentOffset;
	private double speed;

	private DoubleSupplier translationXSupplier;
	private DoubleSupplier translationYSupplier;
	
    private double staticConst = .34;

	private boolean auto;

	// during tele
	public AutoAlignWithHubCommand(LimelightSubsystem limelight, DrivetrainSubsystem drivetrain,
			DoubleSupplier translationXSupplier,
			DoubleSupplier translationYSupplier) {
		this.limelight = limelight;
		this.drivetrain = drivetrain;

		this.translationXSupplier = translationXSupplier;
		this.translationYSupplier = translationYSupplier;

		tController.setSetpoint(0);

		addRequirements(drivetrain);
	}

	// during auto
	public AutoAlignWithHubCommand(LimelightSubsystem limelight, DrivetrainSubsystem drivetrain) {
		this.limelight = limelight;
		this.drivetrain = drivetrain;

		this.translationXSupplier = () -> 0;
		this.translationYSupplier = () -> 0;

		tController.setSetpoint(0);
		tController.setTolerance(3);

		addRequirements(drivetrain);
	}

	@Override
	public void initialize() {
	}

	@Override
	public void execute() {
		currentOffset = limelight.getXOffset();
		speed = tController.calculate(currentOffset);

		drivetrain.drive(ChassisSpeeds.fromFieldRelativeSpeeds(translationXSupplier.getAsDouble(),
				translationYSupplier.getAsDouble(),
				speed + Math.copySign(staticConst, tController.getPositionError()),
				drivetrain.getPose().getRotation()));
	}

	@Override
	public void end(boolean interrupted) {
	}

	@Override
  public boolean isFinished() {
	  if (auto && tController.atSetpoint()) {
		  return true;
	  }
      return false;
  }
}
