// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ClimberSubsystem;

public class RotateWindmillCommand extends CommandBase {

	private ClimberSubsystem climber;
	private double windmillAngle;

	public RotateWindmillCommand(ClimberSubsystem climber, double windmillAngle) {
		this.climber = climber;
		this.windmillAngle = windmillAngle;
		addRequirements(climber);
	}

	@Override
	public void initialize() {
		climber.setWindmillAngle(windmillAngle);
	}

	@Override
	public void execute() {}

	@Override
	public void end(boolean interrupted) {}

	@Override
	public boolean isFinished() {
		return climber.windmillAtTarget();
	}
}
