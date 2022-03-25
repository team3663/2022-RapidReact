// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ClimberSubsystem;
import frc.robot.subsystems.ClimberSubsystem.HookPosition;

public class ExtendElevatorCommand extends CommandBase {

	private ClimberSubsystem climber;
	private double elevatorPosition;

	public ExtendElevatorCommand(ClimberSubsystem climber, double elevatorPosition) {
		this.climber = climber;
		this.elevatorPosition = elevatorPosition;

		addRequirements(climber);
	}

	@Override
	public void initialize() {
	}

	@Override
	public void execute() {
		climber.setRedHookPosition(HookPosition.Grab);
		climber.setElevatorPosition(elevatorPosition);
	}

	@Override
	public void end(boolean interrupted) {
	}

	@Override
	public boolean isFinished() {
		return climber.elevatorAtTarget();
	}
}
