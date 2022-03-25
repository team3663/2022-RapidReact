// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ClimberSubsystem;
import frc.robot.subsystems.ClimberSubsystem.HookPosition;

public class ClimbCommand extends CommandBase {

	private ClimberSubsystem climber;
	private double windmillAngle;
	private boolean redHookFirst;

	public ClimbCommand(ClimberSubsystem climber, double windmillAngle, boolean redHookFirst) {

    this.climber = climber;
    this.windmillAngle = windmillAngle;
    
    addRequirements(climber);
	}

	@Override
	public void initialize() {
	}

	@Override
	public void execute() {
		if (redHookFirst) {
			climber.setRedHookPosition(HookPosition.Locked);
			if (!redHookAtTarget) {
				return;
			}
		}
		else {
			climber.setBlueHookPosition(HookPosition.Locked);
		}
	}

	// Called once the command ends or is interrupted.
	@Override
	public void end(boolean interrupted) {
	}

	// Returns true when the command should end.
	@Override
	public boolean isFinished() {
		return false;
	}
}
