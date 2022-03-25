package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ClimberSubsystem;

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
		climber.setElevatorPosition(elevatorPosition);
	}

	@Override
	public void execute() {}

	@Override
	public void end(boolean interrupted) {}

	@Override
	public boolean isFinished() {
		return climber.elevatorAtTarget();
	}
}
