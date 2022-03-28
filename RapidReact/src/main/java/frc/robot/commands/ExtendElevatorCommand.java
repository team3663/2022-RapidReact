package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ClimberSubsystem;

public class ExtendElevatorCommand extends CommandBase {

	private ClimberSubsystem climber;

	public ExtendElevatorCommand(ClimberSubsystem climber) {
		this.climber = climber;

		addRequirements(climber);
	}

	@Override
	public void initialize() {
		climber.elevatorUp();
	}

	@Override
	public void execute() {}

	@Override
	public void end(boolean interrupted) {}

	@Override
	public boolean isFinished() {
		return climber.getElevatorExtended();
	}
}
