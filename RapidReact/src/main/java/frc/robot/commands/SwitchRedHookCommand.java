package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ClimberSubsystem;
import frc.robot.subsystems.ClimberSubsystem.HookState;

public class SwitchRedHookCommand extends CommandBase {

	private ClimberSubsystem climber;
	private HookState hookPosition;

	public SwitchRedHookCommand(ClimberSubsystem climber, HookState hookPosition) {
		this.climber = climber;
		this.hookPosition = hookPosition;

		addRequirements(climber);
	}

	@Override
  	public void initialize() {
    	climber.setRedHookState(hookPosition);
  	}

	@Override
	public void execute() {
	}

	@Override
	public void end(boolean interrupted) {
	}

	@Override
	public boolean isFinished() {
		return climber.getRedHookState() == hookPosition;
	}
}
