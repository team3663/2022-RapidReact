package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.drivers.Pixy;
import frc.robot.subsystems.DrivetrainSubsystem;
import io.github.pseudoresonance.pixy2api.Pixy2CCC.Block;

public class AutoFollowCargoCommand extends CommandBase {
  private DrivetrainSubsystem drivetrainSubsystem;
  private Pixy pixy;
  private PIDController translationXPidController = new PIDController(0.001, 0, 0); // tune
  private PIDController rotationPidController = new PIDController(0.025, 0, 0); // tune
  
  private double lastXOffset = 0;

  public AutoFollowCargoCommand(DrivetrainSubsystem drivetrainSubsystem, Pixy pixy) {

    this.drivetrainSubsystem = drivetrainSubsystem;
    addRequirements(drivetrainSubsystem);

    this.pixy = pixy;

    translationXPidController.setSetpoint(2000);
    rotationPidController.setSetpoint(155);
  }

  @Override
  public void initialize() {

    System.out.println("start following cargo");
    drivetrainSubsystem.resetPosition();
    drivetrainSubsystem.resetGyroscope();
    pixy.turnOnLights();
  }

  @Override
  public void execute() {
    Block cargo = pixy.getLargestBlock();
    double cargoArea = pixy.getArea(cargo);

    // if no ball is detected, rotate towards the edge from which the ball disappears
    if (cargo == null) {
      if (lastXOffset < 155){
        drivetrainSubsystem.drive(new ChassisSpeeds(0, 0, 1));
      }
      else {
        drivetrainSubsystem.drive(new ChassisSpeeds(0, 0, -1));
      }
    }
    // rotate to the largest ball seen by the pixy camera
    else {
      double cargoXOffset = cargo.getX();
      lastXOffset = cargoXOffset;

      double rotationSpeed = rotationPidController.calculate(cargoXOffset);
      double translationXSpeed = translationXPidController.calculate(cargoArea);

      drivetrainSubsystem.drive(new ChassisSpeeds(translationXSpeed, 0, rotationSpeed));
    }
  }

  @Override
  public void end(boolean interrupted) {
    drivetrainSubsystem.drive(new ChassisSpeeds(0, 0, 0));
    pixy.turnOffLights();
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}