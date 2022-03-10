package frc.robot.utils;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;

public class ControllerHelper {

    private XboxController controller;

    public ControllerHelper( XboxController controller) {
        this.controller = controller;
    }
    
    public double modifyAxis(double value) {
        value = deadband(value, 0.1);
        value = Math.copySign(value * value, value);

        return value;
    }

    public void rumble(boolean rumble) {
        double intensity  = rumble ? 1.0 : 0.0;
        controller.setRumble(RumbleType.kLeftRumble, intensity);
        controller.setRumble(RumbleType.kLeftRumble, intensity);
    }

    private double deadband(double value, double deadband) {
        if (Math.abs(value) > deadband) {
            if (value > 0.1) {
                return (value - deadband) / (1.0 - deadband);
            } else {
                return (value + deadband) / (1.0 - deadband);
            }
        } else {
            return 0.0;
        }
    }
}
