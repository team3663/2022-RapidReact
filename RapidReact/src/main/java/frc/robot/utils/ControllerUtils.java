package frc.robot.utils;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;

public class ControllerUtils {

    private static XboxController drive;
    private static XboxController operator;

    public static double modifyAxis(double value) {
        value = deadband(value, 0.1);
        value = Math.copySign(value * value, value);

        return value;
    }

    public static void setControllers(XboxController drive, XboxController operator)
    {
        ControllerUtils.drive = drive;
        ControllerUtils.operator = operator;
    }

    public static void DriverRumble(boolean rumble) {
        double intensity  = rumble ? 1.0 : 0.0;
        drive.setRumble(RumbleType.kLeftRumble, intensity);
        drive.setRumble(RumbleType.kLeftRumble, intensity);
    }

    public static void OperatorRumble(boolean rumble) {
        double intensity  = rumble ? 1.0 : 0.0;
        operator.setRumble(RumbleType.kLeftRumble, intensity);
        operator.setRumble(RumbleType.kLeftRumble, intensity);
    }

    private static double deadband(double value, double deadband) {
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
