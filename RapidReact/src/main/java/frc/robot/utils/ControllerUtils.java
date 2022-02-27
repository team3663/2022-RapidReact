package frc.robot.utils;

public class ControllerUtils {
    private static final double DEADBAND = 0.05;

    /**
     * 
     * @param value
     * @return options: linear, squared, cubed, squareRoot, cubicRoot
     */
    public static double modifyAxis(double value) {
        return squared(deadband(value, DEADBAND));
    }

    /**
     * equal sensitivity across the entire stick range
     * @param value
     * @return
     */
    private static double linear(double value) {
        return value;
    }

    /**
     * Square: less sensitive near the center
     * @param value
     * @return
     */
    private static double squared(double value) {
        return value * value;
    }

    /**
     * Cubed: much more sensitive near the outer range of the stick
     * @param value
     * @return
     */
    private static double cubed(double value) {
        return value * value * value;
    }

    /**
     * SquareRoot: more sensitive near the center of the stick
     * @param value
     * @return
     */
    private static double squareRoot(double value) {
        return  Math.sqrt(value);
    }

    /**
     * CubicRoot: much more sensitive near the outer range of the stick
     * @param value
     * @return
     */
    private static double cubicRoot(double value) {
        return Math.cbrt(value);
    }

    /**
     * return 0.0 if the value is inside deadband:  -deadband < value < +deadband
     * otherwise, value is returned unchanged
     * @param value
     * @param deadband
     * @return
     */
    private static double deadband(double value, double deadband) {
        if (Math.abs(value) < deadband) {
            value = 0.0;
        }
        return value;
    }
}
