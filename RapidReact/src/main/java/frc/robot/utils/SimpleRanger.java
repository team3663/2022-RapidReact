package frc.robot.utils;

public class SimpleRanger implements Ranger {

    private final double DISTANCE_LOB = 0;
    private final double ANGLE_LOB = 77;
    private final double SPEED_LOB = 2320;

    private final double DISTANCE_0 = 0.1;
    private final double DISTANCE_1 = 1.535;
    private final double DISTANCE_2 = 2.652;
    private final double DISTANCE_3 = 3.6;

    private final double ANGLE_0 = 77;
    private final double ANGLE_1 = 74;
    private final double ANGLE_2 = 67;
    private final double ANGLE_3 = 67;

    private final double SPEED_0 = 2400;
    private final double SPEED_1 = 2700;
    private final double SPEED_2 = 2900;
    private final double SPEED_3 = 3600;

    private final int DISTANCE_COLUMN_INDEX = 0;
    private final int ANGLE_COLUMN_INDEX = 1;
    private final int SPEED_COLUMN_INDEX = 2;

    public double[][] KNOWN_DATA = new double[][] {
        {DISTANCE_0, ANGLE_0, SPEED_0},
        {DISTANCE_1, ANGLE_1, SPEED_1},
        {DISTANCE_2, ANGLE_2, SPEED_2},
        {DISTANCE_3, ANGLE_3, SPEED_3}
    };

    public double[][] PRESET_DATA = new double[][] {
        {DISTANCE_LOB, ANGLE_LOB, SPEED_LOB},
    };

    public enum InterpolationMode {
        ANGLE,
        SPEED
    }

    
    public FiringSolution getFiringSolution(double range) {

        int speed = 0;
        double angle= 0;

        // preset point
        for (int i = 0; i < PRESET_DATA.length; i ++) {
            if (range == PRESET_DATA[i][DISTANCE_COLUMN_INDEX]) {
                speed = (int) Math.round(PRESET_DATA[i][SPEED_COLUMN_INDEX]);
                angle = PRESET_DATA[i][ANGLE_COLUMN_INDEX];
                return new FiringSolution((int) Math.round(speed), angle);
            }
        }
        
        // beyond endpoints
        if (range <= DISTANCE_0) {
            return new FiringSolution((int) Math.round(SPEED_0), ANGLE_0);
        }
        if (range >= DISTANCE_3) {
            return new FiringSolution((int) Math.round(SPEED_3), ANGLE_3);
        }

        // find the two data points to be used
        int distanceHigherBoundIndex = 0;
        for (int i = 0; i < KNOWN_DATA.length; i ++) {
            double distanceReference = KNOWN_DATA[i][DISTANCE_COLUMN_INDEX];
            if (range < distanceReference) {
                distanceHigherBoundIndex = i;
                break;
            }
        }

        speed = (int) Math.round(linearInterpolation(range, distanceHigherBoundIndex, InterpolationMode.SPEED));
        angle = linearInterpolation(range, distanceHigherBoundIndex, InterpolationMode.ANGLE);

        return new FiringSolution(speed, angle);
    }

    public FiringSolution getFiringSolution(String name)
    {
        return null;
    }
    
    private double linearInterpolation(double range, int distanceHigherBoundIndex, InterpolationMode mode) {
        
        int modeIndex;
        if (mode.equals(InterpolationMode.ANGLE)) {
            modeIndex = ANGLE_COLUMN_INDEX;
        }
        else {
            modeIndex = SPEED_COLUMN_INDEX;
        }

        double x1 = KNOWN_DATA[distanceHigherBoundIndex - 1][DISTANCE_COLUMN_INDEX];
        double x2 = KNOWN_DATA[distanceHigherBoundIndex][DISTANCE_COLUMN_INDEX];
        double y1 = KNOWN_DATA[distanceHigherBoundIndex  - 1][modeIndex];
        double y2 = KNOWN_DATA[distanceHigherBoundIndex][modeIndex];

        double x = range;

        double m = (y2 - y1) / (x2 - x1);
        double y = m * x + y1 - m * x1;
        
        return y;
    }
}
