package frc.robot.utils;

public class SimpleRanger implements Ranger {

    private final double DISTANCE_0 = 0;
    private final double DISTANCE_1 = 1.285;
    private final double DISTANCE_2 = 2.066;
    private final double DISTANCE_3 = 2.780;
    private final double DISTANCE_4 = 3.435;
    private final double DISTANCE_5 = 4.099;
    private final double DISTANCE_6 = 4.741;

    private final double ANGLE_0 = 85;
    private final double ANGLE_1 = 79;
    private final double ANGLE_2 = 75;
    private final double ANGLE_3 = 70;
    private final double ANGLE_4 = 67;
    private final double ANGLE_5 = 67;
    private final double ANGLE_6 = 67;

    private final double SPEED_0 = 1200; 
    private final double SPEED_1 = 1200; // actual 1225
    private final double SPEED_2 = 1350;
    private final double SPEED_3 = 1500;
    private final double SPEED_4 = 1800;
    private final double SPEED_5 = 1950;
    private final double SPEED_6 = 2200;

    private final int DISTANCE_COLUMN_INDEX = 0;
    private final int ANGLE_COLUMN_INDEX = 1;
    private final int SPEED_COLUMN_INDEX = 2;

    public double[][] KNOWN_DATA = new double[][] {
        {DISTANCE_0, ANGLE_0, SPEED_0},
        {DISTANCE_1, ANGLE_1, SPEED_1},
        {DISTANCE_2, ANGLE_2, SPEED_2},
        {DISTANCE_3, ANGLE_3, SPEED_3},
        {DISTANCE_4, ANGLE_4, SPEED_4},
        {DISTANCE_5, ANGLE_5, SPEED_5},
        {DISTANCE_6, ANGLE_6, SPEED_6},
    };

    private final double ANGLE_LOB = ANGLE_0;
    private final int SPEED_LOB = (int) Math.round(SPEED_0);

    public enum InterpolationMode {
        ANGLE,
        SPEED
    }

    
    public FiringSolution getFiringSolution(double range) {

        // beyond endpoints
        if (range <= DISTANCE_0) {
            return new FiringSolution((int) Math.round(SPEED_0), ANGLE_0);
        }
        if (range >= DISTANCE_6) {
            return new FiringSolution((int) Math.round(SPEED_6), ANGLE_6); 
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

        int speed = (int) Math.round(linearInterpolation(range, distanceHigherBoundIndex, InterpolationMode.SPEED));
        double angle = linearInterpolation(range, distanceHigherBoundIndex, InterpolationMode.ANGLE);

        return new FiringSolution(speed, angle);
    }

    public FiringSolution getFiringSolution(String name)
    {
        int speed = 0;
        double angle = 0;

        if (name.equals("lob")) {
            speed = SPEED_LOB;
            angle = ANGLE_LOB;
        }

        return new FiringSolution(speed, angle);
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
