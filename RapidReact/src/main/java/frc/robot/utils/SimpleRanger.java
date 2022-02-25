package frc.robot.utils;

public class SimpleRanger implements Ranger {

    // TODO collect data
    public final int DISTANCE_0 = 0;
    public final int DISTANCE_1 = 0;
    public final int DISTANCE_2 = 0;
    public final int DISTANCE_3 = 0;
    public final int DISTANCE_4 = 0;

    public final int ANGLE_0 = 0;
    public final int ANGLE_1 = 0;
    public final int ANGLE_2 = 0;
    public final int ANGLE_3 = 0;
    public final int ANGLE_4 = 0;

    public final int SPEED_0 = 0;
    public final int SPEED_1 = 0;
    public final int SPEED_2 = 0;
    public final int SPEED_3 = 0;
    public final int SPEED_4 = 0;

    public final int DISTANCE_COLUMN_INDEX = 0;
    public final int ANGLE_COLUMN_INDEX = 0;
    public final int SPEED_COLUMN_INDEX = 0;

    public int[][] KNOWN_DATA = new int[][] {
        {DISTANCE_0, ANGLE_0, SPEED_0},
        {DISTANCE_1, ANGLE_1, SPEED_1},
        {DISTANCE_2, ANGLE_2, SPEED_2},
        {DISTANCE_3, ANGLE_3, SPEED_3},
        {DISTANCE_4, ANGLE_4, SPEED_4}
    };

    public enum InterpolationMode {
        ANGLE,
        SPEED
    }

    
    public FiringSolution getFiringSolution(double range) {

        // beyond endpoints
        if (range <= DISTANCE_0) {
            return new FiringSolution(SPEED_0, ANGLE_0);
        }
        if (range >= DISTANCE_4) {
            return new FiringSolution(SPEED_4, ANGLE_4);
        }

        // find the two data points to be used
        int distanceHigherBoundIndex = 0;
        for (int i = 0; i < KNOWN_DATA.length; i ++) {
            int distanceReference = KNOWN_DATA[i][DISTANCE_COLUMN_INDEX];
            if (range < distanceReference) {
                distanceHigherBoundIndex = i;
                break;
            }
        }

        int speed = (int) Math.round(linearInterpolation(range, distanceHigherBoundIndex, InterpolationMode.SPEED));
        double angle = linearInterpolation(range, distanceHigherBoundIndex, InterpolationMode.ANGLE);

        return new FiringSolution(speed, angle);
    }

    public double linearInterpolation(double range, int distanceHigherBoundIndex, InterpolationMode mode) {
        
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
