package frc.robot.utils;

public class SimpleRanger implements Ranger {

    // TO DO retune
    private final int DISTANCE_0 = 0;
    private final int DISTANCE_1 = 5;
    private final int DISTANCE_2 = 10;
    private final int DISTANCE_3 = 15;

    private final int ANGLE_0 = 77;
    private final int ANGLE_1 = 74;
    private final int ANGLE_2 = 67;
    private final int ANGLE_3 = 67;

    private final int SPEED_0 = 2400;
    private final int SPEED_1 = 2700;
    private final int SPEED_2 = 2900;
    private final int SPEED_3 = 3600;


    private final int DISTANCE_COLUMN_INDEX = 0;
    private final int ANGLE_COLUMN_INDEX = 1;
    private final int SPEED_COLUMN_INDEX = 2;

    public int[][] KNOWN_DATA = new int[][] {
        {DISTANCE_0, ANGLE_0, SPEED_0},
        {DISTANCE_1, ANGLE_1, SPEED_1},
        {DISTANCE_2, ANGLE_2, SPEED_2},
        {DISTANCE_3, ANGLE_3, SPEED_3}
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
        if (range >= DISTANCE_3) {
            return new FiringSolution(SPEED_3, ANGLE_3);
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
