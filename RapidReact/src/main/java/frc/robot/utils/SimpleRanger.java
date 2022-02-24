package frc.robot.utils;

public class SimpleRanger implements Ranger {

    // TODO collect data
    public final int DISTANCE_1 = 0;
    public final int DISTANCE_2 = 0;
    public final int DISTANCE_3 = 0;
    public final int DISTANCE_4 = 0;
    public final int DISTANCE_5 = 0;

    public final int HOOD_ANGLE_1 = 0;
    public final int HOOD_ANGLE_2 = 0;
    public final int HOOD_ANGLE_3 = 0;
    public final int HOOD_ANGLE_4 = 0;
    public final int HOOD_ANGLE_5 = 0;

    public final int RPM_1 = 0;
    public final int RPM_2 = 0;
    public final int RPM_3 = 0;
    public final int RPM_4 = 0;
    public final int RPM_5 = 0;

    public final int DISTANCE_COLUMN_INDEX = 0;
    public final int HOOD_ANGLE_COLUMN_INDEX = 0;
    public final int RPM_COLUMN_INDEX = 0;

    public int[][] KNOWN_DATA = new int[][] {
        {DISTANCE_1, HOOD_ANGLE_1, RPM_1},
        {DISTANCE_2, HOOD_ANGLE_2, RPM_2},
        {DISTANCE_3, HOOD_ANGLE_3, RPM_3},
        {DISTANCE_4, HOOD_ANGLE_4, RPM_4},
        {DISTANCE_5, HOOD_ANGLE_5, RPM_5},
    };

    public enum InterpolationMode {
        HOOD_ANGLE,
        RPM
    }

    
    public FiringSolution getFiringSolution(double range) {

        // beyond endpoints
        if (range <= DISTANCE_1) {
            return new FiringSolution(RPM_1, HOOD_ANGLE_1);
        }
        if (range >= DISTANCE_5) {
            return new FiringSolution(RPM_5, HOOD_ANGLE_5);
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

        int rpm = (int) Math.round(linearInterpolation(range,distanceHigherBoundIndex, InterpolationMode.RPM));
        double hoodAngle = linearInterpolation(range, distanceHigherBoundIndex, InterpolationMode.HOOD_ANGLE);

        return new FiringSolution(rpm, hoodAngle);
    }

    public double linearInterpolation(double range, int distanceHigherBoundIndex, InterpolationMode mode) {
        
        int MODE_INDEX;
    
        if (mode.equals(InterpolationMode.HOOD_ANGLE)) {
            MODE_INDEX = HOOD_ANGLE_COLUMN_INDEX;
        }
        else {
            MODE_INDEX = RPM_COLUMN_INDEX;
        }

        double x1 = KNOWN_DATA[distanceHigherBoundIndex - 1][DISTANCE_COLUMN_INDEX];
        double x2 = KNOWN_DATA[distanceHigherBoundIndex][DISTANCE_COLUMN_INDEX];
        double y1 = KNOWN_DATA[distanceHigherBoundIndex - 1][MODE_INDEX];
        double y2 = KNOWN_DATA[distanceHigherBoundIndex][MODE_INDEX];

        double x = range;

        double m = (y2 - y1) / (x2 - x1);
        double y = m * x + y1 - m * x1;
        
        return y;
    }
}
