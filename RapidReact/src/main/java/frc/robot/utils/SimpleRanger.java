package frc.robot.utils;

public class SimpleRanger implements Ranger {

    
    public FiringSolution getFiringSolution(double distance) {

        // beyond endpoints
        if (distance <= DISTANCE_1) {
            return new FiringSolution(RPM_1, HOOD_ANGLE_1);
        }
        if (distance >= DISTANCE_5) {
            return new FiringSolution(RPM_5, HOOD_ANGLE_5);
        }

        // find the two data points to be used
        int distanceHigherBoundIndex = 0;
        for (int i = 0; i < KNOWN_DATA.length; i ++) {
            int distanceReference = KNOWN_DATA[i][DISTANCE_COLUMN_INDEX];
            if (distance < distanceReference) {
                distanceHigherBoundIndex = i;
                break;
            }
        }

        int rpm = (int) Math.round(linearInterpolation(distance,distanceHigherBoundIndex, InterpolationMode.RPM));
        double hoodAngle = linearInterpolation(distance, distanceHigherBoundIndex, InterpolationMode.HOOD_ANGLE);

        return new FiringSolution(rpm, hoodAngle);
    }

    public double linearInterpolation(double distance, int distanceHigherBoundIndex, InterpolationMode mode) {
        
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

        double x = distance;

        double m = (y2 - y1) / (x2 - x1);
        double y = m * x + y1 - m * x1;
        
        return y;
    }
}
