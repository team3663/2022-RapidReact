package frc.robot.utils;

import org.ejml.simple.SimpleMatrix;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.Num;


public class Y_CubicRanger {

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

        int speed = (int) Math.round(cubicInterpolation(range, distanceHigherBoundIndex, InterpolationMode.SPEED));
        double angle = cubicInterpolation(range, distanceHigherBoundIndex, InterpolationMode.ANGLE);

        return new FiringSolution(speed, angle);
    }

    public double cubicInterpolation(double range, int distanceHigherBoundIndex, InterpolationMode mode) {
        
        int modeIndex;
        if (mode.equals(InterpolationMode.ANGLE)) {
            modeIndex = ANGLE_COLUMN_INDEX;
        }
        else {
            modeIndex = SPEED_COLUMN_INDEX;
        }

        int smallestRowIndex;
        if (distanceHigherBoundIndex < 3) {
            smallestRowIndex = 0;
        }
        else {
            smallestRowIndex = 1;
        }

        double x1 = KNOWN_DATA[smallestRowIndex][DISTANCE_COLUMN_INDEX];
        double x2 = KNOWN_DATA[smallestRowIndex + 1][DISTANCE_COLUMN_INDEX];
        double x3 = KNOWN_DATA[smallestRowIndex + 2][DISTANCE_COLUMN_INDEX];
        double x4 = KNOWN_DATA[smallestRowIndex + 3][DISTANCE_COLUMN_INDEX];
        double y1 = KNOWN_DATA[smallestRowIndex][modeIndex];
        double y2 = KNOWN_DATA[smallestRowIndex + 1][modeIndex];
        double y3 = KNOWN_DATA[smallestRowIndex + 2][modeIndex];
        double y4 = KNOWN_DATA[smallestRowIndex + 3][modeIndex];

        double x = range;

        Matrix m = new Matrix(new SimpleMatrix(
             new double[][] {
                 {Math.pow(x1, 3), Math.pow(x1, 2), Math.pow(x1, 1), Math.pow(x1, 0)},
                 {Math.pow(x2, 3), Math.pow(x2, 2), Math.pow(x2, 1), Math.pow(x2, 0)},
                 {Math.pow(x3, 3), Math.pow(x3, 2), Math.pow(x3, 1), Math.pow(x3, 0)},
                 {Math.pow(x4, 3), Math.pow(x4, 2), Math.pow(x4, 1), Math.pow(x4, 0)},
             }
         ));

        Matrix n = new Matrix(new SimpleMatrix(
             new double[][] {
                 {y1},
                 {y2},
                 {y3},
                 {y4},
             }
         ));

        Matrix result = m.inv().times(n);

        double a = result.get(0, 0);
        double b = result.get(1, 0);
        double c = result.get(2, 0);
        double d = result.get(3, 0);

        double y = a * Math.pow(x, 3) + b * Math.pow(x, 2) + c * Math.pow(x, 1) + d * Math.pow(x, 0);

        return y;
    }
}
