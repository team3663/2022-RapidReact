package frc.robot.utils;

public interface Ranger {

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

    public FiringSolution getFiringSolution(double distance);
}
