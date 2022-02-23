package frc.robot.utils;

public class SimpleRanger implements Ranger {

    public FiringSolution getFiringSolution(double range) {

        return new FiringSolution(0, 0.0);
    }
}
