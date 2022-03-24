package frc.robot.subsystems;

public class ClimberSubsystem {

    /**
     * Defined hook positions.
     * 
     * Grab - Opened to grab the next bar.
     * Release - Opened to release the current bar.
     * Locked - Locked to hold onto a bar.
     */
    public enum HookPosition {
        Grab,
        Release,
        Locked
    }


    /**
     * Initilalize the climber subsystem
     */
    public ClimberSubsystem() {
    }


    /**
     * Set target angle for windmill to rotate to.
     * 
     * @param angle - Target angle.
     */
    public void setWindmillAngle(double angle) {

    }


    /**
     * Check to see if the windwill has reached the current target angle.
     * 
     * @return - True if the windmill has reached the target position.
     */
    public boolean getWindmillReady() {
        return false;
    }


    /**
     * Set the position of the Red climbing hook
     * 
     * @param position
     */
    public void setRedHookPosition(HookPosition position) {

    }


    /**
     * Set the position of the Blue climbing hook
     * 
     * @param position
     */
    public void setBlueHookPosition(HookPosition position) {

    }

}