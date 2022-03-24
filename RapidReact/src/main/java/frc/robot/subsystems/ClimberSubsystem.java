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
     * 
     * Elevator - One Neo  gearbox ???, Winch drum diameter ???
     * Windmill - 2 Neo with 100:1 gearboxes, chain 20-tooth drive gear, 72-tooth driven gear
     * Hooks - Neo 550, ??? gear reduction
     *
     */
    public ClimberSubsystem() {
    }


    /**
     * Set target position of the elevator.
     * 
     * @param position
     */
    public void setElevatorPosition(double position) {

    }


    /**
     * Check to see if the elevator has reached the target position.
     * 
     * @return - True if elevator has reached the target position.
     */
    public boolean getElevatorReady() {
        return false;
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