package frc.robot.subsystems.intake;

/**
 * IO interface for the intake subsystem
 */
public interface IntakeIO {
    /**
     * Sets the power applied to the intake motor
     * 
     * @param intakePower Power to apply to the intake motor [-1, 1]
     */
    void setPower(double intakePower);

    /**
     * Sets the intake position
     * 
     * @param desiredPosition The position the intake should move to
     */
    void setPosition(boolean desiredPosition);

    /**
     * Gets the current position of the intake
     */
    boolean currentPosition();

}
