package frc.robot.Constants;

/**
 * Represents the operational state of the shooter.
 */
public enum ShooterState {
    /** Shooter is running at full speed forward. */
    On,
    /** Shooter is running at reduced speed (unused but available for jam clearing). */
    Rev,
    /** Shooter is off. */
    Off,
    /** Shooter is running backwards. */
    Backwards,
    /** Shooter is running forward at manual slow speed. */
    ManualForward,
    /** Shooter is running backward at manual slow speed. */
    ManualBackward
}
