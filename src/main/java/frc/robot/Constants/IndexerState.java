package frc.robot.Constants;

/**
 * Represents the operational state of the indexer.
 */
public enum IndexerState {
    /** Indexing towards the shooter. */
    SHOOTER,
    /** Indexing towards the hopper/storage. */
    HOPPER,
    /** Indexer is off. */
    OFF,
    /** Manual slow forward. */
    MANUAL_FORWARD,
    /** Manual slow backward. */
    MANUAL_BACKWARD
}
