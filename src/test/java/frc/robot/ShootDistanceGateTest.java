package frc.robot;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import frc.robot.Constants.RobotConstants.ShooterSubsystemConstants;
import org.junit.jupiter.api.Test;

/**
 * Tests that the single source of truth for the shoot-distance gate honors the dashboard
 * ShootRangeOffset. This is what makes the dashboard "Set Shoot Range Offset" move both the
 * drive-to-curve target and the shoot gate together (previously the gate ignored the offset, so any
 * nonzero offset drove the robot out of the only zone where it was allowed to feed).
 */
class ShootDistanceGateTest {

    private static final double MID_RANGE = ShooterSubsystemConstants.midRange;
    private static final double TOL = ShooterSubsystemConstants.shootingDistanceToleranceMeters;

    @Test
    void toleranceIsFivePercentOfMidRange() {
        assertEquals(MID_RANGE * 0.05, TOL, 1e-9);
    }

    @Test
    void zeroOffsetMatchesMidRange() {
        assertTrue(ShooterSubsystemConstants.isWithinShootingDistance(MID_RANGE, 0.0, TOL));
    }

    @Test
    void positiveOffsetMovesTheGateOutward() {
        // Robot sitting at midRange + 1.0 must be accepted when the operator sets +1.0 ...
        assertTrue(ShooterSubsystemConstants.isWithinShootingDistance(MID_RANGE + 1.0, 1.0, TOL));
        // ... and must be rejected at midRange (this was the bug: gate ignored the offset).
        assertFalse(ShooterSubsystemConstants.isWithinShootingDistance(MID_RANGE, 1.0, TOL));
    }

    @Test
    void negativeOffsetMovesTheGateInward() {
        assertTrue(ShooterSubsystemConstants.isWithinShootingDistance(MID_RANGE - 0.5, -0.5, TOL));
        assertFalse(ShooterSubsystemConstants.isWithinShootingDistance(MID_RANGE, -0.5, TOL));
    }

    @Test
    void toleranceBoundaryIsInclusive() {
        assertTrue(ShooterSubsystemConstants.isWithinShootingDistance(MID_RANGE + TOL - 1e-9, 0.0, TOL));
        assertFalse(ShooterSubsystemConstants.isWithinShootingDistance(MID_RANGE + TOL + 1e-6, 0.0, TOL));
    }
}
