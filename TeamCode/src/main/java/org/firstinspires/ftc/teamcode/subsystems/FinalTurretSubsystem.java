package org.firstinspires.ftc.teamcode.subsystems;

import static org.firstinspires.ftc.teamcode.hdware.final_Globals.*;

import com.arcrobotics.ftclib.command.SubsystemBase;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.hdware.FinalHardware;

/*
 * Turret (turret1 + mirrored turret2, 1:1).
 *
 * FIXED : held at TELE_TURRET_POS (teleop default), re-sent whenever the value changes
 * TRACK : holds the field angle TURRET_TARGET_DEG while the robot turns
 *         (turret angle = target - robot heading), clipped to TURRET_MIN/MAX_POS
 */
public class FinalTurretSubsystem extends SubsystemBase {

    private final FinalHardware robot;

    public FinalTurretSubsystem(FinalHardware robot) {
        this.robot = robot;
    }

    public enum TURRET {
        FIXED,
        TRACK
    }

    public TURRET turretState = TURRET.FIXED;
    public double sentPos = Double.NaN;
    private double headingDeg = 0;

    public void updateState(TURRET currentState) {
        this.turretState = currentState;
    }

    /** Robot heading (radians, counter-clockwise +) used for TRACK. Call every loop. */
    public void setHeading(double headingRad) {
        this.headingDeg = Math.toDegrees(headingRad);
    }

    /** Turret angle relative to the robot (deg) that points at TURRET_TARGET_DEG. */
    public double wantedRobotDeg() {
        return AngleUnit.normalizeDegrees(TURRET_TARGET_DEG - headingDeg);
    }

    @Override
    public void periodic() {
        double wanted = turretState == TURRET.TRACK
                ? FinalHardware.turretAngleToPosition(wantedRobotDeg())
                : TELE_TURRET_POS;
        // each servo write is a hub round trip, only send changes
        if (wanted == sentPos) return;
        robot.setTurretPosition(wanted);
        sentPos = wanted;
    }
}
