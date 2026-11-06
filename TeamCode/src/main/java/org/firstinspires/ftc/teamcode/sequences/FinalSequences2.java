package org.firstinspires.ftc.teamcode.sequences;

import static com.pedropathing.ivy.commands.Commands.conditional;
import static com.pedropathing.ivy.commands.Commands.instant;
import static com.pedropathing.ivy.groups.Groups.parallel;

import com.pedropathing.ivy.Command;

import org.firstinspires.ftc.teamcode.subsystems.FinalIntake2Subsystem;
import org.firstinspires.ftc.teamcode.subsystems.FinalIntake2Subsystem.INTAKE;
import org.firstinspires.ftc.teamcode.subsystems.FinalIntake2Subsystem.SHOT;
import org.firstinspires.ftc.teamcode.subsystems.FinalSensorThread2;
import org.firstinspires.ftc.teamcode.subsystems.FinalShooter2Subsystem;
import org.firstinspires.ftc.teamcode.subsystems.FinalShooter2Subsystem.FLAP;
import org.firstinspires.ftc.teamcode.subsystems.FinalShooter2Subsystem.MODE;
import org.firstinspires.ftc.teamcode.subsystems.FinalShooter2Subsystem.SERVO2;
import org.firstinspires.ftc.teamcode.subsystems.FinalShooter2Subsystem.SHOOTER;

/*
 * ============================== FINAL SEQUENCES 2 (Ivy) ==============================
 *
 * Button actions for FinalCommandTeleOp2, built only from the subsystems' state commands.
 *
 *   parallel(a, b, c)            do a, b and c together, right now (same loop)
 *   conditional(test, yes, no)   checks test when the button is pressed, then does yes or no
 *
 * Every state command changes only its own part, so the order inside parallel() doesn't matter.
 */
public class FinalSequences2 {

    // =========================== START ===========================

    /** First loop after START: shooter off, POLLEN, flap + servo2 closed, intake off. */
    public static Command initSequence(FinalIntake2Subsystem intakeSS, FinalShooter2Subsystem shooterSS) {
        return parallel(
                shooterSS.shooterCommand(SHOOTER.OFF),
                shooterSS.flapCommand(FLAP.CLOSE),
                shooterSS.servo2Command(SERVO2.CLOSE),
                shooterSS.forceModeCommand(MODE.POLLEN),
                intakeSS.intakeCommand(INTAKE.OFF),
                intakeSS.shotCommand(SHOT.IDLE)
        );
    }

    // =========================== INTAKE ===========================

    /**
     * A - intake on / off.
     * ON: pollen count -> 0, any shot ends, flap CLOSED, NECTAR if one is inside else POLLEN.
     */
    public static Command intakeToggle(FinalIntake2Subsystem intakeSS, FinalShooter2Subsystem shooterSS) {
        return conditional(intakeSS::isRunning,
                intakeSS.intakeCommand(INTAKE.OFF),                       // running -> off
                intakeStart(intakeSS, shooterSS)                           // stopped -> start
        );
    }

    /** Intake ON (A when stopped, also used by autonomous): count -> 0, shot ends, flap CLOSED, mode from the piece inside. */
    public static Command intakeStart(FinalIntake2Subsystem intakeSS, FinalShooter2Subsystem shooterSS) {
        return parallel(
                intakeSS.resetCountCommand(),
                intakeSS.shotCommand(SHOT.IDLE),
                shooterSS.flapCommand(FLAP.CLOSE),
                conditional(intakeSS::nectarInside,
                        shooterSS.modeCommand(MODE.NECTAR, true),
                        shooterSS.modeCommand(MODE.POLLEN, false)),
                intakeSS.intakeCommand(INTAKE.ON)
        );
    }

    /** Dpad left - stop intake + feeder. Closes the flap if full speed had opened it. */
    public static Command intakeStop(FinalIntake2Subsystem intakeSS, FinalShooter2Subsystem shooterSS) {
        return conditional(() -> intakeSS.intakeState == INTAKE.FULL_SPEED,
                parallel(
                        shooterSS.flapCommand(FLAP.CLOSE),
                        intakeSS.intakeCommand(INTAKE.OFF)
                ),
                intakeSS.intakeCommand(INTAKE.OFF)
        );
    }

    /** Dpad right - full speed feed with the flap open (only when stopped and not shooting). */
    public static Command fullSpeed(FinalIntake2Subsystem intakeSS, FinalShooter2Subsystem shooterSS) {
        return conditional(() -> intakeSS.isRunning() || intakeSS.isShooting(),
                nothing(),
                parallel(
                        shooterSS.flapCommand(FLAP.OPEN),
                        intakeSS.intakeCommand(INTAKE.FULL_SPEED)
                )
        );
    }

    // =========================== SHOOTER ===========================

    /** B - shooter on / off. Always back to POLLEN, intake off, ends an LB shot. */
    public static Command shooterToggle(FinalIntake2Subsystem intakeSS, FinalShooter2Subsystem shooterSS) {
        return parallel(
                shooterSS.toggleShooterCommand(),
                intakeSS.shotCommand(SHOT.IDLE),
                intakeSS.intakeCommand(INTAKE.OFF),
                shooterSS.modeCommand(MODE.POLLEN, false)
        );
    }

    /** BACK - POLLEN <-> NECTAR. */
    public static Command modeToggle(FinalShooter2Subsystem shooterSS) {
        return shooterSS.toggleModeCommand();
    }

    // =========================== SHOOTING (LB) ===========================

    /**
     * LB - start a shot, or stop it if one is running.
     * Start: pollen count -> 0, shooter ON, NECTAR if a nectar is on the switches, flap OPEN,
     * SHOT = KICKBACK. FinalIntake2Subsystem runs the rest and closes the flap
     * SHOOT_FLAP_CLOSE_S later by itself.
     */
    public static Command shoot(FinalIntake2Subsystem intakeSS, FinalShooter2Subsystem shooterSS,
                                FinalSensorThread2 sensors) {
        return conditional(intakeSS::isShooting,
                stopShooting(intakeSS, shooterSS),                        // shooting -> stop
                shootStart(intakeSS, shooterSS, sensors)                   // not shooting -> start
        );
    }

    /** Start a shot (LB when not shooting, also used by autonomous): shooter ON, NECTAR if one is on the switches, flap OPEN, KICKBACK. */
    public static Command shootStart(FinalIntake2Subsystem intakeSS, FinalShooter2Subsystem shooterSS,
                                     FinalSensorThread2 sensors) {
        return parallel(
                intakeSS.resetCountCommand(),
                shooterSS.shooterCommand(SHOOTER.ON),
                conditional(() -> sensors.nectarAtSwitch,
                        shooterSS.modeCommand(MODE.NECTAR, true),
                        nothing()),
                intakeSS.shotCommand(SHOT.KICKBACK),
                shooterSS.flapCommand(FLAP.OPEN)
        );
    }

    /** End an LB shot: feeding stops, intake off, flap CLOSED. The shooter keeps spinning. */
    public static Command stopShooting(FinalIntake2Subsystem intakeSS, FinalShooter2Subsystem shooterSS) {
        return parallel(
                intakeSS.shotCommand(SHOT.IDLE),
                intakeSS.intakeCommand(INTAKE.OFF),
                shooterSS.flapCommand(FLAP.CLOSE)
        );
    }

    // =========================== HELPERS ===========================

    /** Does nothing - the "else" side of a conditional. */
    private static Command nothing() {
        return instant(() -> { });
    }
}
