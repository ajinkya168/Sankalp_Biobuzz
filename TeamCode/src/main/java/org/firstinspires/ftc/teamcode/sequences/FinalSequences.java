package org.firstinspires.ftc.teamcode.sequences;

import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.InstantCommand;

import org.firstinspires.ftc.teamcode.subsystems.FinalIntakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.FinalIntakeSubsystem.INTAKE;
import org.firstinspires.ftc.teamcode.subsystems.FinalSensorThread;
import org.firstinspires.ftc.teamcode.subsystems.FinalShooterSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.FinalShooterSubsystem.FLAP;
import org.firstinspires.ftc.teamcode.subsystems.FinalShooterSubsystem.MODE;
import org.firstinspires.ftc.teamcode.subsystems.FinalShooterSubsystem.SERVO2;
import org.firstinspires.ftc.teamcode.subsystems.FinalShooterSubsystem.SHOOTER;

/*
 * Button actions for FinalCommandTeleOp (and FinalVelocityGateTeleOp).
 *
 * Each method returns ONE command = what happens when that button is pressed.
 * They are all InstantCommands: they only change subsystem states, and the
 * subsystems' periodic() turn those states into motor powers every loop.
 *
 *   initSequence   - first loop after START
 *   intakeToggle   - A
 *   shooterToggle  - B
 *   modeToggle     - BACK
 *   shoot          - LB (press again, or wait SHOOT_FLAP_CLOSE_S, to stop)
 *   stopShooting   - end of an LB shot (LB again or the auto close timer)
 *   intakeStop     - dpad left
 *   fullSpeed      - dpad right
 *   servo2Open/Close - dpad down / up
 */
public class FinalSequences {

    // =========================== START ===========================

    /** Start: shooter off, POLLEN, flap + servo2 closed, intake off. */
    public static Command initSequence(FinalIntakeSubsystem intakeSS, FinalShooterSubsystem shooterSS) {
        return new InstantCommand(() -> {
            shooterSS.updateState(SHOOTER.OFF);
            shooterSS.updateState(FLAP.CLOSE);
            shooterSS.updateState(SERVO2.CLOSE);
            shooterSS.forceMode(MODE.POLLEN);
            intakeSS.updateState(INTAKE.OFF);
        });
    }

    // =========================== INTAKE ===========================

    /**
     * A - intake + feeder on / off.
     * Turning ON: pollen count -> 0, any shot ends, flap closes,
     * then NECTAR if a nectar is already inside, otherwise POLLEN.
     */
    public static Command intakeToggle(FinalIntakeSubsystem intakeSS, FinalShooterSubsystem shooterSS) {
        return new InstantCommand(() -> {
            if (intakeSS.isRunning()) {          // running -> just turn it off
                intakeSS.updateState(INTAKE.OFF);
                return;
            }
            intakeSS.resetCount();
            intakeSS.stopShooting();
            shooterSS.updateState(FLAP.CLOSE);
            if (intakeSS.nectarInside()) shooterSS.setMode(MODE.NECTAR, true);
            else shooterSS.setMode(MODE.POLLEN, false);
            intakeSS.updateState(INTAKE.ON);
        });
    }

    /** Dpad left - stop intake + feeder. Closes the flap if full speed had opened it. */
    public static Command intakeStop(FinalIntakeSubsystem intakeSS, FinalShooterSubsystem shooterSS) {
        return new InstantCommand(() -> {
            if (intakeSS.intakeState == INTAKE.FULL_SPEED) shooterSS.updateState(FLAP.CLOSE);
            intakeSS.updateState(INTAKE.OFF);
        });
    }

    /** Dpad right - full speed feed with the flap open (only when stopped and not shooting). */
    public static Command fullSpeed(FinalIntakeSubsystem intakeSS, FinalShooterSubsystem shooterSS) {
        return new InstantCommand(() -> {
            if (intakeSS.isRunning() || intakeSS.shooting) return;
            shooterSS.updateState(FLAP.OPEN);
            intakeSS.updateState(INTAKE.FULL_SPEED);
        });
    }

    // =========================== SHOOTER ===========================

    /** B - shooter on / off. Always goes back to POLLEN and ends an LB shot. */
    public static Command shooterToggle(FinalIntakeSubsystem intakeSS, FinalShooterSubsystem shooterSS) {
        return new InstantCommand(() -> {
            shooterSS.toggleShooter();
            intakeSS.stopShooting();
            shooterSS.setMode(MODE.POLLEN, false);
        });
    }

    /** BACK - manual POLLEN <-> NECTAR. */
    public static Command modeToggle(FinalShooterSubsystem shooterSS) {
        return new InstantCommand(shooterSS::toggleMode);
    }

    // =========================== SHOOTING (LB) ===========================

    /**
     * LB - start a shot, or stop it if one is running.
     * Start: pollen count -> 0, shooter on, NECTAR if a nectar is on the switches,
     * flap OPEN, then FinalIntakeSubsystem does the reverse -> wait -> feed.
     * The teleop ends the shot by itself after SHOOT_FLAP_CLOSE_S.
     */
    public static Command shoot(FinalIntakeSubsystem intakeSS, FinalShooterSubsystem shooterSS,
                                FinalSensorThread sensors) {
        return new InstantCommand(() -> {
            if (intakeSS.shooting) {             // second press = stop
                endShot(intakeSS, shooterSS);
                return;
            }
            intakeSS.resetCount();
            if (!shooterSS.isOn()) shooterSS.updateState(SHOOTER.ON);
            if (sensors.nectarAtSwitch) shooterSS.setMode(MODE.NECTAR, true);
            intakeSS.startShooting();
            shooterSS.updateState(FLAP.OPEN);
        });
    }

    /** End an LB shot: stop feeding + close the flap. The shooter keeps spinning. */
    public static Command stopShooting(FinalIntakeSubsystem intakeSS, FinalShooterSubsystem shooterSS) {
        return new InstantCommand(() -> endShot(intakeSS, shooterSS));
    }

    private static void endShot(FinalIntakeSubsystem intakeSS, FinalShooterSubsystem shooterSS) {
        intakeSS.stopShooting();
        shooterSS.updateState(FLAP.CLOSE);
    }

    // =========================== SERVO2 ===========================

    /** Dpad down. */
    public static Command servo2Open(FinalShooterSubsystem shooterSS) {
        return shooterSS.servo2Command(SERVO2.OPEN);
    }

    /** Dpad up. */
    public static Command servo2Close(FinalShooterSubsystem shooterSS) {
        return shooterSS.servo2Command(SERVO2.CLOSE);
    }
}
