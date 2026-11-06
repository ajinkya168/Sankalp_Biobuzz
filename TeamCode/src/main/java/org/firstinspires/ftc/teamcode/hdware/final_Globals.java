package org.firstinspires.ftc.teamcode.hdware;

import com.acmerobotics.dashboard.config.Config;

/*
 * All tunable values for the Final robot (FinalHardware), shared by FinalCommandTeleOp,
 * the Final subsystems and sequences. Change a value here, or live on FTC Dashboard
 * under "final_Globals".
 *
 * Variable names are the same as sankalp27_Globals so code moves between the two
 * robots without renaming. Sections follow the device order in FinalHardware.
 *
 * Where each value is used:
 *   FinalCommandTeleOp   - DRIVE, INTAKE, SHOOTER gains, HOOD/ADJUST *_POLLEN/NECTAR_POS, FLAP,
 *                          TELE_TURRET_POS, SENSORS, POLLEN COUNT, SHOOTING TELEOP
 *   FINAL: Shooter Test  - SHOOTER_*_VELOCITY + gains, HOOD_* / ADJUST_* steps (get values)
 *   FINAL: Turret Test   - TURRET_*
 *
 * Drive motors and the navX are tuned in pedro/Constants (Pedro follower), not here.
 */
@Config
public class final_Globals {

    //TODO ==================== DRIVE (Pedro follower) ====================
    public static double StrafeSpeed = 1;                 // left stick x (strafe) multiplier
    public static double turnSpeed = 0.6;                 // right stick (turn) multiplier

    //TODO ==================== INTAKE (E motor 0) + FEEDER (E motor 1) ====================
    // intake pulls in with NEGATIVE power (same as sankalp27)
    public static double INTAKE_POWER = 0.9;              // A: intake ON
    public static double FEEDER_POWER = 0.3;              // A: feeder ON
    public static double REVERSE_POWER = 0.6;             // X hold: intake + feeder reverse
    public static double FULL_SPEED_POWER = 1.0;          // dpad right: full-speed feed

    //TODO ==================== SHOOTER (shooter1 + shooter2, mounted opposite) ====================
    // directions / enco ders
    public static boolean SHOOTER1_REVERSED = false;       // one motor is reversed so the same power spins both the right way
    public static boolean SHOOTER2_REVERSED = true;
    public static boolean SHOOTER1_ENCODER_REVERSED = true;  // flip if shooter1 vel reads negative while the flywheel spins the right way
    public static boolean SHOOTER2_ENCODER_REVERSED = true;  // flip if shooter2 vel reads negative while the flywheel spins the right way
    public static boolean SHOOTER_USE_BOTH_ENCODERS = true;  // velocity = average of both; false = shooter1 encoder only

    // velocity PIDF (ticks/s): power = kP*err + kI*integral(err) + kD*d(err)/dt + kF*target
    // gains are used by both the teleop and the Shooter Test, *_VELOCITY only by the Shooter Test
    // (teleop velocities are TELE_*_VELOCITY under SHOOTING TELEOP)
    // POLLEN
    public static double SHOOTER_POLLEN_VELOCITY = 1100;  // ticks/s
    public static double POLLEN_kP = 0.001;
    public static double POLLEN_kI = 0.0;
    public static double POLLEN_kD = 0.0;
    public static double POLLEN_kF = 0.000383;             // ~1 / (max ticks/s at full power)
    // NECTAR
    public static double SHOOTER_NECTAR_VELOCITY = 1150;  // ticks/s
    public static double NECTAR_kP = 0.001;
    public static double NECTAR_kI = 0.0;
    public static double NECTAR_kD = 0.0;
    public static double NECTAR_kF = 0.00038;
    // shooter2's own kP / kF for OUR PIDF (FinalShooterPIDFSubsystem) - POLLEN_k* / NECTAR_k* above are shooter1,
    // kI / kD stay shared per mode
    public static double POLLEN2_kP = 0.001;
    public static double POLLEN2_kF = 0.000383;
    public static double NECTAR2_kP = 0.001;
    public static double NECTAR2_kF = 0.00038;

    // common
    public static double SHOOTER_I_MAX = 0.2;             // integral term never adds more than this much power
    public static double SHOOTER_READY_TOL = 50;          // |error| below this (ticks/s) = "AT SPEED"
    public static double SHOOTER_VELOCITY_STEP = 50;      // bumpers in Shooter Test
    public static boolean SHOOTER_VOLTAGE_COMP = true;    // scale power by NOMINAL_VOLTAGE / battery voltage
    public static double NOMINAL_VOLTAGE = 12.0;

    // HUB velocity PIDF (setVelocityPIDFCoefficients + setVelocity) - FINAL: Shooter Velocity PIDF Test
    // hub units, not the same numbers as POLLEN_k*: kF = 32767 / (max ticks/s at full power)
    // kP per motor - raise the one that recovers slower after a shot
    public static double SHOOTER1_HUB_kP = 0.05;
    public static double SHOOTER2_HUB_kP = 0.05;
    public static double SHOOTER_HUB_kI = 0.0;
    public static double SHOOTER_HUB_kD = 0;
    // kF per motor so both reach the same velocity: kF_new = kF_old * target / measured velocity of that motor
    public static double SHOOTER1_HUB_kF = 12.8;
    public static double SHOOTER2_HUB_kF = 12.1;          // shooter2 read ~1120 vs shooter1 ~1200 at 11.75
    public static double SHOOTER_HUB_TARGET_VELOCITY = 1120; // ticks/s

    //TODO ==================== HOOD (hood1 + hood2 servos, mounted opposite) ==================== (Shooter Test)
    // hood1 gets the position, hood2 gets the mirror (1 - position) because it faces the other way.
    public static double HOOD_INIT_POS = 0.252;           // hood1 = 0.805, hood2 = 1 - 0.805 = 0.195
    public static double HOOD_POLLEN_POS = 0.252;         // hood for POLLEN shots
    public static double HOOD_NECTAR_POS = 0.252;         // hood for NECTAR shots
    public static double HOOD_MIN_POS = 0.0;              // soft limits
    public static double HOOD_MAX_POS = 1.0;
    public static double HOOD2_TRIM = 0.0;                // added to hood2 only, if the two servos don't line up exactly
    public static double HOOD_STEP = 0.01;                // dpad up / down
    public static double HOOD_FINE_STEP = 0.001;          // dpad right / left

    //TODO ==================== SIDE ADJUSTABLE SERVO (adjust) ==================== (Shooter Test)
    public static double ADJUST_INIT_POS = 0.7814;
    public static double ADJUST_POLLEN_POS = 0.7584;      // adjust servo for POLLEN shots
    public static double ADJUST_NECTAR_POS = 0.8514;      // adjust servo for NECTAR shots
    public static double ADJUST_MIN_POS = 0.0;            // soft limits
    public static double ADJUST_MAX_POS = 1.0;
    public static double ADJUST_STEP = 0.01;              // LT + dpad up / down in Shooter Test
    public static double ADJUST_FINE_STEP = 0.001;        // LT + dpad right / left in Shooter Test
    public static double ADJUST_SPEED = 0.25;             // position per second at full left stick in Shooter Test

    //TODO ==================== FLAP (E servo 0) + SERVO2 (E servo 1) ====================
    public static double flapOpen = 0.1;                  // LB shoot / dpad right full speed
    public static double flapClose = 0.7767;              // init, intake, nectar on the switches
    public static double servo2Open = 0.7783;             // dpad down
    public static double servo2Close = 0.0;               // init, dpad up

    //TODO ==================== TURRET (turret1 + turret2 servos, mounted opposite, 1:1) ==================== (Turret Test)
    // turret1 gets the position, turret2 gets the mirror (1 - position) because it faces the other way.
    public static double TURRET_INIT_POS = 0.1;           // turret facing robot-forward (0 deg)
    public static double TURRET_MIN_POS = 0.0;            // soft limits - keep the turret off its hard stops / wiring
    public static double TURRET_MAX_POS = 1.0;
    public static double TURRET_SERVO_RANGE_DEG = 300;    // degrees the servo (= turret, 1:1) turns from position 0 to 1
    public static boolean TURRET_REVERSED = false;        // flip if a + angle turns the turret clockwise (should be counter-clockwise)
    public static double TURRET2_TRIM = 0.0;              // added to turret2 only, if the two servos don't line up exactly
    public static double TURRET_TARGET_DEG = 0;           // field angle to track (0 = the way the robot faced at heading zero)
    public static double TURRET_TARGET_STEP_DEG = 5;      // dpad left / right step in Turret Test

    //TODO ==================== SENSORS (limit1 C0, limit2 C1, b1 C2, b2 C3) ====================
    public static boolean B1_BROKEN_WHEN_LOW = true;      // broken beam reads getState() == false
    public static boolean B2_BROKEN_WHEN_LOW = true;
    public static boolean LIMIT1_PRESSED_WHEN_LOW = true; // pressed limit switch reads getState() == false
    public static boolean LIMIT2_PRESSED_WHEN_LOW = true;
    public static long WATCHER_SLEEP_MS = 2;              // sensor thread rest between polls; 0 hogs the hub and lags the drive

    //TODO ==================== POLLEN COUNT (b1) ====================
    public static double MIN_BREAK_MS = 10;               // shorter breaks are noise
    public static double MIN_CLEAR_MS = 25;               // beam must be clear this long before the next pollen can count
    public static int FEEDER_STOP_COUNT = 3;              // feeder stops once this many pollen are counted...
    public static double FEEDER_STOP_DELAY_MS = 1000;     // ...this long afterwards
    public static int MAX_POLLEN = 4;                     // intake + feeder stop at this count

    //TODO ==================== SHOOTING TELEOP (FinalCommandTeleOp) ====================
    // own velocities so the Shooter Test keeps its own. Gains: POLLEN_k* / NECTAR_k*, adjust: ADJUST_*_POS.
    public static double TELE_POLLEN_VELOCITY = 1100;     // ticks/s
    public static double TELE_NECTAR_VELOCITY = 1210;     // ticks/s
    public static double TELE_TURRET_POS = 0.1;           // turret held fixed here, from init to the end

    // feeding while shooting
    public static double SHOOT_FEED_DELAY_S = 0.3;        // intake + feeder OFF, flap opens, intake + feeder start this much later
    public static double SHOOT_REVERSE_S = 0.1;           // feeder reversed this long first...
    public static double SHOOT_REVERSE_POWER = 0.05;      // ...at this power, then normal feeding
    public static double SHOOT_INTAKE_POWER = 0.5;        // intake power while shooting
    public static double SHOOT_FEEDER_POWER = 0.18 ;        // feeder power while shooting
    public static double SHOOT_FLAP_CLOSE_S = 3;        // LB: flap closes + shot ends this long after LB (0 = never, LB again to stop)
    // Final Velocity Gate TeleOp only: feed at SHOOT_*_POWER only when (target - velocity) <= SHOOT_VELOCITY_TOL,
    // otherwise feeder SHOOT_SLOW_FEEDER_POWER + intake SHOOT_SLOW_INTAKE_POWER until the shooter recovers
    public static double SHOOT_VELOCITY_TOL = 10;         // ticks/s
    public static double SHOOT_SLOW_FEEDER_POWER = 0.1;
    public static double SHOOT_SLOW_INTAKE_POWER = 0.0;

    // POLLEN / NECTAR mode switching (follows the piece on the limit switches)
    public static double NECTAR_DETECT_MS = 20;           // both switches pressed this long = nectar arrived -> NECTAR
    public static double NECTAR_CLEAR_S = 0.25;           // both released this long = nectar shot -> POLLEN
    public static double POLLEN_ADJUST_DELAY_S = 0.2;     // NECTAR -> POLLEN: adjust servo (+ hood) moves this much later
    public static double ADJUST_MOVE_S = 0.15;            // feed waits this long after the adjust servo moves
    public static double NECTAR_INTERVAL_S = 0.15;        // feed waits this long every time a nectar presses both switches

    //TODO ==================== AUTONOMOUS (opmodes/auto) ====================
    // shooting uses the same TELE_*_VELOCITY / SHOOT_* values as the TeleOp
    public static double AUTO_TURRET_POS = 0.5;           // turret held fixed here for the whole auto
    public static double AUTO_TURRET_FLOWER_POS = 0.1;           // turret held fixed here for the whole auto
    public static double AUTO_SPINUP_TIMEOUT_S = 2.5;     // shoot anyway if the flywheel isn't at speed after this long
    public static double AUTO_SHOT_TIMEOUT_S = 3.0;       // a shot never lasts longer than this (also used when SHOOT_FLAP_CLOSE_S = 0)
    public static double AUTO_PATH_TIMEOUT_S = 6.0;       // move on if a path isn't finished after this long (stuck / pushed)
    public static double AUTO_INTAKE_SETTLE_S = 0.2;      // keep the intake running this long after a collect path ends
}
