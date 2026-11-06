package org.firstinspires.ftc.teamcode.hdware;

import com.acmerobotics.dashboard.config.Config;

/*
 * All tunable values for the sankalp27 robot (Sankalp27Hardware), shared by
 * teleOP/Sankalp27TeleOp and every sankalp27 test OpMode. Change a value here (or live
 * on FTC Dashboard under "sankalp27_Globals") and every sankalp27 OpMode uses it.
 *
 * navX name / direction stay in pedroPathing/Constants (NAVX_*) because the Pedro
 * follower builds the localizer from there.
 */
@Config
public class sankalp27_Globals {

    //TODO -------------------- DRIVE --------------------
    public static double StrafeSpeed = 1;
    public static double turnSpeed = 0.6;

    //TODO -------------------- FLAP (E servo 0) + SERVO2 (E servo 1) -------------------- (tune with Sankalp27GetValues)
    public static double flapOpen = 0.1; //0.5
    public static double flapClose = 0.7767;//0.8872;
    public static double flapNectar = 0.3983;//0.8872;
    public static double servo2Open = 0.7783;
    public static double servo2Close = 0.0;
    public static double flapCloseDelay = 1.5; // seconds before the flap goes back to flapClose once nectar (both limit switches pressed) is gone

    //TODO -------------------- INTAKE (E motor 0) + FEEDER (E motor 1) --------------------
    public static double INTAKE_POWER = 0.7;
    public static double FEEDER_POWER = 0.5;
    public static double REVERSE_POWER = 0.6;
    public static double FULL_SPEED_POWER = 1.0;          // dpad right full-speed feed

    //TODO -------------------- SENSORS --------------------
    public static boolean B1_BROKEN_WHEN_LOW = true;      // broken beam reads getState() == false
    public static boolean B2_BROKEN_WHEN_LOW = true;
    public static boolean LIMIT1_PRESSED_WHEN_LOW = true; // pressed limit switch reads getState() == false
    public static boolean LIMIT2_PRESSED_WHEN_LOW = true;
    public static long WATCHER_SLEEP_MS = 2;              // sensor thread rest between polls; 0 hogs the hub and lags the drive

    //TODO -------------------- POLLEN COUNT (b1) --------------------
    public static double MIN_BREAK_MS = 10;               // shorter breaks are noise
    public static double MIN_CLEAR_MS = 25;               // beam must be clear this long before the next pollen can count
    public static int FEEDER_STOP_COUNT = 3;              // feeder stops once this many pollen are counted...
    public static double FEEDER_STOP_DELAY_MS = 1000;     // ...this long afterwards
    public static int MAX_POLLEN = 4;                     // intake + feeder stop at this count

    //TODO -------------------- TURRET (turret1 + turret2 servos, mounted opposite, 1:1 to the turret) -------------------- (Turret Test)
    // turret1 gets the position, turret2 gets the mirror (1 - position) because it faces the other way.
    public static double TURRET_SERVO_RANGE_DEG = 300;    // degrees the servo (= turret, 1:1) turns from position 0 to 1
    public static double TURRET_INIT_POS = 0.5;           // init position = turret facing robot-forward (0 deg)
    public static double TURRET_MIN_POS = 0.0;            // soft limits - keep the turret off its hard stops / wiring
    public static double TURRET_MAX_POS = 1.0;
    public static boolean TURRET_REVERSED = false;        // flip if a + angle turns the turret clockwise (should be counter-clockwise)
    public static double TURRET2_TRIM = 0.0;              // added to turret2 only, if the two servos don't line up exactly
    public static double TURRET_TARGET_DEG = 0;           // field angle to track (0 = the way the robot faced at heading zero)
    public static double TURRET_TARGET_STEP_DEG = 5;      // dpad left / right step in Turret Test

    //TODO -------------------- SHOOTER (shooter1 + shooter2, mounted opposite) -------------------- (Shooter Test)
    public static boolean SHOOTER1_REVERSED = true;       // one motor is reversed so the same power spins both the right way
    public static boolean SHOOTER2_REVERSED = false;
    public static boolean SHOOTER1_ENCODER_REVERSED = true;  // flip if shooter1 vel reads negative while the flywheel spins the right way
    public static boolean SHOOTER2_ENCODER_REVERSED = true;  // flip if shooter2 vel reads negative while the flywheel spins the right way
    public static boolean SHOOTER_USE_BOTH_ENCODERS = true; // velocity = average of both; false = shooter1 encoder only

    // custom PIDF on velocity (ticks/s): power = kP*err + kI*integral(err) + kD*d(err)/dt + kF*target
    public static double SHOOTER_POLLEN_VELOCITY = 1100;  // ticks/s
    public static double POLLEN_kP = 0.005;
    public static double POLLEN_kI = 0.0;
    public static double POLLEN_kD = 0.0;
    public static double POLLEN_kF = 0.00038;             // ~1 / (max ticks/s at full power)

    public static double SHOOTER_NECTAR_VELOCITY = 1150 ;  // ticks/s
    public static double NECTAR_kP = 0.2;
    public static double NECTAR_kI = 0.0;
    public static double NECTAR_kD = 0.0;
    public static double NECTAR_kF = 0.00038;

    public static double SHOOTER_I_MAX = 0.2;             // integral term never adds more than this much power
    public static double SHOOTER_READY_TOL = 50;          // |error| below this (ticks/s) = "AT SPEED"
    public static double SHOOTER_VELOCITY_STEP = 50;      // bumpers in Shooter Test
    public static boolean SHOOTER_VOLTAGE_COMP = true;    // scale power by NOMINAL_VOLTAGE / battery voltage
    public static double NOMINAL_VOLTAGE = 12.0;

    //TODO -------------------- HOOD (hood1 + hood2 servos, mounted opposite) -------------------- (Shooter Test)
    // hood1 gets the position, hood2 gets the mirror (1 - position) because it faces the other way.
    public static double HOOD_INIT_POS = 0.805;           // hood1 = 0.805, hood2 = 1 - 0.805 = 0.195
    public static double HOOD_MIN_POS = 0.0;
    public static double HOOD_MAX_POS = 1.0;
    public static double HOOD2_TRIM = 0.0;                // added to hood2 only, if the two servos don't line up exactly
    public static double HOOD_STEP = 0.01;                // dpad up / down
    public static double HOOD_FINE_STEP = 0.001;          // dpad right / left
    public static double HOOD_POLLEN_POS = 0.265;         // hood for POLLEN shots (get value in Shooter Test)
    public static double HOOD_NECTAR_POS = 0.265;         // hood for NECTAR shots (get value in Shooter Test)

    //TODO -------------------- SIDE ADJUSTABLE SERVO -------------------- (Shooter Test)
    public static double ADJUST_INIT_POS = 0.7814;
    public static double ADJUST_MIN_POS = 0.0;
    public static double ADJUST_MAX_POS = 1.0;
    public static double ADJUST_POLLEN_POS = 0.7584;      // adjust servo for POLLEN shots (get value in Shooter Test)
    public static double ADJUST_NECTAR_POS = 0.8514;      // adjust servo for NECTAR shots (get value in Shooter Test)
    public static double ADJUST_STEP = 0.01;              // LT + dpad up / down in Shooter Test
    public static double ADJUST_FINE_STEP = 0.001;        // LT + dpad right / left in Shooter Test
    public static double ADJUST_SPEED = 0.25;           // position per second at full left stick in Shooter Test

    //TODO -------------------- SHOOTING TELEOP (teleOP/Sankalp27ShootingTeleOp) --------------------
    // own velocities so Shooter Test / Sankalp27TeleOp keep theirs. Gains: POLLEN_k* / NECTAR_k*, adjust: ADJUST_*_POS.
    public static double TELE_POLLEN_VELOCITY = 1082
             ;     // ticks/s
    public static double TELE_NECTAR_VELOCITY = 1150;     // ticks/s
    public static double TELE_TURRET_POS = 0.4;           // turret held fixed here, from init to the end
    public static double SHOOT_FEED_DELAY_S = 0.3;        // LB: intake + feeder OFF, flap opens, intake + feeder start this much later
    public static double POLLEN_ADJUST_DELAY_S = 0.2;     // B / back NECTAR -> POLLEN: adjust servo (+ hood) moves this much later
    // mode follows the piece on the limit switches (Sankalp27ShootingTeleOp)
    public static double NECTAR_DETECT_MS = 20;           // both switches pressed this long = nectar arrived -> NECTAR
    public static double NECTAR_CLEAR_S = 0.25;           // both released this long = nectar shot -> POLLEN
    public static double ADJUST_MOVE_S = 0.15;            // shooting: feed waits this long after the adjust servo moves
    public static double NECTAR_INTERVAL_S = 0.15;        // shooting: feed waits this long every time a nectar presses both switches
    public static double SHOOT_REVERSE_S = 0.1;           // LB: feeder reversed this long first...
    public static double SHOOT_REVERSE_POWER = 0.05;       // ...at this power, then normal feeding
    public static double SHOOT_INTAKE_POWER = 0.5;        // intake + feeder power while shooting
    public static double SHOOT_FEEDER_POWER = 0.2;
}
