package org.firstinspires.ftc.teamcode.hdware;

import static org.firstinspires.ftc.teamcode.hdware.final_Globals.*;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoImplEx;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

/*
 * Hardware for the Final robot.
 *
 * Drive motors and the navX are owned by Pedro (pedro/Constants.java),
 * use Constants.create(hardwareMap) to get the follower.
 *
 * Control Hub
 *   Motor 0-3  - fr, fl, br, bl         -> Constants.drivetrainConfig
 *   I2C        - navX (navx)            -> Constants.localizerConfig
 *   Digital 0  - limit1
 *   Digital 1  - limit2
 *   Digital 2  - b1 (beam break, intake entrance)
 *   Digital 3  - b2 (beam break, below the limit switches)
 * Expansion Hub
 *   Motor 0    - intake
 *   Motor 1    - feeder
 *   Servo 0    - flap (end of feeder)
 *   Servo 1    - servo2
 *   shooter1 / shooter2 - shooter motors, mounted opposite (final_Globals.SHOOTER*_REVERSED)
 *   hood1 / hood2       - hood servos, mounted opposite (hood2 gets 1 - position)
 *   turret1 / turret2   - turret servos, mounted opposite (turret2 gets 1 - position), 1:1
 *   adjust              - side adjustable servo
 */
public class FinalHardware {

    public DcMotorEx intake, feeder;
    public DcMotorEx shooter1, shooter2;
    public Servo flap, hood1, hood2, adjust, turret1, turret2, servo2;

    public DigitalChannel limit1, limit2, b1, b2;
    public VoltageSensor voltageSensor;
    public static FinalHardware instance = null;

    public static FinalHardware getInstance() {
        if (instance == null) {
            instance = new FinalHardware();
        }
        return instance;
    }

    public void init(HardwareMap hardwareMap) {

        //TODO Intake
        intake = hardwareMap.get(DcMotorEx.class, "intake");
        intake.setDirection(DcMotorSimple.Direction.FORWARD);
        intake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        //TODO Feeder
        feeder = hardwareMap.get(DcMotorEx.class, "feeder"); //E - 1
        feeder.setDirection(DcMotorSimple.Direction.FORWARD);
        feeder.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        feeder.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        //TODO Shooter - RUN_WITHOUT_ENCODER: the velocity PIDF is ours (VelocityPIDF), the encoders still count
        shooter1 = hardwareMap.get(DcMotorEx.class, "shooter1");
        shooter1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        shooter1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        shooter1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        shooter1.setDirection(SHOOTER1_REVERSED ? DcMotorSimple.Direction.REVERSE : DcMotorSimple.Direction.FORWARD);

        shooter2 = hardwareMap.get(DcMotorEx.class, "shooter2");
        shooter2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        shooter2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        shooter2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        shooter2.setDirection(SHOOTER2_REVERSED ? DcMotorSimple.Direction.REVERSE : DcMotorSimple.Direction.FORWARD);


        //TODO hood

        hood1 = hardwareMap.get(ServoImplEx.class, "hood1");
        hood2 = hardwareMap.get(ServoImplEx.class, "hood2");

        //TODO side adjustable servo
        adjust = hardwareMap.get(ServoImplEx.class, "adjust");
        //TODO Flap
        flap = hardwareMap.get(Servo.class, "flap");

        //TODO turret
        turret1 = hardwareMap.get(ServoImplEx.class, "turret1");
        turret2 = hardwareMap.get(ServoImplEx.class, "turret2");

        servo2 = hardwareMap.get(ServoImplEx.class, "servo2"); //E - 1

        limit1 = hardwareMap.get(DigitalChannel.class, "limit1"); //C - 0
        limit1.setMode(DigitalChannel.Mode.INPUT);
        limit2 = hardwareMap.get(DigitalChannel.class, "limit2"); //C - 1
        limit2.setMode(DigitalChannel.Mode.INPUT);

        b1 = hardwareMap.get(DigitalChannel.class, "b1"); //C - 2
        b1.setMode(DigitalChannel.Mode.INPUT);
        b2 = hardwareMap.get(DigitalChannel.class, "b2"); //C - 3
        b2.setMode(DigitalChannel.Mode.INPUT);

        voltageSensor = hardwareMap.voltageSensor.iterator().next();
    }

    //TODO ==================== sensors ====================

    /** b1 beam (intake entrance) broken. */
    public boolean isB1Broken() {
        return B1_BROKEN_WHEN_LOW != b1.getState();
    }

    /** b2 beam (below the limit switches) broken. */
    public boolean isB2Broken() {
        return B2_BROKEN_WHEN_LOW != b2.getState();
    }

    public boolean isLimit1Pressed() {
        return LIMIT1_PRESSED_WHEN_LOW != limit1.getState();
    }

    public boolean isLimit2Pressed() {
        return LIMIT2_PRESSED_WHEN_LOW != limit2.getState();
    }

    //TODO ==================== turret ====================

    /** Both turret servos: turret1 gets pos, turret2 the mirror. pos is clipped to the soft limits. Returns what was sent. */
    public double setTurretPosition(double pos) {
        pos = Range.clip(pos, TURRET_MIN_POS, TURRET_MAX_POS);
        turret1.setPosition(pos);
        turret2.setPosition(pos);
//        turret2.setPosition(Range.clip(1 - pos + TURRET2_TRIM, 0, 1));
        return pos;
    }

    /** Servo position for a turret angle relative to the robot (deg, counter-clockwise +, 0 = forward), unclipped. */
    public static double turretAngleToPosition(double robotRelativeDeg) {
        double sign = TURRET_REVERSED ? -1 : 1;
        return TURRET_INIT_POS + sign * AngleUnit.normalizeDegrees(robotRelativeDeg) / TURRET_SERVO_RANGE_DEG;
    }

    /** Turret angle relative to the robot (deg) for a servo position - inverse of turretAngleToPosition. */
    public static double turretPositionToAngle(double pos) {
        double sign = TURRET_REVERSED ? -1 : 1;
        return sign * (pos - TURRET_INIT_POS) * TURRET_SERVO_RANGE_DEG;
    }

    /** Points the turret at a robot-relative angle (deg). Returns the position sent (clipped to the soft limits). */
    public double setTurretAngle(double robotRelativeDeg) {
        return setTurretPosition(turretAngleToPosition(robotRelativeDeg));
    }

    //TODO ==================== shooter ====================

    /** Both shooter motors (directions already set so the same power spins both the right way). */
    public void setShooterPower(double power) {
        shooter1.setPower(power);
        shooter2.setPower(power);
    }

    /** Shooter velocity in ticks/s: average of both encoders, or shooter1 only. */
    public double getShooterVelocity() {
        double v1 = getShooter1Velocity();
        return SHOOTER_USE_BOTH_ENCODERS ? (v1 + getShooter2Velocity()) / 2 : v1;
    }

    /** shooter1 encoder ticks/s, sign-corrected so spinning the right way reads positive. */
    public double getShooter1Velocity() {
        return SHOOTER1_ENCODER_REVERSED ? -shooter1.getVelocity() : shooter1.getVelocity();
    }

    /** shooter2 encoder ticks/s, sign-corrected so spinning the right way reads positive. */
    public double getShooter2Velocity() {
        return SHOOTER2_ENCODER_REVERSED ? -shooter2.getVelocity() : shooter2.getVelocity();
    }

    //TODO ==================== hood + adjust ====================

    /** Both hood servos: hood1 gets pos, hood2 the mirror. pos is clipped to HOOD_MIN_POS..HOOD_MAX_POS. Returns what was sent. */
    public double setHood(double pos) {
        pos = Range.clip(pos, HOOD_MIN_POS, HOOD_MAX_POS);
        hood1.setPosition(pos);
        hood2.setPosition(Range.clip(1 - pos + HOOD2_TRIM, 0, 1));
        return pos;
    }

    /** Side adjustable servo, clipped to ADJUST_MIN_POS..ADJUST_MAX_POS. Returns what was sent. */
    public double setAdjust(double pos) {
        pos = Range.clip(pos, ADJUST_MIN_POS, ADJUST_MAX_POS);
        adjust.setPosition(pos);
        return pos;
    }
}
