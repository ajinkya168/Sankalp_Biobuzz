package org.firstinspires.ftc.teamcode.subsystems;

import com.pedropathing.ivy.Command;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.robot.RobotConstants;
import org.firstinspires.ftc.teamcode.robot.RobotConstants.FlowerLevel;

import static com.pedropathing.ivy.commands.Commands.instant;
import static com.pedropathing.ivy.commands.Commands.waitUntil;
import static com.pedropathing.ivy.groups.Groups.sequential;

/** Lifts a NECTAR up the wall and deposits it on top of a FLOWER during endgame. */
public class FlowerSubsystem {

    private final DcMotorEx lift;
    private final Servo wrist;
    private final Servo claw;

    public FlowerSubsystem(HardwareMap hardwareMap) {
        lift = hardwareMap.get(DcMotorEx.class, RobotConstants.FLOWER_LIFT_MOTOR_NAME);
        wrist = hardwareMap.get(Servo.class, RobotConstants.FLOWER_WRIST_SERVO_NAME);
        claw = hardwareMap.get(Servo.class, RobotConstants.FLOWER_CLAW_SERVO_NAME);

        lift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        lift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        setClawClosed();
        setWristStow();
    }

    // ---- Raw hardware access, used directly by FlowerTest for jogging/tuning ----

    public void setLiftTarget(int ticks) {
        lift.setTargetPosition(ticks);
        lift.setPower(RobotConstants.FLOWER_LIFT_POWER);
    }

    public int getLiftPosition() {
        return lift.getCurrentPosition();
    }

    public boolean liftAt(int ticks) {
        return Math.abs(getLiftPosition() - ticks) <= RobotConstants.FLOWER_LIFT_TOLERANCE_TICKS;
    }

    /** Nudges the lift's target by a fraction of one loop's worth of ticks; call every loop while jogging. */
    public void jogLift(double power) {
        setLiftTarget(getLiftPosition() + (int) (power * RobotConstants.FLOWER_JOG_TICKS_PER_LOOP));
    }

    private void setWristStow() {
        wrist.setPosition(RobotConstants.FLOWER_WRIST_STOW_POSITION);
    }

    private void setWristScore() {
        wrist.setPosition(RobotConstants.FLOWER_WRIST_SCORE_POSITION);
    }

    private void setClawOpen() {
        claw.setPosition(RobotConstants.FLOWER_CLAW_OPEN_POSITION);
    }

    private void setClawClosed() {
        claw.setPosition(RobotConstants.FLOWER_CLAW_CLOSED_POSITION);
    }

    // ---- Ivy commands ----

    public Command goTo(FlowerLevel level) {
        return instant(() -> setLiftTarget(level.ticks))
                .then(waitUntil(() -> liftAt(level.ticks)))
                .requiring(lift);
    }

    public Command deployWrist() {
        return instant(this::setWristScore).requiring(wrist);
    }

    public Command stowWrist() {
        return instant(this::setWristStow).requiring(wrist);
    }

    public Command openClaw() {
        return instant(this::setClawOpen).requiring(claw);
    }

    public Command closeClaw() {
        return instant(this::setClawClosed).requiring(claw);
    }

    /** Raises to FLOWER height, swings the wrist out, and drops the NECTAR onto the FLOWER. */
    public Command scoreOnFlower() {
        return sequential(
                goTo(FlowerLevel.SCORE),
                deployWrist(),
                openClaw()
        );
    }

    /** Returns to a safe travel position with the claw closed. */
    public Command stowAll() {
        return sequential(
                stowWrist(),
                closeClaw(),
                goTo(FlowerLevel.INTAKE)
        );
    }
}
