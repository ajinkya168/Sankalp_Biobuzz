package org.firstinspires.ftc.teamcode.subsystems;

import com.pedropathing.ivy.Command;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.robot.RobotConstants;

import java.util.function.DoubleSupplier;

import static com.pedropathing.ivy.commands.Commands.infinite;
import static com.pedropathing.ivy.commands.Commands.instant;
import static com.pedropathing.ivy.commands.Commands.waitMs;
import static com.pedropathing.ivy.groups.Groups.sequential;

/** Launches POLLEN/NECTAR up into the HIVE's upward-facing scoring chamber. */
public class SSHooterSubsystem {

    private final DcMotorEx leftFlywheel;
    private final DcMotorEx rightFlywheel;
    private final Servo feeder;

    public SSHooterSubsystem(HardwareMap hardwareMap) {
        leftFlywheel = hardwareMap.get(DcMotorEx.class, RobotConstants.SHOOTER_LEFT_MOTOR_NAME);
        rightFlywheel = hardwareMap.get(DcMotorEx.class, RobotConstants.SHOOTER_RIGHT_MOTOR_NAME);
        feeder = hardwareMap.get(Servo.class, RobotConstants.SHOOTER_FEEDER_SERVO_NAME);

        leftFlywheel.setDirection(DcMotorSimple.Direction.FORWARD);
        rightFlywheel.setDirection(DcMotorSimple.Direction.REVERSE);
        leftFlywheel.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightFlywheel.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        feeder.setPosition(RobotConstants.SHOOTER_REST_POSITION);
    }

    public void setVelocity(double ticksPerSecond) {
        leftFlywheel.setVelocity(ticksPerSecond);
        rightFlywheel.setVelocity(ticksPerSecond);
    }

    public double getVelocity() {
        return leftFlywheel.getVelocity();
    }

    public boolean atTargetVelocity(double target) {
        return Math.abs(getVelocity() - target) <= RobotConstants.SHOOTER_VELOCITY_TOLERANCE;
    }

    /**
     * Spins the flywheels forever at {@code ticksPerSecond}, leaving them spinning if raced/interrupted -
     * for composing into an autonomous sequence with {@code .until(...)} ahead of a {@link #fireOne()}.
     */
    public Command spinUp(double ticksPerSecond) {
        return infinite(() -> setVelocity(ticksPerSecond)).requiring(leftFlywheel, rightFlywheel);
    }

    /** Spins the flywheels while scheduled, and spins them down the moment it's cancelled - pair with {@code toggleWhenPressed}/{@code whenHeld}. */
    public Command spinUpHeld(double ticksPerSecond) {
        return spinUpHeld(() -> ticksPerSecond);
    }

    /** Same as {@link #spinUpHeld(double)}, but reads the target velocity fresh every cycle - for a live-tunable binding. */
    public Command spinUpHeld(DoubleSupplier ticksPerSecondSupplier) {
        return Command.build()
                .setExecute(() -> setVelocity(ticksPerSecondSupplier.getAsDouble()))
                .setDone(() -> false)
                .setEnd(endCondition -> setVelocity(0))
                .requiring(leftFlywheel, rightFlywheel);
    }

    public Command stop() {
        return instant(() -> setVelocity(0)).requiring(leftFlywheel, rightFlywheel);
    }

    /** Kicks one scoring element into the HIVE chamber. Does not touch flywheel speed. */
    public Command fireOne() {
        return sequential(
                instant(() -> feeder.setPosition(RobotConstants.SHOOTER_FEED_POSITION)),
                waitMs(RobotConstants.SHOOTER_FEED_TIME_MS),
                instant(() -> feeder.setPosition(RobotConstants.SHOOTER_REST_POSITION))
        ).requiring(feeder);
    }

    /** Spins up, waits until at speed, fires once, then spins back down. */
    public Command shootOnce(double ticksPerSecond) {
        return sequential(
                spinUp(ticksPerSecond).until(() -> atTargetVelocity(ticksPerSecond)),
                fireOne(),
                stop()
        );
    }
}
