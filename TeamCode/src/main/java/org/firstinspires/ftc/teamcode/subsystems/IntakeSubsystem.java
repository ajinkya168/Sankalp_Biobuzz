package org.firstinspires.ftc.teamcode.subsystems;

import com.pedropathing.ivy.Command;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.robot.RobotConstants;

import static com.pedropathing.ivy.commands.Commands.instant;

/** Collects POLLEN/NECTAR off the field and feeds them toward the SSHooter/Flower mechanisms. */
public class IntakeSubsystem {

    private final DcMotorEx motor;

    public IntakeSubsystem(HardwareMap hardwareMap) {
        motor = hardwareMap.get(DcMotorEx.class, RobotConstants.INTAKE_MOTOR_NAME);
        motor.setDirection(DcMotorSimple.Direction.FORWARD);
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    public void setPower(double power) {
        motor.setPower(power);
    }

    public double getPower() {
        return motor.getPower();
    }

    /** Runs while scheduled, and stops the moment it's cancelled - pair with {@code Binding.whileTrue}. */
    public Command in() {
        return runAt(RobotConstants.INTAKE_IN_POWER);
    }

    /** Runs while scheduled, and stops the moment it's cancelled - pair with {@code Binding.whileTrue}. */
    public Command out() {
        return runAt(RobotConstants.INTAKE_OUT_POWER);
    }

    /** One-shot stop, for use outside of a held {@link #in()}/{@link #out()} (e.g. in an autonomous sequence). */
    public Command off() {
        return instant(() -> setPower(0)).requiring(motor);
    }

    private Command runAt(double power) {
        return Command.build()
                .setExecute(() -> setPower(power))
                .setDone(() -> false)
                .setEnd(endCondition -> setPower(0))
                .requiring(motor);
    }
}
