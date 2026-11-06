package org.firstinspires.ftc.teamcode.subsystems;

import static org.firstinspires.ftc.teamcode.hdware.final_Globals.*;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.hdware.FinalHardware;
import org.firstinspires.ftc.teamcode.hdware.VelocityPIDF;

/*
 * Shooter flywheel (shooter1 + shooter2) held by OUR velocity PIDF (VelocityPIDF), not the hub's.
 * Each motor has its own loop on its own encoder:
 *   shooter1 - getShooter1Velocity(), POLLEN_k* / NECTAR_k*
 *   shooter2 - getShooter2Velocity(), POLLEN2_kP / POLLEN2_kF or NECTAR2_kP / NECTAR2_kF,
 *              kI / kD shared with shooter1 (POLLEN_kI/kD or NECTAR_kI/kD)
 *
 * The velocity getters apply SHOOTER1/2_ENCODER_REVERSED, so this works even when an encoder counts
 * against its motor's power (the hub loop can't use that). Gains are power units, not hub units.
 * Each power is scaled by NOMINAL_VOLTAGE / battery when SHOOTER_VOLTAGE_COMP, clipped to 0..1
 * (never backwards), sent with setPower - directions from FinalHardware (SHOOTER1/2_REVERSED).
 *
 * Not a CommandScheduler subsystem in a LinearOpMode: call periodic() once per loop there.
 */
public class FinalShooterPIDFSubsystem extends SubsystemBase {

    private final FinalHardware robot;
    private final VelocityPIDF pidf1 = new VelocityPIDF();
    private final VelocityPIDF pidf2 = new VelocityPIDF();

    private boolean on = false;
    private boolean nectar = false;     // gain set: false = POLLEN, true = NECTAR
    private double targetVelocity = 0;  // ticks/s while on

    // readouts for telemetry (velocity = average, for at speed)
    public double target = 0, velocity = 0, v1 = 0, v2 = 0, power1 = 0, power2 = 0, voltage = 12;
    private final ElapsedTime voltageTimer = new ElapsedTime();
    private static final double VOLTAGE_READ_MS = 100; // battery voltage is a single hub read - not every loop

    public FinalShooterPIDFSubsystem(FinalHardware robot) {
        this.robot = robot;
        // our own PIDF sends power - the hub must not run its velocity loop
        for (DcMotorEx m : new DcMotorEx[]{robot.shooter1, robot.shooter2}) {
            m.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            m.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        }
        robot.setShooterPower(0);
        voltage = robot.voltageSensor.getVoltage();
    }

    private void resetPidf() {
        pidf1.reset();
        pidf2.reset();
    }

    public void setOn(boolean on) {
        if (this.on != on) resetPidf();
        this.on = on;
    }

    public boolean isOn() {
        return on;
    }

    /** Gain set: true = NECTAR, false = POLLEN. */
    public void setNectar(boolean nectar) {
        if (this.nectar != nectar) resetPidf();
        this.nectar = nectar;
    }

    public void setTarget(double ticksPerSecond) {
        targetVelocity = ticksPerSecond;
    }

    public boolean atSpeed() {
        return on && Math.abs(target - velocity) <= SHOOTER_READY_TOL;
    }

    public VelocityPIDF pidf1() {
        return pidf1;
    }

    public VelocityPIDF pidf2() {
        return pidf2;
    }

    @Override
    public void periodic() {
        v1 = robot.getShooter1Velocity();
        v2 = robot.getShooter2Velocity();
        velocity = (v1 + v2) / 2;
        if (voltageTimer.milliseconds() >= VOLTAGE_READ_MS) {
            voltageTimer.reset();
            voltage = robot.voltageSensor.getVoltage();
        }

        target = on ? targetVelocity : 0;
        if (on) {
            double out1 = nectar
                    ? pidf1.update(target, v1, NECTAR_kP, NECTAR_kI, NECTAR_kD, NECTAR_kF, SHOOTER_I_MAX)
                    : pidf1.update(target, v1, POLLEN_kP, POLLEN_kI, POLLEN_kD, POLLEN_kF, SHOOTER_I_MAX);
            double out2 = nectar
                    ? pidf2.update(target, v2, NECTAR2_kP, NECTAR_kI, NECTAR_kD, NECTAR2_kF, SHOOTER_I_MAX)
                    : pidf2.update(target, v2, POLLEN2_kP, POLLEN_kI, POLLEN_kD, POLLEN2_kF, SHOOTER_I_MAX);
            double comp = SHOOTER_VOLTAGE_COMP && voltage > 1 ? NOMINAL_VOLTAGE / voltage : 1;
            power1 = Range.clip(out1 * comp, 0, 1); // never drive the flywheel backwards
            power2 = Range.clip(out2 * comp, 0, 1);
        } else {
            power1 = 0;
            power2 = 0;
        }
        robot.shooter1.setPower(power1);
        robot.shooter2.setPower(power2);
    }

    public void stop() {
        on = false;
        resetPidf();
        power1 = 0;
        power2 = 0;
        robot.setShooterPower(0);
    }
}
