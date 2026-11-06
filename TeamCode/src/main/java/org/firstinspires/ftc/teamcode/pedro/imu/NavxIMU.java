package org.firstinspires.ftc.teamcode.pedro.imu;

import com.pedropathing.revhub.localizers.CustomIMU;
import com.pedropathing.utils.Angle;
import com.qualcomm.hardware.kauailabs.NavxMicroNavigationSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;

/*
 * navX-Micro / navX2-Micro heading source for Pedro's TwoWheelLocalizer.
 *
 * Robot Configuration: add the navX on an I2C port as "navX Micro Navigation Sensor"
 * and name it the same as TwoWheelConfig.imuName (default "navx").
 *
 * The SDK driver already flips the navX yaw so counter-clockwise is positive, which is
 * what Pedro expects. If heading goes the wrong way (navX mounted upside down), pass
 * reversed = true.
 */
public class NavxIMU implements CustomIMU {

    // navX calibrates itself on power up, keep the robot still during init
    public static double CALIBRATION_TIMEOUT_SECONDS = 3.0;

    private final double direction;
    private NavxMicroNavigationSensor navx;
    private double yawOffset = 0.0;

    public NavxIMU() {
        this(false);
    }

    public NavxIMU(boolean reversed) {
        this.direction = reversed ? -1.0 : 1.0;
    }

    @Override
    public void initialize(HardwareMap hardwareMap, String name) {
        navx = hardwareMap.get(NavxMicroNavigationSensor.class, name);

        ElapsedTime timer = new ElapsedTime();
        while (navx.isCalibrating() && timer.seconds() < CALIBRATION_TIMEOUT_SECONDS) {
            try {
                Thread.sleep(20);
            } catch (InterruptedException e) {
                Thread.currentThread().interrupt();
                break;
            }
        }

        resetYaw();
    }

    // radians, counter-clockwise positive
    @Override
    public double getHeading() {
        return Angle.normalize(rawYaw() - yawOffset);
    }

    // navX has no yaw reset through the SDK driver, so zero it in software
    @Override
    public void resetYaw() {
        yawOffset = rawYaw();
    }

    private double rawYaw() {
        return direction * navx.getAngularOrientation(
                AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.RADIANS).firstAngle;
    }
}
