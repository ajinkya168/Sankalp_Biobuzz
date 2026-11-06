# navX for Pedro Pathing 3.0

This folder is self contained. It only depends on the FTC SDK and `com.pedropathing:revhub`.

| File | What it does |
|---|---|
| `NavxIMU.java` | navX Micro / navX2 Micro as a Pedro `CustomIMU` (radians, counter-clockwise positive) |
| `ImuSetup.java` | Picks navX or Control Hub IMU, creates it, and prints the Constants line for tuners |

## Robot Configuration

Add the navX on a Control Hub I2C port as **navX Micro Navigation Sensor**, name it `navx`.
Keep the robot still during init, the navX calibrates on power up (up to 3 s).

## Use it in any Pedro localizer that takes a CustomIMU

Two Wheel:
```java
public static TwoWheelConfig localizerConfig = new TwoWheelConfig(c -> {
    c.imuName.set("navx");
    c.imu.set(new NavxIMU());          // new NavxIMU(true) if heading goes the wrong way
    ...
});
```

Three Wheel + IMU:
```java
public static ThreeWheelIMUConfig localizerConfig = new ThreeWheelIMUConfig(c -> {
    c.imuName.set("navx");
    c.imu.set(new NavxIMU());
    ...
});
```

Switch back to the Control Hub IMU without touching anything else:
```java
c.imuName.set("imu");
c.imu.set(ImuSetup.controlHub(
        RevHubOrientationOnRobot.LogoFacingDirection.UP,
        RevHubOrientationOnRobot.UsbFacingDirection.BACKWARD).create());
```

## Move to a new Pedro project

1. Copy this `imu` folder into the new project's `TeamCode/.../pedro/`.
   If the package is different, change the `package` line at the top of both files.
2. In `Constants`, set `c.imu.set(new NavxIMU())` and `c.imuName.set("navx")`.
3. In any tuner that builds its IMU with `new RevHubIMU(...)`, replace it with
   `ImuSetup.create()` and its printed code with `ImuSetup.code()`.
   `procedures/TwoWheelTuner.java` in this project is the example to copy.

## Check it

Run **Tuning > Tests > Localization Test**, then spin the robot counter-clockwise.
Heading should go up. If it goes down, use `new NavxIMU(true)`.
