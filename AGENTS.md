# AGENTS.md

This repo is a fork of [Pedro-Pathing/Quickstart](https://github.com/Pedro-Pathing/Quickstart) (`upstream`) for the 2026–2027 season. FTC SDK **12.0.0**. Team robot code belongs in the `TeamCode` module only.

## Language

Write Kotlin unless there is a specific reason to use Java. New OpModes and team logic go in `TeamCode/src/main/kotlin/org/firstinspires/ftc/teamcode/`.

The Pedro Quickstart package `org.firstinspires.ftc.teamcode.pedro` stays Java. It matches upstream, and the tuning library loads `@Tuner` procedures from that package. Do not rewrite it in Kotlin.

## Do not edit

- `FtcRobotController` samples (`org.firstinspires.ftc.robotcontroller.external.samples`). Copy a sample into `TeamCode` and edit the copy.
- `FtcOpModeRegister`. Register OpModes with `@TeleOp` or `@Autonomous`.
- The root Gradle plugin block and `build.common.gradle`. Build customizations go in `TeamCode/build.gradle`.
- SDK libraries (`RobotCore`, `Hardware`, `Vision`, and the rest) are Maven dependencies, not source.

## This robot

Hardware configuration `bi2w`. `hardwareMap` names must match these strings. Use `HardwareNames`.

| Device | Name |
|---|---|
| Control Hub 0 | `backLeftMotor` |
| Control Hub 1 | `backRightMotor` |
| Control Hub 2 | `frontLeftMotor` |
| Control Hub 3 | `frontRightMotor` |
| Control Hub I2C 1 | `pinpoint` |
| Expansion Hub 0 | `flywheel1` |
| Expansion Hub 1 | `flywheel2` |
| Expansion Hub 2 | `feed` |
| Expansion Hub 3 | `intake` |

Directions already chosen: both left drive motors reversed, intake reversed, both flywheels reversed. The same power command goes to both flywheels, and they are not opposing.

`flywheel1` has an encoder plugged into Expansion Hub port 0. `flywheel2` does not. Bench OpModes keep `RUN_WITHOUT_ENCODER` and read flywheel1 position and velocity.

## OpModes

Use `LinearOpMode`: `runOpMode`, `waitForStart`, then `while (opModeIsActive())`. `sleep()` is allowed. A linear OpMode cannot override `init`, `loop`, or `stop`.

Call `telemetry.update()` at the end of the loop. The SDK sends telemetry at most about every 250 ms.

`DcMotor.RunMode` and `ZeroPowerBehavior` are unchanged in SDK 12. Bench OpModes here use `RUN_WITHOUT_ENCODER` and `BRAKE`.

## SDK 12 AprilTags

`getDetections()` returns a mix of `AprilTagSingleDetection` and `AprilTagClusterDetection`. Check `instanceof` and cast before reading `id`, `metadata`, corners, or center. `ftcPose` is on the parent type. A single detection's `metadata` may be null. A tag that belongs to a cluster is never also returned as a single detection.

## Pedro Pathing 3

Dependencies: `com.pedropathing:revhub:3.0.1` and `com.pedropathing:tuning:1.0.1`.

`Constants.create(HardwareMap)` is still a stub and returns null. Construct `Follower` as `(Localizer, Drivetrain, Algorithm)`. Ignore the swapped order in the stub comment.

Register a tuner with a **static, no-argument** `@Tuner` method on `Tuning` that returns a `Procedure`. Only the mecanum direction tuner is registered today. Do not invent `@Tuner` methods that the scanner will not see.

Build with `./gradlew :TeamCode:assembleDebug`. Java and Kotlin target 1.8, so the Java 8 deprecation warnings are expected.

Official references: [FTC docs](https://ftc-docs.firstinspires.org/), [SDK 12.0 release](https://github.com/FIRST-Tech-Challenge/FtcRobotController/releases/tag/v12.0), [AprilTag clusters](https://ftc-docs.firstinspires.org/apriltag-clusters), [Pedro installation](https://pedropathing.com/docs/pathing/installation), [Pedro constants](https://pedropathing.com/docs/pathing/tuning/constants).
