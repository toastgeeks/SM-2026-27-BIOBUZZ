package org.firstinspires.ftc.teamcode.autotune;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import java.util.ArrayList;
import java.util.Collections;
import java.util.List;

/**
 * Tunes the velocity PIDF of whichever motors you pick, with nothing to configure. After Start:
 * <pre>
 * 1. Pick the motors. D-pad up/down moves, A selects (the first one picked is measured),
 *    D-pad left/right sets the speed it spins up to, in ticks per second,
 *    hold Y to spin the highlighted motor so you can see which one it is,
 *    right bumper starts the tune.
 * 2. Each picked motor is spun alone. A = turns the right way, B = reverse it.
 * 3. The tuner runs. X aborts at any time.
 * 4. Copy the printed P/I/D/F and motor directions into your subsystem.
 * </pre>
 */
@TeleOp(name = "Auto Tune Flywheel", group = "Tuning")
public class AutoTuneFlywheel extends LinearOpMode {
    private static final double IDENTIFY_POWER = 0.15;
    private String logPath = "";
    private boolean logOk;
    // The speed each spin-up goes to, in ticks per second. 0 means automatic: two thirds of the top speed that is measured.
    // The band that is scored is always two thirds to 95% of the top speed. It is static so the value you set stays for the next run.
    private static double operatingSpeed = 0;

    @Override
    public void runOpMode() {
        List<String> names = new ArrayList<>(hardwareMap.getAllNames(DcMotorEx.class));
        Collections.sort(names);

        telemetry.addLine(names.isEmpty() ? "No motors found in the robot configuration."
                : "Clear the area around the mechanism, then press start. The flywheel will run at full power for a few seconds to measure its top speed.");
        telemetry.update();
        waitForStart();
        if (names.isEmpty()) return;

        List<String> picked = pickMotors(names);
        if (picked == null) return;

        FlywheelAutoTuner tuner = new FlywheelAutoTuner(hardwareMap, picked.toArray(new String[0]));
        tuner.setOperatingSpeed(operatingSpeed);
        logPath = "/sdcard/FIRST/autotune/autotune-" + System.currentTimeMillis() + ".csv";
        logOk = tuner.setLogFile(new java.io.File(logPath));
        run(tuner);
    }

    /** Returns the picked motors in the order they were picked, or null if the OpMode stopped. */
    private List<String> pickMotors(List<String> names) {
        List<String> picked = new ArrayList<>();
        int cursor = 0;
        boolean lastUp = false, lastDown = false, lastA = false, lastLeft = false, lastRight = false;
        DcMotorEx spinning = null;

        while (opModeIsActive()) {
            if (gamepad1.dpad_up && !lastUp) cursor = (cursor + names.size() - 1) % names.size();
            if (gamepad1.dpad_down && !lastDown) cursor = (cursor + 1) % names.size();
            if (gamepad1.a && !lastA) {
                String name = names.get(cursor);
                if (!picked.remove(name)) picked.add(name);
            }
            if (gamepad1.dpad_left && !lastLeft) operatingSpeed = Math.max(0, operatingSpeed - 50);
            if (gamepad1.dpad_right && !lastRight) operatingSpeed += 50;
            lastUp = gamepad1.dpad_up;
            lastDown = gamepad1.dpad_down;
            lastA = gamepad1.a;
            lastLeft = gamepad1.dpad_left;
            lastRight = gamepad1.dpad_right;

            // Spin the highlighted motor only while Y is held
            DcMotorEx wanted = gamepad1.y ? hardwareMap.get(DcMotorEx.class, names.get(cursor)) : null;
            if (spinning != null && spinning != wanted) spinning.setPower(0);
            if (wanted != null) wanted.setPower(IDENTIFY_POWER);
            spinning = wanted;

            if (gamepad1.right_bumper && !picked.isEmpty()) {
                if (spinning != null) spinning.setPower(0);
                return picked;
            }

            telemetry.addLine("Pick the motors for this mechanism");
            telemetry.addLine("D-pad up/down: move | A: select | hold Y: spin it | right bumper: start");
            telemetry.addLine(operatingSpeed > 0
                    ? String.format("D-pad left/right: speed it spins up to %.0f ticks/s (0 = automatic)", operatingSpeed)
                    : "D-pad left/right: speed it spins up to: automatic, two thirds of the top speed it measures");
            for (int i = 0; i < names.size(); i++) {
                int order = picked.indexOf(names.get(i)) + 1;
                telemetry.addLine((i == cursor ? "> " : "  ") + names.get(i) + (order > 0 ? "  [" + order + "]" : ""));
            }
            telemetry.update();
        }
        if (spinning != null) spinning.setPower(0);
        return null;
    }

    private void run(FlywheelAutoTuner tuner) {
        boolean lastA = false, lastB = false;
        while (opModeIsActive() && !tuner.isDone()) {
            if (gamepad1.x) tuner.abort();
            if (gamepad1.a && !lastA) tuner.confirmDirection();
            if (gamepad1.b && !lastB) tuner.reverseDirection();
            lastA = gamepad1.a;
            lastB = gamepad1.b;
            tuner.update();

            boolean confirming = tuner.getPhase() == FlywheelAutoTuner.Phase.CONFIRM_DIRECTION;
            telemetry.addData("Phase", tuner.getPhase());
            telemetry.addData("Status", tuner.getStatus());
            if (!confirming) {
                double left = tuner.getSecondsRemaining();
                telemetry.addData("Time left", "%d:%02d", (int) left / 60, (int) left % 60);
                double target = tuner.getTarget();
                telemetry.addData("Target / velocity", "%.0f / %.0f (%.0f%% of target)",
                        target, tuner.getVelocity(), target > 0 ? 100 * tuner.getVelocity() / target : 0);
                double[] pidf = tuner.getCurrentPIDF();
                telemetry.addData("Trying P / I / D / F", "%.6f / %.6f / %.6f / %.3f", pidf[0], pidf[1], pidf[2], pidf[3]);
            }
            String warning = tuner.getWarning();
            if (warning != null) telemetry.addData("WARNING", warning);
            telemetry.addLine(confirming ? "A = correct direction, B = reverse it, X = stop"
                    : "X = stop (the flywheel stops and nothing is saved)");
            telemetry.update();
        }

        FlywheelTuneResult result = tuner.getResult();
        boolean failed = tuner.getPhase() == FlywheelAutoTuner.Phase.FAILED;
        while (opModeIsActive() && result != null) {
            if (failed) {
                // Half-tuned values are not worth pasting into a subsystem
                telemetry.addLine("Tuning stopped, no values to use");
                telemetry.addData("Reason", result.notes);
            } else {
                telemetry.addLine(result.passed ? "Tuning finished" : "Tuning did not fully converge");
                telemetry.addLine(result.asConstants());
                if (result.encoderInverted) {
                    telemetry.addLine("VELOCITY FEEDBACK WORKS BACKWARDS: only F was tuned, P, I and D are 0.");
                }
                telemetry.addData("Directions", result.directions);
                telemetry.addData("Voltage", "%.1f V", result.voltage);
                telemetry.addData("Hold error", "%.1f%%", result.meanError * 100);
                telemetry.addData("Overshoot (of target)", "%.1f%%", result.overshoot * 100);
                telemetry.addData("Within +-2% of target after", result.settleSeconds >= 0
                        ? String.format("%.2f s", result.settleSeconds) : "never got there");
                telemetry.addData("Spin-up to 75% of top speed", result.spinUpSeconds >= 0
                        ? String.format("%.2f s", result.spinUpSeconds) : "never reached it");
                telemetry.addData("Ramp error", "%.1f%%", result.rampError * 100);
                telemetry.addData("Error per target", result.holdReport);
                telemetry.addData("Diagnostics", result.diagnostics);
                telemetry.addData("Search", result.searchReport);
                telemetry.addData("Notes", result.notes);
            }
            telemetry.addData("Log file", logOk ? logPath : "COULD NOT CREATE " + logPath);
            String warning = tuner.getWarning();
            if (warning != null) telemetry.addData("WARNING", warning);
            telemetry.update();
        }
    }
}
