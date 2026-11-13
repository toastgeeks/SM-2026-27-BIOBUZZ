package org.firstinspires.ftc.teamcode.tuning

import com.qualcomm.hardware.lynx.LynxModule
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.DcMotorEx
import com.qualcomm.robotcore.hardware.DcMotorSimple
import com.qualcomm.robotcore.hardware.PIDFCoefficients
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit
import org.firstinspires.ftc.teamcode.HardwareNames
import java.io.File
import java.util.Locale
import java.util.Properties

/**
 * Runs flywheel tests that a laptop sends over adb, and writes the measurements back.
 *
 * The laptop (tools/flywheel_remote.py) writes [CMD_FILE] as key=value lines. Each command has a
 * larger `seq` than the last. This OpMode runs it once, writes run-<seq>.csv, and updates
 * [STATUS_FILE]. Nothing else on the Control Hub is touched.
 *
 * Commands:
 *  - action=velocity: set p, i, d, f on both flywheels, hold `from` ticks/s for `settle` s,
 *    then step to `to` ticks/s for `duration` s.
 *  - action=power: open-loop `power` for `duration` s (for measuring top speed).
 *  - feed=1 on either action runs the feed and intake at full power during the step (not the
 *    settle), so balls dropped in are shot while the flywheel holds speed.
 *
 * Safety: every run is time-limited and the flywheels stop after it. Targets are clamped.
 * Commands already waiting when the OpMode initializes are ignored. Gamepad 1 X aborts a run,
 * and Stop on the Driver Station stops everything.
 */
@TeleOp(name = "Remote Flywheel Tuner", group = "Tuning")
class RemoteFlywheelTuner : LinearOpMode() {
    private lateinit var motors: List<DcMotorEx>
    private lateinit var feed: DcMotorEx
    private lateinit var intake: DcMotorEx
    private lateinit var hubs: List<LynxModule>
    private var lastStatus = "ready"

    private data class Command(
        val seq: Long,
        val action: String,
        val pidf: PIDFCoefficients,
        val from: Double,
        val to: Double,
        val settle: Double,
        val duration: Double,
        val power: Double,
        val feed: Boolean,
    )

    override fun runOpMode() {
        hubs = hardwareMap.getAll(LynxModule::class.java)
        hubs.forEach { it.bulkCachingMode = LynxModule.BulkCachingMode.MANUAL }
        motors = listOf(HardwareNames.FLYWHEEL, HardwareNames.FLYWHEEL_2).map { name ->
            hardwareMap.get(DcMotorEx::class.java, name).apply {
                direction = DcMotorSimple.Direction.REVERSE
                zeroPowerBehavior = DcMotor.ZeroPowerBehavior.BRAKE
                mode = DcMotor.RunMode.STOP_AND_RESET_ENCODER
                mode = DcMotor.RunMode.RUN_WITHOUT_ENCODER
                power = 0.0
            }
        }
        // Same directions as the Feed and Intake subsystems.
        feed = hardwareMap.get(DcMotorEx::class.java, HardwareNames.FEED).apply {
            direction = DcMotorSimple.Direction.FORWARD
            mode = DcMotor.RunMode.RUN_WITHOUT_ENCODER
            zeroPowerBehavior = DcMotor.ZeroPowerBehavior.BRAKE
            power = 0.0
        }
        intake = hardwareMap.get(DcMotorEx::class.java, HardwareNames.INTAKE).apply {
            direction = DcMotorSimple.Direction.REVERSE
            mode = DcMotor.RunMode.RUN_WITHOUT_ENCODER
            zeroPowerBehavior = DcMotor.ZeroPowerBehavior.BRAKE
            power = 0.0
        }
        DIR.mkdirs()

        // Ignore anything left over from an earlier session.
        var lastSeq = readSeq() ?: -1L
        writeStatus("ready", lastSeq, "waiting for start")

        while (opModeInInit()) {
            telemetry.addLine("Remote Flywheel Tuner")
            telemetry.addLine("Press start, then let the laptop drive the tests.")
            telemetry.addLine("Keep clear of the shooter. Gamepad 1 X aborts a run.")
            telemetry.update()
            sleep(50)
        }
        writeStatus("ready", lastSeq, "running, waiting for commands")

        try {
            while (opModeIsActive()) {
                val seq = readSeq()
                if (seq != null && seq > lastSeq) {
                    lastSeq = seq
                    val command = try {
                        readCommand()
                    } catch (e: IllegalArgumentException) {
                        writeStatus("error", seq, e.message ?: "bad command")
                        null
                    }
                    if (command != null) execute(command)
                }
                telemetry.addLine("Waiting for the laptop")
                telemetry.addData("Last command", lastSeq)
                telemetry.addData("Last result", lastStatus)
                telemetry.update()
                sleep(100)
            }
        } finally {
            stopMotors()
            writeStatus("stopped", lastSeq, "OpMode stopped")
        }
    }

    private fun execute(c: Command) {
        writeStatus("running", c.seq, c.action)
        val rows = StringBuilder("t,phase,target,v1,v2,pos1,pos2,amps1,amps2,feedAmps,volts\n")
        val battery = hardwareMap.voltageSensor.iterator().next()

        if (c.action == "velocity") {
            motors.forEach {
                it.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, c.pidf)
                it.mode = DcMotor.RunMode.RUN_USING_ENCODER
            }
        } else {
            motors.forEach { it.mode = DcMotor.RunMode.RUN_WITHOUT_ENCODER }
        }

        val start = System.nanoTime()
        val total = c.settle + c.duration
        var aborted = false
        var lastPhase = ""
        while (true) {
            val elapsed = (System.nanoTime() - start) / 1e9
            if (elapsed >= total) break
            if (!opModeIsActive() || gamepad1.x) {
                aborted = true
                break
            }
            val phase = if (elapsed < c.settle) "settle" else "step"
            val target = if (c.action == "velocity") (if (phase == "settle") c.from else c.to) else c.power
            if (phase != lastPhase) {
                if (c.action == "velocity") motors.forEach { it.velocity = target }
                else motors.forEach { it.power = target }
                val feeding = c.feed && phase == "step"
                feed.power = if (feeding) 1.0 else 0.0
                intake.power = if (feeding) 1.0 else 0.0
                lastPhase = phase
            }

            hubs.forEach { it.clearBulkCache() }
            val v1 = motors[0].velocity
            val v2 = motors[1].velocity
            val a1 = motors[0].getCurrent(CurrentUnit.AMPS)
            val a2 = motors[1].getCurrent(CurrentUnit.AMPS)
            val feedAmps = if (c.feed) feed.getCurrent(CurrentUnit.AMPS) else 0.0
            rows.append(String.format(Locale.US, "%.4f,%s,%.1f,%.1f,%.1f,%d,%d,%.2f,%.2f,%.2f,%.2f\n",
                elapsed - c.settle, phase, target, v1, v2, motors[0].currentPosition,
                motors[1].currentPosition, a1, a2, feedAmps, battery.voltage))

            telemetry.addLine("Running command ${c.seq}: ${c.action}")
            telemetry.addData("Target", "%.0f", target)
            telemetry.addData("flywheel1 / flywheel2", "%.0f / %.0f", v1, v2)
            if (c.feed && phase == "step") telemetry.addLine("FEED + INTAKE ON: drop balls in now.")
            else if (c.feed) telemetry.addLine("Spinning up. Feed starts in %.1f s.".format(c.settle - elapsed))
            telemetry.addLine("Gamepad 1 X aborts.")
            telemetry.update()
        }
        stopMotors()

        val csv = File(DIR, "run-${c.seq}.csv")
        csv.writeText(rows.toString())
        lastStatus = if (aborted) "aborted ${c.seq}" else "done ${c.seq}"
        writeStatus(if (aborted) "aborted" else "done", c.seq, csv.name)
    }

    private fun stopMotors() {
        if (::feed.isInitialized) feed.power = 0.0
        if (::intake.isInitialized) intake.power = 0.0
        if (!::motors.isInitialized) return
        motors.forEach {
            it.mode = DcMotor.RunMode.RUN_WITHOUT_ENCODER
            it.power = 0.0
        }
    }

    private fun readProperties(): Properties? {
        val file = File(DIR, CMD_FILE)
        if (!file.exists()) return null
        return try {
            Properties().apply { file.reader().use { load(it) } }
        } catch (e: Exception) {
            null // half-written file; try again next loop
        }
    }

    private fun readSeq(): Long? = readProperties()?.getProperty("seq")?.toLongOrNull()

    private fun readCommand(): Command {
        val p = readProperties() ?: throw IllegalArgumentException("command file vanished")
        fun d(key: String, default: Double): Double =
            p.getProperty(key)?.toDoubleOrNull() ?: default

        val action = p.getProperty("action", "velocity")
        require(action == "velocity" || action == "power") { "action must be velocity or power" }
        val from = d("from", 0.0)
        val to = d("to", 0.0)
        val power = d("power", 0.0)
        val settle = d("settle", 0.0)
        val duration = d("duration", 3.0)
        require(from in 0.0..MAX_TARGET && to in 0.0..MAX_TARGET) { "targets must be 0..$MAX_TARGET" }
        require(power in 0.0..1.0) { "power must be 0..1" }
        require(settle in 0.0..MAX_SETTLE) { "settle must be 0..$MAX_SETTLE s" }
        require(duration in 0.1..MAX_DURATION) { "duration must be 0.1..$MAX_DURATION s" }
        val pidf = PIDFCoefficients(d("p", 0.0), d("i", 0.0), d("d", 0.0), d("f", 0.0))
        require(listOf(pidf.p, pidf.i, pidf.d, pidf.f).all { it >= 0 && it.isFinite() }) {
            "gains must be finite and not negative"
        }
        val feed = d("feed", 0.0) > 0
        return Command(p.getProperty("seq").toLong(), action, pidf, from, to, settle, duration, power, feed)
    }

    private fun writeStatus(state: String, seq: Long, detail: String) {
        val tmp = File(DIR, "$STATUS_FILE.tmp")
        tmp.writeText("state=$state\nseq=$seq\ndetail=$detail\n")
        tmp.renameTo(File(DIR, STATUS_FILE))
    }

    companion object {
        val DIR = File("/sdcard/FIRST/claude-tune")
        const val CMD_FILE = "cmd.properties"
        const val STATUS_FILE = "status.properties"
        const val MAX_TARGET = 3000.0
        const val MAX_SETTLE = 6.0
        const val MAX_DURATION = 20.0
    }
}
