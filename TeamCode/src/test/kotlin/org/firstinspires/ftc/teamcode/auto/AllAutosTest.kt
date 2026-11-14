package org.firstinspires.ftc.teamcode.auto

import org.firstinspires.ftc.teamcode.auto.route.RouteException
import org.firstinspires.ftc.teamcode.auto.route.RoutePlanner
import org.junit.Assert.assertTrue
import org.junit.Assert.fail
import org.junit.Test
import java.io.File
import java.lang.reflect.Modifier
import java.util.jar.JarFile

/**
 * Plans every [RouteAuto] in this package, so a new auto is checked without adding a test.
 * Any route error or warning (HIVE, wall, flywheel) fails here, the same as it would show at INIT.
 */
class AllAutosTest {
    /** Class names in the auto package, read from the folder or jar that holds [RouteAuto]. */
    private fun autoClassNames(): List<String> {
        val dir = RouteAuto::class.java.`package`!!.name.replace('.', '/') + "/"
        val source = File(RouteAuto::class.java.protectionDomain!!.codeSource.location.toURI())
        val paths = if (source.isDirectory) {
            File(source, dir).listFiles().orEmpty().map { dir + it.name }
        } else {
            JarFile(source).use { jar -> jar.entries().toList().map { it.name } }
        }
        return paths
            .filter { it.startsWith(dir) && it.endsWith(".class") && '/' !in it.removePrefix(dir) }
            .map { it.removeSuffix(".class").replace('/', '.') }
    }

    private fun autos(): List<Class<out RouteAuto>> =
        autoClassNames().map { Class.forName(it) }
            .filter { RouteAuto::class.java.isAssignableFrom(it) && !Modifier.isAbstract(it.modifiers) }
            .map { it.asSubclass(RouteAuto::class.java) }
            .sortedBy { it.simpleName }

    @Test fun findsAutos() {
        val names = autos().map { it.simpleName }
        assertTrue("found $names", "FarStartParkRed" in names && "LoadingZoneParkBlue" in names)
    }

    @Test fun everyAutoPlansCleanly() {
        val autos = autos()
        assertTrue("no autos found", autos.isNotEmpty())
        val problems = autos.flatMap { auto ->
            val spec = auto.getDeclaredConstructor().newInstance().spec
            try {
                RoutePlanner.plan(spec).warnings.map { "${auto.simpleName}: $it" }
            } catch (e: RouteException) {
                listOf("${auto.simpleName}: ${e.message}")
            }
        }
        if (problems.isNotEmpty()) fail(problems.joinToString("\n"))
    }
}
