# Route autos

A route auto is a short list of steps: where the robot starts, where it drives, and when it shoots or waits. You write the route in Kotlin, and the robot follows it with Pedro Pathing.

```kotlin
@Autonomous(name = "Far Start Park Red", group = "Auto")
class FarStartParkRed : RouteAuto(
    route(start = C6 against FAR, facing = FAR) {
        wait(5000)
        shoot(velocity = 1137, feedSeconds = 3)
        driveTo(C5, facing = RED)
        driveTo(at(A, 5.5).offWall(RED, gap = 6))
    },
)
```

Read it top to bottom. The robot starts in C6, touching the far wall and facing it. It waits 5 seconds and shoots. It drives to the center of C5, turning to face red on the way. Then it drives into column A, halfway between rows 5 and 6, and stops 6 inches short of the red wall.

## The field

This is the field as seen from the audience. Every route uses it, red and blue alike; nothing is mirrored.

```
                               FAR wall
           A        B        C        D        E        F
 6.5   +--------+--------+--------+--------+--------+--------+
       |        |        |        |        |        |        |
   6   |        |  o B6  |        |        |        |        |
 5.5   +--------+---x----+--------+--------+--------*--------+
       |  red   |        |        |        |        |        |
   5   |loading |        |        |        |        |        |
 4.5   +--------+--------+--------+--------+--------+--------+
       |        |        |                 |        |        | B
   4  R|        |        |      HIVE       |        |        | L
 3.5  E+--------+--------+                 +--------+--------+ U
      D|        |        |     C3:D4       |        |        | E
   3   |        |        |                 |        |        |
 2.5   +--------+--------+--------+--------+--------+--------+
       |        |        |        |        |        |  blue  |
   2   |        |        |        |        |        |loading |
 1.5   +--------+--------+--------+--------+--------+--------+
       |        |        |        |        |        |        |
   1   |        |        |   C1   |        |        |        |
 0.5   +--------+--------+--------+--------+--------+--------+
     A-0.5    A+0.5    B+0.5    C+0.5    D+0.5    E+0.5    F+0.5
                            AUDIENCE wall
```

| Mark | Code | Meaning |
|---|---|---|
| `o` | `at(B, 6)` or `B6` | center of tile B6 |
| `x` | `at(B, 5.5)` | column B, on the line between rows 5 and 6 |
| `*` | `at(E + 0.5, 5.5)` | the corner where E5, E6, F5 and F6 meet |

Each tile is 24 in square. The HIVE fills C3:D4, and the robot must stay out of it.

## Naming a spot

Every spot is `at(column, row)`.

- **Whole numbers are tile centers.** Row `5` is the middle of row 5, and column `B` is the middle of column B.
- **Add `.5` for the line between tiles.** Row `5.5` is the line between rows 5 and 6. Column `A + 0.5` is the line between A and B.
- **Anything in between works.** `at(A, 5.25)` is a quarter of the way from A5's center toward A6.
- **Tile names are shorthand.** `C1` means `at(C, 1)`.

The numbers along the left edge and under the diagram show these values for every grid line. The walls are at row 0.5 and 6.5, and at columns `A - 0.5` and `F + 0.5`.

## Walls

The robot is about 18 in square, so its center is 9 in from its edge. Two helpers place it relative to a wall:

| Code | Result |
|---|---|
| `C6 against FAR` | slides to touch the far wall |
| `F2.offWall(BLUE, gap = 6)` | slides to stop 6 in short of the blue wall |

The walls are `AUDIENCE`, `FAR`, `RED` (the A side), and `BLUE` (the F side). Starting against a wall is fine. **Ending touching a wall is a penalty**, so end with `offWall`. Both helpers work on any spot, such as `at(A, 5.5).offWall(RED, gap = 6)`.

## Facing

`facing` takes `AUDIENCE`, `FAR`, `RED`, `BLUE`, or `degrees(45)`. Degrees count counterclockwise from facing blue, so `degrees(90)` faces the far wall. A curve can also use `FORWARD` or `BACKWARD` to face along the path.

The start always needs a `facing`. On a later step, leave it out to keep the current heading. A new heading turns smoothly over the whole leg, the short way round.

## Steps

| Step | What the robot does |
|---|---|
| `driveTo(spot)` | drives in a straight line |
| `driveTo(spot, facing = RED)` | drives in a straight line, turning on the way |
| `curveTo(spot, via = other)` | drives a curve bent toward `other`; it does not pass through `other`. Add more `via` spots after it for more bend. |
| `shoot(velocity = 1137, feedSeconds = 3)` | stops and brakes, spins the flywheels to the velocity (ticks/s), runs the feed and intake for the given time, then turns them all off |
| `spinUp(velocity = 1137)` | starts the flywheels and goes straight on to the next step |
| `shoot(feedSeconds = 3)` | the same as `shoot` above, at the velocity from the last `spinUp` |
| `wait(500)` | stops and brakes for 500 ms |

`shoot` starts feeding once the flywheel is within 25 ticks/s of the target. If it isn't there after 4 s, it feeds anyway and telemetry says so.

The flywheels take about 1.6 s to reach speed, so when the robot drives before it shoots, spin up first and let them get there on the way:

```kotlin
spinUp(velocity = 1137)
driveTo(C5, facing = RED)   // flywheels spin up during the drive
shoot(feedSeconds = 3)      // feeds as soon as they're at speed
```

Each shot turns the flywheels off, so a second shot needs its own `spinUp` or velocity.

After the last step, the robot holds its final spot for 1 s, then stops and brakes. It doesn't keep holding, because a spot a little past a wall would make it push into the wall until the end of AUTO.

## Checks

`RoutePlanner` checks every route against the robot's footprint at its heading along each leg. Facing square to the field, the robot sticks out 9 in from its center; at 45 degrees, its corners stick out 12.7 in. It warns when:

- the robot hits the HIVE, or passes within 2 in of it;
- the robot goes into a wall, including a corner swinging into one during a turn;
- the route ends touching a wall;
- a shot or spin-up asks for more than 1460 ticks/s, about the top speed on a drained battery;
- the route ends with the flywheels still spinning from a `spinUp`.

Warnings show on the Driver Station at INIT, under `!! CHECK ROUTE !!`, along with every leg and its end point. Some mistakes are errors instead: a route with no steps, a leg that ends where it starts, a shot or wait of 0 or less, or `shoot(feedSeconds = ...)` with no `spinUp` before it. On an error, the Driver Station shows it and the robot does not move.

`AllAutosTest` finds every `RouteAuto` in the `auto` package and fails on any warning or error, so a new auto is checked without writing a test for it.

## Writing a new auto

1. Copy one of the autos in `auto/` and give it a new class name and `@Autonomous(name = ...)`.
2. Sketch the route on the diagram above, then write the steps.
3. Run `./gradlew :TeamCode:testDebugUnitTest`. Fix anything `AllAutosTest` reports.
4. Deploy, select the auto, and press INIT. Check the leg list and look for warnings before pressing start.

## Inches

Telemetry shows the robot's position in inches, with the origin in the A1 corner (red wall, audience wall). The x value runs toward blue, and y toward the far wall. To convert:

- x = (column − 0.5) × 24, where A is 1 and F is 6. So column B is x = 36.
- y = (row − 0.5) × 24. So row 5.5 is y = 120.

Heading 0 faces blue, and 90 degrees faces the far wall.

## Files

| File | Holds |
|---|---|
| `Field.kt` | spot names (`at`, `C1`, `against`, `offWall`), facings, and the `route { }` builder |
| `RoutePlanner.kt` | turns a route into steps and runs the checks; no robot code, so it runs in unit tests |
| `RouteCommands.kt` | turns the planned steps into one Ivy command, run through the same `Scheduler` as Teleop; talks to the robot only through `RouteDrive` and `RouteMechanisms`, so it runs in unit tests with fake hardware |
| `../RouteAuto.kt` | the OpMode: shows the route at INIT, connects Pedro and the `Shooter`, `Feed`, and `Intake` commands to `RouteCommands`, and runs the scheduler |
| `TeamCode/src/test/.../RoutePlannerTest.kt` | tests for spots, headings, and checks |
| `TeamCode/src/test/.../RouteCommandsTest.kt` | runs routes through the Ivy scheduler and checks the order of what the robot does |
| `TeamCode/src/test/.../AllAutosTest.kt` | plans every auto and fails on any warning |
