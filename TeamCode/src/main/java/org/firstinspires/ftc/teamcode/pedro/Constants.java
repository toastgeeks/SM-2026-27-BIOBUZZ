package org.firstinspires.ftc.teamcode.pedro;

import com.pedropathing.algorithm.Foresight;
import com.pedropathing.algorithm.ForesightConfig;
import com.pedropathing.controllers.Controller;
import com.pedropathing.follower.Follower;
import com.pedropathing.math.Matrix;
import com.pedropathing.math.Vector2D;
import com.pedropathing.revhub.drivetrains.Mecanum;
import com.pedropathing.revhub.drivetrains.MecanumConfig;
import com.pedropathing.revhub.localizers.PinpointConfig;
import com.pedropathing.revhub.localizers.PinpointLocalizer;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.HardwareNames;

public class Constants {
    public static MecanumConfig drivetrainConfig = new MecanumConfig(
            c -> {
                // Same motors and directions as DriveTrainTest on sweet2.
                // Positive power rolls each wheel forward.
                c.frontLeftName.set(HardwareNames.FRONT_LEFT);
                c.frontRightName.set(HardwareNames.RIGHT_FRONT);
                c.backLeftName.set(HardwareNames.BACK_LEFT);
                c.backRightName.set(HardwareNames.BACK_RIGHT);

                c.frontLeftDirection.set(DcMotorSimple.Direction.REVERSE);
                c.frontRightDirection.set(DcMotorSimple.Direction.REVERSE);
                c.backLeftDirection.set(DcMotorSimple.Direction.REVERSE);
                c.backRightDirection.set(DcMotorSimple.Direction.FORWARD);
            });

    public static PinpointConfig localizerConfig = new PinpointConfig(c -> {
        c.name.set("pinpoint");
        c.podType.set(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        c.xPodOffset.set(5.495435459407297);
        c.yPodOffset.set(-2.8781869843250187); // tuner said +2.878 but measured with yPod FORWARD; sign flips with REVERSED
        c.xPodDirection.set(GoBildaPinpointDriver.EncoderDirection.FORWARD);
        c.yPodDirection.set(GoBildaPinpointDriver.EncoderDirection.REVERSED);
        c.globalDistanceUnit.set(DistanceUnit.INCH);
        c.offsetUnits.set(DistanceUnit.INCH);
    });

    public static ForesightConfig foresightConfig = new ForesightConfig(
            c -> {
                Controller primaryTranslationalForward = Controller.proportional(0.34536109484013233);
                Controller secondaryTranslationalForward = Controller.proportional(0.12760169632501625);
                Controller primaryTranslationalLateral = Controller.proportional(0.5928073592689482);
                Controller secondaryTranslationalLateral = Controller.proportional(0.2190264791455053);

                c.forwardTranslational.set(Controller.piecewise(secondaryTranslationalForward).put(2.5, primaryTranslationalForward));
                c.strafeTranslational.set(Controller.piecewise(secondaryTranslationalLateral).put(2.5, primaryTranslationalLateral));

                c.coast.set(Controller.proportionalFeedforward(0.01326940956580829));
                c.brake.set(Controller.proportionalFeedforward(0.011278998130937046));

                c.headingFeedback.set(Controller.proportional(4.927849800150835));
                c.headingBrakeCoefficients.set(Vector2D.cartesian(0.0508110786963714, 0.00974878119033074));

                c.linearBrakeCoefficients.set(Matrix.diag(0.10663572698266251, 0.03741500180127585));
                c.quadraticBrakeCoefficients.set(Matrix.diag(0.0019264551821271468, 0.003180933004212572));

                c.maxAchievableForwardVelocity.set(73.21790770038498);
                c.maxAchievableStrafeVelocity.set(55.93559077669336);
                c.naturalForwardDeceleration.set(25.96082184296781);
                c.naturalStrafeDeceleration.set(54.14947708121945);
            }
    );

    public static Follower create(HardwareMap h) {
        return new Follower(
                new PinpointLocalizer(h, localizerConfig),
                new Mecanum(h, drivetrainConfig),
                new Foresight(foresightConfig)
        );
    }
}