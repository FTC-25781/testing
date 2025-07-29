package org.firstinspires.ftc.teamcode.auto;

import com.pedropathing.localization.Pose;

public class OffsetUtils {

    public static Pose applyOffset(Pose pose, double dx, double dy) {
        double heading = pose.getHeading();
        double cos = Math.cos(heading);
        double sin = Math.sin(heading);

        double offsetX = dx * cos - dy * sin;
        double offsetY = dx * sin + dy * cos;

        return new Pose(
                pose.getX() - offsetX,
                pose.getY() - offsetY,
                heading
        );
    }

    public static boolean isTurning(double h1, double h2) {
        double delta = Math.abs(h1 - h2);
        return delta > Math.toRadians(5);
    }
}

