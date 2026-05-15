package org.firstinspires.ftc.teamcode.Util;

import com.pedropathing.geometry.Pose;

public enum StartPosition {
    BLUE_BACK(56, 8.355, 90),
    BLUE_FRONT(24, 136, 270),
    RED_BACK(104, 8.355, 90),
    RED_FRONT(120, 136, 270);

    public final Pose pose;
    StartPosition(double x, double y, double deg) {
        this.pose = new Pose(x, y, Math.toRadians(deg));
    }
}