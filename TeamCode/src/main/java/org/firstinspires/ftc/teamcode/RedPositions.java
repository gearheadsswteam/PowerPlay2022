package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.geometry.Pose2d;

import static java.lang.Math.PI;

public class RedPositions {
    public static Pose2d initPose = new Pose2d(-39, -62, PI / 2);
    public static Pose2d[] grabPose = {new Pose2d(-43, -49, PI / 2), new Pose2d(-35, -49, PI / 2), new Pose2d(-33, -47, 1.15)};
    public static Pose2d dropPose = new Pose2d(-24, -33, 0.7);
    public static Pose2d spinnerPose = new Pose2d(-56.5, -57.5, 1);
}
