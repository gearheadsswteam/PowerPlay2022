package org.firstinspires.ftc.teamcode.autonomous.redteam;

import com.acmerobotics.roadrunner.geometry.Pose2d;

import org.firstinspires.ftc.teamcode.drive.PoseStorage;

import static java.lang.Math.PI;

public class RedTeamPositions {
    public static final Pose2d initPose = new Pose2d(-39, -62, PI / 2);
    public static final Pose2d INIT_POSTION_LEFT = initPose;

    //Josh

    public static final Pose2d[] grabPose = {new Pose2d(-44, -56.5, PI / 2), new Pose2d(-36, -56.5, PI / 2), new Pose2d(-36, -54, 1.15)};
    public static final Pose2d dropPose = new Pose2d(-24, -33, 0.7);
    public static final Pose2d spinnerPose = new Pose2d(-58, -57, 1.1);
    public static final Pose2d dropPose2 = new Pose2d(-21, -36, 0.9);
    public static final Pose2d parkPose = new Pose2d(-60, -33, Math.PI);

}
