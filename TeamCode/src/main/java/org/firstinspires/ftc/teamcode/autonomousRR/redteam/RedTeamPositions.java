package org.firstinspires.ftc.teamcode.autonomousRR.redteam;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;

import org.firstinspires.ftc.teamcode.drive.PoseStorage;

public class RedTeamPositions {
    public static final Pose2d INIT_POS = PoseStorage.DEFAULT_AUTONOMOUS_INIT_POSITION;
    public static final Pose2d WG2_START_POS = new Pose2d(-60, -24, 0);;
    public static final Pose2d SHOOTING_POS = new Pose2d(4.21-12, -36.82, 0);
    public static final Pose2d PARK_RING_DROP_POS = new Pose2d(4.21+24, -36.82, -0.1);
    public static final Pose2d SHOOTING_POS_CASE_1 = new Pose2d(2.21-12, -36.82+3, -0.1);
    public static final Pose2d SHOOTING_POS_CASE_4 = new Pose2d(2.21-12, -36.82+3, -0.1);
    public static final Pose2d PARK_POS_CASE_4 = new Pose2d(4.21, -36.82, 0);
    public static final Vector2d RING_GRAB_POS = new Vector2d(-33.52,-19.7);
    public static final Vector2d WOBBLE_GOAL_2_PICKUP_XY= new Vector2d(-57.52, -7.70+48);
    public static final double WOBBLE_GOAL_2_PICKUP_HEADING= 5.121;
    public static final Pose2d PARK_ONLY_POSITION = new Pose2d(6, 0, 0);
    public static final Pose2d PARK_ONLY__RIGHT_LANE_POSITION = new Pose2d(6, -63, 0);
    public static final Pose2d PARK_ONLY__RIGHT_LANE_SHOOTING_POSITION = new Pose2d(-9, -59, 0.27);
    public static final Pose2d PARK_ONLY__RIGHT_LANE_SHOOTING_POSITION_CASE_1 = new Pose2d(-9, -59, 0.27);

}
