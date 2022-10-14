package org.firstinspires.ftc.teamcode.drive;

import com.acmerobotics.roadrunner.geometry.Pose2d;

import org.firstinspires.ftc.teamcode.autonomousRR.AbstractAutonomousOpModeRR;

/**
 * Simple static field serving as a storage medium for the bot's pose.
 * This allows different classes/opmodes to set and read from a central source of truth.
 * A static field allows data to persist between opmodes.
 */
public class PoseStorage {
    public static final Pose2d DEFAULT_AUTONOMOUS_INIT_POSITION = new Pose2d(-60, -48, 0);
    public static Pose2d currentPose = DEFAULT_AUTONOMOUS_INIT_POSITION;
    public static double gyroAngle = 0;
    //Team type
    public static String TEAM_TYPE = AbstractAutonomousOpModeRR.RED_TEAM;
}