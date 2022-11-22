package org.firstinspires.ftc.teamcode.robot;

import com.acmerobotics.roadrunner.geometry.Pose2d;
public class ValueStorage {
    public static int redMultiplier = 1;
    public static int caseDetectionThreshold = 10;
    public static Pose2d lastPose = new Pose2d(0, 0, 0);

    //Values for Roller servo
    public static double ROLLER_DOWN = 0.5;
    public static double ROLLER_UP = 0.25;
    public static double ROLLER_RETRACT = 0.25; //TODO should be changed

    //Values for Gripper
    public static double GRIPPER_HOLDING = 0.46;
    public static double GRIPPER_RELEASE = 0.70;

    //Values for Elevaror height
    public static double [] ELEVATOR_GROUND = new double[]{0.1, 0.2, 0.3};
    public static double [] ELEVATOR_LOW = new double[]{0.1, 0.2, 0.3};
    public static double [] ELEVATOR_MED = new double[]{0.1, 0.2, 0.3};
    public static double [] ELEVATOR_HI = new double[]{0.1, 0.2, 0.3};



}