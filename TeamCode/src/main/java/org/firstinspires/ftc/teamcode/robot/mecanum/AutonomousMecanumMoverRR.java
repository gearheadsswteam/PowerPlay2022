package org.firstinspires.ftc.teamcode.robot.mecanum;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.acmerobotics.roadrunner.trajectory.TrajectoryBuilder;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.robot.GearheadsMecanumRobotRR;

public class AutonomousMecanumMoverRR {
    public SampleMecanumDrive drive;
    public GearheadsMecanumRobotRR robot = null;   // Use a Gearbot's hardware
    private LinearOpMode curOpMode = null;   //current opmode

    /**
     * Constructor
     *
     * @param gearheadsRobot robot to use
     * @param myOpMode       opmode that is executing
     */
    public AutonomousMecanumMoverRR(GearheadsMecanumRobotRR gearheadsRobot, LinearOpMode myOpMode, SampleMecanumDrive mecanumDrive) {
        robot = gearheadsRobot;
        curOpMode = myOpMode;
        drive = mecanumDrive;
    }

    public TrajectoryBuilder trajectoryBuilder(Pose2d startPose) {
        return drive.trajectoryBuilder(startPose);
    }

    public TrajectoryBuilder trajectoryBuilder(Pose2d startPose, boolean reversed) {
        return drive.trajectoryBuilder(startPose, reversed);
    }

    public TrajectoryBuilder trajectoryBuilder(Pose2d startPose, double startHeading) {
        return drive.trajectoryBuilder(startPose, startHeading);
    }

    public void turnAsync(double angle) {
        drive.turnAsync(angle);
    }

    public void turn(double angle) {
        drive.turn(angle);
    }

    public void followTrajectoryAsync(Trajectory trajectory) {
        drive.followTrajectoryAsync(trajectory);
    }

    public void followTrajectory(Trajectory trajectory) {
        drive.followTrajectory(trajectory);
    }

    public void setPoseEstimate(Pose2d initPos){
        drive.setPoseEstimate(initPos);
    }

    public Pose2d getPoseEstimate(){
        return drive.getPoseEstimate();
    }
}
