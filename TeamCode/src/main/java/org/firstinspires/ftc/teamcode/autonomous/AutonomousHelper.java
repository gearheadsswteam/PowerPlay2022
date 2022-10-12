package org.firstinspires.ftc.teamcode.autonomous;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

public class AutonomousHelper {
    private SampleMecanumDrive mecanumDriveRR;

    public AutonomousHelper(SampleMecanumDrive mecDrive){
        mecanumDriveRR = mecDrive;
    }

    public static Pose2d getPose2D(Vector2d vector2d){
        return new Pose2d(vector2d.getX(), vector2d.getY());
    }

    public static Vector2d getVector2d(Pose2d pose2d){
        return new Vector2d(pose2d.getX(), pose2d.getY());
    }

    public Trajectory moveRobot_splineToSplineHeading(Pose2d startPosition, Pose2d endPosition){
        Trajectory traj1 = mecanumDriveRR.trajectoryBuilder(startPosition, 0)
                .splineToSplineHeading(endPosition, 0)
                .build();

        return traj1;
    }

    public Trajectory moveRobot_splineTo(Pose2d startPosition, Pose2d endPosition){
        Trajectory traj1 = mecanumDriveRR.trajectoryBuilder(startPosition, 0).
                splineTo(getVector2d(endPosition), 0)
                .build();

        return traj1;
    }

    public Trajectory moveRobot_lineToSplineHeading(Pose2d startPosition, Pose2d endPosition){
        Trajectory traj1 = mecanumDriveRR.trajectoryBuilder(startPosition, 0).
                lineToSplineHeading(endPosition)
                .build();

        return traj1;
    }





}
