package org.firstinspires.ftc.teamcode.autonomousRR.redteam.auto1;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

public class RedAuto1 {

    Pose2d initPose = new Pose2d(64.26, -36.42, 3.12);
    Pose2d pose3 = new Pose2d(10.40, -31.38, 2.43);
    Pose2d parkPose1 = new Pose2d(14.90, -58.18, 3.16);
    Pose2d parkPose2 = new Pose2d(13.5, -35.49, 3.12);
    Pose2d parkPose3 = new Pose2d(15.54, -11.19, 3.15);

    TrajectorySequence traj1;
    TrajectorySequence traj2;

    SampleMecanumDrive mecanumDriveRR;

    public RedAuto1(SampleMecanumDrive mecanumDriveRR) {
        this.mecanumDriveRR = mecanumDriveRR;
    }

    public void initCoreRoute() {
        mecanumDriveRR.setPoseEstimate(initPose);
        traj1 = mecanumDriveRR.trajectorySequenceBuilder(initPose)
                .lineToLinearHeading(parkPose2)
             //   .waitSeconds(1)
                .lineToLinearHeading(pose3)
            //    .waitSeconds(1)
                .lineToLinearHeading(parkPose2)
            //    .waitSeconds(1)
                .lineToLinearHeading(parkPose1)
                .build();
    }

    public void initParkRoute(int caseId) {
        if (caseId == 1) {
            traj2 = mecanumDriveRR.trajectorySequenceBuilder(traj1.end())
                    .lineTo(new Vector2d(parkPose1.getX(), parkPose1.getY()))
                    .build();
        }
        if (caseId == 3) {
            traj2 = mecanumDriveRR.trajectorySequenceBuilder(traj1.end())
                    .lineTo(new Vector2d(parkPose3.getX(), parkPose3.getY()))
                    .build();
        } else {
            //Park 2 no-op as the robot is already there in traj 1
        }

    }


    public void executeDrive(int caseID) {
        initCoreRoute();
        initParkRoute(caseID);
        mecanumDriveRR.followTrajectorySequence(traj1);
        if (traj2 != null) {
            mecanumDriveRR.followTrajectorySequence(traj2);
        }
    }
}



