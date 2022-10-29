package org.firstinspires.ftc.teamcode.autonomousRR.redteam.auto2;

import com.acmerobotics.roadrunner.geometry.Pose2d;

import org.firstinspires.ftc.teamcode.autonomousRR.redteam.AutoCase;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

public class RedAutoCase3 implements AutoCase {

    Pose2d initPose = new Pose2d(64.26,-36.42, 3.12);
    //    Pose2d pose1 = new Pose2d(60.41,-33.69,2.47);
//    Pose2d pose2 = new Pose2d(46.03,-34.68,6.27);
    Pose2d pose3 = new Pose2d(10.40,-31.38,2.43);
    Pose2d parkPose1 = new Pose2d(14.90,-58.18,3.16);
    Pose2d parkPose2 = new Pose2d(13.5,-35.49,3.12);
    Pose2d parkPose3 = new Pose2d(15.54,-11.19,3.15);


    TrajectorySequence traj3;
    SampleMecanumDrive mecanumDriveRR;

    public RedAutoCase3(SampleMecanumDrive mecanumDriveRR){
        this.mecanumDriveRR = mecanumDriveRR;
    }

    @Override
    public void executeDrive() {
        mecanumDriveRR.setPoseEstimate(initPose);
        traj3 = mecanumDriveRR.trajectorySequenceBuilder(initPose)
                .lineToLinearHeading(parkPose2)
                .waitSeconds(1)
                .lineToLinearHeading(pose3)
                .waitSeconds(1)
                .lineToLinearHeading(parkPose2)
                .waitSeconds(1)
                .lineToLinearHeading(parkPose3)
                .build();
        mecanumDriveRR.followTrajectorySequence(traj3);
    }
}