package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.teamcode.ValueStorage.liftPositions;
import static java.lang.Math.PI;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

@Autonomous (name = "AutonomousBlue4_5" , group = "Blue")
public class AutonomousBlue4_5 extends AutonomousBlue4 {
    Pose2d configParkPose = new Pose2d(66, 38, -PI/2);
    TrajectorySequence traj7;
    TrajectorySequence traj8;

    @Override
    public void runOpMode() {
        initRobot();
        setupTrajForEverythingExceptPark();
        setupTrajForPark();
        runEverythingExceptPark();
        drive.followTrajectorySequence(traj7);
        drive.followTrajectorySequence(traj8);
    }

    protected void setupTrajForPark() {
        /**
         * Go the warehouse and take a cargo
         * TM1: Resets the lift , starts the intake, & gate up
         */

        traj7 = drive.trajectorySequenceBuilder(dropPose2)
                .setReversed(true)
                .addTemporalMarker(0.5, () -> {lift.setTargetPosition(liftPositions[3]);
                    intake.setPower(0);})
                .splineToSplineHeading(new Pose2d(9, 60, PI), PI - 2)
                .setVelConstraint(SampleMecanumDrive.getVelocityConstraint(30, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH))
                .splineToConstantHeading(new Vector2d(13, 65), 0)
                .lineTo(parkPose.vec())
                .resetVelConstraint()
                .build();
        traj8 = drive.trajectorySequenceBuilder(parkPose)
                .setTangent(-PI/2)
                .splineTo(configParkPose.vec(), 0)
                .build();

    }
}
