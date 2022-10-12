package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

import static com.qualcomm.robotcore.hardware.DcMotorSimple.Direction.REVERSE;
import static java.lang.Math.PI;
import static org.firstinspires.ftc.teamcode.ValueStorage.armDown;
import static org.firstinspires.ftc.teamcode.ValueStorage.armRest;
import static org.firstinspires.ftc.teamcode.ValueStorage.bucketDown;
import static org.firstinspires.ftc.teamcode.ValueStorage.bucketRest;
import static org.firstinspires.ftc.teamcode.ValueStorage.bucketUp;
import static org.firstinspires.ftc.teamcode.ValueStorage.cameraDetectionFrames;
import static org.firstinspires.ftc.teamcode.ValueStorage.clawClosed;
import static org.firstinspires.ftc.teamcode.ValueStorage.clawOpen;
import static org.firstinspires.ftc.teamcode.ValueStorage.gateDown;
import static org.firstinspires.ftc.teamcode.ValueStorage.gateUp;
import static org.firstinspires.ftc.teamcode.ValueStorage.liftPositions;
import static org.firstinspires.ftc.teamcode.ValueStorage.parkConfig;

@Autonomous (name = "AutonomousRed4_5" , group = "Red")
public class AutonomousRed4_5 extends AutonomousRed4 {
    Pose2d configParkPose = new Pose2d(62, -41, PI/2);
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
                .addTemporalMarker(0.5, () -> {
                    lift.setTargetPosition(liftPositions[3]);
                    intake.setPower(0);
                })
                .splineToSplineHeading(new Pose2d(9, -59, PI), 2 + PI)
                .setVelConstraint(SampleMecanumDrive.getVelocityConstraint(30, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH))
                .splineToConstantHeading(new Vector2d(13, -65), 0)
                .lineTo(parkPose.vec())
                .resetVelConstraint()
                .build();
        traj8 = drive.trajectorySequenceBuilder(parkPose)
                .setTangent(PI/2)
                .splineTo(configParkPose.vec(), 0)
                .build();

    }
}
