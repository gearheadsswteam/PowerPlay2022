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
import static org.firstinspires.ftc.teamcode.ValueStorage.liftPositions;

@Autonomous (name = "AutonomousRed1_5" , group = "Red")
public class AutonomousRed1_5 extends AutonomousRed1 {
    //Wait for partner to move out of parking position
    Pose2d waitPose = new Pose2d(-30, -63, PI);
    //Parking position
    Pose2d parkPose = new Pose2d(38, -63, PI);
    //Configurable parking delay
    long PARK_DELAY = 1500;
    //Initialize parking trajectories
    Trajectory traj5;
    Trajectory traj6;

    @Override
    public void runOpMode() {
        initRobot();
        setupTrajFordeliverCargoAndDuck();
        setupTrajForpark();
        runEverythingExceptPark();
        drive.followTrajectory(traj5);
        sleep(PARK_DELAY);
        drive.followTrajectory(traj6);
        ValueStorage.lastPose = drive.getPoseEstimate();
    }
    //Set up parking trajectories
    void setupTrajForpark() {
        /**
         * Park in the storage unit
         */
        traj5 = drive.trajectoryBuilder(dropPose)
                .lineToLinearHeading(waitPose)
                .addTemporalMarker(0.5, () -> {lift.setTargetPosition(liftPositions[3]);})
                .build();
        traj6 = drive.trajectoryBuilder(waitPose)
                .lineTo(parkPose.vec())
                .build();

    }

}