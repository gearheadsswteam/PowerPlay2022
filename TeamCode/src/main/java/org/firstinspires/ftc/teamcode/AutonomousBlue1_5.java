package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.teamcode.ValueStorage.liftPositions;
import static java.lang.Math.PI;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous (name = "AutonomousBlue1_5" , group = "Blue")
public class AutonomousBlue1_5 extends AutonomousBlue1 {
    //Wait for partner to move out of parking position
    Pose2d waitPose = new Pose2d(-30, 65, PI);
    //Parking position
    Pose2d parkPose = new Pose2d(42, 72, PI);

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
        runEverythingExceptParkBlue();
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