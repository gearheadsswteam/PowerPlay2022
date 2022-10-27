package org.firstinspires.ftc.teamcode.autonomousRR.redteam;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.autonomousRR.AbstractAutonomousOpModeRR;
import org.firstinspires.ftc.teamcode.robot.GearheadsMecanumRobotRR;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

import java.sql.SQLOutput;

@Autonomous(name = "RedAutonomousModeRR", group = "Red")
public class RedAutonomousModeRR extends AbstractAutonomousOpModeRR {

    Pose2d initPose = new Pose2d(0,0, 0);


    public RedAutonomousModeRR() {
        super.TEAM_TYPE = AbstractAutonomousOpModeRR.RED_TEAM;
    }

    @Override
    protected void initOpModeBeforeStart() {
        super.initOpModeBeforeStart();
        mecanumDriveRR.setPoseEstimate(initPose);
        sleep(500);
        telemetry.addData("Status", "Initialized");
        telemetry.update();
    }


    @Override
    protected void initOpModeAfterStart() {

    }

    @Override
    protected void executeOpMode() {
        if (signal == 1) {
            System.out.println("Execute Case 1");
        } else if(signal == 2) {
            System.out.println("Execute Case 2");
        }  else if (signal == 3) {
            System.out.println("Execute Case 3");
        }
    }
}
