package org.firstinspires.ftc.teamcode.autonomousRR.redteam.auto1;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.autonomousRR.AbstractAutonomousOpModeRR;
import org.firstinspires.ftc.teamcode.autonomousRR.redteam.auto2.RedAuto2;

@Autonomous(name = "RedAutonomousModeRR1", group = "Red")
public class RedAutonomousModeRR1 extends AbstractAutonomousOpModeRR {

    Pose2d initPose = new Pose2d(0,0, 0);


    public RedAutonomousModeRR1() {
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
        signal = robot.signalDetector.getCaseDetected();
        signal = 3;
        RedAuto1 redAuto1 = new RedAuto1(mecanumDriveRR);
        redAuto1.executeDrive(signal);
    }
}
