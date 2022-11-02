package org.firstinspires.ftc.teamcode.autonomousRR.redteam.auto3;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.autonomousRR.AbstractAutonomousOpModeRR;
import org.firstinspires.ftc.teamcode.autonomousRR.redteam.auto3.RedAuto3;

@Autonomous(name = "RedAutonomousModeRR3", group = "Red")
public class RedAutonomousModeRR3 extends AbstractAutonomousOpModeRR {

    Pose2d initPose = new Pose2d(0, 0, 0);


    public RedAutonomousModeRR3() {
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
        signal = 1;
        RedAuto3 redAuto3 = new RedAuto3(mecanumDriveRR);
        redAuto3.executeDrive(signal);
    }
}
