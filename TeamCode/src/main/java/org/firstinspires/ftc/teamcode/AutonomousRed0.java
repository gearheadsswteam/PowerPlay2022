package org.firstinspires.ftc.teamcode;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous(name = "AutonomousRed0" , group = "Red")
public class AutonomousRed0 extends LinearOpMode {
    public void runOpMode() {
        waitForStart();
        ValueStorage.lastIntakeState = 0;
        ValueStorage.lastLiftState = 0;
        ValueStorage.lastArmState = 0;
        ValueStorage.redMultiplier = 1;
        ValueStorage.lastPose = new Pose2d(-39, -62, Math.PI/2);
    }
}
