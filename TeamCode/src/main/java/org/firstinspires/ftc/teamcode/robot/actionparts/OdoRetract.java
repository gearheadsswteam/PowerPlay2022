package org.firstinspires.ftc.teamcode.robot.actionparts;

import com.qualcomm.robotcore.hardware.Servo;

public class OdoRetract {
    private Servo x;
    private Servo y;
    private Servo z;
    private double UP_POSITION;
    private double DOWN_POSITION;

    public OdoRetract(Servo x, Servo y, Servo z) {
        this.x = x;
        this.y = y;
        this.z = z;
    }

    public void activateOdo () {
        x.setPosition(DOWN_POSITION);
        y.setPosition(DOWN_POSITION);
        z.setPosition(DOWN_POSITION);
    }

    public void deactivateOdo () {
         x.setPosition(UP_POSITION);
         y.setPosition(UP_POSITION);
         z.setPosition(UP_POSITION);
    }

    public void initialize() {
        x.setDirection(Servo.Direction.FORWARD);
        y.setDirection(Servo.Direction.FORWARD);
        z.setDirection(Servo.Direction.FORWARD);
    }



}


