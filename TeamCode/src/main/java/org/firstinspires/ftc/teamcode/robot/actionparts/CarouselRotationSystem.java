package org.firstinspires.ftc.teamcode.robot.actionparts;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

/**
 * Subsystem that manages the spinning of the Carousel
 */
public class CarouselRotationSystem {
    //The Continuous servo used for Carousel spinning
    private CRServo carouselSpinServo;

    /**
     * Consttuctor
     * @param carouselSpinServo
     */
    public CarouselRotationSystem(CRServo carouselSpinServo) {
        this.carouselSpinServo = carouselSpinServo;
    }

    /**
     * Initialize the Subsystem
     */

    public void initialize() {
        carouselSpinServo.setDirection(DcMotorSimple.Direction.FORWARD);
    }

    /**
     * Spins the carousel clockwise
      */
    public void rotateClockWise() {
        carouselSpinServo.setPower(1);
    }

    /**
     * Spins the carousel anti clockwise
     */

    public void rotateAntiClockWise() {
        carouselSpinServo.setPower(-1);
    }

    /**
     * Stops the carousel
     */
    public void stop() {
        carouselSpinServo.setPower(0);
    }
}
