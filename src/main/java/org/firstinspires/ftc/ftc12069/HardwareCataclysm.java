package org.firstinspires.ftc.ftc12069;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * Created by Calvin on 16/10/27.
 */

public class HardwareCataclysm
{
    public DcMotor LBMotor   = null;
    public DcMotor  RBMotor  = null;
    public DcMotor  flickMotor    = null;
    public Servo conveyorServo = null;
    public Servo armLeft = null;
    public Servo armRight = null;

    public static final double FLICK_POWER = 1;
    public static final double FLICK_POWER_REVERSE = -1;

    /* local OpMode members. */
    HardwareMap hwMap           =  null;
    private ElapsedTime period  = new ElapsedTime();

    public HardwareCataclysm(){

    }

    public void init(HardwareMap ahwMap) {
        // Save reference to Hardware map
        hwMap = ahwMap;

        // Define and Initialize Motors
        LBMotor = hwMap.dcMotor.get("leftBackMotor");
        RBMotor = hwMap.dcMotor.get("rightBackMotor");
        flickMotor = hwMap.dcMotor.get("flickArm");

        LBMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        RBMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        flickMotor.setDirection(DcMotor.Direction.FORWARD);

        // Set all motors to zero power
        LBMotor.setPower(0);
        RBMotor.setPower(0);
        flickMotor.setPower(0);

        // Set all motors to run without encoders.
        // May want to use RUN_USING_ENCODERS if encoders are installed.
        LBMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        RBMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        flickMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // Define conveyor servos
        conveyorServo = hwMap.servo.get("conveyorServo");
        armLeft = hwMap.servo.get("armLeft");
        armRight = hwMap.servo.get("armRight");

        conveyorServo.setPosition(0);
        armLeft.setPosition(0.5);
        armRight.setPosition(0.5);
    }

    public void waitForTick(long periodMs) {

        long remaining = periodMs - (long) period.milliseconds();

        // sleep for the remaining portion of the regular cycle period.
        if (remaining > 0) {
            try {
                Thread.sleep(remaining);
            } catch (InterruptedException e) {
                Thread.currentThread().interrupt();
            }
        }

        // Reset the cycle clock for the next pass.
        period.reset();
    }
}
