package org.firstinspires.ftc.ftc12069;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cColorSensor;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.OpticalDistanceSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * Created by Calvin on 16/10/27.
 */


public class HardwareCataclysm
{
    public DcMotor LBMotor   = null;
    public DcMotor RBMotor  = null;
    public DcMotor flickMotor    = null;
    public DcMotor conveyorMotor = null;
    public DcMotor collectionMotor = null;

    public OpticalDistanceSensor opticalDistanceSensor = null;
    public ModernRoboticsI2cGyro gyro = null;
    public ModernRoboticsI2cColorSensor colorSensorMR = null;

    /* local OpMode members. */
    HardwareMap hwMap           =  null;
    private ElapsedTime period  = new ElapsedTime();

    public HardwareCataclysm(){

    }

    public void init(HardwareMap ahwMap) {
        // Save reference to Hardware map
        hwMap = ahwMap;

        // Define and Initialize Motors
        //AHO3ELKU
        LBMotor = hwMap.dcMotor.get("LBMotor");
        RBMotor = hwMap.dcMotor.get("RBMotor");

        //AHO3ELKE
        flickMotor = hwMap.dcMotor.get("flickarm");
        conveyorMotor = hwMap.dcMotor.get("conveyor");

        //AL00VTAC
        collectionMotor = hwMap.dcMotor.get("collection");

        //Sensors
        opticalDistanceSensor = hwMap.opticalDistanceSensor.get("opticalDistanceSensor");
        colorSensorMR = (ModernRoboticsI2cColorSensor) hwMap.colorSensor.get("colorSensorMR");
        gyro = (ModernRoboticsI2cGyro) hwMap.gyroSensor.get("gyro");

        // Set run directions
        LBMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        RBMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        flickMotor.setDirection(DcMotor.Direction.FORWARD);
        conveyorMotor.setDirection(DcMotor.Direction.FORWARD);
        collectionMotor.setDirection(DcMotorSimple.Direction.FORWARD);

        //Initialize sensors
        gyro.calibrate();

        // Set stopped behaviors
        LBMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        RBMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        flickMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        conveyorMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        collectionMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Set all motors to zero power
        LBMotor.setPower(0);
        RBMotor.setPower(0);
        flickMotor.setPower(0);
        conveyorMotor.setPower(0);
        collectionMotor.setPower(0);

        // Set all motors to run without encoders.
        // May want to use RUN_USING_ENCODERS if encoders are installed.
        LBMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        RBMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        flickMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        conveyorMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        collectionMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);


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
