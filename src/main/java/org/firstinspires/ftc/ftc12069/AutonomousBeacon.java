package org.firstinspires.ftc.ftc12069;

import com.qualcomm.robotcore.eventloop.opmode.*;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.LightSensor;
import com.qualcomm.robotcore.hardware.OpticalDistanceSensor;
import com.qualcomm.robotcore.util.ElapsedTime;




//@Disabled

@com.qualcomm.robotcore.eventloop.opmode.Autonomous(name = "Pushbot: Auto drive to beacon with lego color sensor", group = "Pushbot")

// @Disabled
public class AutonomousBeacon extends LinearOpMode {
    private ElapsedTime runtime = new ElapsedTime();

    HardwareCataclysm robot = new HardwareCataclysm();   // Use Cataclysms hardware

        /* Declare OpMode members. */

    // could also use HardwarePushbotMatrix class.
    //--LightSensor lightSensor;      // Primary LEGO Light sensor,
    // OpticalDistanceSensor   lightSensor;   // Alternative MR ODS sensor
   //--- OpticalDistanceSensor opticalDistanceSensor;   // Alternative MR ODS sensor
         // Hardware Device Object
    ////////////////////////////////////////////////////
    static final double WHITE_THRESHOLD = 0.2;  // spans between 0.1 - 0.5 from dark to light
    static final double APPROACH_SPEED = 0.5;

    @Override
    public void runOpMode() {

        robot.init(hardwareMap);

        // get a reference to our Light Sensor object.
       //lightSensor = hardwareMap.lightSensor.get("sensor_light");                // Primary LEGO Light Sensor
        //  lightSensor = hardwareMap.opticalDistanceSensor.get("sensor_ods");  // Alternative MR ODS sensor.

        // turn on LED of light sensor.
      //----  robot.lightSensor.enableLed(true);


        // Wait for the game to start (driver presses PLAY)
        // Abort this loop is started or stopped.
        while (!(isStarted() || isStopRequested())) {

            // Display the light level while we are waiting to start
            telemetry.addData("Light Level", robot.lightSensor.getLightDetected());
            telemetry.update();
            idle();
        }
        DriveForward(0.55, -1);
        turn(0.3, -0.3, 0.6);
        DriveForward(0.65, -1);
        turn(-0.3, 0.3, 0.6);
        DriveForward(0.5, -1);
        line();

        DriveForward(0.2, -1);
        line();

        DriveForward(1.5, 1);
    }

    //sense line and remain driving on line
    public void line() {
        //double reflectance = opticalDistanceSensor.getLightDetected();
      //  robot.LBMotor.setPower(APPROACH_SPEED);
      //  robot.RBMotor.setPower(APPROACH_SPEED);

        // run until the white line is seen OR the driver presses STOP;
        while (opModeIsActive() && (robot.lightSensor.getLightDetected() < WHITE_THRESHOLD)) {
            //run til robot is 1 cm away


                // Display the light level while we are looking for the line
                telemetry.addData("Light Level", robot.lightSensor.getLightDetected());
                telemetry.update();

                while(robot.lightSensor.getLightDetected() < WHITE_THRESHOLD){
                    robot.LBMotor.setPower(0.2);
                    robot.RBMotor.setPower(0.2);
                }

                    robot.LBMotor.setPower(0.0);
                    robot.RBMotor.setPower(0.0);

            beacon();
        }
    }

    //sense color of beacon and press button
    public void beacon() {



    }

     public boolean wallDetection(double distance) {
        boolean detection;

        double reflectance = robot.opticalDistanceSensor.getLightDetected();
        if (reflectance <= distance) {
            detection = true;
        } else {
            detection = false;
        }
        return detection;
    }

    public void DriveForward(double time, int power) {


        robot.LBMotor.setPower(power);
        robot.RBMotor.setPower(power);
        runtime.reset();
        while (opModeIsActive() && (runtime.seconds() < time)) {
            telemetry.addData("Path", "Leg 1: %2.5f S Elapsed", runtime.seconds());
            telemetry.update();
        }
        robot.LBMotor.setPower(0);
        robot.RBMotor.setPower(0);
    }

    public void turn(double lPower, double rPower, double time) {
        robot.LBMotor.setPower(lPower);
        robot.RBMotor.setPower(rPower);
        runtime.reset();
        while (opModeIsActive() && (runtime.seconds() < time)) {
            telemetry.addData("Path", "Leg 1: %2.5f S Elapsed", runtime.seconds());
            telemetry.update();
        }
        robot.LBMotor.setPower(0);
        robot.RBMotor.setPower(0);
    }
}
