/*
All rights reserved.

Redistribution and use in source and binary forms, with or without modification,
are permitted (subject to the limitations in the disclaimer below) provided that
the following conditions are met:

Redistributions of source code must retain the above copyright notice, this list
of conditions and the following disclaimer.

Redistributions in binary form must reproduce the above copyright notice, this
list of conditions and the following disclaimer in the documentation and/or
other materials provided with the distribution.

Neither the name of Robert Atkinson nor the names of his contributors may be used to
endorse or promote products derived from this software without specific prior
written permission.

NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
"AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESSFOR A PARTICULAR PURPOSE
ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR
TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF
THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/
package org.firstinspires.ftc.ftc12069;

//import android.app.Activity;
//import android.graphics.Color;
//import android.view.View;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;
//import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
//import com.qualcomm.robotcore.hardware.LightSensor;
import com.qualcomm.robotcore.hardware.OpticalDistanceSensor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcontroller.external.samples.HardwarePushbot;

/**
 * This file illustrates the concept of driving a path based on Gyro heading and encoder counts.
 * It uses the common Pushbot hardware class to define the drive on the robot.
 * The code is structured as a LinearOpMode
 * <p>
 * The code REQUIRES that you DO have encoders on the wheels,
 * otherwise you would use: PushbotAutoDriveByTime;
 * <p>
 * This code ALSO requires that you have a Modern Robotics I2C gyro with the name "gyro"
 * otherwise you would use: PushbotAutoDriveByEncoder;
 * <p>
 * This code requires that the drive Motors have been configured such that a positive
 * power command moves them forward, and causes the encoders to count UP.
 * <p>
 * This code uses the RUN_TO_POSITION mode to enable the Motor controllers to generate the run profile
 * <p>
 * In order to calibrate the Gyro correctly, the robot must remain stationary during calibration.
 * This is performed when the INIT button is pressed on the Driver Station.
 * This code assumes that the robot is stationary when the INIT button is pressed.
 * If this is not the case, then the INIT should be performed again.
 * <p>
 * Note: in this example, all angles are referenced to the initial coordinate frame set during the
 * the Gyro Calibration process, or whenever the program issues a resetZAxisIntegrator() call on the Gyro.
 * <p>
 * The angle of movement/rotation is assumed to be a standardized rotation around the robot Z axis,
 * which means that a Positive rotation is Counter Clock Wise, looking down on the field.
 * This is consistent with the FTC field coordinate conventions set out in the document:
 * ftc_app\doc\tutorial\FTC_FieldCoordinateSystemDefinition.pdf
 * <p>
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

/*
* runs gyro to turn right 45 degrees
* use optical distance sensor
* use color sensor
* press beacon button
* drive straight towards center and launch balls
* push ball off center and park in landing spot
 */
@com.qualcomm.robotcore.eventloop.opmode.Autonomous(name = "Pushbot: Auto Drive By Gyro to Beacon", group = "Pushbot")

//@Disabled
public class Autonomous extends LinearOpMode {


    /////////////// Gyro //////////////////////////////////////////
    /* Declare OpMode members. */
    HardwareCataclysm robot = new HardwareCataclysm();   // Use Cataclysms hardware
    ModernRoboticsI2cGyro gyro = null;                    // Additional Gyro device

    static final double COUNTS_PER_MOTOR_REV = 1440;    // eg: TETRIX Motor Encoder
    static final double DRIVE_GEAR_REDUCTION = 2.0;     // This is < 1.0 if geared UP
    static final double WHEEL_DIAMETER_INCHES = 4.0;     // For fi guring circumference
    static final double COUNTS_PER_INCH = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * 3.1415);

    // These constants define the desired driving/control characteristics
    // The can/should be tweaked to suite the specific robot drive train.
    static final double DRIVE_SPEED = 0.7;     // Nominal speed for better accuracy.
    static final double TURN_SPEED = 0.5;     // Nominal half speed for better accuracy.

    static final double HEADING_THRESHOLD = 1;      // As tight as we can make it with an integer gyro
    static final double P_TURN_COEFF = 0.1;     // Larger is more responsive, but also less stable
    static final double P_DRIVE_COEFF = 0.15;     // Larger is more responsive, but also less stable
    //////////////////////////////////////////////////////////////

    private ElapsedTime holdTimer = new ElapsedTime();

    ////////////////////////// light sensor //////////////

    // OpticalDistanceSensor lightSensor;   // Alternative MR ODS sensor

    static final double WHITE_THRESHOLD = 0.2;  // spans between 0.1 - 0.5 from dark to light
    // static final double APPROACH_SPEED = 0.5;
    ////////////////////////////////////////////////////

    /////////////////////// color sensor ///////////////////
    ColorSensor colorSensor;    // Hardware Device Object
    /////////////////////////////////////////////////////////


    @Override
    public void runOpMode() {
        // get a reference to our ColorSensor object.
        //colorSensor = hardwareMap.colorSensor.get("sensor_color");

        // Set the LED in the beginning

        colorSensor.enableLed(true);
        robot.init(hardwareMap);
        gyro = (ModernRoboticsI2cGyro) hardwareMap.gyroSensor.get("gyro");

        waitForStart(); //wait for driver to press play
        Gyro(90, 0.2);
        //gyro(distance, angle, holdTime, is beginning, is ending)

    }

    //used to drive to wall and use the beacon - gives orders to use gyroDrive, gyroTurn, and gyroHold class
    public void Gyro(int angle, double holdTime) {
        /*
         * Initialize the standard drive system variables.
         * The init() method of the hardware class does most of the work here
         */
       /* robot.init(hardwareMap);
        gyro = (ModernRoboticsI2cGyro) hardwareMap.gyroSensor.get("gyro");*/

        // Ensure the robot it stationary, then reset the encoders and calibrate the gyro.
        robot.LBMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        //robot.RBMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        // Send telemetry message to alert driver that we are calibrating;
        telemetry.addData(">", "Calibrating Gyro");    //
        telemetry.update();

        gyro.calibrate();

        // make sure the gyro is calibrated before continuing
        while (!isStopRequested() && gyro.isCalibrating()) {
            sleep(50);
            idle();
        }

        telemetry.addData(">", "Robot Ready.");    //
        telemetry.update();

        robot.LBMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        //robot.RBMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Wait for the game to start (Display Gyro value), and reset gyro before we move..
        while (!isStarted()) {
            telemetry.addData(">", "Robot Heading = %d", gyro.getIntegratedZValue());
            telemetry.update();
            idle();
        }
        gyro.resetZAxisIntegrator();

        gyroDrive(DRIVE_SPEED, 10.0, 0.0);    // Drive FWD 48 inches
        gyroTurn(TURN_SPEED, angle);         // Turn  CW to 90 Degrees - turn right towards wall
        gyroHold(TURN_SPEED, angle, holdTime);    // Hold 45 Deg heading for a 0.2 second
        gyroDrive(DRIVE_SPEED, 15.0, 0.0);    // Drive FWD 20 inches - drive forwards towards wall
        gyroHold(TURN_SPEED, 0.0, holdTime);
        gyroTurn(TURN_SPEED, -90); // turn left - parallel to wall
        gyroHold(TURN_SPEED, -90.0, holdTime);
        drivingToLine(DRIVE_SPEED, 20.0, 0.0); //drives staight until it detects line
        gyroHold(TURN_SPEED, 0.0, holdTime);
        gyroDrive(DRIVE_SPEED, 2.0, 0.0);    // Drive FWD 20 inches - drive forwards towards wall
        gyroHold(TURN_SPEED, 0.0, holdTime);
        drivingToLine(DRIVE_SPEED, 20.0, 0.0); //drives staight until it detects line

        telemetry.addData("Path", "Complete");
        telemetry.update();
    }

    /**
     * Method to drive on a fixed compass bearing (angle), based on encoder counts.
     * Move will stop if either of these conditions occur:
     * 1) Move gets to the desired position
     * 2) Driver stops the opmode running.
     *
     * @param speed    Target speed for forward motion.  Should allow for _/- variance for adjusting heading
     * @param distance Distance (in inches) to move from current position.  Negative distance means move backwards.
     * @param angle    Absolute Angle (in Degrees) relative to last gyro reset.
     *                 0 = fwd. +ve is CCW from fwd. -ve is CW from forward.
     *                 If a relative angle is required, add/subtract from current heading.
     */
    //used to drive straight to wall and drive to center to release balls
    public void gyroDrive(double speed, double distance, double angle) {
        //beginning is for code to drive straight to wall which is 1st step

        int newLeftTarget;
        //int newRightTarget;
        int moveCounts;
        double max;
        double error;
        double steer;
        double leftSpeed;
        double rightSpeed;

        //while (wallDetection(15.0) == false) {
        // Ensure that the opmode is still active
        if (opModeIsActive()) {


            // Determine new target position, and pass to motor controller
            moveCounts = (int) (distance * COUNTS_PER_INCH);
            newLeftTarget = robot.LBMotor.getCurrentPosition() + moveCounts;
            // newRightTarget = robot.RBMotor.getCurrentPosition() + moveCounts;

            // Set Target and Turn On RUN_TO_POSITION
            robot.LBMotor.setTargetPosition(newLeftTarget);
            // robot.RBMotor.setTargetPosition(newRightTarget);

            robot.LBMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            // robot.RBMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // start motion.
            speed = Range.clip(Math.abs(speed), 0.0, 1.0);
            robot.LBMotor.setPower(speed);
            robot.RBMotor.setPower(speed);

            // keep looping while we are still active, and BOTH motors are running.
            while (opModeIsActive() &&
                    (robot.LBMotor.isBusy() && robot.RBMotor.isBusy())) {

                // adjust relative speed based on heading error.
                error = getError(angle);
                steer = getSteer(error, P_DRIVE_COEFF);

                // if driving in reverse, the motor correction also needs to be reversed
                if (distance < 0)
                    steer *= -1.0;

                leftSpeed = speed - steer;
                rightSpeed = speed + steer;

                // Normalize speeds if any one exceeds +/- 1.0;
                max = Math.max(Math.abs(leftSpeed), Math.abs(rightSpeed));
                if (max > 1.0) {
                    leftSpeed /= max;
                    rightSpeed /= max;
                }

                robot.LBMotor.setPower(leftSpeed);
                robot.RBMotor.setPower(rightSpeed);
            }
        }

    }

    /**
     * Method to spin on central axis to point in a new direction.
     * Move will stop if either of these conditions occur:
     * 1) Move gets to the heading (angle)
     * 2) Driver stops the opmode running.
     *
     * @param speed Desired speed of turn.
     * @param angle Absolute Angle (in Degrees) relative to last gyro reset.
     *              0 = fwd. +ve is CCW from fwd. -ve is CW from forward.
     *              If a relative angle is required, add/subtract from current heading.
     */
    //turn robot to certain angle
    public void gyroTurn(double speed, int angle) {

        int currentheading = robot.gyro.getIntegratedZValue();
        int tollerance = 1;
        double startTime = holdTimer.time();
        int degreesToTravel = robot.gyro.getIntegratedZValue() - angle;

        while (Math.abs(robot.gyro.getIntegratedZValue() - angle) > tollerance && holdTimer.time() < startTime + 4) {

            if (Math.abs(angle - currentheading) < 25)
                speed = .07f;
            telemetry.addData("degreesToTravel", degreesToTravel);
            telemetry.addData("current degree", currentheading);
            telemetry.addData("heading", robot.gyro.getHeading());
            telemetry.addData("speed", speed);
            telemetry.update();
            if (currentheading < angle) {
                robot.LBMotor.setPower(0);
                robot.RBMotor.setPower(speed);
                //turn left
            } else {
                robot.LBMotor.setPower(speed);
                robot.RBMotor.setPower(0);
                //turn right
            }
        }

        robot.LBMotor.setPower(0);
        robot.RBMotor.setPower(0);
    }

/////////////////////////

    /**
     * Method to obtain & hold a heading for a finite amount of time
     * Move will stop once the requested time has elapsed
     *
     * @param speed    Desired speed of turn.
     * @param angle    Absolute Angle (in Degrees) relative to last gyro reset.
     *                 0 = fwd. +ve is CCW from fwd. -ve is CW from forward.
     *                 If a relative angle is required, add/subtract from current heading.
     * @param holdTime Length of time (in seconds) to hold the specified heading.
     */
    public void gyroHold(double speed, double angle, double holdTime) {
        // keep looping while we have time remaining.
        holdTimer.reset();
        while (opModeIsActive() && (holdTimer.time() < holdTime)) {
            // Update telemetry & Allow time for other processes to run.
            onHeading(speed, angle, P_TURN_COEFF);
            telemetry.update();
        }

        // Stop all motion;
        robot.LBMotor.setPower(0);
        robot.RBMotor.setPower(0);
    }

    /**
     * Perform one cycle of closed loop heading control.
     *
     * @param speed  Desired speed of turn.
     * @param angle  Absolute Angle (in Degrees) relative to last gyro reset.
     *               0 = fwd. +ve is CCW from fwd. -ve is CW from forward.
     *               If a relative angle is required, add/subtract from current heading.
     * @param PCoeff Proportional Gain coefficient
     * @return
     */
    boolean onHeading(double speed, double angle, double PCoeff) {
        double error;
        double steer;
        boolean onTarget = false;
        double leftSpeed;
        double rightSpeed;

        // determine turn power based on +/- error
        error = getError(angle);

        if (Math.abs(error) <= HEADING_THRESHOLD) {
            steer = 0.0;
            leftSpeed = 0.0;
            rightSpeed = 0.0;
            onTarget = true;
        } else {
            steer = getSteer(error, PCoeff);
            rightSpeed = speed * steer;
            leftSpeed = -rightSpeed;
        }

        // Send desired speeds to motors.
        robot.LBMotor.setPower(leftSpeed);
        robot.RBMotor.setPower(rightSpeed);

        // Display it for the driver.
        telemetry.addData("Target", "%5.2f", angle);
        telemetry.addData("Err/St", "%5.2f/%5.2f", error, steer);
        telemetry.addData("Speed.", "%5.2f:%5.2f", leftSpeed, rightSpeed);

        return onTarget;
    }

    /**
     * getError determines the error between the target angle and the robot's current heading
     *
     * @param targetAngle Desired angle (relative to global reference established at last Gyro Reset).
     * @return error angle: Degrees in the range +/- 180. Centered on the robot's frame of reference
     * +ve error means the robot should turn LEFT (CCW) to reduce error.
     */
    public double getError(double targetAngle) {

        double robotError;

        // calculate error in -179 to +180 range  (
        robotError = targetAngle - gyro.getIntegratedZValue();
        while (robotError > 180) robotError -= 360;
        while (robotError <= -180) robotError += 360;
        return robotError;
    }

    /**
     * returns desired steering force.  +/- 1 range.  +ve = steer left
     *
     * @param error  Error angle in robot relative degrees
     * @param PCoeff Proportional Gain Coefficient
     * @return
     */
    public double getSteer(double error, double PCoeff) {
        return Range.clip(error * PCoeff, -1, 1);
    }

    //sense line and remain driving on line

    public void drivingToLine(double speed, double distance, double angle) {

        int newLeftTarget;
        int moveCounts;
        double max;
        double error;
        double steer;
        double leftSpeed;
        double rightSpeed;

        // Ensure that the opmode is still active
        if (opModeIsActive()) {
            while (lineDetection() == false) {
                // Determine new target position, and pass to motor controller
                moveCounts = (int) (distance * COUNTS_PER_INCH);
                newLeftTarget = robot.LBMotor.getCurrentPosition() + moveCounts;

                // Set Target and Turn On RUN_TO_POSITION
                robot.LBMotor.setTargetPosition(newLeftTarget);

                robot.LBMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                // start motion.
                speed = Range.clip(Math.abs(speed), 0.0, 1.0);
                robot.LBMotor.setPower(speed);
                robot.RBMotor.setPower(speed);

                // keep looping while we are still active, and BOTH motors are running.
                while (opModeIsActive() &&
                        (robot.LBMotor.isBusy() && robot.RBMotor.isBusy())) {

                    // adjust relative speed based on heading error.
                    error = getError(angle);
                    steer = getSteer(error, P_DRIVE_COEFF);

                    // if driving in reverse, the motor correction also needs to be reversed
                    if (distance < 0)
                        steer *= -1.0;

                    leftSpeed = speed - steer;
                    rightSpeed = speed + steer;

                    // Normalize speeds if any one exceeds +/- 1.0;
                    max = Math.max(Math.abs(leftSpeed), Math.abs(rightSpeed));
                    if (max > 1.0) {
                        leftSpeed /= max;
                        rightSpeed /= max;
                    }

                    robot.LBMotor.setPower(leftSpeed);
                    robot.RBMotor.setPower(rightSpeed);
                }
            }
            robot.LBMotor.setPower(0);
            robot.RBMotor.setPower(0);

            beacon();
        }
    }

    public boolean lineDetection() {
        if (robot.lightSensor.getRawLightDetected() < WHITE_THRESHOLD || robot.lightSensor.getLightDetected() > WHITE_THRESHOLD) {
            return false;
        } else {
            return true;
        }
    }

    public void line() {

       /* //double reflectance = opticalDistanceSensor.getLightDetected();
        robot.LBMotor.setPower(APPROACH_SPEED);
        robot.RBMotor.setPower(APPROACH_SPEED);

        // run until the white line is seen OR the driver presses STOP;
        //while (opModeIsActive() && (lightSensor.getLightDetected() < WHITE_THRESHOLD)) {


        // Display the light level while we are looking for the line
        telemetry.addData("Light Level", robot.lightSensor.getLightDetected());
        telemetry.update();

        if (robot.lightSensor.getLightDetected() < WHITE_THRESHOLD) {
            robot.LBMotor.setPower(0.0);
            robot.RBMotor.setPower(0.2);
        } else
            robot.LBMotor.setPower(0.2);
        robot.RBMotor.setPower(0.0);


        robot.LBMotor.setPower(0);
        robot.RBMotor.setPower(0);

        // Turn off RUN_TO_POSITION
        robot.LBMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        //  robot.RBMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        //beacon();*/
    } //line detection for old plan


    //sense color of beacon and press button
    public void beacon() {

        if (colorSensor.red() > 5) {
            robot.LBMotor.setTargetPosition(10);
            //robot.RBMotor.setTargetPosition(0);
            robot.LBMotor.setPower(0.5);
            robot.RBMotor.setPower(0);

            sleep(100);

            robot.LBMotor.setTargetPosition(0);
            // robot.RBMotor.setTargetPosition(0);
            robot.LBMotor.setPower(-0.5);
            robot.RBMotor.setPower(0);

            sleep(500);


        } else {
            robot.LBMotor.setTargetPosition(10);
            robot.LBMotor.setPower(0.5);
            robot.RBMotor.setPower(0.5);

            sleep(100);
            robot.LBMotor.setTargetPosition(0);
            robot.LBMotor.setPower(0);
            robot.RBMotor.setPower(0);
            //----------------
            robot.LBMotor.setTargetPosition(-10);
            //robot.RBMotor.setTargetPosition(10);
            robot.LBMotor.setPower(-0.5);
            robot.RBMotor.setPower(0);

            sleep(100);

            robot.LBMotor.setTargetPosition(0);
            // robot.RBMotor.setTargetPosition(0);
            robot.LBMotor.setPower(-0.5);
            robot.RBMotor.setPower(0);
        }
    }


}
