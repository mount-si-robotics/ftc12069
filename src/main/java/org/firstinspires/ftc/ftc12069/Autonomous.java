package org.firstinspires.ftc.ftc12069;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.OpticalDistanceSensor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

//import org.firstinspires.ftc.robotcontroller.external.samples.HardwarePushbot;

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

    private static final double COUNTS_PER_MOTOR_REV = 1440;    // eg: TETRIX Motor Encoder
    private static final double DRIVE_GEAR_REDUCTION = 2.0;     // This is < 1.0 if geared UP
    private static final double WHEEL_DIAMETER_INCHES = 4.0;     // For fi guring circumference
    private static final double COUNTS_PER_INCH = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * 3.1415);

    // These constants define the desired driving/control characteristics
    // The can/should be tweaked to suite the specific robot drive train.
    private static final double DRIVE_SPEED = 0.7;     // Nominal speed for better accuracy.
    private static final double TURN_SPEED = 0.5;     // Nominal half speed for better accuracy.

    private static final double HEADING_THRESHOLD = 1;      // As tight as we can make it with an integer gyro
    private static final double P_TURN_COEFF = 0.1;     // Larger is more responsive, but also less stable
    private static final double P_DRIVE_COEFF = 0.15;     // Larger is more responsive, but also less stable
    //////////////////////////////////////////////////////////////

    private ElapsedTime holdTimer = new ElapsedTime();

    ////////////////////////// light sensor //////////////

    OpticalDistanceSensor opticalDistanceSensor;   // Alternative MR ODS sensor

    private static final double WHITE_THRESHOLD = 0.2;  // spans between 0.1 - 0.5 from dark to light
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
        robot.init(hardwareMap);
       // gyro = (ModernRoboticsI2cGyro) hardwareMap.gyroSensor.get("gyro");

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

    private void drivingToLine(double speed, double distance, double angle) {

        int newLeftTarget;
        int moveCounts;
        double max;
        double error;
        double steer;
        double leftSpeed;
        double rightSpeed;

        // Ensure that the opmode is still active
        if (opModeIsActive()) {
            while (lineDetection()) {
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

    private boolean lineDetection() {
        return !(opticalDistanceSensor.getRawLightDetected() < WHITE_THRESHOLD || opticalDistanceSensor.getLightDetected() > WHITE_THRESHOLD);
    }

    public void line() {

        //double reflectance = lightSensor.getLightDetected();
        /*robot.LBMotor.setPower(APPROACH_SPEED);
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
    private void beacon() {

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

