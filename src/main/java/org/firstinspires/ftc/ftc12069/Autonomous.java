    package org.firstinspires.ftc.ftc12069;


    import android.graphics.Color;

    import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;
 //   import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
    import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
    import com.qualcomm.robotcore.hardware.ColorSensor;
    import com.qualcomm.robotcore.hardware.DcMotor;
    import com.qualcomm.robotcore.hardware.GyroSensor;
    import com.qualcomm.robotcore.hardware.OpticalDistanceSensor;
 //   import com.qualcomm.robotcore.hardware.Servo;
    import com.qualcomm.robotcore.util.ElapsedTime;

    //@com.qualcomm.robotcore.eventloop.opmode.Autonomous(name = "Pushbot: AutoRed Thomas's code", group = "Pushbot")
    @com.qualcomm.robotcore.eventloop.opmode.Autonomous(name = "Pushbot: Auto Drive By Gyro to Beacon", group = "Pushbot")

    public class Autonomous extends LinearOpMode {
        HardwareCataclysm robot = new HardwareCataclysm();   // Use Cataclysms hardware
    /* Declare OpMode members. */

        private ElapsedTime runtime = new ElapsedTime();
        public final float CIRCUMFENCE = (float) (4.00 * Math.PI);
        public final int ENCODERTICKS = 1120;
        public final float GEARRATIO = .5f;


        private DcMotor LBMotor;
        private DcMotor RBMotor;

        // private Servo buttonPusher;
        private ColorSensor modernRobotics;
        private OpticalDistanceSensor ODS;
        private GyroSensor sensorType;
        private ModernRoboticsI2cGyro Gyro;
        GyroSensor sensorGyro = hardwareMap.gyroSensor.get("gyro");

        @Override
        public void runOpMode() {
            telemetry.addData("Status", "Initialized");
            telemetry.update();


            RBMotor = hardwareMap.dcMotor.get("RBMotor");
            LBMotor = hardwareMap.dcMotor.get("LBMotor");
            //LBMotor.setDirection(DcMotor.Direction.REVERSE);
            // buttonPusher = hardwareMap.servo.get("ButtonPusherCRServo");
            modernRobotics = hardwareMap.colorSensor.get("MRCSensor");
            ODS = hardwareMap.opticalDistanceSensor.get("ODSensor");
            sensorType = hardwareMap.gyroSensor.get("GSensor");

            Gyro = (ModernRoboticsI2cGyro) sensorType;

            // buttonPusher.setPosition(0);

            LBMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            RBMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            checkEncoder();
            LBMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            RBMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            Gyro.calibrate();
            while (Gyro.isCalibrating()) {
            }
            waitForStart();
            runtime.reset();

            moveRobot2(12, .2f);
            turnUsingRightMotors(-45, .2f);
            moveRobot2(60, .2f, -35);
            turnUsingLeftMotors(0, .2f, 0);
            setMotorSpeed(.2f);
            sleep(750);
            CheckBeaconForRed(-.2f, 5);
        }

        public void moveRobot(double distance, float speed) {
            double ticksToInches = (ENCODERTICKS * GEARRATIO) / CIRCUMFENCE;
            int PositionTarget1 = LBMotor.getCurrentPosition() + (int) (distance * ticksToInches);
            int PositionTarget2 = RBMotor.getCurrentPosition() + (int) (distance * ticksToInches);

            LBMotor.setTargetPosition(PositionTarget1);
            RBMotor.setTargetPosition(PositionTarget2);

            LBMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            RBMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            setMotorSpeed(speed);
            while (LBMotor.isBusy()) {
                telemetry.addData("angle", Gyro.getHeading());
                telemetry.update();
            }

            setMotorSpeed(0);
            LBMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            RBMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }

        public void moveRobot2(double distance, float speed) {
            sleep(100);
            int targetAngle = -Gyro.getIntegratedZValue();
            double startTime = runtime.time();

            int headingerror;
            int currentheading;
            float driveConstant = .003f;
            float midPower = speed;
            float drivesteering;
            float leftPower, rightPower;
            double ticksToInches = (ENCODERTICKS * GEARRATIO) / CIRCUMFENCE;
            int PositionTarget1 = LBMotor.getCurrentPosition() + (int) (distance * ticksToInches);
            int PositionTarget2 = RBMotor.getCurrentPosition() + (int) (distance * ticksToInches);

            LBMotor.setTargetPosition(PositionTarget1);
            RBMotor.setTargetPosition(PositionTarget2);

            LBMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            RBMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            setMotorSpeed(speed);

            while (LBMotor.isBusy() && runtime.time() < startTime + 6) {

                currentheading = -Gyro.getIntegratedZValue();

                headingerror = targetAngle - currentheading;
                drivesteering = headingerror * driveConstant;
                leftPower = midPower + drivesteering;
                if (leftPower > 1)
                    leftPower = 1;
                if (leftPower < 0)
                    leftPower = 0;
                rightPower = midPower - drivesteering;
                if (rightPower > 1)
                    rightPower = 1;
                if (rightPower < 0)
                    rightPower = 0;
                LBMotor.setPower(leftPower);
                RBMotor.setPower(rightPower);
                telemetry.addData("leftPower",
                        leftPower);
                telemetry.addData("rightPower",
                        rightPower);
                telemetry.addData("drivestering",
                        drivesteering);
                telemetry.addData("angle",
                        currentheading);
                telemetry.update();
            }
            setMotorSpeed(0);
            LBMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            RBMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }

        public void moveRobot2(double distance, float speed, int targetAngle) {
            sleep(100);
            double startTime = runtime.time();

            int headingerror;
            int currentheading;
            float driveConstant = .003f;
            float midPower = speed;
            float drivesteering;
            float leftPower, rightPower;
            double ticksToInches = (ENCODERTICKS * GEARRATIO) / CIRCUMFENCE;
            int PositionTarget1 = LBMotor.getCurrentPosition() + (int) (distance * ticksToInches);
            int PositionTarget2 = RBMotor.getCurrentPosition() + (int) (distance * ticksToInches);

            LBMotor.setTargetPosition(PositionTarget1);
            RBMotor.setTargetPosition(PositionTarget2);

            LBMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            setMotorSpeed(speed);

            while (LBMotor.isBusy() && runtime.time() < startTime + 6) {

                currentheading = -Gyro.getIntegratedZValue();

                headingerror = targetAngle - currentheading;
                drivesteering = headingerror * driveConstant;
                leftPower = midPower + drivesteering;
                if (leftPower > 1)
                    leftPower = 1;
                if (leftPower < 0)
                    leftPower = 0;
                rightPower = midPower - drivesteering;
                if (rightPower > 1)
                    rightPower = 1;
                if (rightPower < 0)
                    rightPower = 0;
                LBMotor.setPower(leftPower);
                RBMotor.setPower(rightPower);
                telemetry.addData("leftPower",
                        leftPower);
                telemetry.addData("rightPower",
                        rightPower);
                telemetry.addData("drivestering",
                        drivesteering);
                telemetry.addData("angle",
                        currentheading);
                telemetry.update();
            }
            setMotorSpeed(0);
            LBMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }

        public boolean MotorsBusy() {
            return LBMotor.isBusy() && RBMotor.isBusy();
        }

        public void setMotorSpeed(float speed) {
            LBMotor.setPower(speed);
            RBMotor.setPower(speed);
        }

        public void checkEncoder() {
            while (!LBMotor.getMode().equals(DcMotor.RunMode.STOP_AND_RESET_ENCODER))
                LBMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            RBMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        }


        public void CheckBeaconForBlue(float speed, float waitTime) {
            float hsvValues[] = {0F, 0F, 0F};
            double startTime = runtime.time();
            setMotorSpeed(speed);
            Color.RGBToHSV((modernRobotics.red() * 255) / 800, (modernRobotics.green() * 255) / 800, (modernRobotics.blue() * 255) / 800, hsvValues);
            while (hsvValues[0] < 150 && runtime.time() < startTime + waitTime) {
                Color.RGBToHSV((modernRobotics.red() * 255) / 800, (modernRobotics.green() * 255) / 800, (modernRobotics.blue() * 255) / 800, hsvValues);
                telemetry.addData("Hue", hsvValues[0]);
                telemetry.addData("startTime", startTime);
                telemetry.addData("timer", runtime.time());
                telemetry.update();
            }
            setMotorSpeed(0);
        }

        public void CheckRedColor() {
            float hsvValues[] = {0F, 0F, 0F};
            double startTime = runtime.time();
            boolean isRed = false;
            Color.RGBToHSV((modernRobotics.red() * 255) / 800, (modernRobotics.green() * 255) / 800, (modernRobotics.blue() * 255) / 800, hsvValues);
            Color.RGBToHSV((modernRobotics.red() * 255) / 800, (modernRobotics.green() * 255) / 800, (modernRobotics.blue() * 255) / 800, hsvValues);
            if (hsvValues[0] < 20)
                isRed = true;

            telemetry.addData("isRed", isRed);
            telemetry.update();
            sleep(2000);
        }

        public void CheckBeaconForRed(float speed, float waitTime) {
            float hsvValues[] = {0F, 0F, 0F};
            double startTime = runtime.time();
            setMotorSpeed(speed);
            Color.RGBToHSV((modernRobotics.red() * 255) / 800, (modernRobotics.green() * 255) / 800, (modernRobotics.blue() * 255) / 800, hsvValues);
            while (hsvValues[0] > 20 && runtime.time() < startTime + waitTime) {
                Color.RGBToHSV((modernRobotics.red() * 255) / 800, (modernRobotics.green() * 255) / 800, (modernRobotics.blue() * 255) / 800, hsvValues);
                telemetry.addData("Hue", hsvValues[0]);
                telemetry.addData("startTime", startTime);
                telemetry.addData("timer", runtime.time());
                telemetry.update();
            }
            setMotorSpeed(0);
        }



        public void turn(int degrees, float speed) {
            int currentheading = -Gyro.getIntegratedZValue();
            int tollerance = 1;
            double startTime = runtime.time();

            while (Math.abs(currentheading - degrees) > tollerance && runtime.time() < startTime + 6) {
                currentheading = -Gyro.getIntegratedZValue();
                if (Math.abs(degrees - currentheading) < 25)
                    speed = .07f;
                telemetry.addData("check", Math.abs(currentheading - degrees));
                telemetry.addData("current heading", currentheading);
                telemetry.addData("degrees", degrees);
                telemetry.addData("speed", speed);
                telemetry.update();
                if (currentheading < degrees)
                    turnRight(speed);
                else
                    turnLeft(speed);
            }
            setMotorSpeed(0);
        }

        public void turnUsingLeftMotors(int degrees, float speed, float tollerance) {
            int currentheading = -Gyro.getIntegratedZValue();
            double startTime = runtime.time();

            while (Math.abs(currentheading - degrees) > tollerance && runtime.time() < startTime + 4) {
                currentheading = -Gyro.getIntegratedZValue();
                if (Math.abs(degrees - currentheading) < 25)
                    speed = .07f;
                telemetry.addData("check", Math.abs(currentheading - degrees));
                telemetry.addData("current heading", currentheading);
                telemetry.addData("degrees", degrees);
                telemetry.addData("speed", speed);
                telemetry.update();
                if (currentheading < degrees) {
                    LBMotor.setPower(speed);
                } else {
                    LBMotor.setPower(-speed);
                }
            }
            setMotorSpeed(0);
        }

        public void turnUsingRightMotors(int degrees, float speed) {
            int currentheading = -Gyro.getIntegratedZValue();
            int tollerance = 1;
            double startTime = runtime.time();

            while (Math.abs(currentheading - degrees) > tollerance && runtime.time() < startTime + 4) {
                currentheading = -Gyro.getIntegratedZValue();
                if (Math.abs(degrees - currentheading) < 25)
                    speed = .07f;
                telemetry.addData("check", Math.abs(currentheading - degrees));
                telemetry.addData("current heading", currentheading);
                telemetry.addData("degrees", degrees);
                telemetry.addData("speed", speed);
                telemetry.update();
                if (currentheading < degrees) {
                    RBMotor.setPower(-speed);
                } else {
                    RBMotor.setPower(speed);
                }
            }
            setMotorSpeed(0);
        }

        public void searchForWhiteLine(float speed) {
            double startTime = runtime.time();
            setMotorSpeed(speed);
            while (ODS.getRawLightDetected() < .3
                    && runtime.time() < startTime + 4) {
                telemetry.addData("lightBack", ODS.getRawLightDetected());
            }
            setMotorSpeed(0);
        }

        public void turnLeft(float speed) {
            RBMotor.setPower(speed);
            LBMotor.setPower(-speed);
        }

        public void turnRight(float speed) {
            RBMotor.setPower(-speed);
            LBMotor.setPower(speed);
        }
    }
