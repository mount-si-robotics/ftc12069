/*
Copyright (c) 2016 Robert Atkinson

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

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.Range;

@TeleOp(name = "Catacylsm Linear OpMode - WIP V3.5", group = "Cataclysm Hardware OpModes")
public class TeleopLinearOpMode extends LinearOpMode {

    /* Declare OpMode members. */
    HardwareCataclysm robot = new HardwareCataclysm();   // Use Cataclysms hardware map

    @Override
    public void runOpMode() {
        double left;
        double right;
        double max;
        double conveyorServoPos;
        double changeRate;

        /* Initialize the hardware variables.
         * The init() method of the hardware class does all the work here
         */
        robot.init(hardwareMap);

        // Send telemetry message to signify robot waiting;
        conveyorServoPos = 0;
        telemetry.addData("Say", "Robot Initialized");
        telemetry.update();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

            // Run wheels in tank drive mode
            left = -gamepad1.left_stick_y;
            right = -gamepad1.right_stick_y;

            // Normalize the values so neither exceed +/- 100
            max = Math.max(Math.abs(left), Math.abs(right));
            if (max > 1.0) {
                left /= max;
                right /= max;
            }

            robot.LBMotor.setPower(-left);
            robot.RBMotor.setPower(-right);


            // Use gamepad2 y and a buttons to control flick arm
            if (gamepad2.y) {
                robot.flickMotor.setPower(robot.FLICK_POWER);

                telemetry.addData("Arm Status", "Flick Motor: Forward");
                telemetry.update();
            }
            else if (gamepad2.a) {
                robot.flickMotor.setPower(robot.FLICK_POWER_REVERSE);

                telemetry.addData("Arm Status", "Flick Motor: Reverse");
                telemetry.update();
            }
            else {
                robot.flickMotor.setPower(0.0);

                telemetry.addData("Arm Status", "Flick Motor: Stopped");
                telemetry.update();
            }


            // Use gamepad2 d-pad to control conveyor belt direction
            // Add code to print postition to screen
            if (gamepad2.dpad_up) {
                robot.conveyorMotor.setPower(1);

                telemetry.addData("Collection Status", "Conveyor: Forward");
                telemetry.update();
            }
            else if (gamepad2.dpad_down) {
                robot.conveyorMotor.setPower(-1);

                telemetry.addData("Collection Status", "Conveyor: Reverse");
                telemetry.update();
            }
            else {
                robot.conveyorMotor.setPower(0);
                telemetry.addData("Collection Status", "Conveyor Stopped");
                telemetry.update();
            }


            // Use the bumpers to control the ball collection mechanism
            if (gamepad2.left_bumper) {
                robot.collectionMotor.setPower(1);
                telemetry.addData("Collection Arm Status", "Moving Down");
                telemetry.update();
            }
            else if (gamepad2.right_bumper) {
                robot.collectionMotor.setPower(-1);

                telemetry.addData("Collection Arm Status", "Moving Up");
                telemetry.update();
            }
            else {
                robot.collectionMotor.setPower(0);

                telemetry.addData("Collection Arm Status", "Stopped");
                telemetry.update();
            }

            telemetry.update();
            robot.waitForTick(0);
        }
    }
}