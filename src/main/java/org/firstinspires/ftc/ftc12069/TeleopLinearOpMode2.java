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

@TeleOp(name = "Catacylsm Linear OpMode (Manjesh/Hari) - V2", group = "Cataclysm Hardware OpModes")
public class TeleopLinearOpMode2 extends LinearOpMode {

    /* Declare OpMode members. */
    HardwareCataclysm robot = new HardwareCataclysm();   // Use Cataclysms hardware map

    @Override
    public void runOpMode() {
        double left;
        double right;
        double max;
        double breaks;

        /* Initialize the hardware variables.
         * The init() method of the hardware class does all the work here
         */
        robot.init(hardwareMap);

        // Send telemetry message to signify robot waiting;
        telemetry.addData("Say", "Robot Initialized");
        telemetry.update();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

            // Run wheels in tank drive mode
            left = (-gamepad1.left_stick_y);
            right = (-gamepad1.right_stick_y);
            breaks = (Math.abs(gamepad1.right_trigger));

            // Normalize the values so neither exceed +/- 100
            max = Math.max(Math.abs(left), Math.abs(right));
            if (max > 1.0) {
                left /= max;
                right /= max;
            }

            robot.LBMotor.setPower(left - breaks);
            robot.RBMotor.setPower(right - breaks);


            // Use gamepad2 y and b buttons to control flick arm
            if (gamepad2.y) {
                robot.flickMotor.setPower(1);
            }
            else if (gamepad2.b) {
                robot.flickMotor.setPower(-1);
            }
            else {
                robot.flickMotor.setPower(0.0);
            }


            // Use gamepad2 d-pad to control conveyor belt direction
            if (gamepad2.dpad_up) {
                robot.conveyorMotor.setPower(-0.13);
            }
            else if (gamepad2.dpad_down) {
                robot.conveyorMotor.setPower(0.13);
            }
            else {
                robot.conveyorMotor.setPower(0);
            }


            // Use the bumpers on controller 1 ato control the ball collection mechanism
            if (gamepad2.left_bumper) {
                robot.collectionMotor.setPower(0.5);
            }
            else if (gamepad2.right_bumper) {
                robot.collectionMotor.setPower(-0.5);
            }
            else if (gamepad1.left_bumper) {
                robot.collectionMotor.setPower(-0.5);
            }
            else if (gamepad1.right_bumper){
                robot.collectionMotor.setPower(0.5);
            }
            else {
                robot.collectionMotor.setPower(0);
            }


            telemetry.update();
            robot.waitForTick(0);
        }
    }
}