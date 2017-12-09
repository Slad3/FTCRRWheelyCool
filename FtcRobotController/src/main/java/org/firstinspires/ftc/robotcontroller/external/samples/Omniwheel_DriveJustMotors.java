/* Copyright (c) 2017 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.robotcontroller.external.samples;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.Range;

/**
 * This OpMode uses the common HardwareK9bot class to define the devices on the robot.
 * All device access is managed through the HardwareK9bot class. (See this class for device names)
 * The code is structured as a LinearOpMode
 *
 * This particular OpMode executes a basic Tank Drive Teleop for the K9 bot
 * It raises and lowers the arm using the Gampad Y and A buttons respectively.
 * It also opens and closes the claw slowly using the X and B buttons.
 *
 * Note: the configuration of the servos is such that
 * as the arm servo approaches 0, the arm position moves up (away from the floor).
 * Also, as the claw servo approaches 0, the claw opens up (drops the game element).
 *
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

@TeleOp(name="Omniwheel_Drive_Just_Motors", group="K9bot")
//@Disabled
public class Omniwheel_DriveJustMotors extends LinearOpMode
{

    /* Declare OpMode members. */
    HardwareK9bot   robot            =   new HardwareK9bot();
    //ColorSensor color_sensor;
    double          leftPosition     =   robot.LEFT_HOME;                  // Servo safe position
    double          rightPosition    =   robot.RIGHT_HOME;               // Servo safe position
    double          liftSpeed        =   1;
    double          lifterRSpeed     =   0;
    double          lifterLSpeed     =   0;
    double          driveSpeed       =   1;
    //double          red              ;
    //double          blue            ; // =   0;
    //double          green           ; // =   0;
    final double    LEFT_HOME        =   robot.LEFT_HOME;
    final double    RIGHT_HOME       =   robot.RIGHT_HOME;               //Gucci
    final double    LEFT_MAX_RANGE   =   robot.LEFT_MAX_RANGE;
    final double    LEFT_MIN_RANGE   =   robot.LEFT_MIN_RANGE;               //Gucci
    final double    RIGHT_MAX_RANGE  =   robot.RIGHT_MAX_RANGE;
    final double    RIGHT_MIN_RANGE  =   robot.RIGHT_MIN_RANGE;               //Gucci
    final double    LEFT_SPEED       =   0.03 ;                            // sets rate to move servo
    final double    RIGHT_SPEED      =   0.03 ;                            // sets rate to move servo

    @Override
    public void runOpMode()
    {
        double frontLeft = 0;
        double frontRight = 0;
        double backLeft = 0;
        double backRight = 0;
        double Lift;
        double LifterR;
        double LifterL;

        /* Initialize the hardware variables.
         * The init() method of the hardware class does all the work here
         */
        robot.init(hardwareMap);

        // Send telemetry message to signify robot waiting;
        telemetry.addData("Say", "Hello Driver");    //
        telemetry.update();
        //color_sensor = hardwareMap.colorSensor.get("color");
        // Wait for the game to start (driver presses PLAY)
        waitForStart();


        // run until the end of the match (driver presses STOP)
        while (opModeIsActive())
        {

            if (gamepad2.a)
                liftSpeed = 1;
            if (gamepad2.b)
                liftSpeed = 0.5;

            //red = color_sensor.red();
            //green = color_sensor.green();
            //blue = color_sensor.blue();

            driveSpeed = 1;

            if (gamepad1.right_bumper)
            {
                driveSpeed = 0.5;
            }

            if (gamepad1.left_bumper)
            {
                driveSpeed = 0.25;
            }

            // Run wheels in tank mode (note: The joystick goes negative when pushed forwards, so negate it)
            frontLeft  = ( gamepad1.left_stick_x - gamepad1.left_stick_y - gamepad1.right_stick_x)/2 * driveSpeed; //Front right
            frontRight = ( gamepad1.left_stick_x + gamepad1.left_stick_y - gamepad1.right_stick_x)/2 * driveSpeed; //Front left
            backLeft   = (-gamepad1.left_stick_x - gamepad1.left_stick_y - gamepad1.right_stick_x)/2 * driveSpeed;  //Back right
            backRight  = (-gamepad1.left_stick_x + gamepad1.left_stick_y - gamepad1.right_stick_x)/2 * driveSpeed; //Back left
            Lift = gamepad2.left_stick_y * liftSpeed;

            lifterRSpeed = 0;
            lifterLSpeed = 0;

            if (gamepad2.dpad_down)
            {
                lifterRSpeed = 0.1;
                lifterLSpeed = 0.1;
            }
            else if (gamepad2.dpad_up)
            {
                lifterRSpeed = -0.1;
                lifterLSpeed = -0.1;
            }
            if (gamepad2.dpad_left)
            {
                lifterLSpeed = 0.05;
            }
            if (gamepad2.dpad_right)
            {
                lifterRSpeed = 0.05;
            }

            robot.FL_drive.setPower(frontLeft);
            robot.FR_drive.setPower(frontRight);
            robot.BL_drive.setPower(backLeft);
            robot.BR_drive.setPower(backRight);
            robot.Lift.setPower(Lift);
           // robot.LifterL.setPower(-lifterLSpeed);
           // robot.LifterR.setPower(lifterRSpeed);

            // Left servo going in means more
            if (gamepad2.left_bumper)
                leftPosition += LEFT_SPEED;
            else if (gamepad2.y)
                leftPosition = LEFT_MIN_RANGE;

            // Right servo going in means less
            if (gamepad2.right_bumper)
                rightPosition -= RIGHT_SPEED;
            else if (gamepad2.y)
                rightPosition = RIGHT_MAX_RANGE;

            if (gamepad2.x)
            {
                rightPosition = 0.96;
                leftPosition = 0.44;
            }

            //color_sensor.red();   // Red channel value
            //color_sensor.green(); // Green channel value
            //color_sensor.blue();  // Blue channel value

            //color_sensor.alpha(); // Total luminosity
            //color_sensor.argb();  // Combined color value

            // Move both servos to new position.
            leftPosition  = Range.clip(leftPosition, robot.LEFT_MIN_RANGE, robot.LEFT_MAX_RANGE);
            robot.Left.setPosition(leftPosition);
            rightPosition = Range.clip(rightPosition, robot.RIGHT_MIN_RANGE, robot.RIGHT_MAX_RANGE);
            robot.Right.setPosition(rightPosition);

            // Send telemetry message to signify robot running;
            telemetry.addData("Left",   "%.2f", leftPosition);
            telemetry.addData("Right",  "%.2f", rightPosition);
            telemetry.addData("frontLeft",  "%.2f", frontLeft);
            telemetry.addData("frontRight", "%.2f", frontRight);
            telemetry.addData("backLeft",  "%.2f", backLeft);
            telemetry.addData("backRight", "%.2f", backRight);
            telemetry.addData("Lift", "%.2f", Lift);
            //telemetry.addData("LifterR", "%.2f", lifterRSpeed);
            //telemetry.addData("LifterL", "%.2f", lifterLSpeed);
            //telemetry.addData("Red", "%.2f", color_sensor.red());


            telemetry.update(); //Limited to 100x per second

            // Pause for 40 mS each cycle = update 25 times a second.
            sleep(40);
        }
    }
}

