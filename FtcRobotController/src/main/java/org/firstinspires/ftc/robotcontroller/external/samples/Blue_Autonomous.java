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

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.GyroSensor;
import com.qualcomm.robotcore.hardware.I2cAddr;
import com.qualcomm.robotcore.hardware.I2cDevice;
import com.qualcomm.robotcore.hardware.I2cDeviceSynch;
import com.qualcomm.robotcore.hardware.I2cDeviceSynchImpl;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * This file illustrates the concept of driving a path based on time.
 * It uses the common Pushbot hardware class to define the drive on the robot.
 * The code is structured as a LinearOpMode
 *
 * The code assumes that you do NOT have encoders on the wheels,
 *   otherwise you would use: PushbotAutoDriveByEncoder;
 *
 *   The desired path in this example is:
 *   - Drive forward for 3 seconds
 *   - Spin right for 1.3 seconds
 *   - Drive Backwards for 1 Second
 *   - Stop and close the claw.
 *
 *  The code is written in a simple form with no optimizations.
 *  However, there are several ways that this type of sequence could be streamlined,
 *
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

@Autonomous(name="Blue Autonomous", group="K9bot")
//@Disabled
public class Blue_Autonomous extends LinearOpMode {

    /* Declare OpMode members. */
    HardwareK9bot         robot   = new HardwareK9bot();   // Use a Pushbot's hardware
    private ElapsedTime     runtime = new ElapsedTime();

    int heading;       // Heading left/right. Integer between 0 and 359

    boolean bPrevState = false;
    boolean bCurrState = false;
    boolean bLedOn = true; // bLedOn represents the state of the LED.

    GyroSensor sensorGyro;  // General Gyro Sensor allows us to point to the sensor in the configuration file.
    ModernRoboticsI2cGyro mrGyro;  // ModernRoboticsI2cGyro allows us to .getIntegratedZValue()

    ColorSensor colorSensor;    // Hardware Device Object

    byte[] range1Cache; //The read will return an array of bytes. They are stored in this variable
    I2cDevice RANGE1;
    I2cDeviceSynch RANGE1Reader;

    I2cAddr RANGE1ADDRESS = new I2cAddr(0x14); // Default I2C address for MR Range (7-bit)

    public static final int RANGE1_REG_START = 0x04; // Register to start reading
    public static final int RANGE1_READ_LENGTH = 2; // Number of byte to read

    // Moves or turns robot in specified direction, power, and duration. Then stops.
    public void movePower(String movement, double power, double duration) {
        double startTime = getRuntime();
        switch (movement) {
            case "forward":
                runtime.reset();
                while (runtime.seconds() < duration) {
                    robot.FL_drive.setPower(power);
                    robot.FR_drive.setPower(power * -1);
                    robot.BL_drive.setPower(power);
                    robot.BR_drive.setPower(power * -1);
                }
                break;
            case "backward":
                runtime.reset();
                while (runtime.seconds() < duration) {
                    robot.FL_drive.setPower(power * -1);
                    robot.FR_drive.setPower(power);
                    robot.BL_drive.setPower(power * -1);
                    robot.BR_drive.setPower(power);
                }
                break;
            case "right":
                runtime.reset();
                while (runtime.seconds() < duration) {
                    robot.FL_drive.setPower(power);
                    robot.FR_drive.setPower(power);
                    robot.BL_drive.setPower(power * -1);
                    robot.BR_drive.setPower(power * -1);
                }
                break;
            case "left":
                runtime.reset();
                while (runtime.seconds() < duration) {
                    robot.FL_drive.setPower(power * -1);
                    robot.FR_drive.setPower(power * -1);
                    robot.BL_drive.setPower(power);
                    robot.BR_drive.setPower(power);
                }
                break;
            case "leftTurn":
                runtime.reset();
                while (runtime.seconds() < duration) {
                    robot.FL_drive.setPower(power);
                    robot.FR_drive.setPower(power);
                    robot.BL_drive.setPower(power);
                    robot.BR_drive.setPower(power);
                }
                break;
            case "rightTurn":
                runtime.reset();
                while (runtime.seconds() < duration) {
                    robot.FL_drive.setPower(power * -1);
                    robot.FR_drive.setPower(power * -1);
                    robot.BL_drive.setPower(power * -1);
                    robot.BR_drive.setPower(power * -1);
                }
                break;
            case "Lift":
                runtime.reset();
                while (runtime.seconds() < duration) {
                    robot.Lift.setPower(power);
                }
                break;
        }
        motorStop();
    }

    // Moves or turns robot in specified direction, power, and duration. Does not stop itself.
    public void smoothMovePower(String movement, double power, double duration) {
        double startTime = getRuntime();
        switch (movement) {
            case "forward":
                runtime.reset();
                while (runtime.seconds() < duration) {
                    robot.FL_drive.setPower(power);
                    robot.FR_drive.setPower(power * -1);
                    robot.BL_drive.setPower(power);
                    robot.BR_drive.setPower(power * -1);
                }
                break;
            case "backward":
                runtime.reset();
                while (runtime.seconds() < duration) {
                    robot.FL_drive.setPower(power * -1);
                    robot.FR_drive.setPower(power);
                    robot.BL_drive.setPower(power * -1);
                    robot.BR_drive.setPower(power);
                }
                break;
            case "right":
                runtime.reset();
                while (runtime.seconds() < duration) {
                    robot.FL_drive.setPower(power);
                    robot.FR_drive.setPower(power);
                    robot.BL_drive.setPower(power * -1);
                    robot.BR_drive.setPower(power * -1);
                }
                break;
            case "left":
                runtime.reset();
                while (runtime.seconds() < duration) {
                    robot.FL_drive.setPower(power * -1);
                    robot.FR_drive.setPower(power * -1);
                    robot.BL_drive.setPower(power);
                    robot.BR_drive.setPower(power);
                }
                break;
            case "leftTurn":
                runtime.reset();
                while (runtime.seconds() < duration) {
                    robot.FL_drive.setPower(power);
                    robot.FR_drive.setPower(power);
                    robot.BL_drive.setPower(power);
                    robot.BR_drive.setPower(power);
                }
                break;
            case "rightTurn":
                runtime.reset();
                while (runtime.seconds() < duration) {
                    robot.FL_drive.setPower(power * -1);
                    robot.FR_drive.setPower(power * -1);
                    robot.BL_drive.setPower(power * -1);
                    robot.BR_drive.setPower(power * -1);
                }
                break;
        }
    }

    // Sets power of all drive motors to zero.
    public void motorStop() {
        robot.FL_drive.setPower(0);
        robot.FR_drive.setPower(0);
        robot.BL_drive.setPower(0);
        robot.BR_drive.setPower(0);
        robot.Lift.setPower(0);
    }

    // Sets power of all drive motors to zero.
    public void motorWait(double duration) {

        movePower("forward", 0, duration);
    }

    // Fixes any headings passed into it into the range of 0 - 360.
    public int cleanUp(int input) {
        if (input == 360)
            return 0;
        if (input < 0)
            return (input + 360);
        if (input > 360)
            return (input - 360);
        return input;
    }

    // Turns a number of degrees compared to where the robot was when the program started. Positive numbers turn left.
    public void turnAbsolute(int target) {
        heading = 360 - mrGyro.getHeading();  // Reverse direction of heading to match the integrated value
        heading = cleanUp(heading);
        int heading = cleanUp(360 - mrGyro.getHeading()); //Set variable to gyro readings
        for (int i = 0; i < 2; i++) {
            while (Math.abs(heading - target) > 180) {  //Continue while the robot direction is further than 180 degrees from the target
                if (heading < target)
                    smoothMovePower("leftTurn", 1, 0.5);
                else if (heading > target)
                    smoothMovePower("rightTurn", 1, 0.5);
                heading = cleanUp(360 - mrGyro.getHeading());  //Set variable to gyro reading
                telemetry.addData("heading", String.format("%03d", heading));
                telemetry.update();
            }
            while (Math.abs(heading - target) > 90) {  //Continue while the robot direction is further than 90 degrees from the target
                if (Math.abs(heading - target) > 270)
                    break;
                if (heading > target)
                    smoothMovePower("leftTurn", 1, 0.5);
                else if (heading < target)
                    smoothMovePower("rightTurn", 1, 0.5);
                heading = cleanUp(360 - mrGyro.getHeading());  //Set variable to gyro reading
                telemetry.addData("heading", String.format("%03d", heading));
                telemetry.update();
            }
            while (Math.abs(heading - target) > 45) {  //Continue while the robot direction is further than 45 degrees from the target
                if (Math.abs(heading - target) > 315)
                    break;
                if (heading > target)
                    smoothMovePower("leftTurn", 1, 0.25);
                else if (heading < target)
                    smoothMovePower("rightTurn", 1, 0.25);
                heading = cleanUp(360 - mrGyro.getHeading());  //Set variable to gyro reading
                telemetry.addData("heading", String.format("%03d", heading));
                telemetry.update();
            }
            while (Math.abs(heading - target) > 10) {  //Continue while the robot direction is further than 10 degrees from the target
                if (Math.abs(heading - target) > 350)
                    break;
                if (heading > target)
                    movePower("leftTurn", 0.25, 0.25);
                else if (heading < target)
                    movePower("rightTurn", 0.25, 0.25);
                heading = cleanUp(360 - mrGyro.getHeading());  //Set variable to gyro reading
                telemetry.addData("heading", String.format("%03d", heading));
                telemetry.update();
            }
        }
        motorStop();
        movePower("forward", 0, 0.25);
        motorStop();
        heading = cleanUp(360 - mrGyro.getHeading());  //Set variable to gyro reading
        int degreesError = heading - target; // If positive left, if negative right
        if (degreesError > 0)
            movePower("leftTurn", 1, degreesError / 90);
        else
            movePower("rightTurn", 1, Math.abs(degreesError) / 90);
        motorStop();
    }

    public String colorTranspose() {
        colorSensor.enableLed(bLedOn);
        int redd = colorSensor.red();
        int greenn = colorSensor.green();
        int bluee = colorSensor.blue();
        if ((redd + greenn + bluee) < 5)
            return "none";
        if (redd > greenn && redd > bluee)
            return "red";
        if (greenn > bluee)
            return "green";
        return "blue";
    }

    // Reads the gyro
    public void gyroRead() {
        heading = 360 - mrGyro.getHeading();  // Reverse direction of heading to match the integrated value
        heading = cleanUp(heading);
        telemetry.addData("Heading ", String.format("%03d", heading));
    }

    // Reads the range sensor
    public void rangeRead() {
        range1Cache = RANGE1Reader.read(RANGE1_REG_START, RANGE1_READ_LENGTH);
    }

    public void knockBall(String team) {
        robot.BallArm.setPosition(robot.BALL_ARM_DOWN);
        sleep(1000);
        runtime.reset();
        while (opModeIsActive() && (runtime.seconds() < 1)) {
            telemetry.addData("Waiting 1 Second", "%2.5f S Elapsed", runtime.seconds());
            telemetry.update();
        }
        String ballColor = colorTranspose();
        telemetry.addData("Got color transpose:", ballColor);
        telemetry.update();
        double duration = 0.5;
        double power = 0.25;
        if (team.equals(ballColor)) {
            smoothMovePower("rightTurn", power, duration);
            robot.BallArm.setPosition(robot.BALL_ARM_UP);
            smoothMovePower("leftTurn", power, duration);
        } else if (ballColor.equals("none")) {
            motorStop();
            robot.BallArm.setPosition(robot.BALL_ARM_UP);
        } else {
            smoothMovePower("leftTurn", power, duration);
            robot.BallArm.setPosition(robot.BALL_ARM_UP);
            smoothMovePower("rightTurn", power, duration);
        }
        motorStop();
    }

    // Moves robot close enough to back wall to begin correctXAxis.
    public void correctYAxisBackWall() {
        motorStop();
        turnAbsolute(90);
        while (true) {
            rangeRead();
            if (range1Cache[0] < (17 * 2.54)) {
                motorStop();
                break;
            }
            smoothMovePower("forward", .25, .1);
        }
        return;
    }

    @Override
    public void runOpMode() {

        /*
         * Initialize the drive system variables.
         * The init() method of the hardware class does all the work here
         */
        robot.init(hardwareMap);
        sensorGyro = hardwareMap.gyroSensor.get("gyro");  // Point to the gyro in the configuration file
        mrGyro = (ModernRoboticsI2cGyro) sensorGyro;      // ModernRoboticsI2cGyro allows us to .getIntegratedZValue()
        mrGyro.calibrate();  // Calibrate the sensor so it knows where 0 is and what still is. DO NOT MOVE SENSOR WHILE BLUE LIGHT IS SOLID

        colorSensor = hardwareMap.get(ColorSensor.class, "sensor_color");

        RANGE1 = hardwareMap.i2cDevice.get("RANGE1");
        RANGE1Reader = new I2cDeviceSynchImpl(RANGE1, RANGE1ADDRESS, false);
        RANGE1Reader.engage();

        // Send telemetry message to signify robot waiting;
        telemetry.addData("Status", "Ready to run");    //
        telemetry.update();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        // Step through each leg of the path, ensuring that the Auto mode has not been stopped along the way

        // Step 1:  Wait a second
        motorWait(1);
        runtime.reset();
        while (opModeIsActive() && (runtime.seconds() < 1)) {
            telemetry.addData("Waiting 1 Second", "%2.5f S Elapsed", runtime.seconds());
            telemetry.update();
        }

        // Step 2:  Set servos in closed around block
        robot.Left.setPosition(robot.LEFT_GRAB);
        robot.Right.setPosition(robot.RIGHT_GRAB);

        // Step 3:  Raise lift a bit
        movePower("Lift", 0.25, 0.25);
        motorStop();
        runtime.reset();
        while (opModeIsActive() && (runtime.seconds() < 0.25)) {
            telemetry.addData("Lifting", "%2.5f S Elapsed", runtime.seconds());
            telemetry.update();
        }

        // Step 4:  Knock the ball
        knockBall("red");

        sleep(500);

        // Step 5: Drive forward
        movePower("forward", 0.5, 1.5);
        sleep(500);

        // Step 6: Turn to wall
        turnAbsolute(90);

        // Step 7: Correct Y
        correctYAxisBackWall();

        // Step 8: Place block
        movePower("forward", 0.25, 0.75);
        movePower("forward", 0.1, 0.75);
        movePower("forward", 0.05, 3);

        // Step 9:  Stop all motors.
        motorStop();
        robot.Left.setPosition(robot.LEFT_HOME);
        robot.Right.setPosition(robot.RIGHT_HOME);
        sleep(1000);

        // Step 10: Back up
        movePower("backward", 1, 0.5);

        telemetry.addData("Path", "Complete");
        telemetry.update();
        sleep(1000);
    }
}
