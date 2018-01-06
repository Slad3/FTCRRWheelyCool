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

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.I2cDevice;
import com.qualcomm.robotcore.hardware.OpticalDistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.configuration.I2cSensor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.ColorSensor;

/**
 * This is NOT an opmode.
 *
 * This class can be used to define all the specific hardware for a single robot.
 * In this case that robot is a K9 robot.
 *
 * This hardware class assumes the following device names have been configured on the robot:
 * Note:  All names are lower case and some have single spaces between words.
 *
 * Motor channel:  Left  drive motor:        "left_drive"
 * Motor channel:  Right drive motor:        "right_drive"
 * Servo channel:  Servo to raise/lower arm: "arm"
 * Servo channel:  Servo to open/close claw: "claw"
 *
 * Note: the configuration of the servos is such that:
 *   As the arm servo approaches 0, the arm position moves up (away from the floor).
 *   As the claw servo approaches 0, the claw opens up (drops the game element).
 */
public class SimpleHardware
{
    /* Public OpMode members. */
    public DcMotor  FL_drive   =    null;
    public DcMotor  FR_drive  =     null;
    public DcMotor  BL_drive   =    null;
    public DcMotor  BR_drive  =     null;
    public DcMotor  Lift      =     null;
    public Servo    Left         = null;
    public Servo    Right        = null;
    public Servo    BallArm     = null;
    public Servo    FrontBoi     = null;

    public final static double LEFT_GRAB = 0.68;
    public final static double RIGHT_GRAB = 0.40;
    public final static double LEFT_RELEASE  = 0.62;
    public final static double RIGHT_RELEASE  = 0.46;
    public final static double LEFT_MIN_RANGE  = 0.29;
    public final static double LEFT_MAX_RANGE  = 1.00; // Use as home.
    public final static double RIGHT_MIN_RANGE  = 0.05; // Use as home.
    public final static double LEFT_HOME  = 1.00;
    public final static double RIGHT_HOME  = 0.05;
    public final static double RIGHT_MAX_RANGE  = 0.70;
    public final static double BALL_ARM_UP = .00;
    public final static double BALL_ARM_DOWN = 1;
    public final static double FRONT_OUT = .98;
    public final static double FRONT_IN = .0;

    /* Local OpMode members. */
    HardwareMap hwMap  = null;
    private ElapsedTime period  = new ElapsedTime();

    /* Constructor */
    public SimpleHardware()
    {
    }

    /* Initialize standard Hardware interfaces */
    public void init(HardwareMap ahwMap)
    {
        // save reference to HW Map
        hwMap = ahwMap;

        // Define and Initialize Motors
        FL_drive  = hwMap.get(DcMotor.class, "FL_drive");
        FR_drive = hwMap.get(DcMotor.class, "FR_drive");
        BL_drive  = hwMap.get(DcMotor.class, "BL_drive");
        BR_drive = hwMap.get(DcMotor.class, "BR_drive");
        Lift = hwMap.get(DcMotor.class, "Lift");

        // Set all motors to zero power
        FL_drive.setPower(0);
        FR_drive.setPower(0);
        BL_drive.setPower(0);
        BR_drive.setPower(0);
        Lift.setPower(0);

        // Set all motors to run without encoders.
        // May want to use RUN_USING_ENCODERS if encoders are installed.
        FL_drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        FR_drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        BL_drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        BR_drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        Lift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // Define and initialize ALL installed servos.
        Left  = hwMap.get(Servo.class, "Left");
        Right = hwMap.get(Servo.class, "Right");
        BallArm = hwMap.get(Servo.class, "BallArm");
        FrontBoi = hwMap.get(Servo.class, "FrontBoi");

        //the below lines set up the configuration file
        //colorA = (ModernRoboticsI2cColorSensor) hwMap.get("colorA");
        Left.setPosition(LEFT_MAX_RANGE);
        Right.setPosition(RIGHT_MIN_RANGE);
        BallArm.setPosition(BALL_ARM_UP);
        FrontBoi.setPosition(FRONT_IN);
    }
}