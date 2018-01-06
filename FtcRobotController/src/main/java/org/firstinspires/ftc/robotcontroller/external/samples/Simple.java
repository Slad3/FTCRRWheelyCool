package org.firstinspires.ftc.robotcontroller.external.samples;

/*
Modern Robotics Color Sensors Example with color number
Created 9/29/2016 by Colton Mehlhoff of Modern Robotics using FTC SDK 2.2
Reuse permitted with credit where credit is due

Configuration:
I2CDevice "ca" (MRI Color Sensor with I2C address 0x3a (0x1d 7-bit)
I2CDevice "cc" (MRI Color Sensor with default I2C address 0x3c (0x1e 7-bit)

ModernRoboticsI2cColorSensor class is not being used because it can not access color number.
ColorSensor class is not being used because it can not access color number.

To change color sensor I2C Addresses, go to http://modernroboticsedu.com/mod/lesson/view.php?id=96
Support is available by emailing support@modernroboticsinc.com.
*/

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cColorSensor;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.GyroSensor;
import com.qualcomm.robotcore.hardware.I2cAddr;
import com.qualcomm.robotcore.hardware.I2cDevice;
import com.qualcomm.robotcore.hardware.I2cDeviceSynch;
import com.qualcomm.robotcore.hardware.I2cDeviceSynchImpl;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.hardware.OpticalDistanceSensor;


@TeleOp(name = "Simple", group = "K9bot")
//@Autonomous(...) is the other common choice
//@Disabled
public class Simple extends OpMode
{
    /* Declare OpMode members. */
    HardwareK9bot   robot            =   new HardwareK9bot();
    double          leftPosition     =   robot.LEFT_MAX_RANGE;                  // Servo safe position
    double          rightPosition    =   robot.RIGHT_MIN_RANGE;                 // Servo safe position
    double          ballPosition     =   robot.BALL_ARM_UP;
    double          frontPosition     =  robot.FRONT_IN;
    double          liftSpeed        =   1;
    double          driveSpeed       =   1;
    final double    LEFT_SPEED       =   0.03;                            // Sets rate to move servo
    final double    RIGHT_SPEED      =   0.03;

    // Sets rates
    double frontLeft;
    double frontRight;
    double backLeft;
    double backRight;
    double Lift;

    private ElapsedTime runtime = new ElapsedTime();
    private ElapsedTime runtime1 = new ElapsedTime();

    // Code to run ONCE when the driver hits INIT
    @Override
    public void init() // Initializes the hardware variables.
    {
     /* Initialize the hardware variables.
     * The init() method of the hardware class does all the work here
     */
        robot.init(hardwareMap);

        telemetry.addData("Status", "Initialized");

        telemetry.addData("Say", "Hello Driver");
        telemetry.update();
    }

    @Override
    public void init_loop()
    {
    }

    // Code to run ONCE when the driver hits PLAY
    @Override
    public void start()
    {
        runtime.reset();
    }


    public void telemetryList()
    {
        telemetry.addData("Status", "Running: " + runtime1.toString());

        // Send telemetry message to signify robot running;
        telemetry.addData("Left", "%.2f", leftPosition);
        telemetry.addData("Right", "%.2f", rightPosition);
        telemetry.addData("Front", "%.2f", frontPosition);
        telemetry.addData("Ball", "%.2f", ballPosition);

        telemetry.addData("frontLeft", "%.2f", frontLeft);
        telemetry.addData("frontRight", "%.2f", frontRight);
        telemetry.addData("backLeft", "%.2f", backLeft);
        telemetry.addData("backRight", "%.2f", backRight);
        telemetry.addData("Lift", "%.2f", Lift);

        telemetry.update(); // Limited to 100x per second
    }

    // Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
    @Override
    public void loop()
    {

        // Run wheels in tank mode (note: The joystick goes negative when pushed forwards, so negate it)
        frontLeft  = ( gamepad1.left_stick_x - gamepad1.left_stick_y - gamepad1.right_stick_x)/2 * driveSpeed; // Front right
        frontRight = ( gamepad1.left_stick_x + gamepad1.left_stick_y - gamepad1.right_stick_x)/2 * driveSpeed; // Front left
        backLeft   = (-gamepad1.left_stick_x - gamepad1.left_stick_y - gamepad1.right_stick_x)/2 * driveSpeed; // Back right
        backRight  = (-gamepad1.left_stick_x + gamepad1.left_stick_y - gamepad1.right_stick_x)/2 * driveSpeed; // Back left
        Lift = gamepad2.left_stick_y * liftSpeed;


        driveSpeed = 1;
        if (gamepad1.right_bumper)
            driveSpeed = 0.5;
        if (gamepad1.left_bumper)
            driveSpeed = 0.25;


        if (gamepad1.right_bumper)
            frontPosition = robot.FRONT_OUT;
        else if (gamepad1.left_bumper)
            frontPosition = robot.FRONT_IN;

        //Controller 2

        if (gamepad1.back || gamepad2.back)
            telemetryList();

        if (gamepad2.a)
            liftSpeed = 1;
        if (gamepad2.b)
            liftSpeed = 0.5;

        if (gamepad2.x)
        {
            rightPosition = 0.96;
            leftPosition = 0.44;
        }

        // Left servo going in means more
        if (gamepad2.left_bumper)
            leftPosition += LEFT_SPEED;
        else if (gamepad2.y)
            leftPosition = robot.LEFT_MIN_RANGE;

        // Right servo going in means less
        if (gamepad2.right_bumper)
            rightPosition -= RIGHT_SPEED;
        else if (gamepad2.y)
            rightPosition = robot.RIGHT_MAX_RANGE;

        if (gamepad2.dpad_up)
            ballPosition += 0.01;
        if (gamepad2.dpad_down)
            ballPosition -= 0.01;

        robot.FL_drive.setPower(frontLeft);
        robot.FR_drive.setPower(frontRight);
        robot.BL_drive.setPower(backLeft);
        robot.BR_drive.setPower(backRight);
        robot.Lift.setPower(Lift);

        // Move all servos to new position.
        leftPosition  = Range.clip(leftPosition, robot.LEFT_MIN_RANGE, robot.LEFT_MAX_RANGE);
        robot.Left.setPosition(leftPosition);
        rightPosition = Range.clip(rightPosition, robot.RIGHT_MIN_RANGE, robot.RIGHT_MAX_RANGE);
        robot.Right.setPosition(rightPosition);
        robot.FrontBoi.setPosition(frontPosition);
        robot.BallArm.setPosition(ballPosition);

        if (gamepad1.a)
            telemetryList();
    }
}

