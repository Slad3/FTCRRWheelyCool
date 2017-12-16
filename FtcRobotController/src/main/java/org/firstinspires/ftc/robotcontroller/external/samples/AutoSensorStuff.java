
package org.firstinspires.ftc.robotcontroller.external.samples;

/*
Modern Robotics Color Sensors Example with color number
Created 9/29/2016 by Cqaolton Mehlhoff of Modern Robotics using FTC SDK 2.2
Reuse permitted with credit where credit is due

Configuration:
I2CDevice "ca" (MRI Color Sensor with I2C address 0x3a (0x1d 7-bit)
I2CDevice "cc" (MRI Color Sensor with default I2C address 0x3c (0x1e 7-bit)

ModernRoboticsI2cColorSensor class is not being used because it can not access color number.
ColorSensor class is not being used because it can not access color number.

To change color sensor I2C Addresses, go to http://modernroboticsedu.com/mod/lesson/view.php?id=96
Support is available by emailing support@modernroboticsinc.com.
*/

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.configuration.I2cSensor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.I2cAddr;
import com.qualcomm.robotcore.hardware.I2cDevice;
import com.qualcomm.robotcore.hardware.I2cDeviceSynch;
import com.qualcomm.robotcore.hardware.I2cDeviceSynchImpl;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.GyroSensor;
import com.qualcomm.robotcore.util.ElapsedTime;


@TeleOp(name = "Testing Auto Stuff", group = "K9bot")
//@Autonomous(...) is the other common choice
//@Disabled
public class AutoSensorStuff extends OpMode {

    /* Declare OpMode members. */
    HardwareK9bot robot = new HardwareK9bot();
    double leftPosition = robot.LEFT_HOME;                  // Servo safe position
    double rightPosition = robot.RIGHT_HOME;                 // Servo safe position
    double liftSpeed = 1;
    double driveSpeed = 1;
    //final double    LEFT_HOME        =   robot.LEFT_HOME;
    //final double    RIGHT_HOME       =   robot.RIGHT_HOME;
    //final double    LEFT_MAX_RANGE   =   robot.LEFT_MAX_RANGE;
    final double LEFT_MIN_RANGE = robot.LEFT_MIN_RANGE;
    final double RIGHT_MAX_RANGE = robot.RIGHT_MAX_RANGE;
    //final double    RIGHT_MIN_RANGE  =   robot.RIGHT_MIN_RANGE;
    final double LEFT_SPEED = 0.03;                            // sets rate to move servo
    final double RIGHT_SPEED = 0.03;

    // sets rates
    double frontLeft;
    double frontRight;
    double backLeft;
    double backRight;
    double Lift;
    boolean red, blue, green;
    boolean Adetects;
    boolean Cdetects;

    private ElapsedTime runtime = new ElapsedTime();
    private ElapsedTime runtime1 = new ElapsedTime();

    byte[] colorAcache;
    byte[] colorCcache;

    I2cDevice colorA;
    I2cDevice colorC;
    I2cDeviceSynch colorAreader;
    I2cDeviceSynch colorCreader;

    //TouchSensor touch;         //Instance of TouchSensor - for changing color sensor mode

    boolean buttonState = false;  //Tracks the last known state of the touch sensor
    boolean LEDState = true;     //Tracks the mode of the color sensor; Active = true, Passive = false

    //int zAccumulated;  //Total rotation left/right
    int heading;       //Heading left/right. Integer between 0 and 359
    int target = 0;  //Desired angle to turn to
    int xVal, yVal, zVal;  //Momentary rate of rotation in three axis

    GyroSensor sensorGyro;  //General Gyro Sensor allows us to point to the sensor in the configuration file.
    ModernRoboticsI2cGyro mrGyro;  //ModernRoboticsI2cGyro allows us to .getIntegratedZValue()

    public void stop() {
        robot.FL_drive.setPower(0);
        robot.FR_drive.setPower(0);
        robot.BL_drive.setPower(0);
        robot.BR_drive.setPower(0);
    }

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
        }
        stop();
    }

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

    public void smoothMovePowerinstant(String movement, double power) {

        switch (movement) {
            case "forward":
                robot.FL_drive.setPower(power);
                robot.FR_drive.setPower(power * -1);
                robot.BL_drive.setPower(power);
                robot.BR_drive.setPower(power * -1);

                break;

            case "backward":
                robot.FL_drive.setPower(power * -1);
                robot.FR_drive.setPower(power);
                robot.BL_drive.setPower(power * -1);
                robot.BR_drive.setPower(power);
                break;

            case "right":
                robot.FL_drive.setPower(power);
                robot.FR_drive.setPower(power);
                robot.BL_drive.setPower(power * -1);
                robot.BR_drive.setPower(power * -1);
                break;

            case "left":
                robot.FL_drive.setPower(power * -1);
                robot.FR_drive.setPower(power * -1);
                robot.BL_drive.setPower(power);
                robot.BR_drive.setPower(power);
                break;

            case "leftTurn":
                robot.FL_drive.setPower(power);
                robot.FR_drive.setPower(power);
                robot.BL_drive.setPower(power);
                robot.BR_drive.setPower(power);
                break;

            case "rightTurn":
                robot.FL_drive.setPower(power * -1);
                robot.FR_drive.setPower(power * -1);
                robot.BL_drive.setPower(power * -1);
                robot.BR_drive.setPower(power * -1);
                break;
        }
    }


    public void turn(int target) throws InterruptedException {
        turnAbsolute(target + mrGyro.getHeading());
    }

    //This function turns a number of degrees compared to where the robot was when the program started. Positive numbers turn left.
    public void turnAbsolute(int target) {
        int heading = 360 - mrGyro.getHeading(); //Set variables to gyro readings

        while (Math.abs(heading - target) > 90) {  //Continue while the robot direction is further than 90 degrees from the target
            if (heading > target)
                smoothMovePower("leftTurn", 1, 0.5);

            if (heading < target)
                smoothMovePower("rightTurn", 1, 0.5);

            heading = 360 - mrGyro.getHeading();  //Set variables to gyro readings
            telemetry.addData("heading", String.format("%03d", heading));
            telemetry.update();
        }

        while (Math.abs(heading - target) > 45) {  //Continue while the robot direction is further than 45 degrees from the target
            if (heading > target)
                smoothMovePower("leftTurn", 1, 0.25);

            if (heading < target)
                smoothMovePower("rightTurn", 1, 0.25);

            heading = 360 - mrGyro.getHeading();  //Set variables to gyro readings
            telemetry.addData("heading", String.format("%03d", heading));
            telemetry.update();
        }

        while (Math.abs(heading - target) > 10) {  //Continue while the robot direction is further than 10 degrees from the target
            if (heading > target)
                movePower("leftTurn", 0.25, 0.25);

            if (heading < target)
                movePower("rightTurn", 0.25, 0.25);

            heading = 360 - mrGyro.getHeading();  //Set variables to gyro readings
            telemetry.addData("heading", String.format("%03d", heading));
            telemetry.update();
        }
        stop();
    }

    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init() {
     /* Initialize the hardware variables.
     * The init() method of the hardware class does all the work here
     */
        robot.init(hardwareMap);

        telemetry.addData("Status", "Initialized");

        //the below lines set up the configuration file
        colorA = hardwareMap.i2cDevice.get("colorA");
        colorC = hardwareMap.i2cDevice.get("colorC");

        colorAreader = new I2cDeviceSynchImpl(colorA, I2cAddr.create8bit(0x3a), false);
        colorCreader = new I2cDeviceSynchImpl(colorC, I2cAddr.create8bit(0x3c), false);

        colorAreader.engage();
        colorCreader.engage();

        sensorGyro = hardwareMap.gyroSensor.get("gyro");  //Point to the gyro in the configuration file
        mrGyro = (ModernRoboticsI2cGyro) sensorGyro;      //ModernRoboticsI2cGyro allows us to .getIntegratedZValue()
        mrGyro.calibrate();  //Calibrate the sensor so it knows where 0 is and what still is. DO NOT MOVE SENSOR WHILE BLUE LIGHT IS SOLID

        telemetry.addData("Say", "Hello Driver");    //
        telemetry.update();

    }

    @Override
    public void init_loop() {
    }

    /*
     * Code to run ONCE when the driver hits PLAY
     */
    @Override
    public void start() {
        runtime.reset();

        while (mrGyro.isCalibrating()) { //Ensure calibration is complete (usually 2 seconds)
        }

        if (LEDState) {
            colorAreader.write8(3, 0);    //Set the mode of the color sensor using LEDState
            colorCreader.write8(3, 0);    //Set the mode of the color sensor using LEDState
        } else {
            colorAreader.write8(3, 1);    //Set the mode of the color sensor using LEDState
            colorCreader.write8(3, 1);    //Set the mode of the color sensor using LEDState
        }
        //Active - For measuring reflected light. Cancels out ambient light
        //Passive - For measuring ambient light, eg. the FTC Color Beacon
    }

    /*
     * Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
     */




    //Make the sensors actually sensor

    public void correctXAxis(boolean leftSensorDetected, boolean rightSensorDetected) {



        boolean xAxisSet = false;

        if (rightSensorDetected && leftSensorDetected) {
            xAxisSet = true;
        }

        if (!leftSensorDetected && !rightSensorDetected) {

            smoothMovePower("left", 1.0, 3.0);
            smoothMovePower("right", 1.0, 6.0);
        }


        while (!xAxisSet) {

            if (leftSensorDetected && !rightSensorDetected) {

                smoothMovePowerinstant("left", 0.5);
            }

            if (!leftSensorDetected && rightSensorDetected) {

                smoothMovePowerinstant("right", 0.5);
            }

        }
    }

    public void correctYAxis(double length) {
        //Use distace to wall to correct the Y axis
        boolean distanceSet = false;
        while (distanceSet /*sensor lengthn > length*/) {
            smoothMovePowerinstant("forward", 1.0);
        }

    }

    //Connect the sensor to the method
    public void correctZAxis() {


        boolean blockPresent = false;   //Change to be the sensor
        //Detects if block is in a location
        while (!blockPresent) {
            robot.Lift.setPower(1000);
        }

    }

    public void release() {

        //let go of both servos


        //raise lift above the block to prevent tipping
        robot.Lift.setPower(1);

        //smoothMovePower("backwards", 1.0, .5);

        //return;

    }

    public void orient() {
        //Compares current orientations to preset orientation

        //correctYaw(presetYaw);

        //correctXAxis();

        //correctYAxis(general distance);

        //correctZAxis();

        //correctXAxis();

        //correctYAxis(A lot closer to the walll);


    }

    public void autoPlace() {

        while (gamepad1.right_trigger > 50.0) {

            //orient();

            //pauses for 1.5 seconds for human confirmation
            //sleep(1.5);

            // while (/*Until block is in place/sensor detects that block is in place*/)
            //move forward very slowly


            //release();

        }

        //This function turns a number of degrees compared to where the robot is. Positive numbers turn left.


    }




        @Override
    public void loop() {
        telemetry.addData("Status", "Running: " + runtime1.toString());

        //zAccumulated = mrGyro.getIntegratedZValue();  //Set variables to gyro readings

        heading = 360 - mrGyro.getHeading();  //Reverse direction of heading to match the integrated value
        if (heading == 360)
            heading = 0;

        xVal = mrGyro.rawX() / 128;  //Lowest 7 bits are noise
        yVal = mrGyro.rawY() / 128;
        zVal = mrGyro.rawZ() / 128;

        //The below two if() statements ensure that the mode of the color sensor is changed only once each time the touch sensor is pressed.
        //The mode of the color sensor is saved to the sensor's long term memory. Just like flash drives, the long term memory has a life time in the 10s or 100s of thousands of cycles.
        //This seems like a lot but if your program wrote to the long term memory every time though the main loop, it would shorten the life of your sensor.

        if (true) {  //If the touch sensor is just now being pressed (was not pressed last time through the loop but now is)

            buttonState = true;                   //Change touch state to true because the touch sensor is now pressed
            LEDState = !LEDState;                //Change the LEDState to the opposite of what it was
            if (LEDState) {
                colorAreader.write8(3, 0);    //Set the mode of the color sensor using LEDState
                colorCreader.write8(3, 0);    //Set the mode of the color sensor using LEDState
            } else {
                colorAreader.write8(3, 1);    //Set the mode of the color sensor using LEDState
                colorCreader.write8(3, 1);    //Set the mode of the color sensor using LEDState
            }
        }

        if (true) { //If the touch sensor is now pressed
            buttonState = false;                  //Set the buttonState to false to indicate that the touch sensor was released
        }


        colorAcache = colorAreader.read(0x04, 1);
        colorCcache = colorCreader.read(0x04, 1);



        if(colorAcache[0] == 8 || colorAcache[0] == 10 || colorAcache[0] == 3)
            Adetects = true;

        if(colorCcache[0] == 8 || colorCcache[0] == 10 || colorCcache[0] == 3)
            Cdetects = true;






        if (gamepad2.a)
            liftSpeed = 1;
        if (gamepad2.b)
            liftSpeed = 0.5;



        if (gamepad2.a)
            movePower("left", 0.5, 0.5);

        if (gamepad2.b)

            movePower("right", 0.5, 0.5);

        driveSpeed = 1;
        if (gamepad1.right_bumper)
            driveSpeed = 0.5;
        if (gamepad1.left_bumper)
            driveSpeed = 0.25;

        // Run wheels in tank mode (note: The joystick goes negative when pushed forwards, so negate it)
        frontLeft = (gamepad1.left_stick_x - gamepad1.left_stick_y - gamepad1.right_stick_x) / 2 * driveSpeed; //Front right
        frontRight = (gamepad1.left_stick_x + gamepad1.left_stick_y - gamepad1.right_stick_x) / 2 * driveSpeed; //Front left
        backLeft = (-gamepad1.left_stick_x - gamepad1.left_stick_y - gamepad1.right_stick_x) / 2 * driveSpeed;  //Back right
        backRight = (-gamepad1.left_stick_x + gamepad1.left_stick_y - gamepad1.right_stick_x) / 2 * driveSpeed; //Back left
        Lift = gamepad2.left_stick_y * liftSpeed;

        robot.FL_drive.setPower(frontLeft);
        robot.FR_drive.setPower(frontRight);
        robot.BL_drive.setPower(backLeft);
        robot.BR_drive.setPower(backRight);
        robot.Lift.setPower(Lift);

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

        if (gamepad2.x){
            rightPosition = 0.96;
            leftPosition = 0.44;
        }


        if (gamepad1.y){}
        if (gamepad1.a){
            correctXAxis(Cdetects, Adetects);
        }
        if (gamepad1.b)



        if (gamepad1.dpad_up)

            movePower("forward", 1, 0.5);
        if (gamepad1.dpad_down)

            movePower("backward", 1, 0.5);
        if (gamepad1.dpad_left)

            movePower("leftTurn", 1, 0.5);
        if (gamepad1.dpad_right)

            movePower("rightTurn", 1, 0.5);

        // Move both servos to new position.
        leftPosition = Range.clip(leftPosition, robot.LEFT_MIN_RANGE, robot.LEFT_MAX_RANGE);
        robot.Left.setPosition(leftPosition);
        rightPosition = Range.clip(rightPosition, robot.RIGHT_MIN_RANGE, robot.RIGHT_MAX_RANGE);
        robot.Right.setPosition(rightPosition);

        // Send telemetry message to signify robot running;
        telemetry.addData("Left", "%.2f", leftPosition);
        telemetry.addData("Right", "%.2f", rightPosition);
        telemetry.addData("frontLeft", "%.2f", frontLeft);
        telemetry.addData("frontRight", "%.2f", frontRight);
        telemetry.addData("backLeft", "%.2f", backLeft);
        telemetry.addData("backRight", "%.2f", backRight);
        telemetry.addData("Lift", "%.2f", Lift);

        //display values
        telemetry.addData("1 #A", colorAcache[0] & 0xFF);
        telemetry.addData("2 #C", colorCcache[0] & 0xFF);

        telemetry.addData("3 A", colorAreader.getI2cAddress().

                get8Bit());
        telemetry.addData("4 A", colorCreader.getI2cAddress().

                get8Bit());

        telemetry.addData("1. heading", String.format("%03d", heading));  //Display variables to Driver Station Screen
        telemetry.addData("2. target", String.format("%03d", target));
        telemetry.addData("3. X", String.format("%03d", xVal));
        telemetry.addData("4. Y", String.format("%03d", yVal));
        telemetry.addData("5. Z", String.format("%03d", zVal));

        telemetry.update(); //Limited to 100x per second

    }
}












