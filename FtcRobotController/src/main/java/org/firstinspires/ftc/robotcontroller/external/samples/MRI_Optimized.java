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
        import android.app.Activity;
        import android.graphics.Color;
        import android.view.View;
        import com.qualcomm.ftcrobotcontroller.R;
        import com.qualcomm.robotcore.eventloop.opmode.Disabled;
        import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
        import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
        import com.qualcomm.robotcore.hardware.ColorSensor;


@TeleOp(name = "Optimization", group = "K9bot")
//@Autonomous(...) is the other common choice
//@Disabled
public class MRI_Optimized extends OpMode
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
    boolean red, blue;
    boolean firstCycle = true;

    private ElapsedTime runtime = new ElapsedTime();
    private ElapsedTime runtime1 = new ElapsedTime();

    byte[] range1Cache; //The read will return an array of bytes. They are stored in this variable
    I2cDevice RANGE1;
    I2cDeviceSynch RANGE1Reader;

    OpticalDistanceSensor ods = robot.ods;

    double odsReadingRaw;
    static double odsReadingLinear;

    //ColorSensor colorSensor;    // Hardware Device Object
    boolean bPrevState = false;
    boolean bCurrState = false;
    boolean bLedOn = true; // bLedOn represents the state of the LED.

    //sensor value between 0 and 1023
    int raw1;
    int state = 0;
    int count = 0;

    //int zAccumulated;  // Total rotation left/right
    int heading;       // Heading left/right. Integer between 0 and 359
    int target = 0;  // Desired angle to turn to
    int temp, temp1;
    double temp2;
    double degreesPer10thSecond = 0;
    double degreesPerSecond = 0;

    GyroSensor sensorGyro;  // General Gyro Sensor allows us to point to the sensor in the configuration file.
    ModernRoboticsI2cGyro mrGyro;  // ModernRoboticsI2cGyro allows us to .getIntegratedZValue()

    I2cAddr RANGE1ADDRESS = new I2cAddr(0x14); // Default I2C address for MR Range (7-bit)
    public static final int RANGE1_REG_START = 0x04; // Register to start reading
    public static final int RANGE1_READ_LENGTH = 2; // Number of byte to read

    // Sets power of all drive motors to zero.
    public void motorStop()
    {
        robot.FL_drive.setPower(0);
        robot.FR_drive.setPower(0);
        robot.BL_drive.setPower(0);
        robot.BR_drive.setPower(0);
    }

    // Moves or turns robot in specified direction, power, and duration. Then stops.
    public void movePower(String movement, double power, double duration)
    {
        double startTime = getRuntime();
        switch(movement)
        {
            case "forward":
                runtime.reset();
                while (runtime.seconds() < duration)
                {
                    robot.FL_drive.setPower(power);
                    robot.FR_drive.setPower(power * -1);
                    robot.BL_drive.setPower(power);
                    robot.BR_drive.setPower(power * -1);
                }
                break;

            case "backward":
                runtime.reset();
                while (runtime.seconds() < duration)
                {
                    robot.FL_drive.setPower(power * -1);
                    robot.FR_drive.setPower(power);
                    robot.BL_drive.setPower(power * -1);
                    robot.BR_drive.setPower(power);
                }
                break;

            case "right":
                runtime.reset();
                while (runtime.seconds() < duration)
                {
                    robot.FL_drive.setPower(power);
                    robot.FR_drive.setPower(power);
                    robot.BL_drive.setPower(power * -1);
                    robot.BR_drive.setPower(power * -1);
                }
                break;

            case "left":
                runtime.reset();
                while (runtime.seconds() < duration)
                {
                    robot.FL_drive.setPower(power * -1);
                    robot.FR_drive.setPower(power * -1);
                    robot.BL_drive.setPower(power);
                    robot.BR_drive.setPower(power);
                }
                break;

            case "leftTurn":
                runtime.reset();
                while (runtime.seconds() < duration)
                {
                    robot.FL_drive.setPower(power);
                    robot.FR_drive.setPower(power);
                    robot.BL_drive.setPower(power);
                    robot.BR_drive.setPower(power);
                }
                break;

            case "rightTurn":
                runtime.reset();
                while (runtime.seconds() < duration)
                {
                    robot.FL_drive.setPower(power * -1);
                    robot.FR_drive.setPower(power * -1);
                    robot.BL_drive.setPower(power * -1);
                    robot.BR_drive.setPower(power * -1);
                }
                break;
        }
        motorStop();
    }

    // Moves or turns robot in specified direction, power, and duration. Does not stop itself.
    public void smoothMovePower(String movement, double power, double duration)
    {
        double startTime = getRuntime();
        switch(movement)
        {
            case "forward":
                runtime.reset();
                while (runtime.seconds() < duration)
                {
                    robot.FL_drive.setPower(power);
                    robot.FR_drive.setPower(power * -1);
                    robot.BL_drive.setPower(power);
                    robot.BR_drive.setPower(power * -1);
                }
                break;

            case "backward":
                runtime.reset();
                while (runtime.seconds() < duration)
                {
                    robot.FL_drive.setPower(power * -1);
                    robot.FR_drive.setPower(power);
                    robot.BL_drive.setPower(power * -1);
                    robot.BR_drive.setPower(power);
                }
                break;

            case "right":
                runtime.reset();
                while (runtime.seconds() < duration)
                {
                    robot.FL_drive.setPower(power);
                    robot.FR_drive.setPower(power);
                    robot.BL_drive.setPower(power * -1);
                    robot.BR_drive.setPower(power * -1);
                }
                break;

            case "left":
                runtime.reset();
                while (runtime.seconds() < duration)
                {
                    robot.FL_drive.setPower(power * -1);
                    robot.FR_drive.setPower(power * -1);
                    robot.BL_drive.setPower(power);
                    robot.BR_drive.setPower(power);
                }
                break;

            case "leftTurn":
                runtime.reset();
                while (runtime.seconds() < duration)
                {
                    robot.FL_drive.setPower(power);
                    robot.FR_drive.setPower(power);
                    robot.BL_drive.setPower(power);
                    robot.BR_drive.setPower(power);
                }
                break;

            case "rightTurn":
                runtime.reset();
                while (runtime.seconds() < duration)
                {
                    robot.FL_drive.setPower(power * -1);
                    robot.FR_drive.setPower(power * -1);
                    robot.BL_drive.setPower(power * -1);
                    robot.BR_drive.setPower(power * -1);
                }
                break;
        }
    }

    // Turns a number of degrees compared to where the robot is. Positive numbers turn left.
    public void changeAngle(int degreesFromCurrentAngle, double degreesPer10thSecond)
    {
        if (degreesFromCurrentAngle < 0)
        {
            degreesFromCurrentAngle = Math.abs(degreesFromCurrentAngle);
            movePower("leftTurn", 0.25, degreesFromCurrentAngle / degreesPerSecond);
        }
        else
        {
            movePower("rightTurn", 0.25, degreesFromCurrentAngle / degreesPerSecond);
        }
        motorStop();
    }

    // Turns a number of degrees compared to where the robot was when the program started. Positive numbers turn left.
    public void turnAbsolute(int target)
    {
        heading = 360 - mrGyro.getHeading();  // Reverse direction of heading to match the integrated value
        heading = cleanUp(heading);

        int heading = cleanUp(360 - mrGyro.getHeading()); //Set variable to gyro readings
        for (int i = 0; i < 2; i++)
        {
            while (Math.abs(heading - target) > 180)
            {  //Continue while the robot direction is further than 180 degrees from the target
                if (heading < target)
                    smoothMovePower("leftTurn", 1, 0.5);
                else if (heading > target)
                    smoothMovePower("rightTurn", 1, 0.5);
                heading = cleanUp(360 - mrGyro.getHeading());  //Set variable to gyro reading
                telemetry.addData("heading", String.format("%03d", heading));
                telemetry.update();
            }

            while (Math.abs(heading - target) > 90)
            {  //Continue while the robot direction is further than 90 degrees from the target
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

            while (Math.abs(heading - target) > 45)
            {  //Continue while the robot direction is further than 45 degrees from the target
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

            while (Math.abs(heading - target) > 10)
            {  //Continue while the robot direction is further than 10 degrees from the target
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

    // Reads the ODS
    public void odsRead()
    {
        odsReadingRaw = ods.getRawLightDetected() / 5;                   //update raw value (This function now returns a value between 0 and 5 instead of 0 and 1 as seen in the video)
        odsReadingLinear = Math.pow(odsReadingRaw, 0.5);
    }

    // Reads the color sensor
    public void colorRead()
    {
        /*
        // check the status of the x button on either gamepad.
        bCurrState = gamepad1.x;
        // check for button state transitions.
        if (bCurrState && (bCurrState != bPrevState))  {

            // button is transitioning to a pressed state. So Toggle LED
            bLedOn = !bLedOn;
        }
        //bPrevState = bCurrState;
        */

        //colorSensor.enableLed(bLedOn);

        //telemetry.addData("LED", bLedOn ? "On" : "Off");
        //telemetry.addData("Red  ", colorSensor.red());
        //telemetry.addData("Green", colorSensor.green());
        //telemetry.addData("Blue ", colorSensor.blue());
        //telemetry.update();
    }

    // Reads the range sensor
    public void rangeRead()
    {
        range1Cache = RANGE1Reader.read(RANGE1_REG_START, RANGE1_READ_LENGTH);
    }

    // Reads the gyro
    public void gyroRead()
    {
        heading = 360 - mrGyro.getHeading();  // Reverse direction of heading to match the integrated value
        heading = cleanUp(heading);
    }

    // Finds heading
    public void findHeading()
    {
        temp = heading;
        // movePower("rightTurn", 0.25, 1.0);
        heading = cleanUp(360 - mrGyro.getHeading());  // Reverse direction of heading to match the integrated value.
        temp1 = heading;
        if (temp > temp1)
            temp1 += 360;
        temp2 = (double) (temp1 - temp);
        degreesPer10thSecond = temp2 / 10.0; // Saves variable for rest of program.
        degreesPerSecond = temp2;
    }

    public void knockBall (String team)
    {
        double timeStart = getRuntime();
        String ballColor = "";
        robot.BallArm.setPosition(robot.BALL_ARM_DOWN);

        // Code to sense color


        movePower("forward", 0, 0.25);

        if (team == ballColor)
        {
            smoothMovePower("rightTurn", .25, 0.25);
            robot.BallArm.setPosition(robot.BALL_ARM_UP);
            smoothMovePower("leftTurn", .25, 0.25);
        }
        else if (ballColor == "none")
        {
            motorStop();
            robot.BallArm.setPosition(robot.BALL_ARM_UP);
        }
        else
        {
            smoothMovePower("leftTurn", .25, 0.25);
            robot.BallArm.setPosition(robot.BALL_ARM_UP);
            smoothMovePower("rightTurn", .25, 0.25);
        }
        motorStop();
    }

    // Fixes any headings passed into it into the range of 0 - 360.
    public int cleanUp(int input)
    {
        if (input == 360)
            return 0;
        if (input < 0)
            return (input + 360);
        if (input > 360)
            return (input - 360);
        return input;
    }

    // Aligns robot with the cipher boxes by scrolling from right to left.
    public void correctXAxisBackWall()
    {
        turnAbsolute(180);
        double startTime = getRuntime();
        double time = getRuntime();
        int counter = 0;
        while (time - startTime < 2.0)
        {
            if (counter % 10 == 0)
                turnAbsolute(180);
            range1Cache = RANGE1Reader.read(RANGE1_REG_START, RANGE1_READ_LENGTH);
            if (range1Cache[0] < 10 * 2.54)
            {
                motorStop();
                movePower("right", 0.5, 0.01); // Find this number
                motorStop();
                return;
            }
            smoothMovePower("left", 1, 0.075);
            time = getRuntime();
            telemetry.addData("Heading", String.format("%03d", heading));
            telemetry.addData("Ultra Sonic", range1Cache[0] & 0xFF);
            telemetry.addData("Counter", counter);
            counter++;
        }
        motorStop();
    }

    // Moves robot close enough to back wall to begin correctXAxis.
    public void correctYAxisBackWall()
    {

        int counter = 0;
        turnAbsolute(180);
        movePower("backward", 1, 0.5);
        motorStop();
        while (true)
        {
            if (counter % 10 == 0)
                turnAbsolute(180);
            range1Cache = RANGE1Reader.read(RANGE1_REG_START, RANGE1_READ_LENGTH);
            if (range1Cache[0] < 17 * 2.54)
            {
                motorStop();
                turnAbsolute(180);
                return;
            }
            smoothMovePower("forward", 0.5, 0.05);
            telemetry.addData("Heading", String.format("%03d", heading));
            telemetry.addData("Ultra Sonic", range1Cache[0] & 0xFF);
            telemetry.addData("Counter", counter);
            counter++;
        }
    }

    // First cycle gyro initialization
    public void firstCycleFunc()
    {
        gyroRead();
        telemetry.addData("Gyroscope good", 0);

        findHeading();
        telemetry.addData("Heading good", 0);

        colorRead();
        telemetry.addData("Color sensor good", 0);

        rangeRead();
        telemetry.addData("Range sensor good", range1Cache);

        odsRead();
        telemetry.addData("Optical distance sensor good", odsReadingRaw);

        telemetry.clearAll();
        telemetry.addData("First run good", 0);
    }

    // Orients the robot to place blocks
    public void orient()
    {
        correctYAxisBackWall();
        correctXAxisBackWall();
        //correctZAxis();
    }

    // Code to run ONCE when the driver hits INIT
    @Override
    public void init() // Initializes the hardware variables.
    {
     /* Initialize the hardware variables.
     * The init() method of the hardware class does all the work here
     */
        // get a reference to our ColorSensor object.
        //colorSensor = hardwareMap.colorSensor.get("sensor_color");

        // Set the LED in the beginning
        robot.init(hardwareMap);

        sensorGyro = hardwareMap.gyroSensor.get("gyro");  // Point to the gyro in the configuration file
        mrGyro = (ModernRoboticsI2cGyro)sensorGyro;      // ModernRoboticsI2cGyro allows us to .getIntegratedZValue()
        mrGyro.calibrate();  // Calibrate the sensor so it knows where 0 is and what still is. DO NOT MOVE SENSOR WHILE BLUE LIGHT IS SOLID

        RANGE1 = hardwareMap.i2cDevice.get("RANGE1");
        RANGE1Reader = new I2cDeviceSynchImpl(RANGE1, RANGE1ADDRESS, false);
        RANGE1Reader.engage();

        // get a reference to our ColorSensor object.
        //ColorSensor colorSensor = hardwareMap.get(ColorSensor.class, "sensor_color");

        telemetry.addData("Status", "Initialized");
        telemetry.update();
    }

    @Override
    public void init_loop()
    {
        telemetry.addData("Say", "Hello Driver");
        telemetry.update();
    }

    // Code to run ONCE when the driver hits PLAY
    @Override
    public void start()
    {
        runtime.reset();

        while (mrGyro.isCalibrating()) // Ensure calibration is complete (usually 2 seconds)
        {
        }
        // Set the LED in the beginning
        //colorSensor.enableLed(bLedOn);
    }

    // Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
    @Override
    public void loop()
    {

        if (firstCycle)
        {
            firstCycleFunc();
            firstCycle = false;
        }

        // Run wheels in tank mode (note: The joystick goes negative when pushed forwards, so negate it)
        frontLeft = (gamepad1.left_stick_x - gamepad1.left_stick_y - gamepad1.right_stick_x) / 2 * driveSpeed; // Front right
        frontRight = (gamepad1.left_stick_x + gamepad1.left_stick_y - gamepad1.right_stick_x) / 2 * driveSpeed; // Front left
        backLeft = (-gamepad1.left_stick_x - gamepad1.left_stick_y - gamepad1.right_stick_x) / 2 * driveSpeed; // Back right
        backRight = (-gamepad1.left_stick_x + gamepad1.left_stick_y - gamepad1.right_stick_x) / 2 * driveSpeed; // Back left
        Lift = gamepad2.left_stick_y * liftSpeed;

        if (gamepad1.a)
            target = cleanUp(target + 15);
        if (gamepad1.b)
            target = cleanUp(target - 15);
        if (gamepad1.x)
            colorRead();
        if (gamepad1.y)
            turnAbsolute(target);

        /* drivespeed stuff
        driveSpeed = 1;
        if (gamepad1.right_bumper)
            driveSpeed = 0.5;
        if (gamepad1.left_bumper)
            driveSpeed = 0.25;
        */

        if (gamepad1.right_bumper)
            frontPosition = robot.FRONT_OUT;
        else if (gamepad1.left_bumper)
            frontPosition = robot.FRONT_IN;

        if (gamepad1.dpad_up)
            movePower("forward", 1, 0.5);
        if (gamepad1.dpad_down)
            correctXAxisBackWall();
        if (gamepad1.dpad_left)
            knockBall("red");
        if (gamepad1.dpad_right)
            orient();

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
        if (gamepad2.dpad_left)
            knockBall("red");
        if (gamepad2.dpad_right)
            knockBall("blue");

        robot.FL_drive.setPower(frontLeft);
        robot.FR_drive.setPower(frontRight);
        robot.BL_drive.setPower(backLeft);
        robot.BR_drive.setPower(backRight);
        robot.Lift.setPower(Lift);

        // Move all servos to new position.
        leftPosition = Range.clip(leftPosition, robot.LEFT_MIN_RANGE, robot.LEFT_MAX_RANGE);
        robot.Left.setPosition(leftPosition);
        rightPosition = Range.clip(rightPosition, robot.RIGHT_MIN_RANGE, robot.RIGHT_MAX_RANGE);
        robot.Right.setPosition(rightPosition);
        robot.FrontBoi.setPosition(frontPosition);
        robot.BallArm.setPosition(ballPosition);
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
            telemetry.addData("DegreesPer10thSecond", "%.2f", degreesPer10thSecond);

            telemetry.addData("5. heading", String.format("%03d", heading));  // Display variables to Driver Station Screen
            telemetry.addData("6. target", String.format("%03d", target));

            telemetry.addData("7. ODS Raw", odsReadingRaw);

            telemetry.addData("8. Ultra Sonic", range1Cache[0] & 0xFF);
            telemetry.addData("9. range ODS", range1Cache[1] & 0xFF);

            telemetry.addData("10. ODS Raw", odsReadingRaw);
            telemetry.addData("11. ODS linear", odsReadingLinear);

            telemetry.update(); // Limited to 100x per second
        }
    }

