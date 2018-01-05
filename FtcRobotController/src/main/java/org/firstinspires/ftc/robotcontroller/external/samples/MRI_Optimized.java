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
    private double frontLeft;
    private double frontRight;
    private double backLeft;
    private double backRight;
    private double Lift;
    boolean red, blue;
    private boolean firstCycle = true;

    private ElapsedTime runtime = new ElapsedTime();
    private ElapsedTime runtime1 = new ElapsedTime();

    //private byte[] colorAcache;
    private byte[] range1Cache; // The read will return an array of bytes. They are stored in this variable

    //ModernRoboticsI2cColorSensor colorA;
    //I2cDeviceSynch colorAreader;

    //I2cDeviceSynch RANGE1Reader;

    OpticalDistanceSensor ods = robot.ods;

    //double odsReadingRaw;
    //static double odsReadingLinear;

    //sensor value between 0 and 1023
    //int state = 0;

    boolean buttonState = false;  // Tracks the last known state of the gamepad 1 x button
    private boolean LEDState = true;     // Tracks the mode of the color sensor; Active = true, Passive = false

    //int zAccumulated;  // Total rotation left/right
    int heading;       // Heading left/right. Integer between 0 and 359
    private int target = 0;  // Desired angle to turn to
    private int temp, temp1;
    double temp2;
    double degreesPer10thSecond = 0;
    double degreesPerSecond = 0;
    double secondsPerDegree = 0;

    private GyroSensor sensorGyro;  // General Gyro Sensor allows us to point to the sensor in the configuration file.
    private ModernRoboticsI2cGyro mrGyro;  // ModernRoboticsI2cGyro allows us to .getIntegratedZValue()
/*
    private ColorSensor colorA1;
    private ModernRoboticsI2cColorSensor colorA;
    private I2cDeviceSynch colorAreader;
*/

    private I2cDevice RANGE1;
    private I2cDeviceSynch RANGE1Reader;

    private OpticalDistanceSensor ods1;

    private double odsReadingRaw;
    static double odsReadingLinear;

    //sensor value between 0 and 1023
    int raw1;
    int state = 0;
    int count = 0;

    private I2cAddr RANGE1ADDRESS = new I2cAddr(0x14); //Default I2C address for MR Range (7-bit)
    private static final int RANGE1_REG_START = 0x04; //Register to start reading
    private static final int RANGE1_READ_LENGTH = 2; //Number of byte to read

    // Sets power of all drive motors to zero.
    public void stop()
    {
        robot.FL_drive.setPower(0);
        robot.FR_drive.setPower(0);
        robot.BL_drive.setPower(0);
        robot.BR_drive.setPower(0);
    }

    // Moves or turns robot in specified direction, power, and duration. Then stops.
    private void movePower(String movement, double power, double duration)
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
        stop();
    }

    // Moves or turns robot in specified direction, power, and duration. Does not stop itself.
    private void smoothMovePower(String movement, double power, double duration)
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
    private void changeAngle(int degreesFromCurrentAngle, double degreesPer10thSecond)
    {
        if (degreesFromCurrentAngle < 0)
        {
            degreesFromCurrentAngle = Math.abs(degreesFromCurrentAngle);
            movePower("leftTurn", 0.25, secondsPerDegree * degreesFromCurrentAngle);
        }
        else
        {
            movePower("rightTurn", 0.25, secondsPerDegree * degreesFromCurrentAngle);
        }
        stop();
    }

    // Turns a number of degrees compared to where the robot was when the program started. Positive numbers turn left.
    private void turnAbsolute(int target)
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
        stop();
        movePower("forward", 0, 0.25);
        stop();
        heading = cleanUp(360 - mrGyro.getHeading());  //Set variable to gyro reading
        int degreesError = heading - target; // If positive left, if negative right
        if (heading > target) // If true, we need to turn left
            movePower("leftTurn", 1, degreesError * secondsPerDegree);
        else
            movePower("rightTurn", 1, degreesError * secondsPerDegree);
        stop();
    }

    //Reads the ODS
    public void odsRead ()
    {

        odsReadingRaw = ods.getRawLightDetected() / 5;                   //update raw value (This function now returns a value between 0 and 5 instead of 0 and 1 as seen in the video)
        odsReadingLinear = Math.pow(odsReadingRaw, 0.5);

    }

    public void knockBall (String team)
    {
    /*
        double timeStart = getRuntime();
        String ballColor;
        robot.BallArm.setPosition(robot.BALL_ARM_DOWN);

        colorAcache = colorAreader.read(0x04, 1);

        switch(colorAcache[0])
        {
            case 10:
                ballColor = "red";
                break;
            case 3:
                ballColor = "blue";
                break;
            default:
                ballColor = "none";
                break;
        }

        movePower("forward", 0, 0.25);

        if (team == ballColor)
        {
            smoothMovePower("rightTurn", .25, 0.25);
            robot.BallArm.setPosition(robot.BALL_ARM_UP);
            smoothMovePower("leftTurn", .25, 0.25);
        }
        else if (ballColor == "none")
        {
            stop();
            robot.BallArm.setPosition(robot.BALL_ARM_UP);
        }
        else
        {
            smoothMovePower("leftTurn", .25, 0.25);
            robot.BallArm.setPosition(robot.BALL_ARM_UP);
            smoothMovePower("rightTurn", .25, 0.25);
        }
        stop();
    */
    }

    // Fixes any headings passed into it into the range of 0 - 360.
    private int cleanUp(int input)
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
    private void correctXAxisBackWall()
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
                stop();
                movePower("right", 0.5, 0.01); // Find this number
                stop();
                return;
            }
            smoothMovePower("left", 1, 0.075);
            time = getRuntime();
            telemetry.addData("Heading", String.format("%03d", heading));
            telemetry.addData("Ultra Sonic", range1Cache[0] & 0xFF);
            telemetry.addData("Counter", counter);
            counter++;
        }
        stop();
    }

    // Moves robot close enough to back wall to begin correctXAxis.
    private void correctYAxisBackWall()
    {

        int counter = 0;
        turnAbsolute(180);
        movePower("backward", 1, 0.5);
        stop();
        while (true)
        {
            if (counter % 10 == 0)
                turnAbsolute(180);
            range1Cache = RANGE1Reader.read(RANGE1_REG_START, RANGE1_READ_LENGTH);
            if (range1Cache[0] < 17 * 2.54)
            {
                stop();
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
    private void firstCycleFunc()
    {
        // Gyroscope
        heading = 360 - mrGyro.getHeading();  // Reverse direction of heading to match the integrated value
        heading = cleanUp(heading);

        temp = heading;
        movePower("rightTurn", 0.25, 1.0);
        heading = cleanUp(360 - mrGyro.getHeading());  // Reverse direction of heading to match the integrated value.
        temp1 = heading;
        if (temp > temp1)
            temp1 += 360;
        temp2 = (double) (temp1 - temp);
        degreesPer10thSecond = temp2 / 10.0; // Saves variable for rest of program.
        degreesPerSecond = temp2;
        secondsPerDegree = 1 / temp2;

        // Color Sensor
        //colorAcache = colorAreader.read(0x04, 1);

        // Range Sensor
        range1Cache = RANGE1Reader.read(RANGE1_REG_START, RANGE1_READ_LENGTH);

        //ODS Sensor
        odsReadingRaw = ods.getRawLightDetected();
    }

    // Orients the robot to place blocks
    private void orient()
    {
        correctYAxisBackWall();
        correctXAxisBackWall();
        //correctZAxis();
    }

    private void sensorUpdate()
    {
        range1Cache = RANGE1Reader.read(RANGE1_REG_START, RANGE1_READ_LENGTH);
        odsReadingRaw = ods1.getRawLightDetected();
        //colorAcache = colorAreader.read(0x04, 1);
        heading = 360 - mrGyro.getHeading();
        heading = cleanUp(heading);

        // Display values
        //telemetry.addData("1. #A", colorAcache[0] & 0xFF);

        //telemetry.addData("2. A", colorAreader.getI2cAddress().get8Bit());

        telemetry.addData("3. heading", String.format("%03d", heading));  // Display variables to Driver Station Screen

        telemetry.addData("4. ODS Raw", odsReadingRaw);

        telemetry.addData("5. Ultra Sonic", range1Cache[0] & 0xFF);
        telemetry.addData("6. range ODS", range1Cache[1] & 0xFF);

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

        telemetry.update(); // Limited to 100x per second
    }

    // Code to run ONCE when the driver hits INIT
    @Override
    public void init() // Initializes the hardware variables.
    {
     /* Initialize the hardware variables.
     * The init() method of the hardware class does all the work here
     */
        robot.init(hardwareMap);

        telemetry.addData("Status", "Initialized");

        //the below lines set up the configuration file
        //colorA1 = hardwareMap.colorSensor.get("colorA");
        //colorA = (ModernRoboticsI2cColorSensor) colorA1;
        //colorAreader = new I2cDeviceSynchImpl((I2cDevice) colorA, I2cAddr.create8bit(0x3a), false);
        //colorAreader.engage();

        sensorGyro = hardwareMap.gyroSensor.get("gyro");  // Point to the gyro in the configuration file
        mrGyro = (ModernRoboticsI2cGyro)sensorGyro;      // ModernRoboticsI2cGyro allows us to .getIntegratedZValue()
        mrGyro.calibrate();  // Calibrate the sensor so it knows where 0 is and what still is. DO NOT MOVE SENSOR WHILE BLUE LIGHT IS SOLID

        RANGE1 = hardwareMap.i2cDevice.get("RANGE1");
        RANGE1Reader = new I2cDeviceSynchImpl(RANGE1, RANGE1ADDRESS, false);
        RANGE1Reader.engage();

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

        while (mrGyro.isCalibrating()) // Ensure calibration is complete (usually 2 seconds)
        {
        }
/*
        if(LEDState)
        {
            colorAreader.write8(3, 0);    //Set the mode of the color sensor using LEDState
        }
        else
        {
            colorAreader.write8(3, 1);    //Set the mode of the color sensor using LEDState
        }
        //Active - For measuring reflected light. Cancels out ambient light
        //Passive - For measuring ambient light, eg. the FTC Color Beacon
        */
    }

    public void telemetryList()
    {
        telemetry.addData("Status", "Running: " + runtime1.toString());

        // Send telemetry message to signify robot running;
        telemetry.addData("Left", "%.2f", leftPosition);
        telemetry.addData("Right", "%.2f", rightPosition);
        telemetry.addData("Front", "%.2f", frontPosition);
        telemetry.addData("Ball", "%.2f", ballPosition);

        /*
        telemetry.addData("frontLeft", "%.2f", frontLeft);
        telemetry.addData("frontRight", "%.2f", frontRight);
        telemetry.addData("backLeft", "%.2f", backLeft);
        telemetry.addData("backRight", "%.2f", backRight);
        */

        telemetry.addData("Lift", "%.2f", Lift);
        telemetry.addData("DegreesPer10thSecond", "%.2f", degreesPer10thSecond);

        // Display values
        //telemetry.addData("1. #A", colorAcache[0] & 0xFF);
        //telemetry.addData("2. #C", colorCcache[0] & 0xFF);

        //telemetry.addData("3. A", colorAreader.getI2cAddress().get8Bit());
        //telemetry.addData("4. C", colorCreader.getI2cAddress().get8Bit());

        telemetry.addData("5. heading", String.format("%03d", heading));  // Display variables to Driver Station Screen
        telemetry.addData("6. target", String.format("%03d", target));

        telemetry.addData("7. ODS Raw", odsReadingRaw);

        telemetry.addData("8. Ultra Sonic", range1Cache[0] & 0xFF);
        telemetry.addData("9. range ODS", range1Cache[1] & 0xFF);

        telemetry.update(); // Limited to 100x per second


    }


    // Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
    @Override
    public void loop()
    {
        telemetry.addData("Status", "Running: " + runtime1.toString());

        if(firstCycle)
        {
            firstCycleFunc();
            firstCycle = false;
        }

        // Run wheels in tank mode (note: The joystick goes negative when pushed forwards, so negate it)
        frontLeft  = ( gamepad1.left_stick_x - gamepad1.left_stick_y - gamepad1.right_stick_x)/2 * driveSpeed; // Front right
        frontRight = ( gamepad1.left_stick_x + gamepad1.left_stick_y - gamepad1.right_stick_x)/2 * driveSpeed; // Front left
        backLeft   = (-gamepad1.left_stick_x - gamepad1.left_stick_y - gamepad1.right_stick_x)/2 * driveSpeed; // Back right
        backRight  = (-gamepad1.left_stick_x + gamepad1.left_stick_y - gamepad1.right_stick_x)/2 * driveSpeed; // Back left
        Lift = gamepad2.left_stick_y * liftSpeed;

        if (gamepad1.a)
            target = cleanUp(target + 15);
        if (gamepad1.b)
            target = cleanUp(target - 15);
        if (gamepad1.y)
            turnAbsolute(target);

        /*
        drivespeed stuff
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
            sensorUpdate();
        if (gamepad1.dpad_right)
            orient();

        // Controller 2
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
            //knockBall("red");
        if (gamepad2.dpad_right)
            //knockBall("blue");

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

        // Send telemetry message to signify robot running;
        telemetry.update(); // Limited to 100x per second
    }
}