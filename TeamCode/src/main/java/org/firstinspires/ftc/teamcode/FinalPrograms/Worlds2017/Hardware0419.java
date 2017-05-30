package org.firstinspires.ftc.teamcode.FinalPrograms.Worlds2017;


import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.FinalPrograms.WAGSAdafruitIMU;

import android.graphics.Color;

import org.lasarobotics.vision.ftc.resq.Beacon;
import org.lasarobotics.vision.opmode.LinearVisionOpMode;
import org.lasarobotics.vision.opmode.extensions.CameraControlExtension;
import org.lasarobotics.vision.util.ScreenOrientation;


/**
 * This is NOT an opmode.
 * <p/>
 * This class can be used to define all the specific hardware for a single robot.
 * In this case that robot is a Pushbot.
 * See PushbotTeleopTank_Iterative and others classes starting with "Pushbot" for usage examples.
 * <p/>
 * This hardware class assumes the following device names have been configured on the robot:
 * Note:  All names are lower case and some have single spaces between words.
 */

public class Hardware0419 {
    /* local OpMode members. */
    HardwareMap hwMap = null;
    private ElapsedTime period = new ElapsedTime();
    private ElapsedTime runtime = new ElapsedTime();



    /* Public OpMode members. */

    //Motor Controller Left:
    //Motor Controller Right:
    /* Drive motors */
    public DcMotor leftMotorFront = null;
    public DcMotor leftMotorBack = null;
    public DcMotor rightMotorFront = null;
    public DcMotor rightMotorBack = null;

    //Motor Controller:
    /* Flywheels */
    public DcMotor ballShooter = null;
    public DcMotor ballShooter2 = null;

    //Motor Controller:
    /* Collection & Lift */
    public DcMotor ballPick = null;
    public DcMotor lift = null;

    //Servo Controller:
    /* Button pushers */
    public CRServo buttonPusherR = null;
    public CRServo buttonPusherL = null;

    /* Shooter latch & Lift release & backup button pusher*/
    public Servo ballLatch = null;
    public Servo liftLatch = null;
    public Servo button = null;

    //Core Interface Device Module:

    WAGSAdafruitIMU imu = null;
    ColorSensor lineTracker = null;
    DigitalChannel hallSwitch = null;

    //Positions
    static final double BALL_LATCH_OPEN = 0.0;
    static final double BALL_LATCH_CLOSED = 0.7;
    static final double LIFT_LATCH_OPEN = 0.7;
    static final double LIFT_LATCH_CLOSED = 0.72;


    final double MAX_POS = 1.0;
    final double MIN_POS = 0;


    /* Motor, Servo, and Sensor Names */
    public static final String lfName = "left_front";
    public static final String lbName = "left_back";
    public static final String rfName = "right_front";
    public static final String rbName = "right_back";

    public static final String bsName = "ball_shoot";
    public static final String bs2Name = "ball_shoot2";
    public static final String bpickName = "ballPick";
    public static final String liftName = "lift";

    public static final String bpRName = "buttonPusherR";
    public static final String bpLName = "buttonPusherL";
    public static final String blName = "ballLatch";
    public static final String llName = "liftLatch";
    public static final String buName = "button";

    public static final String colorSensorName = "color";
    public static final String hallSwitchName = "hallSwitch";
    public static final String imuName = "imu";
    float hsvValues[] = {0F, 0F, 0F};


    /* Other data */

    //Encoder Data
    static final double COUNTS_PER_MOTOR_REV = 1440;    // eg: TETRIX Motor Encoder
    static final double DRIVE_GEAR_REDUCTION = 2.0;     // This is < 1.0 if geared UP
    static final double WHEEL_DIAMETER_INCHES = 4.0;     // For figuring circumference
    static final double COUNTS_PER_INCH = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * 3.1415);

    //Movement Data
    static final double DRIVE_SPEED = 0.6;
    static final double TURN_SPEED = 0.5;

    //Shooter data
    final double SHOOTER_CALIB_VAL = 0.5;
    final double TARGET_SPEED_HIGH = 234.27;
    final double TARGET_SPEED_LOW = 368.07;
    final double TIME_VALUE = 0;
    final double CIRCLE_DIST = 4 * Math.PI / 6;
    final double MAX_POWER = 1.0;
    final double SHOOT_HIGH = 0.55;
    final double SHOOT_LOW = 0.35;

    int magnetsCross = 0;
    boolean newCondition = false;
    double timeValue = 0;
    int magCross = 0;


    /* Constructor */
    public Hardware0419() {

    }

    /* Initialize standard Hardware interfaces */
    public void init(HardwareMap ahwMap, boolean auton) {
        // Save reference to Hardware map
        hwMap = ahwMap;

        // Define and Initialize Motors
        leftMotorFront = hwMap.dcMotor.get(lfName);
        leftMotorBack = hwMap.dcMotor.get(lbName);
        rightMotorFront = hwMap.dcMotor.get(rfName);
        rightMotorBack = hwMap.dcMotor.get(rbName);
        ballShooter = hwMap.dcMotor.get(bsName);
        ballShooter2 = hwMap.dcMotor.get(bs2Name);
        ballPick = hwMap.dcMotor.get(bpickName);
        lift = hwMap.dcMotor.get(liftName);


        buttonPusherR = hwMap.crservo.get(bpRName);
        buttonPusherL = hwMap.crservo.get(bpLName);
        ballLatch = hwMap.servo.get(blName);
        liftLatch = hwMap.servo.get(llName);
        button = hwMap.servo.get(buName);

        lineTracker = hwMap.colorSensor.get(colorSensorName);
        hallSwitch = hwMap.digitalChannel.get(hallSwitchName);
        imu = new WAGSAdafruitIMU(imuName, hwMap);

        //Set motor directions
        leftMotorFront.setDirection(DcMotor.Direction.REVERSE);
        leftMotorBack.setDirection(DcMotor.Direction.REVERSE);
        rightMotorFront.setDirection(DcMotor.Direction.FORWARD);
        rightMotorBack.setDirection(DcMotor.Direction.FORWARD);

        buttonPusherL.setDirection(CRServo.Direction.REVERSE);
        buttonPusherR.setDirection(CRServo.Direction.FORWARD);

        ballShooter.setDirection(DcMotorSimple.Direction.REVERSE);
        ballShooter2.setDirection(DcMotorSimple.Direction.FORWARD);
        ballPick.setDirection(DcMotorSimple.Direction.FORWARD);
        lift.setDirection(DcMotorSimple.Direction.FORWARD);

        leftMotorBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftMotorFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightMotorBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightMotorFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);


        ballShooter.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        ballShooter2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        ballPick.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        lift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        //Add encoders for autonomous
        if (auton) {
            leftMotorBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            leftMotorFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            rightMotorBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            rightMotorFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            ballShooter.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            ballShooter2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            ballShooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            ballShooter2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            leftMotorBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            leftMotorFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            rightMotorBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            rightMotorFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        }

    }

    public void initialize() {
        ballLatch.setPosition(BALL_LATCH_CLOSED); //closed position
        liftLatch.setPosition(LIFT_LATCH_CLOSED); //closed position
        lineTracker.enableLed(false);
    }

    public void resetCamera() {
        LinearVisionOpMode.beacon.setAnalysisMethod(Beacon.AnalysisMethod.FAST);
        LinearVisionOpMode.beacon.setColorToleranceRed(0);
        LinearVisionOpMode.beacon.setColorToleranceBlue(0);

        LinearVisionOpMode.rotation.setIsUsingSecondaryCamera(false);
        LinearVisionOpMode.rotation.disableAutoRotate();
        LinearVisionOpMode.rotation.setActivityOrientationFixed(ScreenOrientation.LANDSCAPE);

        LinearVisionOpMode.cameraControl.setColorTemperature(CameraControlExtension.ColorTemperature.AUTO);
        LinearVisionOpMode.cameraControl.setAutoExposureCompensation();
    }

    public void pause() {
        // Set all motors to zero power
        pauseDrive();

        ballShooter.setPower(0);
        ballShooter2.setPower(0);

        ballPick.setPower(0);
        lift.setPower(0);
    }

    public void pauseDrive() {
        leftMotorFront.setPower(0);
        leftMotorBack.setPower(0);
        rightMotorFront.setPower(0);
        rightMotorBack.setPower(0);
    }

    public void turn(double turnAngle, double speed, LinearVisionOpMode opMode) {
        double newHeading;
        double[] angles = imu.getAngles();
        double currYaw = angles[0];
        if (turnAngle > 0) { //positive param, right turn
            newHeading = currYaw + turnAngle;
            while (opMode.opModeIsActive() && currYaw < newHeading) {
                leftMotorBack.setPower(speed);
                leftMotorFront.setPower(speed);
                rightMotorBack.setPower(-speed);
                rightMotorFront.setPower(-speed);

            }
        } else { //negative param, left turn
            newHeading = currYaw - turnAngle;
            while (opMode.opModeIsActive() && currYaw < newHeading) {
                leftMotorBack.setPower(-speed);
                leftMotorFront.setPower(-speed);
                rightMotorBack.setPower(speed);
                rightMotorFront.setPower(speed);
            }
        }
        pauseDrive();
    }


    public void encoderDrive(double speed,
                             double leftInches, double rightInches,
                             double timeoutS, LinearVisionOpMode opMode) {
        int newLeftTargetF;
        int newLeftTargetB;
        int newRightTargetF;
        int newRightTargetB;

        // Ensure that the opmode is still active

        while (opMode.opModeIsActive()) {

            // Determine new target position, and pass to motor controller
            newLeftTargetF = leftMotorFront.getCurrentPosition() + (int) (leftInches * COUNTS_PER_INCH);
            newLeftTargetB = leftMotorBack.getCurrentPosition() + (int) (leftInches * COUNTS_PER_INCH);
            newRightTargetF = rightMotorFront.getCurrentPosition() + (int) (rightInches * COUNTS_PER_INCH);
            newRightTargetB = rightMotorBack.getCurrentPosition() + (int) (rightInches * COUNTS_PER_INCH);

            leftMotorBack.setTargetPosition(newLeftTargetB);
            leftMotorFront.setTargetPosition(newLeftTargetF);
            rightMotorBack.setTargetPosition(newRightTargetB);
            rightMotorFront.setTargetPosition(newRightTargetF);

            // Turn On RUN_TO_POSITION
            leftMotorBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            leftMotorFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rightMotorBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rightMotorFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // reset the timeout time and start motion.
            runtime.reset();
            leftMotorBack.setPower(Math.abs(speed));
            leftMotorFront.setPower(Math.abs(speed));
            rightMotorBack.setPower(Math.abs(speed));
            rightMotorFront.setPower(Math.abs(speed));


            // keep looping while we are still active, and there is time left, and both motors are running.
            while (opMode.opModeIsActive() &&
                    (runtime.seconds() < timeoutS) &&
                    (leftMotorBack.isBusy() && leftMotorFront.isBusy() &&
                            rightMotorBack.isBusy() && leftMotorFront.isBusy())) {

                // Display it for the driver.
                opMode.telemetry.addData("Path1", "Running to %7d :%7d", newLeftTargetB, newLeftTargetF,
                        newRightTargetB, newRightTargetF);
                opMode.telemetry.addData("Path2", "Running at %7d :%7d",
                        leftMotorBack.getCurrentPosition(),
                        leftMotorFront.getCurrentPosition(),
                        rightMotorBack.getCurrentPosition(),
                        rightMotorFront.getCurrentPosition());
                opMode.telemetry.update();
            }

            // Stop all motion;
           pauseDrive();

            // Turn off RUN_TO_POSITION
            leftMotorBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            leftMotorFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            rightMotorBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            rightMotorFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
    }


    public void shootParticles() {
            ballShooter.setPower(1.0 * SHOOTER_CALIB_VAL); //shoot
            ballShooter2.setPower(1.0 * SHOOTER_CALIB_VAL);
            ballPick.setPower(1.0);
    }

    public void shootParticlesHallSwitch(boolean highSpeed, LinearVisionOpMode opMode) {
        if(!highSpeed) {
            if(hallSwitch.getState() == false) {
                magCross++;
                timeValue = runtime.time();
                runtime.reset();
                opMode.telemetry.addData("MagnetCrossed: ", true);

            }

            if(CIRCLE_DIST / timeValue < SHOOT_LOW) { //run at 0.35
                double newPwr = ballShooter.getPower();
                ballShooter.setPower(newPwr++);
                ballShooter2.setPower(newPwr++);
                opMode.telemetry.addData("Speed: ", "adjust");

            }

            else {
                ballShooter.setPower(SHOOT_LOW);
                ballShooter2.setPower(SHOOT_LOW);
            }

            opMode.telemetry.addData("# times magnets", magCross);

        }

        else if (highSpeed) {
            runtime.reset();
            if(hallSwitch.getState() == false) {
                magCross++;
                timeValue = runtime.time();
                runtime.reset();
                opMode.telemetry.addData("MagnetCrossed: ", true);

            }

            if(CIRCLE_DIST / timeValue < SHOOT_HIGH) { //run at 0.55
                double newPwr = ballShooter.getPower();
                ballShooter.setPower(newPwr++);
                ballShooter2.setPower(newPwr++);
                opMode.telemetry.addData("Speed: ", "adjust");

            }

            else {
                ballShooter.setPower(SHOOT_HIGH);
                ballShooter2.setPower(SHOOT_HIGH);
            }

            opMode.telemetry.addData("# times magnets", magCross);

        }

        else {
            ballShooter.setPower(0);
            ballShooter2.setPower(0);

        }
    }

    /*
     * Drives forward from starting position and shoots two particles
     * Servos & motors move as particles are shot
     */
    public void driveShootParticles(double firstBallSec, double secondBallSec, LinearVisionOpMode opMode) {
        runtime.reset();
        while (opMode.opModeIsActive() && (runtime.seconds() < 1)) {
            pauseDrive();
        }

        encoderDrive(DRIVE_SPEED, 7, 7, 3.0, opMode); //move forward

        pauseDrive();

        ballLatch.setPosition(BALL_LATCH_OPEN); //open

        runtime.reset();
        while (opMode.opModeIsActive() && (runtime.seconds() < firstBallSec)) {
            shootParticlesHallSwitch(true, opMode);
        }

        ballPick.setPower(1.0);


        runtime.reset();
        while (opMode.opModeIsActive() && (runtime.seconds() < secondBallSec)) {
            shootParticlesHallSwitch(true, opMode);
        }

        runtime.reset();
        while (opMode.opModeIsActive() && (runtime.seconds() < 0.1)) {
            pauseDrive();
        }

        ballLatch.setPosition(BALL_LATCH_OPEN); //open

        pauseDrive();

        encoderDrive(DRIVE_SPEED, 1, 1, 0.5, opMode); //move forward until 10

        ballLatch.setPosition(BALL_LATCH_CLOSED);

        pause();
    }

    public void encoderDriveStrafe(boolean right, double speed,
                                   double leftInches, double rightInches,
                                   double timeoutS, LinearVisionOpMode opMode) {
        int newLeftTargetF;
        int newLeftTargetB;
        int newRightTargetF;
        int newRightTargetB;

        // Ensure that the opmode is still active
        if (opMode.opModeIsActive()) {

            if (right) { //strafe right
                // Determine new target position, and pass to motor controller
                newLeftTargetF = leftMotorFront.getCurrentPosition() + (int) (leftInches * COUNTS_PER_INCH);
                newLeftTargetB = leftMotorBack.getCurrentPosition() + (int) (leftInches * COUNTS_PER_INCH);
                newRightTargetF = rightMotorFront.getCurrentPosition() + (int) (rightInches * COUNTS_PER_INCH);
                newRightTargetB = rightMotorBack.getCurrentPosition() + (int) (rightInches * COUNTS_PER_INCH);

                leftMotorBack.setTargetPosition(newLeftTargetB);
                leftMotorFront.setTargetPosition(newLeftTargetF);
                rightMotorBack.setTargetPosition(newRightTargetB);
                rightMotorFront.setTargetPosition(newRightTargetF);

                // Turn On RUN_TO_POSITION
                leftMotorBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                leftMotorFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                rightMotorBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                rightMotorFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                // reset the timeout time and start motion.
                runtime.reset();
                leftMotorBack.setPower(Math.abs(speed));
                leftMotorFront.setPower(Math.abs(-speed));
                rightMotorBack.setPower(Math.abs(-speed));
                rightMotorFront.setPower(Math.abs(speed));


                // keep looping while we are still active, and there is time left, and both motors are running.
                while (opMode.opModeIsActive() &&
                        (runtime.seconds() < timeoutS) &&
                        (leftMotorBack.isBusy() && leftMotorFront.isBusy() &&
                                rightMotorBack.isBusy() && leftMotorFront.isBusy())) {

                    // Display it for the driver.
                    opMode.telemetry.addData("Path1", "Running to %7d :%7d", newLeftTargetB, newLeftTargetF,
                            newRightTargetB, newRightTargetF);
                    opMode.telemetry.addData("Path2", "Running at %7d :%7d",
                            leftMotorBack.getCurrentPosition(),
                            leftMotorFront.getCurrentPosition(),
                            rightMotorBack.getCurrentPosition(),
                            rightMotorFront.getCurrentPosition());
                    opMode.telemetry.update();
                }

                // Stop all motion;
                pauseDrive();
            } else if (!right) { //strafe left
                // Determine new target position, and pass to motor controller
                newLeftTargetF = leftMotorFront.getCurrentPosition() + (int) (leftInches * COUNTS_PER_INCH);
                newLeftTargetB = leftMotorBack.getCurrentPosition() + (int) (leftInches * COUNTS_PER_INCH);
                newRightTargetF = rightMotorFront.getCurrentPosition() + (int) (rightInches * COUNTS_PER_INCH);
                newRightTargetB = rightMotorBack.getCurrentPosition() + (int) (rightInches * COUNTS_PER_INCH);

                leftMotorBack.setTargetPosition(newLeftTargetB);
                leftMotorFront.setTargetPosition(newLeftTargetF);
                rightMotorBack.setTargetPosition(newRightTargetB);
                rightMotorFront.setTargetPosition(newRightTargetF);

                // Turn On RUN_TO_POSITION
                leftMotorBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                leftMotorFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                rightMotorBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                rightMotorFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                // reset the timeout time and start motion.
                runtime.reset();
                leftMotorBack.setPower(Math.abs(-speed));
                leftMotorFront.setPower(Math.abs(speed));
                rightMotorBack.setPower(Math.abs(speed));
                rightMotorFront.setPower(Math.abs(-speed));


                // keep looping while we are still active, and there is time left, and both motors are running.
                while (opMode.opModeIsActive() &&
                        (runtime.seconds() < timeoutS) &&
                        (leftMotorBack.isBusy() && leftMotorFront.isBusy() &&
                                rightMotorBack.isBusy() && leftMotorFront.isBusy())) {

                    // Display it for the driver.
                    opMode.telemetry.addData("Path1", "Running to %7d :%7d", newLeftTargetB, newLeftTargetF,
                            newRightTargetB, newRightTargetF);
                    opMode.telemetry.addData("Path2", "Running at %7d :%7d",
                            leftMotorBack.getCurrentPosition(),
                            leftMotorFront.getCurrentPosition(),
                            rightMotorBack.getCurrentPosition(),
                            rightMotorFront.getCurrentPosition());
                    opMode.telemetry.update();
                }

                // Stop all motion;
                pauseDrive();

            }


            // Turn off RUN_TO_POSITION
            leftMotorBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            leftMotorFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            rightMotorBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            rightMotorFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            pauseDrive();
        }
    }

    public void searchWhiteLine(double whiteValue, double driveSpeed, LinearVisionOpMode opMode) {

        lineTracker.enableLed(true);


        double redColor = lineTracker.red();

        while (opMode.opModeIsActive() && redColor < whiteValue) { //see white
            lineTracker.enableLed(true);
            Color.RGBToHSV(lineTracker.red() * 8, lineTracker.green() * 8, lineTracker.blue() * 8, hsvValues);
            redColor = lineTracker.red();

            leftMotorBack.setPower(0.2);
            leftMotorFront.setPower(driveSpeed);
            rightMotorBack.setPower(driveSpeed);
            rightMotorFront.setPower(driveSpeed);
            opMode.telemetry.addData("red", redColor);
            opMode.telemetry.update();
        }

        opMode.telemetry.addData("Line", "Detected");
        opMode.telemetry.update();

        pauseDrive();


    }

    public void hitBeacon(boolean direction, LinearVisionOpMode opMode) {



        encoderDrive(DRIVE_SPEED, 1.5, 1.5, 1.0, opMode); //right in front of FIRST beacon to look at color

        boolean colorFound = false;
        boolean redRight = false;
        boolean redLeft = false;


        runtime.reset();
        while (opMode.opModeIsActive() && (runtime.seconds() < 0.75)) {
            pauseDrive();

        }

        while (!colorFound) {
            if (opMode.beacon.getAnalysis().getColorString().equals("red, blue")) {
                opMode.telemetry.addData("Color: ", "red is left");
                opMode.telemetry.update();
                redLeft = true;
                colorFound = true;
            } else if (opMode.beacon.getAnalysis().getColorString().equals("blue, red")) {
                opMode.telemetry.addData("Color: ", "red is right");
                opMode.telemetry.update();
                redRight = true;
                colorFound = true;
            }

            opMode.telemetry.addData("Beacon Color", opMode.beacon.getAnalysis().getColorString());
            opMode.telemetry.addData("Beacon Center", opMode.beacon.getAnalysis().getLocationString());
            opMode.telemetry.addData("Beacon Confidence", opMode.beacon.getAnalysis().getConfidenceString());
            opMode.telemetry.addData("Beacon Buttons", opMode.beacon.getAnalysis().getButtonString());
            opMode.telemetry.addData("Screen Rotation", opMode.rotation.getScreenOrientationActual());
            opMode.telemetry.addData("Frame Rate", opMode.fps.getFPSString() + " FPS");
            opMode.telemetry.addData("Frame Size", "Width: " + opMode.width + " Height: " + opMode.height);
            opMode.telemetry.update();

        }
        pauseDrive();

        if (redRight) {   //first beacon!
            encoderDrive(DRIVE_SPEED, 0.6, 0.6, 1, opMode); //moving infront of correct button
            pauseDrive();

//towards beacon

            encoderDriveStrafe(direction, DRIVE_SPEED, 4.0, 4.0, 6, opMode);

            pauseDrive();


            runtime.reset();
            while (opMode.opModeIsActive() && (runtime.seconds() < 0.1)) {
                pauseDrive();
            }

            encoderDriveStrafe(!direction, DRIVE_SPEED, 4.0, 4.0, 6, opMode);

            pauseDrive();


        } else if (redLeft) { //first beacon
            encoderDrive(-DRIVE_SPEED, -1.6, -1.6, 3, opMode); //moving infront of correct button

            pauseDrive();


            encoderDriveStrafe(direction, DRIVE_SPEED, 4.0, 4.0, 6, opMode);

            pauseDrive();


            runtime.reset();
            while (opMode.opModeIsActive() && (runtime.seconds() < 0.1)) {
                pauseDrive();
            }
            
            pauseDrive();
            
            encoderDriveStrafe(!direction, DRIVE_SPEED, 4.0, 4.0, 6, opMode);
            
            pauseDrive();
        }
        
        public void delay(double s, LinearOpMode opMode) {
            runtime.reset();
            while (opMode.opModeIsActive() && (runtime.seconds() < s)) {
                pauseDrive();
            } 
        }


    }


    /***
     * waitForTick implements a periodic delay. However, this acts like a metronome with a regular
     * periodic tick.  This is used to compensate for varying processing times for each cycle.
     * The function looks at the elapsed cycle time, and sleeps for the remaining time interval.
     *
     * @param periodMs Length of wait cycle in mSec.
     * @throws InterruptedException
     */
    public void waitForTick(long periodMs) throws InterruptedException {

        long remaining = periodMs - (long) period.milliseconds();

        // sleep for the remaining portion of the regular cycle period.
        if (remaining > 0)
            Thread.sleep(remaining);

        // Reset the cycle clock for the next pass.
        period.reset();
    }
}

