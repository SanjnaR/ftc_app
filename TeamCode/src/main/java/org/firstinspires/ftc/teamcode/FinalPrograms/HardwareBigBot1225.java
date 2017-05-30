package org.firstinspires.ftc.teamcode.FinalPrograms;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * This is NOT an opmode.
 *
 * This class can be used to define all the specific hardware for a single robot.
 * In this case that robot is a Pushbot.
 * See PushbotTeleopTank_Iterative and others classes starting with "Pushbot" for usage examples.
 *
 * This hardware class assumes the following device names have been configured on the robot:
 * Note:  All names are lower case and some have single spaces between words.
 *
 * Motor channel:  Left  drive motor:        "left_drive"
 * Motor channel:  Right drive motor:        "right_drive"
 */
public class HardwareBigBot1225
{
    /* Public OpMode members. */
    public DcMotor  leftMotorFront   = null;
    public DcMotor  leftMotorBack   = null;
    public DcMotor  rightMotorFront  = null;
    public DcMotor  rightMotorBack  = null;

    public DcMotor ballShooter = null;
    public DcMotor ballShooter2 = null;

    public DcMotor ballPick = null;
    public DcMotor lift = null;

    public CRServo buttonPusherR = null;
    public CRServo buttonPusherL = null;

    public Servo ballLatch = null;
    public Servo liftLatch = null;

    public Servo button = null;

    /* local OpMode members. */
    HardwareMap hwMap           =  null;
    private ElapsedTime period  = new ElapsedTime();

    /* Constructor */
    public HardwareBigBot1225(){

    }

    /* Initialize standard Hardware interfaces */
    public void init(HardwareMap ahwMap) {
        // Save reference to Hardware map
        hwMap = ahwMap;

        // Define and Initialize Motors
        leftMotorFront   = hwMap.dcMotor.get("left_front");
        leftMotorBack   = hwMap.dcMotor.get("left_back");
        rightMotorFront   = hwMap.dcMotor.get("right_front");
        rightMotorBack   = hwMap.dcMotor.get("right_back");
        ballShooter = hwMap.dcMotor.get("ball_shoot");
        ballShooter2 = hwMap.dcMotor.get("ball_shoot2");
        ballPick = hwMap.dcMotor.get("ballPick");
        lift = hwMap.dcMotor.get("lift");




        buttonPusherR = hwMap.crservo.get("buttonPusherR");
        buttonPusherL = hwMap.crservo.get("buttonPusherL");
        ballLatch = hwMap.servo.get("ballLatch");
        button = hwMap.servo.get("button");
        liftLatch = hwMap.servo.get("liftLatch");

        leftMotorFront.setDirection(DcMotor.Direction.REVERSE); // Set to REVERSE if using AndyMark motors
        leftMotorBack.setDirection(DcMotor.Direction.REVERSE); // Set to REVERSE if using AndyMark motors
        rightMotorFront.setDirection(DcMotor.Direction.FORWARD);// Set to FORWARD if using AndyMark motors
        rightMotorBack.setDirection(DcMotor.Direction.FORWARD);// Set to FORWARD if using AndyMark motors

        buttonPusherL.setDirection(CRServo.Direction.REVERSE);
        buttonPusherR.setDirection(CRServo.Direction.FORWARD);

        ballShooter.setDirection(DcMotorSimple.Direction.REVERSE);
        ballShooter2.setDirection(DcMotorSimple.Direction.FORWARD);
        ballPick.setDirection(DcMotorSimple.Direction.FORWARD);
        lift.setDirection(DcMotorSimple.Direction.FORWARD);


        // Set all motors to zero power
        leftMotorFront.setPower(0); // Set to REVERSE if using AndyMark motors
        leftMotorBack.setPower(0); // Set to REVERSE if using AndyMark motors
        rightMotorFront.setPower(0);// Set to FORWARD if using AndyMark motors
        rightMotorBack.setPower(0);// Set to FORWARD if using AndyMark motors

        ballShooter.setPower(0);
        ballShooter2.setPower(0);

        ballPick.setPower(0);
        lift.setPower(0);


        // Set all motors to run without encoders.
        // May want to use RUN_USING_ENCODERS if encoders are installed.
     leftMotorBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
     leftMotorFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
     rightMotorBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
     rightMotorFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);


        ballShooter.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        ballShooter2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        ballPick.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        lift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

    }


    /***
     *
     * waitForTick implements a periodic delay. However, this acts like a metronome with a regular
     * periodic tick.  This is used to compensate for varying processing times for each cycle.
     * The function looks at the elapsed cycle time, and sleeps for the remaining time interval.
     *
     * @param periodMs  Length of wait cycle in mSec.
     * @throws InterruptedException
     */
    public void waitForTick(long periodMs) throws InterruptedException {

        long  remaining = periodMs - (long)period.milliseconds();

        // sleep for the remaining portion of the regular cycle period.
        if (remaining > 0)
            Thread.sleep(remaining);

        // Reset the cycle clock for the next pass.
        period.reset();
    }
}

