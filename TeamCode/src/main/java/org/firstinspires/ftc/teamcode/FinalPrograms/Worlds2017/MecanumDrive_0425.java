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
package org.firstinspires.ftc.teamcode.FinalPrograms.Worlds2017;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.OpticalDistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;




@TeleOp(name="World Championships Teleop", group="Champions Challenge")
//@Disabled
public class MecanumDrive_0425 extends OpMode{

    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();

    Hardware0419 WAGSbot = new Hardware0419();

    //percentage power of the ball shooter
    final double SHOOTER_CALIB_VAL = 1.0;
    final double CIRCLE_DIST = 4 * Math.PI / 6;
    final double MAX_POWER = 1.0;
    final double SHOOT_HIGH = 0.55;
    final double SHOOT_LOW = 0.35;

    final double TARGET_SPEED_HIGH = 234.27;
    final double TARGET_SPEED_LOW = 368.07;

    double timeValue = 0;
    int magCross = 0;
    OpticalDistanceSensor ods;
    DigitalChannel hallSwitch;

    final double MAX_POS = 1.0;
    final double MIN_POS = 0;
    boolean newCondition = false;

    @Override
    public void init() {

        WAGSbot.init(hardwareMap, false); //initialize hardware map, not auton
        ods = hardwareMap.opticalDistanceSensor.get("ods");
        // Send telemetry message to show robot waiting
        telemetry.addData("Say", "Hello Driver");
        updateTelemetry(telemetry);

        WAGSbot.ballLatch.setPosition(0.7);//it was 0.4
        WAGSbot.button.setPosition(0.45);
        WAGSbot.liftLatch.setPosition(0.72);
        ColorSensor colorSensor = hardwareMap.colorSensor.get("color");
        hallSwitch = hardwareMap.digitalChannel.get("hallSwitch");
        colorSensor.enableLed(false);
        ods.enableLed(false);
        telemetry.addData("reset:", "encoders");
        updateTelemetry(telemetry);

//
//        WAGSbot.ballShooter.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        WAGSbot.ballShooter2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        WAGSbot.ballShooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        WAGSbot.ballShooter2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);


        telemetry.update();

        telemetry.addData("Init:", "Complete");
        updateTelemetry(telemetry);
    }

    @Override
    public void init_loop() {

    }


    @Override
    public void start() {
    }


    @Override
    public void loop() {

        //values for drive motor power for mecanum drive
        double leftF = 0;
        double leftB = 0;
        double rightF = 0;
        double rightB = 0;

        double power = 0;
        double power2 = 0;
        int index = 0;

        boolean ballFlag = true;

        //values for drive motor power for tank drive
        double left = 0;
        double right = 0;

        //setting new drivers1 to gamepad1 values
        Gamepad drivers1 = new Gamepad();

        drivers1.left_stick_y = -gamepad1.left_stick_y;
        drivers1.right_stick_x = gamepad1.right_stick_x;
        drivers1.left_stick_x = gamepad1.left_stick_x;
        drivers1.right_stick_y = -gamepad1.right_stick_y;



     //create dead zone to prevent robot from drifting diagonally
    if(drivers1.left_stick_x > -0.5 && drivers1.left_stick_x < 0.5 && drivers1.left_stick_y != 0 ||
       drivers1.left_stick_x != 0 &&drivers1.left_stick_y > -0.5 && drivers1.left_stick_y < 0.5 ||
       drivers1.right_stick_x != 0) {

        //right - left shift & turn
        rightF = (drivers1.left_stick_y - drivers1.right_stick_x) + (drivers1.left_stick_x);
        rightB = (drivers1.left_stick_y - drivers1.right_stick_x) + (-drivers1.left_stick_x);
        leftF = (drivers1.left_stick_y + drivers1.right_stick_x) + -drivers1.left_stick_x;
        leftB = (drivers1.left_stick_y + drivers1.right_stick_x) + (drivers1.left_stick_x);

    }

        //setting values from joystick to the motors

        if(gamepad1.x) {
           newCondition = !newCondition;
        }

        if(newCondition) {
            WAGSbot.leftMotorFront.setPower(leftF * 0.23);
            WAGSbot.leftMotorBack.setPower(leftB * 0.23);
            WAGSbot.rightMotorFront.setPower(rightF * 0.23);
            WAGSbot.rightMotorBack.setPower(rightB * 0.23);
        }

       else {
            WAGSbot.leftMotorFront.setPower(leftF);
            WAGSbot.leftMotorBack.setPower(leftB);
            WAGSbot.rightMotorFront.setPower(rightF);
            WAGSbot.rightMotorBack.setPower(rightB);
        }



        power2 = -gamepad2.right_stick_y;

        WAGSbot.buttonPusherL.setPower(0.0);
        WAGSbot.buttonPusherR.setPower(0.0);


        if(gamepad2.right_trigger > 0) {
            WAGSbot.button.setPosition(Servo.MAX_POSITION);
        }

        else if(gamepad2.right_bumper) {
            WAGSbot.button.setPosition(Servo.MIN_POSITION);
        }

        else {
            WAGSbot.button.setPosition(0.45);
        }

        if(gamepad2.left_bumper) {
            WAGSbot.liftLatch.setPosition(0.83); //down
        }

        else if (gamepad2.left_trigger > 0) {
            WAGSbot.liftLatch.setPosition(0.72);
        }

        //when A is pressed, move collector in
        if(gamepad2.a) {
            WAGSbot.ballPick.setPower(MAX_POWER);
        }

        //when B is pressed, stop
        if (gamepad2.b) {
            WAGSbot.ballPick.setPower(0.0);
        }

        //when X is pressed, move collector out
        if(gamepad2.x) {
            WAGSbot.ballPick.setPower(-MAX_POWER);
        }


        if(gamepad2.y) {
                WAGSbot.ballLatch.setPosition(0.05); //0.15
        }
        else {
            WAGSbot.ballLatch.setPosition(0.7);//it was 0.4
        }

        //set power to Y value on second joystick
        power = -gamepad2.left_stick_y;
        power2 = -gamepad2.right_stick_y;


        //if joystick is moved up then move the ball shooter
        //creates 3 areas of deadzone


//        if(power < 0) {
//            runtime.reset();
//            if(hallSwitch.getState() == false) {
//                magCross++;
//                timeValue = runtime.time();
//                runtime.reset();
//                telemetry.addData("MagnetCrossed: ", true);
//
//            }
//
//            if(CIRCLE_DIST / timeValue < TARGET_POWER_LOW) {
//                double newPwr = WAGSbot.ballShooter.getPower();
//                WAGSbot.ballShooter.setPower(newPwr++);
//                WAGSbot.ballShooter2.setPower(newPwr++);
//                telemetry.addData("Speed: ", "adjust");
//
//            }
//
//            else if (CIRCLE_DIST / timeValue > TARGET_POWER_LOW){
//                double newPwr = WAGSbot.ballShooter.getPower();
//                WAGSbot.ballShooter.setPower(newPwr--);
//                WAGSbot.ballShooter2.setPower(newPwr--);
//                telemetry.addData("Speed: ", "adjust");
//                telemetry.addData("greater: ", "greater");
//            }
//
//            else {
//
//                WAGSbot.ballShooter.setPower(TARGET_POWER_LOW);
//                WAGSbot.ballShooter2.setPower(TARGET_POWER_LOW);
//                telemetry.addData("Eq: ", "equal");
//            }
//
//            telemetry.addData("# times magnets", magCross);
//
//        }
//
//        else if(power > 0) { //down7
//
//            WAGSbot.ballShooter.setPower(TARGET_POWER_HIGH);
//            WAGSbot.ballShooter2.setPower(TARGET_POWER_HIGH);
//
//        }
//
//        else {
//            WAGSbot.ballShooter.setPower(0);
//            WAGSbot.ballShooter2.setPower(0);
//        }


        if(power < 0) { //low power
            runtime.reset();
            if(hallSwitch.getState() == false) {
                magCross++;
                timeValue = runtime.time();
                runtime.reset();
                telemetry.addData("MagnetCrossed: ", true);

            }

            if(CIRCLE_DIST / timeValue < SHOOT_LOW) { //run at 0.35
                double newPwr = WAGSbot.ballShooter.getPower();
                WAGSbot.ballShooter.setPower(newPwr++);
                WAGSbot.ballShooter2.setPower(newPwr++);
                telemetry.addData("Speed: ", "adjust");

            }

            else {
                WAGSbot.ballShooter.setPower(SHOOT_LOW);
                WAGSbot.ballShooter2.setPower(SHOOT_LOW);
            }

            telemetry.addData("# times magnets", magCross);

        }

        else if(power > 0) { //high power
            runtime.reset();
            if(hallSwitch.getState() == false) {
                magCross++;
                timeValue = runtime.time();
                runtime.reset();
                telemetry.addData("MagnetCrossed: ", true);

            }

            if(CIRCLE_DIST / timeValue < SHOOT_HIGH) { //run at 0.55
                double newPwr = WAGSbot.ballShooter.getPower();
                WAGSbot.ballShooter.setPower(newPwr++);
                WAGSbot.ballShooter2.setPower(newPwr++);
                telemetry.addData("Speed: ", "adjust");

            }

            else {
                WAGSbot.ballShooter.setPower(SHOOT_HIGH);
                WAGSbot.ballShooter2.setPower(SHOOT_HIGH);
            }

            telemetry.addData("# times magnets", magCross);

        }

        else {
            WAGSbot.ballShooter.setPower(0);
            WAGSbot.ballShooter2.setPower(0);
        }









        if(power2 >= 0) {
            WAGSbot.lift.setPower(power2 * 0.5);
        }

        else if(power2 < 0) {
            WAGSbot.lift.setPower(power2 * 0.5);
        }

        else {
            WAGSbot.lift.setPower(0);
        }




            // Send telemetry message to signify robot running;
            telemetry.addData("condition", newCondition);
            telemetry.addData("voltage: ", hardwareMap.voltageSensor.get("Motor Controller 1").getVoltage());
            telemetry.addData("ballshoot1", WAGSbot.ballShooter.getPower());
            telemetry.addData("power2", WAGSbot.ballShooter2.getPower());
            telemetry.addData("left front", "%.2f", leftF);
            telemetry.addData("right front", "%.2f", rightF);
            telemetry.addData("left back", "%.2f", leftB);
            telemetry.addData("right back", "%.2f", rightB);
            telemetry.addData("left x", "%.2f", drivers1.right_stick_x);
            updateTelemetry(telemetry);
        }

    @Override
    public void stop() {
    }

}
