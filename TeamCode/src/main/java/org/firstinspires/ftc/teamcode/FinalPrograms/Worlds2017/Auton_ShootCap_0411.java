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

import org.firstinspires.ftc.teamcode.FinalPrograms.WAGSAdafruitIMU;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.OpticalDistanceSensor;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.opencv.core.Size;

/*
 * Alliance: RED/BLUE
 * Objective: 
 *    Score two particle 
 *    Knock cap ball off of wooden base
 * Enhancements: 
 *    Hall Switch speed control 
 *    Encoder drive
 * Other: N/A
*/


@Autonomous(name = "2R/B: WORLDS Shoot & Cap", group = "Robot")
//@Disabled                            // Uncomment this to add to the opmode list
public class Auton_ShootCap_0411 extends LinearVisionOpMode {
    int frameCount = 0;

    Hardware0419 robot = new Hardware0419();
    private ElapsedTime runtime = new ElapsedTime();
    ColorSensor colorSensor;

    OpticalDistanceSensor ods;
//    DeviceInterfaceModule cdim;
//    OpticalDistanceSensor opticalDistanceSensor;


    static final double COUNTS_PER_MOTOR_REV = 1440;    // eg: TETRIX Motor Encoder
    static final double DRIVE_GEAR_REDUCTION = 2.0;     // This is < 1.0 if geared UP
    static final double WHEEL_DIAMETER_INCHES = 4.0;     // For figuring circumference
    static final double COUNTS_PER_INCH = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * 3.1415);
    static final double DRIVE_SPEED = 0.6;
    static final double TURN_SPEED = 0.5;

    static final double LATCH_OPEN = 0.0;
    static final double LATCH_CLOSE = 0.95;


    static final int LED_CHANNEL = 5;
    float hsvValues[] = {0F, 0F, 0F};
    final float values[] = hsvValues;

    // State used for updating telemetry

    //----------------------------------------------------------------------------------------------
    // Main logic
    //----------------------------------------------------------------------------------------------

    @Override
    public void runOpMode() throws InterruptedException {

        robot.init(hardwareMap, true);
        robot.initialize();
        telemetry.update();
        setCamera(Cameras.SECONDARY); //selfie camera
        setFrameSize(new Size(900, 900));
        enableExtension(VisionOpMode.Extensions.BEACON);         //Beacon detection
        enableExtension(VisionOpMode.Extensions.ROTATION);       //Automatic screen rotation correction
        enableExtension(VisionOpMode.Extensions.CAMERA_CONTROL); //Manual camera control

        robot.resetCamera();

        telemetry.addData("Ready", "Ready to start.");

        waitForStart();

        robot.lineTracker.enableLed(false);

        robot.driveShootParticles(0.35, 1.50, this);


        robot.encoderDrive(DRIVE_SPEED, 5, 5, 0.5, this); //move forward until 10


        robot.ballLatch.setPosition(0.7);

        robot.pauseDrive();


        robot.encoderDrive(DRIVE_SPEED, 13, 13, 9, this);


        sleep(1000);

        telemetry.addData("Path", "Complete");
        telemetry.update();


    }

}





