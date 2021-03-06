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

import android.graphics.Color;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.OpticalDistanceSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.opencv.core.Size;

/*
 * Alliance: RED
 * Objective: 
 *    Score two particle 
 *    Hit both beacons
 *    Park on corner vortex
 * Enhancements: 
 *    Hall Switch speed control 
 *    Computer vision
 *    Encoder drive
 *    IMU auto alignment    
 * Other: N/A
*/

@Autonomous(name = "FAR Red: WORLDS Beacon & Corner", group = "Red")
//@Disabled                            // Uncomment this to add to the opmode list
public class AutonRed_BeaconCorner_0411 extends LinearVisionOpMode {

    Hardware0419 robot = new Hardware0419();

    final int WHITE_VALUE = 15;
    final double TURN_SPEED = 0.25;
    final double DRIVE_SPEED_FAST = 0.75;
    final double DRIVE_SPEED = 0.5;
    final double DRIVE_SPEED_SLOW = 0.2;
    boolean direction = false;

    float hsvValues[] = {0F, 0F, 0F};

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

        robot.turn(30, TURN_SPEED, this);

        robot.encoderDrive(DRIVE_SPEED_FAST, 24, 24, 8.0, this); //move forward towards beacon ish area //22

        robot.turn(5, TURN_SPEED, this);

        robot.searchWhiteLine(WHITE_VALUE, DRIVE_SPEED_SLOW, this);

        robot.hitBeacon(direction, this);

        //move to next beacon

        robot.encoderDrive(DRIVE_SPEED, -10, -10, 3.0, this); //other beacon

        robot.searchWhiteLine(WHITE_VALUE, DRIVE_SPEED_SLOW, this);

        robot.hitBeacon(direction, this);

        //park on corner

        robot.encoderDrive(DRIVE_SPEED_FAST, -3, -3, 1, this);

        robot.encoderDrive(DRIVE_SPEED_FAST, 2, -2, 1, this);

        robot.encoderDrive(DRIVE_SPEED_FAST, -16, -16, 5, this);

        robot.pauseDrive();

        sleep(1000);

        telemetry.addData("Path", "Complete");
        telemetry.update();


    }
}





