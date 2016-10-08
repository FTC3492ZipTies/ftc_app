/* Copyright (c) 2014, 2015 Qualcomm Technologies Inc

All rights reserved.

Redistribution and use in source and binary forms, with or without modification,
are permitted (subject to the limitations in the disclaimer below) provided that
the following conditions are met:

Redistributions of source code must retain the above copyright notice, this list
of conditions and the following disclaimer.

Redistributions in binary form must reproduce the above copyright notice, this
list of conditions and the following disclaimer in the documentation and/or
other materials provided with the distribution.

Neither the name of Qualcomm Technologies Inc nor the names of its contributors
may be used to endorse or promote products derived from this software without
specific prior written permission.

NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
"AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE. */

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.GyroSensor;
import com.qualcomm.robotcore.hardware.LightSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.UltrasonicSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * Demonstrates empty OpMode
 */
@Autonomous(name = "Concept: NullOp", group = "Concept")
public class Commands extends OpMode {

    DcMotor BRMotor, BLMotor, FRMotor, FLMotor;
    ColorSensor color;
    GyroSensor gyro;
    LightSensor lightF, lightB, lightR, lightL;
    UltrasonicSensor ultra;
    Servo grabServo;

    //DT measurements only apply to drive trains with 4 powered wheels. Width is between the centers of
    // the front wheels, and the length is between the centers of the wheels on one of the sides.
    static double wheelDia = 4.0, DTLength = 13.0, DTWidth = 16.0;

    //this is the number of encoder ticks per motor rotation. AndyMark motors are typically 7 ticks
    // times the gear ratio. an AM 40 would have 280 ticks.
    static int ticksPerRotation = 140;


    private ElapsedTime runtime = new ElapsedTime();

    //run the left motors
    void runLMotors(double power) {
        BLMotor.setPower(power);
        FLMotor.setPower(power);
    }

    //run the right motors
    void runRMotors(double power) {
        BRMotor.setPower(power);
        FRMotor.setPower(power);
    }

    //run the front motors
    void runFrMotors(double power) {
        FRMotor.setPower(power);
        FLMotor.setPower(power);
    }

    //run the rear motors
    void runReMotors(double power) {
        BRMotor.setPower(power);
        BLMotor.setPower(power);
    }

    //run all the drive motors
    void runAllMotors(double power) {
        runFrMotors(power);
        runReMotors(power);
    }

    //find the ticks it takes to run a distance.
    int distToTicks (double distance){
        int tickPerDist = (int) (ticksPerRotation / (wheelDia * Math.PI));
        int totalTicks = (int) (tickPerDist * distance);
        return totalTicks;
    }

    //drive forward a certain distance in inches
    void driveForward(double distance, double power){
        int initPos = FRMotor.getCurrentPosition();
        int endPos;

        //check drive direction
        if (power > 0){
            //if forwards the endPos should be be larger than the initPos
            endPos =  initPos + distToTicks(distance);
            //run the motors until we reach this distance
            while (endPos > initPos){
                runAllMotors(power);
            }
        }
        else if (power < 0){
            //if backwards, the endPos should be smaller than the initPos
            endPos =  initPos - distToTicks(distance);
            //run the motors until we reach this distance
            while (endPos < initPos){
                runAllMotors(power);
            }
        }

        stopAllMotors();
    }

    //turn a certain number of degrees using the motor encoders
    void encoderTurn(int degrees, double power){
        int initPos = FRMotor.getCurrentPosition();
        int endPos;

        //check turn direction
        if (degrees > 0){
            //will be turning right


        }
    }

    //stop all of the drive motors
    void stopAllMotors() {
        runAllMotors(0);
    }

    @Override
    public void init() {
        telemetry.addData("Status", "Initialized");
        FRMotor = hardwareMap.dcMotor.get("FRMotor");
        FLMotor = hardwareMap.dcMotor.get("FLMotor");
        BRMotor = hardwareMap.dcMotor.get("BRMotor");
        BLMotor = hardwareMap.dcMotor.get("BLMotor");

        color = hardwareMap.colorSensor.get("color");
    }

    /*
       * Code to run when the op mode is first enabled goes here
       * @see com.qualcomm.robotcore.eventloop.opmode.OpMode#start()
       */
    @Override
    public void init_loop() {
    }

    /*
     * This method will be called ONCE when start is pressed
     * @see com.qualcomm.robotcore.eventloop.opmode.OpMode#loop()
     */
    @Override
    public void start() {
        runtime.reset();
        driveForward(70, 1);
    }

    /*
     * This method will be called repeatedly in a loop
     * @see com.qualcomm.robotcore.eventloop.opmode.OpMode#loop()
     */
    @Override
    public void loop() {
        telemetry.addData("Status", "Run Time: " + runtime.toString());
    }
}
