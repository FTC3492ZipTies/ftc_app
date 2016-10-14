package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

/**
 * Created by Ki on 10/13/2016.
 */

@Autonomous(name = "Autonomoose v0.1")
public class Autonomous00 extends Commands{
    @Override
    public void init() {
        driveForward(70, 1);
        encoderTurn(180, .5);
        gyroTurn(-180, .5);
    }

}
