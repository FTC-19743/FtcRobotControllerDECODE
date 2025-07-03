package org.firstinspires.ftc.teamcode.testcode;

import android.annotation.SuppressLint;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;



@TeleOp(name = "GamepadTest ", group = "Test Code")
public class GamepadTest extends LinearOpMode {


    @SuppressLint("DefaultLocale")
    public void runOpMode() {

        telemetry.addLine("Ready to start");
        telemetry.update();
        waitForStart();


        while (opModeIsActive()) {
            if(gamepad1.dpadUpWasReleased()) {
                telemetry.addLine("Released");
            }
        }
    }
}