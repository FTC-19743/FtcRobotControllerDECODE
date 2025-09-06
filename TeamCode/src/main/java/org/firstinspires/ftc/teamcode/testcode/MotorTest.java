package org.firstinspires.ftc.teamcode.testcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.libs.teamUtil;

@Config
@TeleOp(name = "Motor Test", group = "Test Code")
public class MotorTest extends LinearOpMode {

    public static double PowerIncrement = 0.1;
    public static double CurrentPower = 0;



    @Override
    public void runOpMode(){
        FtcDashboard dashboard = FtcDashboard.getInstance();
        telemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry()); // write telemetry to Driver Station and Dashboard

        DcMotorEx motor = hardwareMap.get(DcMotorEx.class,"motor");

        teamUtil.init(this);

        waitForStart();

        if (isStopRequested()) {
            return;
        }

        while (opModeIsActive()) {
            motor.setPower(CurrentPower);
            if(gamepad1.dpadUpWasReleased()){
                CurrentPower+=PowerIncrement;
            }
            if(gamepad1.dpadDownWasReleased()){
                CurrentPower-=PowerIncrement;
            }

            TelemetryPacket packet = new TelemetryPacket();
            dashboard.sendTelemetryPacket(packet);
            telemetry.addLine("Current Power: " + CurrentPower);

            telemetry.update();
        }
    }


}
