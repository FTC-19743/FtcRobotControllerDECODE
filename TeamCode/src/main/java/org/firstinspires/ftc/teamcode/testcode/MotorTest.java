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
    public static double VelocityIncrement = 100;
    public static final float openPos = 0.39f;
    public static final float launchPos = 0.51f;
    public double CurrentPower = 0;
    public double currentVelocity = 0;




    @Override
    public void runOpMode(){
        FtcDashboard dashboard = FtcDashboard.getInstance();
        telemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry()); // write telemetry to Driver Station and Dashboard

        DcMotorEx motor = hardwareMap.get(DcMotorEx.class,"motor");
        DcMotorEx motorL = hardwareMap.get(DcMotorEx.class,"motorL");
        DcMotorEx motorR = hardwareMap.get(DcMotorEx.class,"motorR");

        Servo pusher = hardwareMap.get(Servo.class,"pusher");

        teamUtil.init(this);

        waitForStart();

        if (isStopRequested()) {
            return;
        }
        pusher.setPosition(openPos);
        while (opModeIsActive()) {
            motor.setPower(CurrentPower);
            //motorL.setPower(CurrentPower);
            //motorR.setPower(-CurrentPower);
            motorL.setVelocity(currentVelocity);
            motorR.setVelocity(-currentVelocity);
            if(gamepad1.dpadUpWasReleased()){
                CurrentPower+=PowerIncrement;
                currentVelocity+=VelocityIncrement;
            }
            if(gamepad1.dpadDownWasReleased()){
                CurrentPower-=PowerIncrement;
                currentVelocity-=VelocityIncrement;
            }
            if(gamepad1.aWasReleased()) {
                pusher.setPosition(launchPos);
                teamUtil.pause(400);
                pusher.setPosition(openPos);
            }

            TelemetryPacket packet = new TelemetryPacket();
            dashboard.sendTelemetryPacket(packet);
            telemetry.addLine("Current Power: " + CurrentPower);
            telemetry.addLine("currentVelocity: " + currentVelocity);
            telemetry.addLine("ReportedVelocity: " + motorL.getVelocity()+", "+motorR.getVelocity());
            telemetry.update();
        }
    }


}
