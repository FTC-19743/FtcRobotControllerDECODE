package org.firstinspires.ftc.teamcode.testcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.libs.teamUtil;

@Config
@TeleOp(name = "Shooter Test", group = "Test Code")
public class ShooterTest extends LinearOpMode {

    public static double PowerIncrement = 0.1;
    public static double VelocityIncrement = 100;
    public static double SmallVelocityIncrement  = 10;
    public static final float openPos = 0.39f;
    public static final float launchPos = 0.51f;
    public double CurrentPower = 0;
    public double currentRVelocity = 0;
    public double currentLVelocity = 0;
    public static double shooterIncrement = 0.11;
    public static double shooterP = 50;
    public static double shooterI = 1;
    public static double shooterD = 0.5;
    public double shooterF = 0;
    public static int pusherPause = 400;




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
        teamUtil.log("PIDF coefficients: " + motorL.getPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER));
        while (opModeIsActive()) {
            motorL.setVelocityPIDFCoefficients(shooterP, shooterI, shooterD, shooterF);
            motorR.setVelocityPIDFCoefficients(shooterP, shooterI, shooterD, shooterF);
            motor.setPower(CurrentPower);
            //motorL.setPower(CurrentPower);
            //motorR.setPower(-CurrentPower);
            motorL.setVelocity(currentLVelocity);
            motorR.setVelocity(-currentRVelocity);
            if(gamepad1.dpadUpWasReleased()){
                CurrentPower+=PowerIncrement;
                currentLVelocity+=VelocityIncrement;
            }
            if(gamepad1.dpadDownWasReleased()){
                CurrentPower-=PowerIncrement;
                currentLVelocity-=VelocityIncrement;
            }
            if(gamepad1.dpadRightWasReleased()){
                CurrentPower+=PowerIncrement;
                currentRVelocity+=VelocityIncrement;
            }
            if(gamepad1.dpadLeftWasReleased()){
                CurrentPower-=PowerIncrement;
                currentRVelocity-=VelocityIncrement;
            }
            if(gamepad1.aWasPressed()) {
                pusher.setPosition(pusher.getPosition()-shooterIncrement);
            }
            if(gamepad1.yWasPressed()) {
                pusher.setPosition(pusher.getPosition()+shooterIncrement);
            }
            if(gamepad1.bWasPressed()) {
                pusher.setPosition(pusher.getPosition()-shooterIncrement);
                teamUtil.pause(pusherPause);
                pusher.setPosition(pusher.getPosition()-shooterIncrement);
                teamUtil.pause(pusherPause);
                pusher.setPosition(pusher.getPosition()-shooterIncrement);
            }

            TelemetryPacket packet = new TelemetryPacket();
            dashboard.sendTelemetryPacket(packet);
            telemetry.addLine("Current Power: " + CurrentPower);
            telemetry.addLine("currentVelocity: " + currentRVelocity + ", " + currentLVelocity);
            telemetry.addLine("ReportedVelocity: " + motorL.getVelocity()+", "+motorR.getVelocity());
            telemetry.addData("Reported Left Velocity: " , motorL.getVelocity());
            telemetry.addData("Reported Right Velocity: " , motorR.getVelocity());

            telemetry.update();
        }
    }


}
