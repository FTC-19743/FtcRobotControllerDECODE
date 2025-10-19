package org.firstinspires.ftc.teamcode.testCode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.libs.teamUtil;

@Config
@TeleOp(name = "Intake Test", group = "Test Code")
public class IntakeTest extends LinearOpMode {

    public static double PowerIncrement = 0.1;
    public static double VelocityIncrement = 100;
    public static double SmallVelocityIncrement  = 10;
    public static final float openPos = 0.39f;
    public static final float launchPos = 0.51f;
    public double CurrentPower = 0;
    public double currentVelocity = 0;





    @Override
    public void runOpMode(){
        FtcDashboard dashboard = FtcDashboard.getInstance();
        telemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry()); // write telemetry to Driver Station and Dashboard

        DcMotorEx motor = hardwareMap.get(DcMotorEx.class,"motor");
        //DcMotorEx motorL = hardwareMap.get(DcMotorEx.class,"motorL");
        //DcMotorEx motorR = hardwareMap.get(DcMotorEx.class,"motorR");



        teamUtil.init(this);

        waitForStart();

        if (isStopRequested()) {
            return;
        }

        while (opModeIsActive()) {
            motor.setPower(CurrentPower);
            //motorL.setPower(CurrentPower);
            //motorR.setPower(-CurrentPower);

            if(gamepad1.dpadUpWasReleased()){
                CurrentPower+=PowerIncrement;
                currentVelocity+=VelocityIncrement;
            }
            if(gamepad1.dpadDownWasReleased()){
                CurrentPower-=PowerIncrement;
                currentVelocity-=VelocityIncrement;
            }
            if(gamepad1.dpadRightWasReleased()){
                CurrentPower*=-1;
            }

            TelemetryPacket packet = new TelemetryPacket();
            dashboard.sendTelemetryPacket(packet);
            telemetry.addLine("Current Power: " + CurrentPower);
            telemetry.addLine("currentVelocity: " + currentVelocity);

            telemetry.update();
        }
    }


}
