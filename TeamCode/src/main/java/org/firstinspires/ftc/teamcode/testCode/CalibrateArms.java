package org.firstinspires.ftc.teamcode.testCode;



import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;


import org.firstinspires.ftc.teamcode.assemblies.BasicDrive;

import org.firstinspires.ftc.teamcode.assemblies.Intake;
import org.firstinspires.ftc.teamcode.assemblies.Output;

import org.firstinspires.ftc.teamcode.libs.teamUtil;

@Config
@TeleOp(name = "Calibrate Arms", group = "Test Code")
public class CalibrateArms extends LinearOpMode {

    Intake intake;
    Output output;
    BasicDrive drive;
    static public float FLIPPER_TEST_VAL = 0f;

    //Robot robot;  // DO NOT USE, not properly initialized!
    boolean hangCalibrated = false;

    
    public enum Ops {Test_Intake
       
    };
    public static Ops AA_Operation = Ops.Test_Intake;
    public static boolean useCV = true;

    
    private void doAction() {
        switch (AA_Operation) {
            case Test_Intake : testIntake();break;
            
        }
    }
    @Override
    public void runOpMode() throws InterruptedException {
        FtcDashboard dashboard = FtcDashboard.getInstance();
        //FtcDashboard.setDrawDefaultField(false); // enable to eliminate field drawing
        telemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry()); // write telemetry to Driver Station and Dashboard

        teamUtil.init(this);
        teamUtil.alliance = teamUtil.Alliance.RED;
        teamUtil.SIDE=teamUtil.Side.BASKET;
        //Robot robot = new Robot();
        //teamUtil.robot = robot;
        BasicDrive drive = new BasicDrive();
        drive.initalize();
        drive.calibrate();


        intake = new Intake();
        intake.initialize();

        output = new Output();
        //output.initalize();
        //output.calibrate();
        
       
        telemetry.addLine("Ready to start");
        telemetry.update();

        waitForStart();

        if (isStopRequested()) {
            return;
        }

        while (opModeIsActive()) {
           

            if (gamepad1.left_stick_button) {
                teamUtil.logSystemHealth();
            }

            if (AA_Operation==Ops.Test_Intake){
                testIntake();
            }


            // Drawing stuff on the field
            TelemetryPacket packet = new TelemetryPacket();
            dashboard.sendTelemetryPacket(packet);


            // Graphing stuff and putting stuff in telemetry
            //telemetry.addData("Item", data)); // Anything written like this can be graphed against time.  Multiple items can be graphed together
            //telemetry.addData("Velocity", 0);
            //telemetry.addData("Encoder", 0);
            //telemetry.addData("Current Velocity", 0);
            //telemetry.addData("Motor Velocity", 0);
            //intake.axonSlider.loop();

            intake.intakeTelemetry();
           

            telemetry.addLine("Elevator Tolerance: " + intake.elevator.getPIDFCoefficients(DcMotor.RunMode.RUN_TO_POSITION));
            
            telemetry.update();
        }

    }

    public void testIntake() {
        if (gamepad1.rightBumperWasReleased()) {
            intake.calibrate();
            intake.elevator.setVelocity(0);
        }
        if(gamepad1.dpadUpWasReleased()){
            intake.middle_flipper.setPosition(FLIPPER_TEST_VAL);
        }
        if(gamepad1.dpadLeftWasReleased()){
            intake.left_flipper.setPosition(FLIPPER_TEST_VAL);
        }
        if(gamepad1.dpadRightWasReleased()){
            intake.right_flipper.setPosition(FLIPPER_TEST_VAL);
        }
        if(gamepad1.circleWasReleased()){
            intake.intakeStop();
        }
        if(gamepad1.triangleWasReleased()){
            intake.intakeIn();
        }
        if(gamepad1.squareWasReleased()){
            intake.intakeOut();
        }if(gamepad1.crossWasReleased()){
            intake.elevatorToFlippers();
        }if(gamepad1.optionsWasReleased()){
            intake.elevatorToFlippersNoWait();
        }
    }
}
