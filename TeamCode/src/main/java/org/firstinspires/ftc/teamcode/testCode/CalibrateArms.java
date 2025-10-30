package org.firstinspires.ftc.teamcode.testCode;



import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;


import org.firstinspires.ftc.teamcode.assemblies.AxonPusher;
import org.firstinspires.ftc.teamcode.assemblies.Intake;
import org.firstinspires.ftc.teamcode.assemblies.Robot;
import org.firstinspires.ftc.teamcode.assemblies.Shooter;

import org.firstinspires.ftc.teamcode.libs.teamUtil;

@Config
@TeleOp(name = "Calibrate Arms", group = "Test Code")
public class CalibrateArms extends LinearOpMode {

    Robot robot;
    
    static public float FLIPPER_TEST_VAL = 0f;
    
    double aimerPosition = Shooter.AIMER_CALIBRATE;
    
    public enum Ops {
        Test_Intake,
        Test_Shooter,
        Test_Foot
    };
    public static Ops AA_Operation = Ops.Test_Shooter;
    public static boolean useCV = true;

    @Override
    public void runOpMode() throws InterruptedException {
        FtcDashboard dashboard = FtcDashboard.getInstance();
        //FtcDashboard.setDrawDefaultField(false); // enable to eliminate field drawing
        telemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry()); // write telemetry to Driver Station and Dashboard
        teamUtil.init(this);
        
        robot = new Robot();
        robot.initialize();
        //robot.initCV(enableLiveView);// TODO: false for competition

        robot.drive.setHeading(0);
        teamUtil.justRanAuto = false;
        teamUtil.justRanCalibrateRobot = false;

        robot.calibrate();
        telemetry.addLine("Ready to start");
        telemetry.addLine("ALLIANCE : " + teamUtil.alliance);
        telemetry.update();

        waitForStart();

        if (isStopRequested()) {
            return;
        }

        while (opModeIsActive()) {
           

            if (gamepad1.left_stick_button) {
                teamUtil.logSystemHealth();
            }
            switch (AA_Operation) {
                case Test_Intake : testIntake();break;
                case Test_Shooter : testShooter();break;
                case Test_Foot : testFoot();break;
            }

            // Drawing stuff on the field
            TelemetryPacket packet = new TelemetryPacket();
            dashboard.sendTelemetryPacket(packet);

            telemetry.update();

            // Graphing stuff and putting stuff in telemetry
            //telemetry.addData("Item", data)); // Anything written like this can be graphed against time.  Multiple items can be graphed together
            //telemetry.addData("Velocity", 0);
            //telemetry.addData("Encoder", 0);
            //telemetry.addData("Current Velocity", 0);
            //telemetry.addData("Motor Velocity", 0);
            //robot.intake.axonSlider.loop();


        }

    }

    public void testIntake() {
        robot.intake.intakeTelemetry();
        telemetry.addLine("Elevator Tolerance: " + robot.intake.elevator.getPIDFCoefficients(DcMotor.RunMode.RUN_TO_POSITION));

        if (gamepad1.rightBumperWasReleased()) {
            robot.intake.calibrate();
            robot.intake.elevator.setVelocity(0);
        }
        if(gamepad1.dpadUpWasReleased()){
            robot.intake.middle_flipper.setPosition(FLIPPER_TEST_VAL);
        }
        if(gamepad1.dpadLeftWasReleased()){
            robot.intake.left_flipper.setPosition(FLIPPER_TEST_VAL);
        }
        if(gamepad1.dpadRightWasReleased()){
            robot.intake.right_flipper.setPosition(FLIPPER_TEST_VAL);
        }
        if(gamepad1.circleWasReleased()){
            robot.intake.intakeStop();
        }
        if(gamepad1.triangleWasReleased()){
            robot.intake.intakeIn();
        }
        if(gamepad1.squareWasReleased()){
            robot.intake.intakeOut();
        }if(gamepad1.crossWasReleased()){
            robot.intake.elevatorToFlippers();
        }if(gamepad1.optionsWasReleased()){
            robot.intake.elevatorToFlippersNoWait();
        }
    }

    public void testFoot() {
        if(gamepad1.dpadUpWasReleased()){
            robot.setFootPos(Robot.FOOT_EXTENDED_POS);
        }if(gamepad1.dpadDownWasReleased()){
            robot.setFootPos(Robot.FOOT_CALIBRATE_POS);
        }

    }
    public void testShooter(){
        robot.shooter.outputTelemetry();

        if(gamepad1.dpadUpWasReleased()){
            robot.shooter.setShootSpeed(Shooter.SHOOTER_FAR_VELOCITY);
        }if(gamepad1.dpadDownWasReleased()){
            robot.shooter.stopShooter();
        }
        if(gamepad1.dpadLeftWasPressed()){
            robot.shooter.aim(robot.shooter.currentAim()-.01);
        }
        if(gamepad1.dpadRightWasPressed()){
            robot.shooter.aim(robot.shooter.currentAim()+.01);
        }

        if(gamepad1.aWasPressed()){
            robot.shooter.pusher.pushN(1, AxonPusher.RTP_MAX_VELOCITY, 1500);

        }

    }
}
