package org.firstinspires.ftc.teamcode.testCode;

import static org.firstinspires.ftc.teamcode.libs.teamUtil.Alliance.BLUE;
import static org.firstinspires.ftc.teamcode.libs.teamUtil.Alliance.RED;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.assemblies.AxonPusher;
import org.firstinspires.ftc.teamcode.assemblies.Robot;
import org.firstinspires.ftc.teamcode.assemblies.Shooter;
import org.firstinspires.ftc.teamcode.libs.teamUtil;

@Config
@TeleOp(name = "TestAutoPaths", group = "Test Code")
public class TestAutoPaths extends LinearOpMode{
    Robot robot;

    static public boolean USE_ARMS = false;
    public long elapsedTime = 0;

    public enum Ops {
        Goal_Side,
        Human_Side
    };
    static public int RESET_X = 0;
    static public int RESET_Y = 0;
    static public int RESET_H = 0;
    public static Ops AA_Operation = Ops.Goal_Side;
    public static boolean useCV = false;

    @Override
    public void runOpMode() throws InterruptedException {
        FtcDashboard dashboard = FtcDashboard.getInstance();
        //FtcDashboard.setDrawDefaultField(false); // enable to eliminate field drawing
        telemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry()); // write telemetry to Driver Station and Dashboard
        teamUtil.init(this);

        robot = new Robot();
        robot.initialize(false);
        //robot.initCV(enableLiveView);// TODO: false for competition

        robot.drive.setHeading(0);
        teamUtil.justRanAuto = false;
        teamUtil.justRanCalibrateRobot = false;
        teamUtil.alliance = BLUE;

        robot.calibrate();
        telemetry.addLine("Ready to start");
        telemetry.addLine("ALLIANCE : " + teamUtil.alliance);
        telemetry.update();

        waitForStart();

        if (isStopRequested()) {
            return;
        }

        while (opModeIsActive()) {
            telemetry.addLine("ALLIANCE : " + teamUtil.alliance + " SIDE : " + teamUtil.SIDE + "PATTERN: " + teamUtil.pattern);
            telemetry.addLine("Testing: " + AA_Operation);
            telemetry.addLine("Use Arms: "+ USE_ARMS);
            telemetry.addLine("Last Op: "+ elapsedTime);
            robot.drive.loop();
            robot.drive.driveMotorTelemetry();

            if (gamepad1.left_stick_button) {
                teamUtil.logSystemHealth();
            }
            if (gamepad1.leftBumperWasReleased()) {
                robot.setStartLocalizedPosition();
            }

            if (gamepad1.rightBumperWasReleased()) {
                if (teamUtil.alliance== RED) {
                    teamUtil.alliance = teamUtil.Alliance.BLUE;
                } else {
                    teamUtil.alliance = RED;
                }
            }


            switch (AA_Operation) {
                case Goal_Side : testGoalSide();break;
                case Human_Side : testHumanSide();break;
            }
            if(gamepad1.aWasReleased()){
                robot.drive.setRobotPosition(RESET_X,RESET_Y,RESET_H);
            }

            // Drawing stuff on the field
            TelemetryPacket packet = new TelemetryPacket();
            dashboard.sendTelemetryPacket(packet);

            //robot.outputLLPose();
            telemetry.update();
        }

    }

    public void testGoalSide() {
        if(gamepad1.dpadUpWasReleased()){
            long startTime = System.currentTimeMillis();
            robot.goalSide(USE_ARMS);
            robot.drive.stopMotors();
            elapsedTime = System.currentTimeMillis()-startTime;
        }
    }

    public void testHumanSide() {
        long startTime = System.currentTimeMillis();
        if(gamepad1.dpadUpWasReleased()){
            //robot.goalSide(USE_ARMS);
            robot.drive.stopMotors();
            elapsedTime = System.currentTimeMillis()-startTime;
        }
    }

}
