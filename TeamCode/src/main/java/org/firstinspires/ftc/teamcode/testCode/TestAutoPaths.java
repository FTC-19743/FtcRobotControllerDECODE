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
import org.firstinspires.ftc.teamcode.assemblies.Intake;
import org.firstinspires.ftc.teamcode.assemblies.Robot;
import org.firstinspires.ftc.teamcode.assemblies.Shooter;
import org.firstinspires.ftc.teamcode.libs.teamUtil;

@Config
@TeleOp(name = "TestAutoPaths", group = "Test Code")
public class TestAutoPaths extends LinearOpMode{
    Robot robot;

    static public boolean USE_ARMS = false;
    static public boolean USE_INTAKE_DETECTOR = false;
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
    public static long gateLeaveTime = 0;
    public static int GRAB_VEL = 600;
    public static int GRAB_X = -600;
    public static int GRAB_TIME = 3000;

    @Override
    public void runOpMode() throws InterruptedException {
        FtcDashboard dashboard = FtcDashboard.getInstance();
        //FtcDashboard.setDrawDefaultField(false); // enable to eliminate field drawing
        telemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry()); // write telemetry to Driver Station and Dashboard
        teamUtil.init(this);

        robot = new Robot();
        robot.initialize(true);
        //robot.initCV(enableLiveView);// TODO: false for competition

        robot.drive.setHeading(0);
        teamUtil.justRanAuto = false;
        teamUtil.justRanCalibrateRobot = false;
        teamUtil.alliance = RED;

        robot.calibrate();
        telemetry.addLine("Ready to start");
        telemetry.addLine("ALLIANCE : " + teamUtil.alliance);
        telemetry.update();

        waitForStart();

        if (isStopRequested()) {
            return;
        }

        while (opModeIsActive()) {
            telemetry.addLine("ALLIANCE : " + teamUtil.alliance + " SIDE : " + teamUtil.SIDE + " PATTERN: " + teamUtil.pattern);
            telemetry.addLine("Testing: " + AA_Operation);
            telemetry.addLine("Use Arms: "+ USE_ARMS);
            telemetry.addLine("Use Detector: "+ USE_INTAKE_DETECTOR);
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
            if (gamepad1.right_trigger > .5) {
                while (gamepad1.right_trigger > .5) {}
                USE_ARMS = !USE_ARMS;
            }


            switch (AA_Operation) {
                case Goal_Side : testGoalSide();break;
                case Human_Side : testHumanSide();break;
            }
            if(gamepad1.aWasReleased()){
                robot.drive.setRobotPosition(RESET_X,RESET_Y,RESET_H);
            }
            if(gamepad1.yWasReleased()){
                robot.intake.flippersToTransfer();
                robot.intake.calibrateElevators();
            }

            // Drawing stuff on the field
            TelemetryPacket packet = new TelemetryPacket();
            dashboard.sendTelemetryPacket(packet);

            //robot.outputLLPose();
            telemetry.update();
        }

    }

    public String colorLetter(Intake.ARTIFACT color) {
        if (color == Intake.ARTIFACT.PURPLE) return "P";
        else if (color == Intake.ARTIFACT.GREEN) return "G";
        else return "N";
    }
    public String locationLetter(Intake.Location location) {
        if (location == Intake.Location.LEFT) return "L";
        else if (location ==  Intake.Location.CENTER) return "M";
        else if (location ==  Intake.Location.RIGHT) return "R";
        else return "N";
    }

    public void testGoalSide() {
        if(gamepad1.dpadUpWasReleased()){
            long startTime = System.currentTimeMillis();
            robot.goalSideV2(USE_ARMS, USE_INTAKE_DETECTOR, gateLeaveTime);
            robot.drive.stopMotors();
            elapsedTime = System.currentTimeMillis()-startTime;
            teamUtil.log("---------- Elapsed Time: " + elapsedTime);
        }
        if (gamepad1.dpadDownWasPressed()){
            long startTime = System.currentTimeMillis();
            robot.getMoreBalls();
            elapsedTime = System.currentTimeMillis()-startTime;
            robot.drive.driveMotorsHeadingsFR(0,0,1000);
            teamUtil.pause(500);
            robot.drive.stopMotors();
            robot.intake.intakeStop();
            teamUtil.log("---------- Elapsed Time: " + elapsedTime);
        }
        if (gamepad1.dpadRightWasReleased()) {
            robot.intake.resetIntakeDetector();
        }

        if (gamepad1.dpadLeftWasReleased()){
            Intake.INTAKE_IN_POWER = .9;
            robot.intake.intakeNum=0;
            if (robot.limeLightActive()) {
                robot.intake.resetIntakeDetector();
                teamUtil.pause(100);
            }
            robot.intake.startIntakeDetector();
            robot.intake.getReadyToIntake();
            teamUtil.pause(1000);
            long startTime = System.currentTimeMillis();
            robot.grab3(GRAB_VEL, GRAB_X, GRAB_TIME);
            elapsedTime = System.currentTimeMillis()-startTime;
            teamUtil.log("---------- Elapsed Time: " + elapsedTime);
            teamUtil.pause(250);
            robot.intake.intakeStop();
        }

        if (gamepad1.xWasReleased()) {
            // test shot order logic
            for (teamUtil.Pattern currentPattern : teamUtil.Pattern.values()) {
                teamUtil.pattern = currentPattern;
                for (Intake.ARTIFACT left : Intake.ARTIFACT.values()) {
                    Intake.leftLoad = left;
                    for (Intake.ARTIFACT middle : Intake.ARTIFACT.values()) {
                        Intake.middleLoad = middle;
                        for (Intake.ARTIFACT right : Intake.ARTIFACT.values()) {
                            Intake.rightLoad = right;
                            robot.determineShotOrderAutoPattern();
                            teamUtil.log("Pattern: " + teamUtil.pattern +
                                    " Loaded: " + colorLetter(Intake.leftLoad) + colorLetter(Intake.middleLoad) + colorLetter(Intake.rightLoad) +
                                    " Order: " + locationLetter(robot.shotOrder[1]) + locationLetter(robot.shotOrder[2]) + locationLetter(robot.shotOrder[3]));
                        }
                    }
                }
            }
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
