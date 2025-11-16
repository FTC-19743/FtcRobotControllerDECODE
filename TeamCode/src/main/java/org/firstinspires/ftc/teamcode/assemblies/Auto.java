package org.firstinspires.ftc.teamcode.assemblies;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.libs.Blinkin;
import org.firstinspires.ftc.teamcode.libs.teamUtil;
@Config

@Autonomous(name = "Auto", group = "LinearOpMode")
public class Auto extends LinearOpMode {

    Robot robot;
    boolean shootingMode = false;

    public void runOpMode() {
        FtcDashboard dashboard = FtcDashboard.getInstance();
        telemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry()); // write telemetry to Driver Station and Dashboard
        teamUtil.init(this);
        robot = new Robot();
        robot.initialize(false);
        //robot.initCV(enableLiveView);// TODO: false for competition
        teamUtil.justRanAuto = false;
        teamUtil.justRanCalibrateRobot = false;
        robot.calibrate();
        robot.intake.flippersToTransfer();

        while (!gamepad1.aWasReleased() && !isStopRequested()) {
            teamUtil.telemetry.addLine("Alliance: " + teamUtil.alliance);
            teamUtil.telemetry.addLine("Press Bumpers To Change Alliance");
            if (gamepad1.rightBumperWasReleased() || gamepad1.leftBumperWasReleased()) {
                if (teamUtil.alliance == teamUtil.Alliance.BLUE) {
                    teamUtil.alliance = teamUtil.Alliance.RED;
                } else {
                    teamUtil.alliance = teamUtil.Alliance.BLUE;
                }
            }
            teamUtil.telemetry.update();
        }
        if (isStopRequested()) return;

        while (!gamepad1.aWasReleased() && !isStopRequested()) {
            teamUtil.telemetry.addLine("Alliance: " + teamUtil.alliance);
            teamUtil.telemetry.addLine("----------------------------------");
            teamUtil.telemetry.addLine("Side: " + teamUtil.SIDE);
            teamUtil.telemetry.addLine("Press Bumpers To Change Side");
            if (gamepad1.rightBumperWasReleased() || gamepad1.leftBumperWasReleased()) {
                if (teamUtil.SIDE == teamUtil.Side.GOAL) {
                    teamUtil.SIDE = teamUtil.Side.HUMAN;
                } else {
                    teamUtil.SIDE = teamUtil.Side.GOAL;
                }
            }
            teamUtil.telemetry.update();
        }
        if (isStopRequested()) return;

        teamUtil.telemetry.addLine("Alliance: " + teamUtil.alliance);
        teamUtil.telemetry.addLine("Side: " + teamUtil.SIDE);
        teamUtil.telemetry.addLine("----------------------------------");
        teamUtil.telemetry.addLine("Press A to Localize and start CV");
        teamUtil.telemetry.update();
        while (!gamepad1.aWasReleased() && !isStopRequested()) {}
        if (isStopRequested()) return;
        robot.setStartLocalizedPosition();
        robot.initCV(false);


        while (!gamepad1.aWasReleased() && !isStopRequested()) {
            teamUtil.telemetry.addLine("Alliance: " + teamUtil.alliance);
            teamUtil.telemetry.addLine("Side: " + teamUtil.SIDE);
            robot.drive.localizerTelemetry();
            robot.detectPattern();
            teamUtil.telemetry.addLine("----------------------------------");
            teamUtil.telemetry.addLine("Pattern: " + teamUtil.pattern);
            teamUtil.telemetry.addLine("Press A to Finish Setup");
            teamUtil.telemetry.update();
        }
        if (isStopRequested()) return;

        while (!isStarted()) {
            teamUtil.telemetry.addLine("Alliance: " + teamUtil.alliance);
            teamUtil.telemetry.addLine("Side: " + teamUtil.SIDE);
            robot.drive.localizerTelemetry();
            robot.detectPattern();
            teamUtil.telemetry.addLine("----------------------------------");
            teamUtil.telemetry.addLine("Pattern: " + teamUtil.pattern);
            teamUtil.telemetry.addLine("READY TO GO");
            teamUtil.telemetry.update();
        }

        robot.stopCV();
        if (isStopRequested()) return;

        if(teamUtil.alliance == teamUtil.Alliance.RED){
            robot.blinkin.setSignal(Blinkin.Signals.SINELON_RED);
        }else{
            robot.blinkin.setSignal(Blinkin.Signals.SINELON_BLUE);
        }

        waitForStart(); // TODO: Do we need this?
        teamUtil.inInitialization=false;
        //robot.blinkin.setSignal(Blinkin.Signals.OFF);
        if(!isStopRequested()) {
            long startTime = System.currentTimeMillis();
            //teamUtil.pause(delay);

            if (teamUtil.SIDE == teamUtil.Side.GOAL) {
                robot.goalSide(true);
            } else {
                robot.humanSide(true);
            }
            robot.drive.stopMotors();
            robot.drive.waitForRobotToStop(1000);
            long endTime = System.currentTimeMillis();
            long elapsedTime = endTime - startTime;
            teamUtil.log("Elapsed Auto Time Without Wait At End: " + elapsedTime);
            robot.blinkin.setSignal(Blinkin.Signals.OFF);
            //while (opModeIsActive()) { }// don't kill opMode until the last possible moment to allow other threads to finish
            teamUtil.pause(500);
            robot.drive.loop();

            teamUtil.cacheHeading = robot.drive.getHeadingODO();
            teamUtil.cacheY = robot.drive.oQlocalizer.posY_mm;
            teamUtil.cacheX = robot.drive.oQlocalizer.posX_mm;
            teamUtil.justRanAuto = true; // avoid recalibration at start of teleop
        }
    }

}
