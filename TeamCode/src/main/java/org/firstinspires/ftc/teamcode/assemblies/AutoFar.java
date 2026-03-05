package org.firstinspires.ftc.teamcode.assemblies;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.libs.Blinkin;
import org.firstinspires.ftc.teamcode.libs.teamUtil;

@Config

@Autonomous(name = "AutoFAR", group = "LinearOpMode")
public class AutoFar extends LinearOpMode {

    Robot robot;
    boolean getPatternSet = false;
    long cycle1Time = 0;
    long cycle2Time = 0;
    long cycle3Time = 0;
    long cycle4Time = 0;


    public void runOpMode() {
        FtcDashboard dashboard = FtcDashboard.getInstance();
        telemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry()); // write telemetry to Driver Station and Dashboard
        teamUtil.init(this);
        robot = new Robot();
        robot.initialize(true);
        //robot.initCV(enableLiveView);
        teamUtil.justRanAuto = false;
        teamUtil.justRanCalibrateRobot = false;
        robot.calibrate();
        robot.intake.flippersToTransfer();

        teamUtil.SIDE = teamUtil.Side.HUMAN;

        while (!gamepad1.aWasReleased() && !isStopRequested()) {
            teamUtil.telemetry.addLine("Side: " + teamUtil.SIDE);
            teamUtil.telemetry.addLine("----------------------------------");
            teamUtil.telemetry.addLine("Alliance: " + teamUtil.alliance);
            teamUtil.telemetry.addLine("Press Bumpers To Change Alliance");
            if (gamepad1.rightBumperWasReleased() || gamepad1.leftBumperWasReleased()) {
                if (teamUtil.alliance == teamUtil.Alliance.BLUE) {
                    teamUtil.alliance = teamUtil.Alliance.RED;
                    gamepad1.setLedColor(1,0,0, Gamepad.LED_DURATION_CONTINUOUS);
                } else {
                    teamUtil.alliance = teamUtil.Alliance.BLUE;
                    gamepad1.setLedColor(0,0,1,Gamepad.LED_DURATION_CONTINUOUS);
                }
            }

            teamUtil.telemetry.update();
        }
        if (isStopRequested()) return;

        while (!gamepad1.aWasReleased() && !isStopRequested()) {
            teamUtil.telemetry.addLine("Side: " + teamUtil.SIDE);
            teamUtil.telemetry.addLine("Alliance: " + teamUtil.alliance);
            teamUtil.telemetry.addLine("----------------------------------");
            teamUtil.telemetry.addLine("Check Limelight functionality");
            teamUtil.telemetry.addLine("Press Bumpers To Reset Lime Light Detector");

            if(!robot.limeLightActive()){
                robot.intake.startDetector();
            }else{
                robot.intake.detectLoadedArtifactsV2();
                //robot.intake.detectIntakeArtifactsV2();
                robot.intake.signalArtifacts();
            }
            if (gamepad1.rightBumperWasReleased() || gamepad1.leftBumperWasReleased()) {
                robot.intake.resetIntakeDetector();
                robot.intake.detectorMode = Intake.DETECTION_MODE.LOADED;
            }
            teamUtil.telemetry.update();
            teamUtil.pause(100); // avoid too much spam in log
        }
        if (isStopRequested()) return;

        while (!gamepad1.aWasReleased() && !isStopRequested()) {
            teamUtil.telemetry.addLine("Side: " + teamUtil.SIDE);
            teamUtil.telemetry.addLine("Alliance: " + teamUtil.alliance);
            teamUtil.telemetry.addLine("----------------------------------");
            teamUtil.telemetry.addLine("GetPatternSet: " + getPatternSet);
            teamUtil.telemetry.addLine("Press Bumpers To Change");
            if (gamepad1.rightBumperWasReleased() || gamepad1.leftBumperWasReleased()) {
                getPatternSet = !getPatternSet;
            }
            teamUtil.telemetry.update();
        }
        if (isStopRequested()) return;

        while (!gamepad1.aWasReleased() && !isStopRequested()) {
            teamUtil.telemetry.addLine("Side: " + teamUtil.SIDE);
            teamUtil.telemetry.addLine("Alliance: " + teamUtil.alliance);
            teamUtil.telemetry.addLine("GetPatternSet: " + getPatternSet);
            teamUtil.telemetry.addLine("----------------------------------");
            teamUtil.telemetry.addLine("Cycle 1 time: "+ cycle1Time +" ms");
            if (gamepad1.dpadUpWasPressed()) {
                cycle1Time += 1000;
            }
            if(gamepad1.dpadDownWasPressed() && cycle1Time > 0){ // stop negative
                cycle1Time -= 1000;
            }
            teamUtil.telemetry.update();
        }
        if (isStopRequested()) return;

        while (!gamepad1.aWasReleased() && !isStopRequested()) {
            teamUtil.telemetry.addLine("Side: " + teamUtil.SIDE);
            teamUtil.telemetry.addLine("Alliance: " + teamUtil.alliance);
            teamUtil.telemetry.addLine("GetPatternSet: " + getPatternSet);
            teamUtil.telemetry.addLine("Cycle 1 time: "+ cycle1Time +" ms");
            teamUtil.telemetry.addLine("----------------------------------");
            teamUtil.telemetry.addLine("Cycle 2 time: "+ cycle2Time +" ms");
            if (gamepad1.dpadUpWasPressed()) {
                cycle2Time += 1000;
            }
            if(gamepad1.dpadDownWasPressed() && cycle2Time > 0){ // stop negative
                cycle2Time -= 1000;
            }
            teamUtil.telemetry.update();
        }
        if (isStopRequested()) return;

        while (!gamepad1.aWasReleased() && !isStopRequested()) {
            teamUtil.telemetry.addLine("Side: " + teamUtil.SIDE);
            teamUtil.telemetry.addLine("Alliance: " + teamUtil.alliance);
            teamUtil.telemetry.addLine("GetPatternSet: " + getPatternSet);
            teamUtil.telemetry.addLine("Cycle 1 time: "+ cycle1Time +" ms");
            teamUtil.telemetry.addLine("Cycle 2 time: "+ cycle2Time +" ms");
            teamUtil.telemetry.addLine("----------------------------------");
            teamUtil.telemetry.addLine("Cycle 3 time: "+ cycle3Time +" ms");
            if (gamepad1.dpadUpWasPressed()) {
                cycle3Time += 1000;
            }
            if(gamepad1.dpadDownWasPressed() && cycle3Time > 0){ // stop negative
                cycle3Time -= 1000;
            }
            teamUtil.telemetry.update();
        }
        if (isStopRequested()) return;

        while (!gamepad1.aWasReleased() && !isStopRequested()) {
            teamUtil.telemetry.addLine("Side: " + teamUtil.SIDE);
            teamUtil.telemetry.addLine("Alliance: " + teamUtil.alliance);
            teamUtil.telemetry.addLine("GetPatternSet: " + getPatternSet);
            teamUtil.telemetry.addLine("Cycle 1 time: "+ cycle1Time +" ms");
            teamUtil.telemetry.addLine("Cycle 2 time: "+ cycle2Time +" ms");
            teamUtil.telemetry.addLine("Cycle 3 time: "+ cycle3Time +" ms");
            teamUtil.telemetry.addLine("----------------------------------");
            teamUtil.telemetry.addLine("Cycle 4 time: "+ cycle4Time +" ms");
            if (gamepad1.dpadUpWasPressed()) {
                cycle4Time += 1000;
            }
            if(gamepad1.dpadDownWasPressed() && cycle4Time > 0){ // stop negative
                cycle4Time -= 1000;
            }
            teamUtil.telemetry.update();
        }
        if (isStopRequested()) return;


        while (!gamepad1.aWasReleased() && !isStopRequested()) {
            teamUtil.telemetry.addLine("Side: " + teamUtil.SIDE);
            teamUtil.telemetry.addLine("Alliance: " + teamUtil.alliance);
            teamUtil.telemetry.addLine("GetPatternSet: " + getPatternSet);
            teamUtil.telemetry.addLine("Cycle 1 time: "+ cycle1Time +" ms");
            teamUtil.telemetry.addLine("Cycle 2 time: "+ cycle2Time +" ms");
            teamUtil.telemetry.addLine("Cycle 3 time: "+ cycle3Time +" ms");
            teamUtil.telemetry.addLine("Cycle 4 time: "+ cycle4Time +" ms");
            teamUtil.telemetry.addLine("----------------------------------");
            teamUtil.telemetry.addLine("Press A to Localize");
            teamUtil.telemetry.update();
        }
        if (isStopRequested()) return;
        robot.setStartLocalizedPosition();

        while (!isStarted()) {
            teamUtil.telemetry.addLine("Side: " + teamUtil.SIDE);
            teamUtil.telemetry.addLine("Alliance: " + teamUtil.alliance);
            teamUtil.telemetry.addLine("GetPatternSet: " + getPatternSet);
            teamUtil.telemetry.addLine("Cycle 1 time: "+ cycle1Time +" ms");
            teamUtil.telemetry.addLine("Cycle 2 time: "+ cycle2Time +" ms");
            teamUtil.telemetry.addLine("Cycle 3 time: "+ cycle3Time +" ms");
            teamUtil.telemetry.addLine("Cycle 4 time: "+ cycle4Time +" ms");
            teamUtil.telemetry.addLine("----------------------------------");
            teamUtil.telemetry.addLine("READY TO GO");
            teamUtil.telemetry.update();
        }
        robot.intake.setLoadedArtifacts(Intake.ARTIFACT.NONE, Intake.ARTIFACT.NONE, Intake.ARTIFACT.NONE);
        robot.intake.signalArtifacts();
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

            robot.humanSide(true, false, getPatternSet, cycle1Time, cycle2Time, cycle3Time, cycle4Time);

            robot.drive.stopMotors();
            robot.drive.waitForRobotToStop(1000);
            long endTime = System.currentTimeMillis();
            long elapsedTime = endTime - startTime;
            teamUtil.log("Elapsed Auto Time Without Wait At End: " + elapsedTime);
            robot.blinkin.setSignal(Blinkin.Signals.OFF);
            robot.stopLimeLight();
            //while (opModeIsActive()) { }// don't kill opMode until the last possible moment to allow other threads to finish
//            teamUtil.pause(500); related to line above?
            robot.drive.loop();

            teamUtil.cacheHeading = robot.drive.getHeadingODO();
            teamUtil.cacheY = robot.drive.oQlocalizer.posY_mm;
            teamUtil.cacheX = robot.drive.oQlocalizer.posX_mm;
            teamUtil.justRanAuto = true; // avoid recalibration at start of teleop
        }
    }

}
