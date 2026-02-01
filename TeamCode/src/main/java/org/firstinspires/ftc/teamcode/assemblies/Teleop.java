package org.firstinspires.ftc.teamcode.assemblies;


import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

//import org.firstinspires.ftc.teamcode.libs.Blinkin;

import org.firstinspires.ftc.teamcode.libs.Blinkin;
import org.firstinspires.ftc.teamcode.libs.teamUtil;

@Config
@TeleOp(name = "Teleop", group = "LinearOpMode")
public class Teleop extends LinearOpMode {

    Robot robot;

    boolean shootingMode = false;
    
    /*
    public void loopRunTimeCalculate(int loopNumber,boolean button){
        long startTime=0;
        long endTime=0;
        int loopAmount=0;
        int buttonPressNumber=0;
        if(button&&buttonPressNumber==0){
            buttonPressNumber=1;
            startTime=System.currentTimeMillis();
        }
        if(button&&buttonPressNumber==1){
            loopAmount=loopNumber;
            endTime=System.currentTimeMillis();
        }
        long totalRunTime = endTime-startTime;
        long loopTime = totalRunTime/loopAmount;

        //TODO: take away (only for testing)
        telemetry.addLine("Button Press Number" + buttonPressNumber);

        teamUtil.log("Loop Time" + loopTime);
    }

     */


    public void runOpMode() {
        FtcDashboard dashboard = FtcDashboard.getInstance();
        //FtcDashboard.setDrawDefaultField(false); // enable to eliminate field drawing
        telemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry()); // write telemetry to Driver Station and Dashboard
        teamUtil.init(this);


        robot = new Robot();
        robot.initialize(true);
        robot.shooter.flywheelNormal();
        //robot.initCV(enableLiveView);// TODO: false for competition

        if (teamUtil.justRanAuto) {
            robot.drive.setRobotPosition(teamUtil.cacheX, teamUtil.cacheY, teamUtil.cacheHeading);
        } else {
            robot.drive.setHeading(0);

            robot.calibrate();
        }
        teamUtil.justRanAuto = false;
        teamUtil.justRanCalibrateRobot = false;
        boolean endgameMode = false;
        boolean limelightOverride = false;
        boolean footUp = true;

        if (Intake.KEEP_INTAKE_DETECTOR_SNAPSCRIPT_RUNNING) { // If we are going to keep this running the whole time
            robot.startLimeLightPipeline(Robot.PIPELINE_INTAKE); // start it right away
        }
        // TODO: Consider a pause here to avoid spamming log

        telemetry.addLine("Ready to start");
        telemetry.addLine("ALLIANCE : " + teamUtil.alliance);
        telemetry.update();


        while (!opModeIsActive()) {
        /*   
            if(driverGamepad.wasRightBumperPressed()||driverGamepad.wasLeftBumperPressed()){
                if(teamUtil.alliance == teamUtil.Alliance.BLUE){
                    teamUtil.alliance = teamUtil.Alliance.RED;
                }else{
                    teamUtil.alliance= teamUtil.Alliance.BLUE;
                }
            }
            telemetry.addLine("Ready to start");
            telemetry.addLine("ALLIANCE : "+ teamUtil.alliance);
            telemetry.update();
            */
        }


        waitForStart();


        while (opModeIsActive()) {
            robot.drive.loop();
            ////////// Drive

            //robot.drive.setHeldHeading(robot.drive.getGoalHeading());
            robot.drive.universalDriveJoystickV3(
                    gamepad1.left_stick_y * (teamUtil.alliance == teamUtil.Alliance.BLUE ? 1 : -1),
                    gamepad1.left_stick_x * (teamUtil.alliance == teamUtil.Alliance.BLUE ? -1 : 1),
                    gamepad1.right_stick_x,
                    gamepad1.right_trigger > .5, gamepad1.left_trigger > .5,
                    robot.drive.getHeadingODO(),
                    shootingMode);

            if (gamepad1.psWasReleased()) {
                if (shootingMode) {
                    shootingMode = false;
                    robot.blinkin.setSignal(Blinkin.Signals.OFF);
                } else {
                    shootingMode = true;
                }
            }

            if (shootingMode) {
                if(!robot.localizer.localizing.get()) {
                    if (robot.canShoot(robot.drive.robotGoalDistance())) { // TODO: is this the right version of this method? Seems like it using old flywheel speed check logic
                        robot.blinkin.setSignal(Blinkin.Signals.NORMAL_WHITE);
                    } else {
                        robot.blinkin.setSignal(Blinkin.Signals.AIMING);
                    }
                }
            }

            if(!endgameMode && !shootingMode) {
                robot.shooter.setShootSpeed(Math.min(robot.shooter.calculateVelocityV2(robot.drive.robotGoalDistance()), Shooter.MAX_IDLE_FLYWHEEL_VELOCITY));
            }

            if (shootingMode) {
                robot.drive.setHeldHeading(robot.drive.robotGoalHeading());
            }
//            if (gamepad1.rightBumperWasPressed() && gamepad1.leftBumperWasPressed()) {
//                if (teamUtil.alliance == teamUtil.Alliance.BLUE) {
//                    teamUtil.alliance = teamUtil.Alliance.RED;
//                } else {
//                    teamUtil.alliance = teamUtil.Alliance.BLUE;
//                }
//            }
//                if(gamepad1.xWasReleased()){
//                    robot.drive.setHeading(0);
//                }
//                if(gamepad1.aWasReleased()){
//                    robot.drive.setRobotPosition(teamUtil.cacheX,teamUtil.cacheY,teamUtil.cacheHeading);
//                }
            if(gamepad1.leftBumperWasReleased()){
                robot.drive.setHeading(0);
            }
            if(gamepad1.rightBumperWasReleased()){
                if (teamUtil.alliance == teamUtil.Alliance.BLUE) {
                    teamUtil.alliance = teamUtil.Alliance.RED;
                } else {
                    teamUtil.alliance = teamUtil.Alliance.BLUE;
                }
            }

            if (gamepad1.dpadUpWasReleased()) {
                robot.drive.setRobotPosition(robot.drive.oQlocalizer.posX_mm, teamUtil.alliance == teamUtil.Alliance.BLUE ? BasicDrive.RED_ALLIANCE_WALL : BasicDrive.BLUE_ALLIANCE_WALL, 0);
            }
            if (gamepad1.dpadDownWasReleased()) {
                robot.drive.setRobotPosition(robot.drive.oQlocalizer.posX_mm, teamUtil.alliance == teamUtil.Alliance.BLUE ? BasicDrive.BLUE_ALLIANCE_WALL : BasicDrive.RED_ALLIANCE_WALL, 0);
            }
            if (gamepad1.dpadRightWasReleased()) {
                gamepad1.rumble(250);
                robot.drive.setRobotPosition(teamUtil.alliance == teamUtil.Alliance.BLUE ? BasicDrive.SCORE_X : BasicDrive.AUDIENCE_X, robot.drive.oQlocalizer.posY_mm, 0);
            }
            if (gamepad1.dpadLeftWasReleased()) {
                robot.drive.setRobotPosition(teamUtil.alliance == teamUtil.Alliance.BLUE ? BasicDrive.AUDIENCE_X : BasicDrive.SCORE_X, robot.drive.oQlocalizer.posY_mm, 0);
            }

            if (gamepad1.yWasReleased()) {
                robot.drive.setHeldHeading(teamUtil.alliance == teamUtil.Alliance.BLUE ? 270 : 90);
            }
            if (gamepad1.aWasReleased()) {
                robot.drive.setHeldHeading(teamUtil.alliance == teamUtil.Alliance.BLUE ? 90 : 270);
            }
            if (gamepad1.bWasReleased()) {
                robot.drive.setHeldHeading(teamUtil.alliance == teamUtil.Alliance.BLUE ? 180 : 0);
            }
            if (gamepad1.xWasReleased()) {
                robot.drive.setHeldHeading(teamUtil.alliance == teamUtil.Alliance.BLUE ? 0 : 180);
            }

            if(gamepad1.shareWasPressed()){
                robot.localizer.localize(2500);
            }

                ////////////// SHOOTER ///////////////////////////

                if (shootingMode) {
                    if (!robot.shooter.superFastShooting.get() && !robot.shootingArtifactColor.get()) {
                        // adjust shooter flywheel and pitch if we are not actively shooting at the moment
                        robot.shooter.adjustShooterV4(robot.drive.robotGoalDistance());
                    }
                }

                if (gamepad2.rightBumperWasReleased()) {
                    robot.resetFlippersAndPusherNoWait(500);
                }
                if (gamepad2.bWasReleased()) {
                    robot.shootArtifactColorNoWait(Intake.ARTIFACT.GREEN);
                }
                if (gamepad2.xWasReleased()) {
                    robot.shootArtifactColorNoWait(Intake.ARTIFACT.PURPLE);
                }

                if (gamepad2.yWasReleased()) {
                    double distance = robot.drive.robotGoalDistance();
                    if (distance < Shooter.MID_DISTANCE_THRESHOLD) { // lock in shooter to current conditions TODO: This may get undone by the teleop loop!
                        robot.shooter.lockShooter(distance);
                    }
                    robot.shooter.pushOneNoWait();
                }

                if (gamepad2.aWasReleased()) {
                    robot.shooter.pusher.reset(false);
                }
                if (gamepad2.right_trigger > .6f && shootingMode) {
                    if (!robot.shooter.superFastShooting.get()) {
                        robot.shootIfCanTeleop(); // blinkin based on the result?
                    }
                }

                ///////////// ENDGAME //////////////////////////////
                if (gamepad1.touchpadWasPressed()) {
                    robot.drive.setHeldHeading(teamUtil.alliance == teamUtil.Alliance.BLUE ? 315 : 45);
                    if (endgameMode) {  // press again to attempt auto align
                        endgameMode = false;
                    } else { // Press once to enter end game mode
                        endgameMode=true;
                        robot.intake.intakeStop();
                        robot.shooter.setShootSpeed(0);
                    }
                }

//                if(gamepad1.psWasReleased()){
//                    if (endgameMode) {  // press again to attempt auto align
//                        robot.alignForLiftNoWait();
//                    } else { // Press once to enter end game mode
//                        endgameMode=true;
//                        robot.intake.intakeStop();
//                        robot.shooter.setShootSpeed(0);
//                    }
//                }
                if (endgameMode) {
                    if (robot.seeLine()) {
                        robot.blinkin.setSignal(Blinkin.Signals.SEE_LINE);
                    } else {
                        robot.blinkin.setSignal(Blinkin.Signals.OFF);
                    }
                }
                if (gamepad1.optionsWasReleased() && endgameMode) {
                    if(footUp) {
                        robot.setFootPos(Robot.FOOT_EXTENDED_POS);
                        robot.intake.intakeStop();
                        robot.shooter.setShootSpeed(0);
                        footUp=false;
                    }else{
                        robot.setFootPos(Robot.FOOT_CALIBRATE_POS);
                        footUp=true;
                    }
                }



                ////////////// INTAKE ////////////////////////////

                if (gamepad2.dpadUpWasReleased()) {
                    robot.intake.getReadyToIntakeNoWait();
                }
                if (gamepad2.dpadDownWasReleased()) {
                    robot.intake.intakeOut();
                }
                if (gamepad2.dpadLeftWasPressed()) {
                    robot.intake.intakeStop();
                }
                if (gamepad2.dpadRightWasPressed()) {
                    robot.intake.intakeStart();
                }
                if (gamepad2.left_trigger > .8) { // Set up for a manual (Human Player) load
                    robot.intake.flippersToTransfer();
                    robot.intake.intakeOut();
                    robot.intake.detectorMode = Intake.DETECTION_MODE.LOADED; // tell detector to focus on loaded position
                    robot.intake.setLoadedArtifacts(Intake.ARTIFACT.PURPLE, Intake.ARTIFACT.PURPLE, Intake.ARTIFACT.PURPLE); // failsafe in case LL not working
                }

                if (gamepad2.leftBumperWasReleased()) {
                    if (robot.intake.servoPositionIs(robot.intake.left_flipper, Intake.FLIPPER_TRANSFER)) {
                        robot.intake.superFastUnloadNoWait(Intake.leftLoad != Intake.ARTIFACT.NONE, Intake.middleLoad != Intake.ARTIFACT.NONE, Intake.rightLoad != Intake.ARTIFACT.NONE);
                    } else {
                        if (limelightOverride) {
                            robot.intake.setIntakeArtifacts(teamUtil.Pattern.PPG); //Manual override, balls unknown
                        }
                        robot.intake.elevatorToShooterFastNoWait(false);
                    }
                }
                if (!limelightOverride) {
                    if (robot.intake.detectorMode == Intake.DETECTION_MODE.INTAKE) {
                        robot.intake.detectIntakeArtifactsV2();
                    } else if (robot.intake.detectorMode == Intake.DETECTION_MODE.LOADED) {
                        robot.intake.detectLoadedArtifactsV2();
                    }
                }

                if (gamepad2.backWasPressed()) {
                    robot.intake.elevatorToFlippersV2NoWait(true); // use loaded detector
                }

                robot.intake.signalArtifacts();

                robot.outputTelemetry();

                if (gamepad2.optionsWasReleased()) {
                    if (!limelightOverride) {
                        limelightOverride = true;
                    } else {
                        limelightOverride = false;
                        robot.intake.resetIntakeDetector();
                    }
                }


                //telemetry.addData("Left Hang Velocity", robot.hang.hang_Left.getVelocity());
                //telemetry.addData("Right Hang Velocity", robot.hang.hang_Right.getVelocity());
                //telemetry.addLine("Low Bucket Toggled: " + lowBucketToggle);
                //telemetry.addLine("Hang Manual: " + hangManualControl);
                if (endgameMode) {
                    telemetry.addLine("----- END GAME ----");
                }
                telemetry.addLine(String.format("X: %d Y: %d Heading: %.1f", robot.drive.oQlocalizer.posX_mm, robot.drive.oQlocalizer.posY_mm, robot.drive.getHeadingODO()));
                telemetry.addLine("Alliance: " + (teamUtil.alliance == teamUtil.Alliance.RED ? "RED" : "BLUE"));
                //telemetry.addLine("Detector Mode: " + robot.intake.detectorMode + " Left: " + Intake.leftIntake);
                telemetry.update();
            }

            teamUtil.log("shutting down");
            robot.stopLimeLight();

            teamUtil.cacheHeading = robot.drive.getHeadingODO();
            teamUtil.cacheY = robot.drive.oQlocalizer.posY_mm;
            teamUtil.cacheX = robot.drive.oQlocalizer.posX_mm;


        }

    }
