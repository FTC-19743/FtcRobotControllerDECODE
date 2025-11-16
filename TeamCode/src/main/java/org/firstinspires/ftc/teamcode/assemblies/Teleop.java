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

    boolean endgame = false;
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
        robot.initialize(false);
        //robot.initCV(enableLiveView);// TODO: false for competition

        if(teamUtil.justRanAuto){
            robot.drive.setRobotPosition(teamUtil.cacheX,teamUtil.cacheY,teamUtil.cacheHeading);
        }else{
            robot.drive.setHeading(0);

            robot.calibrate();
        }
        teamUtil.justRanAuto = false;
        teamUtil.justRanCalibrateRobot = false;


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
        

            //TODO: FIX ALL STATE MANAGEMENT
            waitForStart();


            while (opModeIsActive()) {
                robot.drive.loop();
                ////////// Drive

                //robot.drive.setHeldHeading(robot.drive.getGoalHeading());
                robot.drive.universalDriveJoystickV3(
                        gamepad1.left_stick_y * (teamUtil.alliance== teamUtil.Alliance.BLUE ? 1 : -1),
                        gamepad1.left_stick_x * (teamUtil.alliance== teamUtil.Alliance.BLUE ? -1 : 1),
                        gamepad1.right_stick_x,
                        gamepad1.right_trigger > .5,gamepad1.left_trigger > .5,
                        robot.drive.getHeadingODO(),
                        shootingMode);

                if(gamepad1.yWasReleased()){
                    if(shootingMode == true){
                        robot.intake.startDetector(false);
                        shootingMode = false;
                        robot.shooter.setShootSpeed(robot.shooter.IDLE_FLYWHEEL_VELOCITY);
                    }else{
                        robot.intake.stopDetector();
                        shootingMode = true;

                    }
                }
                if(shootingMode){
                    if(robot.canShoot()){
                        robot.blinkin.setSignal(Blinkin.Signals.READY_TO_SHOOT);
                    }else{
                        robot.blinkin.setSignal(Blinkin.Signals.AIMING);
                    }
                }
                if(shootingMode){
                    robot.drive.setHeldHeading(robot.drive.robotGoalHeading());
                }
                if(gamepad1.xWasReleased()){
                    robot.drive.setHeading(0);
                }
                if(gamepad1.aWasReleased()){
                    robot.drive.setRobotPosition(teamUtil.cacheX,teamUtil.cacheY,teamUtil.cacheHeading);
                }
                if(gamepad1.dpadUpWasReleased()){
                    robot.drive.setRobotPosition(robot.drive.oQlocalizer.posX_mm,teamUtil.alliance == teamUtil.Alliance.BLUE ? BasicDrive.RED_ALLIANCE_WALL : BasicDrive.BLUE_ALLIANCE_WALL,0);
                }
                if(gamepad1.dpadDownWasReleased()){
                    robot.drive.setRobotPosition(robot.drive.oQlocalizer.posX_mm,teamUtil.alliance == teamUtil.Alliance.BLUE ? BasicDrive.BLUE_ALLIANCE_WALL : BasicDrive.RED_ALLIANCE_WALL,0);
                }
                if(gamepad1.dpadRightWasReleased()){
                    robot.drive.setRobotPosition(teamUtil.alliance == teamUtil.Alliance.BLUE ? BasicDrive.SCORE_X : BasicDrive.AUDIENCE_X, robot.drive.oQlocalizer.posY_mm,0);
                }
                if(gamepad1.dpadLeftWasReleased()){
                    robot.drive.setRobotPosition(teamUtil.alliance == teamUtil.Alliance.BLUE ? BasicDrive.AUDIENCE_X : BasicDrive.SCORE_X, robot.drive.oQlocalizer.posY_mm,0);
                }

                ////////////// SHOOTER ///////////////////////////

                if(gamepad2.rightBumperWasReleased()){
                    robot.shootAllArtifactsNoWait();
                }
                if(gamepad2.bWasReleased()){
                    robot.shootArtifactColorNoWait(Intake.ARTIFACT.GREEN);
                }
                if(gamepad2.xWasReleased()){
                    robot.shootArtifactColorNoWait(Intake.ARTIFACT.PURPLE);
                }
                if(shootingMode){
                    robot.shooter.adjustShooterV2(robot.drive.robotGoalDistance());
                }
                if(gamepad2.yWasReleased()){
                    robot.shooter.pushOne();
                }
                if(gamepad2.aWasReleased()){
                    robot.shooter.pusher.calibrate();
                    robot.shooter.pushOne();
                }


                ///////////// ENDGAME //////////////////////////////

                if(gamepad1.dpadDownWasReleased()){
                    if(endgame){
                        robot.setFootPos(robot.FOOT_EXTENDED_POS);
                    }
                    endgame = true;
                }



                ////////////// INTAKE ////////////////////////////

                if(gamepad2.dpadUpWasReleased()){
                    robot.intake.getReadyToIntakeNoWait();
                }if(gamepad2.dpadDownWasReleased()){
                    robot.intake.intakeOut();
                }if(gamepad2.dpadLeftWasPressed()){
                    robot.intake.intakeStop();
                }if(gamepad2.dpadRightWasPressed()){
                    robot.intake.intakeStart();
                }

                if(gamepad2.leftBumperWasReleased()){
                    robot.intake.elevatorToFlippersV2NoWait();
                }

                robot.outputTelemetry();


                //telemetry.addData("Left Hang Velocity", robot.hang.hang_Left.getVelocity());
                //telemetry.addData("Right Hang Velocity", robot.hang.hang_Right.getVelocity());
                //telemetry.addLine("Low Bucket Toggled: " + lowBucketToggle);
                //telemetry.addLine("Hang Manual: " + hangManualControl);
                telemetry.addLine("ODO X: " + robot.drive.oQlocalizer.posX_mm + " ODO Y: " + robot.drive.oQlocalizer.posY_mm + " ODO Heading: " + robot.drive.getHeadingODO());

                telemetry.update();
            }

        teamUtil.log("shutting down");

        teamUtil.cacheHeading = robot.drive.getHeadingODO();
        teamUtil.cacheY = robot.drive.oQlocalizer.posY_mm;
        teamUtil.cacheX = robot.drive.oQlocalizer.posX_mm;


    }

}
