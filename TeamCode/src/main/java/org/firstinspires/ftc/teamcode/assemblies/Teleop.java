package org.firstinspires.ftc.teamcode.assemblies;


import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;

//import org.firstinspires.ftc.teamcode.libs.Blinkin;

import org.firstinspires.ftc.teamcode.libs.teamUtil;

@Config
@TeleOp(name = "Teleop", group = "LinearOpMode")
public class Teleop extends LinearOpMode {

    Robot robot;

    boolean endgame = false;
    
    
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
        robot.initialize();
        //robot.initCV(enableLiveView);// TODO: false for competition

        robot.drive.setHeading(0);
        teamUtil.justRanAuto = false;
        teamUtil.justRanCalibrateRobot = false;

        robot.calibrate();
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
        }
        
         */
            //TODO: FIX ALL STATE MANAGEMENT
            waitForStart();


            while (opModeIsActive()) {
                ////////// Drive
                // Right bumper resets Heading
                if (gamepad1.rightStickButtonWasReleased() && gamepad1.leftStickButtonWasReleased()) {
                    robot.drive.setRobotPosition(0, 0, 0);
                }
                //robot.drive.setHeldHeading(robot.drive.getGoalHeading());
                robot.drive.universalDriveJoystickV2(
                        gamepad1.left_stick_y * (teamUtil.alliance== teamUtil.Alliance.BLUE ? 1 : -1),
                        gamepad1.left_stick_x * (teamUtil.alliance== teamUtil.Alliance.BLUE ? -1 : 1),
                        gamepad1.right_stick_x,
                        gamepad1.right_trigger > .5,gamepad1.left_trigger > .5,
                        robot.drive.getHeadingODO());

                if (gamepad1.leftBumperWasReleased()) {
                    robot.intake.calibrate();
                    robot.intake.elevator.setVelocity(0);
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
                if(gamepad1.dpadUpWasReleased()){
                    robot.shooter.setShootSpeed(Shooter.SHOOTER_FAR_VELOCITY);
                    //robot.setFootPos(Robot.FOOT_EXTENDED_POS);
                }if(gamepad1.dpadDownWasReleased()){
                    robot.shooter.stopShooter();
                    //robot.setFootPos(Robot.FOOT_CALIBRATE_POS);
                }
                if(gamepad1.dpadRightWasPressed()){
                    robot.intake.unloadToShooter(true);
                    //robot.shooter.aim(robot.shooter.currentAim()+.01);
                }if(gamepad1.dpadLeftWasPressed()){
                    //robot.shooter.aim(robot.shooter.currentAim()-.01);
                }if(gamepad1.rightBumperWasPressed()){
                    robot.shooter.pusher.pushN(1, AxonPusher.RTP_MAX_VELOCITY, 1500);
                }

                if(gamepad2.dpadUpWasReleased()){
                    robot.shooter.setShootSpeed(Shooter.SHOOTER_FAR_VELOCITY);
                }if(gamepad2.dpadDownWasReleased()){
                    robot.shooter.stopShooter();
                }if(gamepad2.dpadRightWasPressed()){
                    robot.shooter.aim(robot.shooter.currentAim()+.01);
                }if(gamepad2.dpadLeftWasPressed()){
                    robot.shooter.aim(robot.shooter.currentAim()-.01);
                }if(gamepad2.aWasPressed()){
                    robot.shootAllArtifacts();
                }if(gamepad2.xWasPressed()){
                    robot.intake.elevatorToFlippers();
                }if(gamepad2.leftBumperWasPressed()){
                    robot.intake.intakeIn();
                }if(gamepad2.rightBumperWasPressed()){
                    robot.intake.intakeStop();
                }

                robot.outputTelemetry();

                robot.drive.loop();
                //telemetry.addData("Left Hang Velocity", robot.hang.hang_Left.getVelocity());
                //telemetry.addData("Right Hang Velocity", robot.hang.hang_Right.getVelocity());
                //telemetry.addLine("Low Bucket Toggled: " + lowBucketToggle);
                //telemetry.addLine("Hang Manual: " + hangManualControl);

                telemetry.update();
            }
        }
    }
}
