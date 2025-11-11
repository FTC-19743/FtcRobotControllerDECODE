package org.firstinspires.ftc.teamcode.testCode;

import static org.firstinspires.ftc.teamcode.testCode.CalibrateDrive.Ops.Test_Power_Consumption;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.assemblies.BasicDrive;

import org.firstinspires.ftc.teamcode.assemblies.Robot;
import org.firstinspires.ftc.teamcode.libs.teamUtil;

@Config
@TeleOp(name = "Calibrate Drive", group = "Test Code")
public class CalibrateDrive extends LinearOpMode {

    Robot robot;
    BasicDrive drive;
    public static int testVelocity = 1000;
    public static float testPower = 1f;
    public static int testEndVelocity = 0;
    public static float testEndPower = .3f;

    public static int testDistance = 100;
    public static boolean powerBraking = false;
    public static double HEADING = 0;
    public static double HELD_HEADING = 0;
    public static int SECONDS = 1;
    public static int botX = 72;
    public static int botY = 72;
    public static float botHeading = 0;
    public static boolean botEndInDeadband = false;

    public static int TELEPORT_X = 0;
    public static int TELEPORT_Y = 0;
    public static int TELEPORT_H =  0;


    public enum Ops {Test_Wiring,
        Test_Rot_Bug,
        Test_Power_Consumption,
        test_MoveToHolding,
        Move_No_Acc_Heading,
        Move_Power_No_Acc_Heading,
        Move_No_Acc_With_Heading,
        Move_Power_No_Acc_With_Heading,
        Find_Max_Forward,
        Find_Max_Left,
        Brake_Test_Forward,
        Brake_Test_Right,
        Reverse_Test,
        Reverse_Test2,
        Test_Spins,
        Test_Move_CMs,
        Test_Holding_Target,
        Test_Holding_Target2,
        Test_Move_To,
        Test_Move_To2,
        Move_CMs_Test,
        Move_Encoder_Target_Test,
        Methods_Test};
    public static Ops AAOP = Ops.Test_Wiring;




    private void doAction() {
        switch (AAOP) {
            case Test_Wiring : testDriveMotorWiring();break;
            case Move_No_Acc_Heading : moveNoAccelerateNoHeadingControl();break;
            case Move_Power_No_Acc_Heading : movePowerNoAccelerateNoHeadingControl();break;
            case Move_No_Acc_With_Heading : moveNoAccelerateWithHeadingControl();break;
            case Move_Power_No_Acc_With_Heading : movePowerNoAccelerateWithHeadingControl();break;
            case Find_Max_Forward : drive.findMaxVelocity(testDistance);break;
            case Find_Max_Left : drive.findMaxStrafeVelocity(testDistance);break;
            case Move_CMs_Test : goForADriveCMs();break;
            case Test_Move_To : testMoveTo();break;
            case Move_Encoder_Target_Test : goForADriveTarget();break;
            case Brake_Test_Forward : brakeTestForward();break;
            case Brake_Test_Right : brakeTestRight();break;
            case Reverse_Test:  reverseTest();break;
            case Reverse_Test2: reverseTest2();break;


        }
    }
    @Override
    public void runOpMode() throws InterruptedException {
        FtcDashboard dashboard = FtcDashboard.getInstance();
        //FtcDashboard.setDrawDefaultField(false); // enable to eliminate field drawing
        telemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry()); // write telemetry to Driver Station and Dashboard
        teamUtil.init(this);
        teamUtil.alliance = teamUtil.Alliance.BLUE;
        teamUtil.SIDE=teamUtil.Side.GOAL;

        robot = new Robot();
        robot.initialize(false);
        //robot.initCV(enableLiveView);// TODO: false for competition

        robot.drive.setHeading(0);
        teamUtil.justRanAuto = false;
        teamUtil.justRanCalibrateRobot = false;

        robot.calibrate();
        drive = robot.drive;

        telemetry.addLine("Ready to start");
        telemetry.addLine("ALLIANCE : " + teamUtil.alliance);
        telemetry.update();

        waitForStart();

        if (isStopRequested()) {
            return;
        }

        while (opModeIsActive()) {
            
            drive.loop(); // keep odometry data up to date
            telemetry.addLine("Alliance: " + teamUtil.alliance);
            drive.driveMotorTelemetry();

            if (gamepad1.left_stick_button) {
                teamUtil.logSystemHealth();
            }
            // Left bumper toggles Alliance
            if (gamepad1.leftBumperWasReleased()) {
                if (teamUtil.alliance == teamUtil.Alliance.RED) {
                    teamUtil.alliance = teamUtil.Alliance.BLUE;
                } else {
                    teamUtil.alliance = teamUtil.Alliance.RED;
                }
            }
            // Right bumper resets Heading
            if (gamepad1.rightBumperWasReleased()) {
                drive.setHeading(0);
            }
            if (gamepad1.rightStickButtonWasReleased()&&gamepad1.leftStickButtonWasReleased()) {
                drive.setRobotPosition(TELEPORT_X,TELEPORT_Y, TELEPORT_H);
            }

            // X makes the selected action happen
            if (gamepad1.xWasReleased()) {
                telemetry.addLine("Do Action");
                doAction();
            }
            if (AAOP==Ops.Test_Spins){
                testSpins();
            } else if (AAOP==Ops.Test_Move_CMs){
                testMoveCMs();
            } else if (AAOP==Ops.Test_Holding_Target){
                testHoldingTarget();
            } else if (AAOP==Ops.Test_Holding_Target2){
                testHoldingTarget2();
            }else if (AAOP==Ops.Test_Move_To){
                testMoveTo();
            } else if (AAOP==Ops.Test_Move_To2){
                testMoveTo2();
            } else if (AAOP==Ops.Reverse_Test){
                reverseTest();
            } else if (AAOP==Ops.Test_Rot_Bug){
                testRotBug();
            } else if (AAOP== Test_Power_Consumption) {
                testPowerConsumption();
            } else if (AAOP==Ops.test_MoveToHolding) {
                testMoveToHolding();
            }else if (AAOP==Ops.Methods_Test) {
                testNewMethods();
        }
            drive.universalDriveJoystickV2(
                    gamepad1.left_stick_y * (teamUtil.alliance== teamUtil.Alliance.BLUE ? 1 : -1),
                    gamepad1.left_stick_x * (teamUtil.alliance== teamUtil.Alliance.BLUE ? -1 : 1),
                    gamepad1.right_stick_x,
                    gamepad1.right_trigger > .5,gamepad1.left_trigger > .5,
                    drive.getHeadingODO());

            // Drawing stuff on the field
            TelemetryPacket packet = new TelemetryPacket();
            packet.fieldOverlay().drawImage("/images/BatBot.jpg", botX, botY, 18, 18);
            //packet.fieldOverlay().drawImage("/dash/powerplay.png", 0, 0, 144, 144);
            dashboard.sendTelemetryPacket(packet);


            // Graphing stuff and putting stuff in telemetry
            //telemetry.addData("Item", data)); // Anything written like this can be graphed against time.  Multiple items can be graphed together
            telemetry.addData("Velocity", 0);
            telemetry.addData("Encoder", 0);
            telemetry.addData("Current Velocity", 0);
            telemetry.addData("Motor Velocity", 0);
            telemetry.addData("YVelocity", 0);
            telemetry.addData("YEncoder", 0);
            telemetry.addData("XVelocity", 0);
            telemetry.addData("XEncoder", 0);

            telemetry.update();
            //sleep(20);
        }
    }
    public void testNewMethods() {

    }

    public void testMoveToHolding() {
        if (gamepad1.dpad_up) {
            drive.moveToXHoldingStrafe(testPower, botX, botY, (int) HEADING, (int) HELD_HEADING, testEndVelocity, null, 0, 3000);
            drive.stopMotors();
        }
    }


    public void testRotBug() {
        if (gamepad1.dpad_up) {
            BasicDrive.newDrivePowerAlgo = true;
        }
        if (gamepad1.dpad_down) {
            BasicDrive.newDrivePowerAlgo = false;
        }
        if (gamepad1.x) {
            float strafeMaxDeclination = BasicDrive.STRAFE_MAX_DECLINATION; // save for later
            BasicDrive.STRAFE_MAX_DECLINATION = 35;
            drive.strafeHoldingStraightPower(testPower, 112 - 300, 500, 0, null, 0, 5000);
            BasicDrive.STRAFE_MAX_DECLINATION = strafeMaxDeclination;
            drive.stopMotors();
        }
        telemetry.addLine("New Algo: " + BasicDrive.newDrivePowerAlgo);
    }

    public void testPowerConsumption () {
        if (gamepad1.dpad_up) {
            drive.straightHoldingStrafePower(testPower, botX, 0, 0);
            drive.straightHoldingStrafePower(testPower, 0, 0, 0);
        } if (gamepad1.dpad_down) {
            drive.straightHoldingStrafePower(testPower, botX, 0, 0);
            drive.setMotorsBrake();
            drive.stopMotors();
            teamUtil.pause(POWER_REVERSE_BRAKING_PAUSE1);
            drive.straightHoldingStrafePower(testPower, 0, 0, 0);
            drive.setMotorsBrake();
            drive.stopMotors();
            teamUtil.pause(POWER_REVERSE_BRAKING_PAUSE1);
        } if (gamepad1.dpad_right) {
            drive.straightHoldingStrafePower(testPower, botX, 0, 0);
            drive.setMotorsActiveBrake();
            teamUtil.pause(POWER_REVERSE_BRAKING_PAUSE1);
            drive.setMotorsWithEncoder();
            drive.straightHoldingStrafePower(testPower, 0, 0, 0);
            drive.setMotorsActiveBrake();
            teamUtil.pause(POWER_REVERSE_BRAKING_PAUSE1);
            drive.setMotorsWithEncoder();
        } else if (gamepad1.dpad_left) {
            drive.straightHoldingStrafePower(testPower, botX-400, 0, 0);
            drive.strafeHoldingStraightPower(testPower, botY-300, botX, 0);
            drive.straightHoldingStrafePower(testPower, 400, botY, 0);
            drive.strafeHoldingStraightPower(testPower, 300, 0, 0);
        } else {
            drive.stopMotors();
        }
    }


    public static int REVERSE_BRAKING_PAUSE1= 150;
    public void reverseTest() {
        if (gamepad1.dpad_up) {
            drive.straightHoldingStrafeEncoder(BasicDrive.MAX_VELOCITY, 300, 0,0,BasicDrive.MAX_VELOCITY, false, null,0,4000);
            drive.stopMotors();
            teamUtil.pause(REVERSE_BRAKING_PAUSE1);
            drive.lastVelocity = BasicDrive.MAX_VELOCITY; // make motors start at full power
            drive.straightHoldingStrafeEncoder(BasicDrive.MAX_VELOCITY, 1100, 0,0,0, false, null,0,4000);
        }
        if (gamepad1.dpad_down) {
            drive.straightHoldingStrafeEncoder(BasicDrive.MAX_VELOCITY, 300, 0,0,BasicDrive.MAX_VELOCITY, false, null,0,4000);
            drive.lastVelocity = BasicDrive.MAX_VELOCITY; // make motors start at full power
            drive.straightHoldingStrafeEncoder(BasicDrive.MAX_VELOCITY, 1100, 0,0,0, false, null,0,4000);
        }

    }

    public static int POWER_REVERSE_BRAKING_PAUSE1= 150;
    public void reverseTest2() {
        long doneTime = System.currentTimeMillis() + (int)(SECONDS*1000);
        drive.loop();
        while (System.currentTimeMillis() < doneTime) {
            drive.driveMotorsHeadingsPower(HEADING, HELD_HEADING,  testPower );
            drive.loop();
        }
        doneTime = System.currentTimeMillis() + (int)(SECONDS*1000);
        while (System.currentTimeMillis() < doneTime) {
            drive.driveMotorsHeadingsPower(HEADING+180, HELD_HEADING,  testPower );
            drive.loop();
        }
        drive.stopMotors();
    }


    public void testDriveMotorWiring() {
        drive.setMotorVelocities(drive.MIN_START_VELOCITY,0,0,0);
        teamUtil.pause(1000);
        drive.setMotorVelocities(0,drive.MIN_START_VELOCITY,0,0);
        teamUtil.pause(1000);
        drive.setMotorVelocities(0,0,drive.MIN_START_VELOCITY,0);
        teamUtil.pause(1000);
        drive.setMotorVelocities(0,0,0,drive.MIN_START_VELOCITY);
        teamUtil.pause(1000);
        drive.stopMotors();
    }

    public void moveNoAccelerateNoHeadingControl () {
        //drive.driveMotorsHeadings(HEADING, drive.getHeading(), testVelocity);
        drive.driveMotorsHeadings(HEADING, drive.getHeadingODO(), testVelocity);
        teamUtil.pause(SECONDS*1000);
        drive.stopMotors();
    }

    public void movePowerNoAccelerateNoHeadingControl () {
        drive.driveMotorsHeadingsPower(HEADING, drive.getHeadingODO(), testPower);
        teamUtil.pause(SECONDS*1000);
        drive.stopMotors();
    }


    public void moveNoAccelerateWithHeadingControl () {
        long doneTime = System.currentTimeMillis() + (int)(SECONDS*1000);
        drive.loop();
        double heldHeading = drive.getHeadingODO();

        while (System.currentTimeMillis() < doneTime) {
            drive.driveMotorsHeadings(HEADING, heldHeading, testVelocity);
            drive.loop();
        }
        drive.stopMotors();
    }

    public void movePowerNoAccelerateWithHeadingControl () {
        long doneTime = System.currentTimeMillis() + (int)(SECONDS*1000);
        drive.loop();
        while (System.currentTimeMillis() < doneTime) {
            drive.driveMotorsHeadingsPower(HEADING, HELD_HEADING,  testPower );
            drive.loop();
        }
        drive.stopMotors();
    }



    public void testSpins() {
        if (gamepad1.dpad_up) {
            drive.spinToHeading(0);
        }
        if (gamepad1.dpad_down) {
            drive.spinToHeading(180);
        }
        if (gamepad1.dpad_left) {
            drive.spinToHeading(270);
        }
        if (gamepad1.dpad_right) {
            drive.spinToHeading(90);
        }
    }

    public void testMoveCMs() {
        if (gamepad1.dpad_up) {
            /*
            drive.setHeading(0);
            drive.moveCm(testVelocity, testDistance, 0, 0,0);
             */
            drive.moveCm(BasicDrive.MAX_VELOCITY,35,135,315,0);

        } else if (gamepad1.dpad_down) {
            drive.setHeading(0);
            drive.moveCm(testVelocity, testDistance, 180, 0,0);
        } else if (gamepad1.dpad_left) {
            drive.setHeading(0);
            drive.moveCm(testVelocity, testDistance, 90, 0,0);
        } else if (gamepad1.dpad_right) {
            drive.setHeading(0);
            drive.moveCm(testVelocity, testDistance, 270, 0,0);
        } else if (gamepad1.y) {
            drive.setHeading(0);
            drive.moveCm(testVelocity, testDistance, 45, 0,0);
        } else if (gamepad1.a) {
            drive.setHeading(0);
            drive.moveCm(testVelocity, testDistance, 225, 0,0);
        } else if (gamepad1.x) {
            drive.setHeading(0);
            drive.moveCm(testVelocity, testDistance, 135, 0,0);
        } else if (gamepad1.b) {
            drive.setHeading(0);
            drive.moveCm(testVelocity, testDistance, 315, 0,0);
        }
    }



    public void testHoldingTarget() {
        if (gamepad1.dpad_up) {
            //drive.setHeading(0);
            drive.loop();
            double startForward = drive.oQlocalizer.posX_mm;
            double forwardTarget = (long) (startForward + testDistance);
            double startStrafe = drive.oQlocalizer.posY_mm+botY;
            drive.straightHoldingStrafeEncoder(testVelocity, forwardTarget,startStrafe,0,testEndVelocity, powerBraking, null,0,3000);
            if (testEndVelocity > 0) {
                teamUtil.pause(1000);
                drive.stopMotors();
            }
        }
        if (gamepad1.dpad_down) {
            drive.setHeading(0);
            drive.loop();
            double startForward = drive.oQlocalizer.posX_mm;
            double forwardTarget = (long) (startForward - testDistance);
            double startStrafe = drive.oQlocalizer.posY_mm+botY;
            drive.straightHoldingStrafeEncoder(testVelocity, forwardTarget,startStrafe,0,testEndVelocity, powerBraking, null,0,3000);
            if (testEndVelocity > 0) {
                teamUtil.pause(1000);
                drive.stopMotors();
            }
        }
        if (gamepad1.dpad_right) {
            //drive.setHeading(0);
            drive.loop();
            double startStrafe = drive.oQlocalizer.posY_mm;
            double strafeTarget = (long) (startStrafe - testDistance);
            double startForward = drive.oQlocalizer.posX_mm+botX;
            drive.strafeHoldingStraightEncoder(testVelocity, strafeTarget,startForward,0,0,null,0,3000);
        }
        if (gamepad1.dpad_left) {
            drive.setHeading(0);
            drive.loop();
            double startStrafe = drive.oQlocalizer.posY_mm;
            double strafeTarget = (long) (startStrafe + testDistance);
            double startForward = drive.oQlocalizer.posX_mm+botX;
            drive.strafeHoldingStraightEncoder(testVelocity, strafeTarget,startForward,0,0,null,0,3000);
        }
    }


    public void testHoldingTarget2() {
        if (gamepad1.dpad_up) {
            //drive.setHeading(0);
            drive.loop();
            drive.loop();
            double startForward = drive.oQlocalizer.posX_mm;
            double forwardTarget = (long) (startForward + testDistance);
            double startStrafe = drive.oQlocalizer.posY_mm+botY;
            drive.straightHoldingStrafePower(testPower, forwardTarget,startStrafe,0, null,0,3000);
            drive.stopMotors();
        }
        if (gamepad1.dpad_down) {
            drive.setHeading(0);
            drive.loop();
            drive.loop();
            double startForward = drive.oQlocalizer.posX_mm;
            double forwardTarget = (long) (startForward - testDistance);
            double startStrafe = drive.oQlocalizer.posY_mm+botY;
            drive.straightHoldingStrafePower(testPower, forwardTarget,startStrafe,0, null,0,3000);
            drive.stopMotors();
        }
        if (gamepad1.dpad_right) {
            drive.loop();
            drive.loop();
            double startStrafe = drive.oQlocalizer.posY_mm;
            double strafeTarget = (long) (startStrafe - testDistance);
            double startForward = drive.oQlocalizer.posX_mm+botX;
            drive.strafeHoldingStraightPower(testPower, strafeTarget,startForward,0,null,0,3000);
            drive.stopMotors();
        }
        if (gamepad1.dpad_left) {
            drive.loop();
            drive.loop();
            double startStrafe = drive.oQlocalizer.posY_mm;
            double strafeTarget = (long) (startStrafe + testDistance);
            double startForward = drive.oQlocalizer.posX_mm+botX;
            drive.strafeHoldingStraightPower(testPower, strafeTarget,startForward,0,null,0,3000);
            drive.stopMotors();
        }
    }

    public void testMoveTo() {
        if (gamepad1.dpad_up){
            drive.moveTo(testVelocity,botY,botX,botHeading,0,null,0,5000);
        }
        if (gamepad1.dpad_left){
            drive.moveTo(testVelocity,botY,botX,botHeading,testEndVelocity,null,0,botEndInDeadband,5000);
        }
        if (gamepad1.dpad_right){
            drive.moveTo(testVelocity,botY,botX,botHeading,testEndVelocity,null,0,botEndInDeadband,5000);
            drive.setMotorPower(0.3);
            teamUtil.pause(500);
            drive.stopMotors();
        }
        if (gamepad1.x) {
            drive.moveToX(testPower, botX, (int) HEADING, (int) HELD_HEADING);
            drive.stopMotors();
        }
        if (gamepad1.y) {
            drive.moveToY(testPower, botY, (int) HEADING, (int) HELD_HEADING);
            drive.stopMotors();
        }
        if (gamepad1.a) {
            drive.moveToX(testPower, botX, (int) HEADING, (int) HELD_HEADING);
            drive.moveToY(testPower, botY, (int) 270, (int) 270);
            drive.stopMotors();
        }
    }

    public void testMoveTo2() {
        if (gamepad1.dpad_up){
            drive.moveToPower(testPower,botY,botX,0,0,null,0,5000);
            drive.stopMotors();
        }
        if (gamepad1.dpad_left){
            drive.moveToPower(testPower,botY,botX,0,testEndPower,null,0,false,5000);
            drive.stopMotors();
        }
        if (gamepad1.x) {
            drive.setRobotPosition(0,0,0);
            drive.moveToPower(testPower,400,400,45,0,null,0,5000);
            drive.moveToPower(testPower,400,0,90,0,null,0,5000);
            drive.moveToPower(testPower,0,0,0,0,null,0,5000);
        }
        if (gamepad1.y) {
            drive.moveToPower(testPower,botY,botX,0,0,null,0,5000);
            drive.setMotorsActiveBrake();
        }
    }


    public void goForADriveCMs() {
        drive.moveCm(testDistance, 90);
        drive.moveCm(testDistance, 180);
        drive.moveCm(testDistance, 270);
        drive.moveCm(testDistance, 0);

        drive.moveCm(testDistance, 45);
        drive.moveCm(testDistance, 135);
        drive.moveCm(testDistance, 225);
        drive.moveCm(testDistance, 315);
    }


    public void goForADriveTarget() {
        drive.loop();
        long startForward = drive.oQlocalizer.posX_mm;
        long forwardTarget = (long) (startForward + testDistance);
        long startStrafe = drive.oQlocalizer.posY_mm;
        long strafeTarget = (long) (startStrafe + testDistance);
        drive.setHeading(0);

        drive.straightHoldingStrafeEncoder(drive.MAX_VELOCITY, forwardTarget,startStrafe,0,0, false, null,0,3000);
        drive.strafeHoldingStraightEncoder(drive.MAX_VELOCITY, strafeTarget,forwardTarget,0,0,null,0,3000);
        drive.straightHoldingStrafeEncoder(drive.MAX_VELOCITY, startForward,strafeTarget,0,0, false, null,0,3000);
        drive.strafeHoldingStraightEncoder(drive.MAX_VELOCITY, startStrafe,startForward,0,0,null,0,3000);
    }

    public void goForADriveAngleTargets() {
        // TBD: Implement a loop using MoveTo()
    }

    public void brakeTestForward () {
        drive.setHeading(0);
        drive.moveCm(testVelocity   , testDistance  , HEADING, 0,testVelocity);
        drive.setMotorsBrake();
        drive.stopMotors();
        drive.loop();
        long startBrakeTime = System.currentTimeMillis();
        while (System.currentTimeMillis() < startBrakeTime+1000) {
            teamUtil.log("vel:"+drive.oQlocalizer.velX_mmS+":enc:"+drive.oQlocalizer.posX_mm);
            telemetry.addData("XVelocity", drive.oQlocalizer.velX_mmS);
            telemetry.addData("XEncoder", drive.oQlocalizer.posX_mm);
            telemetry.update();
            drive.loop();
        }
    }


    public void brakeTestRight () {
        drive.setHeading(0);
        drive.moveCm(testVelocity   , testDistance  , 270, 0,testVelocity);
        drive.setMotorsBrake();
        drive.stopMotors();
        drive.loop();
        long startBrakeTime = System.currentTimeMillis();
        while (System.currentTimeMillis() < startBrakeTime+1000) {
            telemetry.addData("YVelocity", drive.oQlocalizer.velY_mmS);
            telemetry.addData("YEncoder", drive.oQlocalizer.posY_mm);
            teamUtil.log("vel:"+drive.oQlocalizer.velX_mmS+":enc:"+drive.oQlocalizer.posY_mm);
            telemetry.update();
            drive.loop();
        }
        
    }

    public void TuneFrontBraking () {


    }
    

}
