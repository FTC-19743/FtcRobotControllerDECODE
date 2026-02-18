package org.firstinspires.ftc.teamcode.testCode;



import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLStatus;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;


import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.CameraName;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.robotcore.external.navigation.UnnormalizedAngleUnit;
import org.firstinspires.ftc.teamcode.assemblies.AprilTagLocalizer;
import org.firstinspires.ftc.teamcode.assemblies.AxonPusher;
import org.firstinspires.ftc.teamcode.assemblies.Intake;
import org.firstinspires.ftc.teamcode.assemblies.Robot;
import org.firstinspires.ftc.teamcode.assemblies.Shooter;

import org.firstinspires.ftc.teamcode.libs.teamUtil;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;
import java.util.Locale;

@Config
@TeleOp(name = "Calibrate Arms", group = "Test Code")
public class CalibrateArms extends LinearOpMode {

    Robot robot;
    public static double VelocityIncrement = 100;
    public static boolean adjustShootMode = false;
    public double CurrentPower = 0;
    public double currentRVelocity = 0;
    public double currentLVelocity = 0;
    public static float FLIPPER_TEST_VAL = 0f;
    public static boolean SHOOT_3_AUTO = false;

    double aimerPosition = Shooter.AIMER_CALIBRATE;
    
    public enum Ops {
        Test_CVLocalizer,
        Test_IMU,
        Test_Intake,
        Test_DetectorV2,
        Test_Intake_Detector,
        Test_Elevator,
        Test_Shooter,
        Test_CV,
        Test_Foot,
        Test_PIDF,
        Teleport
    };
    public static Ops AA_Operation = Ops.Test_Shooter;
    public static boolean useCV = false;
    AprilTagLocalizer localizer;

    @Override
    public void runOpMode() throws InterruptedException {
        FtcDashboard dashboard = FtcDashboard.getInstance();
        //FtcDashboard.setDrawDefaultField(false); // enable to eliminate field drawing
        telemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry()); // write telemetry to Driver Station and Dashboard
        teamUtil.init(this);

        robot = new Robot();
        robot.initialize(true);
        robot.drive.setHeading(0);
        teamUtil.justRanAuto = false;
        teamUtil.justRanCalibrateRobot = false;
        teamUtil.alliance = teamUtil.Alliance.RED;

        robot.calibrate();
        if (useCV) {
            telemetry.addLine("Initializing CV");
            telemetry.update();
            initCV();
            telemetry.addLine("starting LiveView");
            telemetry.update();

            visionPortal.resumeLiveView();
        }

        localizer = new AprilTagLocalizer();

        telemetry.addLine("Ready to start");
        telemetry.addLine("ALLIANCE : " + teamUtil.alliance);
        telemetry.addLine("SIDE : " + teamUtil.SIDE);
        telemetry.update();
        
        waitForStart();

        if (isStopRequested()) {
            return;
        }

        robot.intake.startDetector();

        while (opModeIsActive()) {
            telemetry.addLine("MODE : " + AA_Operation);
            telemetry.addLine("ALLIANCE : " + teamUtil.alliance + " SIDE : " + teamUtil.SIDE + "PATTERN: " + teamUtil.pattern);

            if (gamepad1.left_stick_button) {
                teamUtil.logSystemHealth();
            }
            switch (AA_Operation) {
                case Test_IMU : testIMU();break;
                case Test_Intake : testIntake();break;
                case Test_Intake_Detector: testIntakeDetector();break;
                case Test_DetectorV2 : testDetectorV2(); break;
                case Test_Elevator: testElevator();break;
                case Test_Shooter : testShooter();break;
                case Test_CV: testCV();break;
                case Test_Foot : testFoot();break;
                case Test_PIDF: shooterPIDF();break;
                case Teleport : teleport();break;
                case Test_CVLocalizer : testLocalizer(); break;
            }

            // Drawing stuff on the field
            TelemetryPacket packet = new TelemetryPacket();
            dashboard.sendTelemetryPacket(packet);

            //robot.outputLLPose();
            telemetry.update();

            // Graphing stuff and putting stuff in telemetry
            //telemetry.addData("Item", data)); // Anything written like this can be graphed against time.  Multiple items can be graphed together
            //telemetry.addData("Velocity", 0);
            //telemetry.addData("Encoder", 0);
            //telemetry.addData("Current Velocity", 0);
            //telemetry.addData("Motor Velocity", 0);
            //robot.intake.axonSlider.loop();


        }
        robot.stopLimeLight();
    }
    private AprilTagProcessor aprilTag;
    private VisionPortal visionPortal;
    private WebcamName webcamL, webcamF, webcamR;

    boolean continousDetection = false;
    public Pose3D lastPose;
    long lastLoopTime = 0;
    boolean detecting = false;
    public void testLocalizer() {
        robot.drive.loop();
        telemetry.addLine("Loop Time: " + (System.currentTimeMillis() - lastLoopTime));
        telemetry.addLine("Intake Detector: " + detecting);
        telemetry.addLine("VP Setup: " + localizer.visionPortalRunning + " Streaming: " + localizer.isStreaming() + " Processing: " + localizer.isProcessing());
        telemetry.addLine("MJPEG: " + localizer.MJPEG);

        lastLoopTime = System.currentTimeMillis();
        robot.drive.localizerTelemetry();

        if (gamepad1.dpadUpWasReleased()) {
            localizer.localize(5000);
            robot.drive.loop();
            String data = String.format(Locale.US, "OQ X: %.0f, Y: %.0f, H: %.1f", (float) robot.drive.oQlocalizer.posX_mm, (float) robot.drive.oQlocalizer.posY_mm, Math.toDegrees(robot.drive.oQlocalizer.heading_rad));
            teamUtil.log(data);
        }
        if (gamepad1.dpadDownWasReleased()) {
            robot.drive.setRobotPosition(TELEPORT_X, TELEPORT_Y, TELEPORT_H);
        }

        if (gamepad1.yWasReleased()) {
            localizer.initCV();
            continousDetection = true;
        }
        if (gamepad1.aWasReleased()) {
            localizer.stopCV();
            continousDetection = false;
        }
        if (gamepad1.xWasReleased()) {
            if (!localizer.visionPortalRunning) {
                localizer.initCV();
            } else {
                localizer.startStreaming();
            }
        }
        if (gamepad1.bWasReleased()) {
            if (localizer.visionPortalRunning) {
                localizer.stopStreaming();
            }
        }
        if (gamepad1.leftBumperWasReleased()) {
            if (!localizer.visionPortalRunning) {
                localizer.initCV();
            } else {
                localizer.startProcessing();
            }
        }
        if (gamepad1.rightBumperWasReleased()) {
            if (localizer.visionPortalRunning) {
                localizer.stopProcessing();
            }
        }
        if (gamepad1.psWasReleased()) {
            detecting = !detecting;
        }
        if (detecting) {
            if (robot.intake.detectorMode == Intake.DETECTION_MODE.INTAKE) {
                robot.intake.detectIntakeArtifactsV2();
                robot.intake.signalArtifacts();
            } else if (robot.intake.detectorMode == Intake.DETECTION_MODE.LOADED) {
                robot.intake.detectLoadedArtifactsV2();
                robot.intake.signalArtifacts();
            } else {
                robot.intake.setRGBSignals(Intake.ARTIFACT.NONE, Intake.ARTIFACT.NONE, Intake.ARTIFACT.NONE);
            }
        }
        if (continousDetection) {
            Pose3D pose = localizer.getFreshRobotPose();
            if (pose != null) {
                lastPose = pose;
            } else {
                pose = lastPose;
            }
            if (pose!= null) {
                double newX=0,newY=0,newH=0 ;

                if (teamUtil.alliance== teamUtil.Alliance.RED) {
                    newX = pose.getPosition().x*-1*25.4 + AprilTagLocalizer.ADJUST_RED_X;
                    newY = pose.getPosition().y*-1*25.4 + AprilTagLocalizer.ADJUST_RED_Y;
                    newH = pose.getOrientation().getYaw(AngleUnit.DEGREES)-90 + AprilTagLocalizer.ADJUST_RED_H;
                } else if (teamUtil.alliance== teamUtil.Alliance.BLUE) {
                    newX = pose.getPosition().x*-1*25.4 + AprilTagLocalizer.ADJUST_BLUE_X;
                    newY = pose.getPosition().y*-1*25.4 + AprilTagLocalizer.ADJUST_BLUE_Y;
                    newH = pose.getOrientation().getYaw(AngleUnit.DEGREES)-90 + AprilTagLocalizer.ADJUST_BLUE_H;
                }

                telemetry.addLine(String.format("Camera: X: %.0f Y: %.0f H: %.1f ", newX, newY, newH));
                telemetry.addLine(String.format("Diff: X: %.0f Y: %.0f H: %.1f ",
                        robot.drive.oQlocalizer.posX_mm - newX,
                        robot.drive.oQlocalizer.posY_mm - newY,
                        Math.toDegrees(robot.drive.oQlocalizer.heading_rad) -newH));

            }
        }
    }
    public void initCV () {
        teamUtil.log("Initializing multi cam vision portal for Calibrate Arms");
        aprilTag = new AprilTagProcessor.Builder().build();
        robot.aprilTag = aprilTag;
        webcamL = hardwareMap.get(WebcamName.class, "webcamleft");
        webcamF = hardwareMap.get(WebcamName.class, "webcamfront");
        webcamR = hardwareMap.get(WebcamName.class, "webcamright");
        CameraName switchableCamera = ClassFactory.getInstance()
                .getCameraManager().nameForSwitchableCamera(webcamL, webcamF, webcamR);
        // Create the vision portal by using a builder.
        visionPortal = new VisionPortal.Builder()
                .setCamera(switchableCamera)
                .addProcessor(aprilTag)
                .build();
        teamUtil.log("Multi cam vision portal built");
    }

    public void testCV () {
        robot.detectPattern();

        if (gamepad1.dpadLeftWasReleased()) {
            visionPortal.setActiveCamera(webcamL);
        }
        if (gamepad1.dpadUpWasReleased()) {
            visionPortal.setActiveCamera(webcamF);
        }
        if (gamepad1.dpadRightWasReleased()) {
            visionPortal.setActiveCamera(webcamR);
        }

        List<AprilTagDetection> currentDetections = aprilTag.getDetections();
        telemetry.addData("# AprilTags Detected", currentDetections.size());
        for (AprilTagDetection detection : currentDetections) {
            if (detection.metadata != null) {
                telemetry.addLine(String.format("\n==== (ID %d) %s", detection.id, detection.metadata.name));
                telemetry.addLine(String.format("XYZ %6.1f %6.1f %6.1f  (inch)", detection.ftcPose.x, detection.ftcPose.y, detection.ftcPose.z));
                telemetry.addLine(String.format("PRY %6.1f %6.1f %6.1f  (deg)", detection.ftcPose.pitch, detection.ftcPose.roll, detection.ftcPose.yaw));
                telemetry.addLine(String.format("RBE %6.1f %6.1f %6.1f  (inch, deg, deg)", detection.ftcPose.range, detection.ftcPose.bearing, detection.ftcPose.elevation));
            } else {
                telemetry.addLine(String.format("\n==== (ID %d) Unknown", detection.id));
                telemetry.addLine(String.format("Center %6.0f %6.0f   (pixels)", detection.center.x, detection.center.y));
            }
        }
        // Add "key" information to telemetry
        telemetry.addLine("\nkey:\nXYZ = X (Right), Y (Forward), Z (Up) dist.");
        telemetry.addLine("PRY = Pitch, Roll & Yaw (XYZ Rotation)");
        telemetry.addLine("RBE = Range, Bearing & Elevation");

    }

    public static int TELEPORT_X, TELEPORT_Y, TELEPORT_H;

    public void teleport() {
        robot.drive.loop(); // get updated localizer data
        robot.drive.driveMotorTelemetry();

        if (gamepad1.dpadUpWasReleased()) {
            robot.drive.setRobotPosition(TELEPORT_X, TELEPORT_Y, TELEPORT_H);
        }
    }

    public static boolean DETECTOR_COLORS = false;

    public void testIMU() {
        robot.drive.loop();

        telemetry.addData("IMU Ang Vel: " , Math.toDegrees(robot.drive.oQlocalizer.velHeading_radS));
        telemetry.addLine(String.format("IMU Ang Vel: %.3f",  robot.drive.oQlocalizer.velHeading_radS));


        if (gamepad1.left_trigger > 0.5) {
            robot.shooter.setShootSpeed(SHOOTER_VELOCITY);
        } else {
            robot.shooter.stopShooter();
        }


    }

    public void testIntakeDetector() {
        robot.intake.intakeTelemetry();
        if(gamepad1.dpadUpWasReleased()){
            //robot.intake.startDetector(DETECTOR_COLORS);
        }
        if(gamepad1.dpadLeftWasReleased()){
            DETECTOR_COLORS = !DETECTOR_COLORS;
            //robot.intake.stopDetector();
            teamUtil.pause(100);
            //robot.intake.startDetector(DETECTOR_COLORS);
        }
        if(gamepad1.dpadDownWasReleased()){
            //robot.intake.stopDetector();
        }
    }

    public static long LLRESET_PAUSE = 100;

    public void testDetectorV2() {

        //telemetry.addLine("LL Status: " + robot.limelight.getStatus());
        telemetry.addLine("DetectorMode: " + robot.intake.detectorMode + " Keep On: " + Intake.KEEP_INTAKE_DETECTOR_SNAPSCRIPT_RUNNING);

        if (robot.intake.detectorMode == Intake.DETECTION_MODE.INTAKE) {
            robot.intake.detectIntakeArtifactsV2();
            robot.intake.signalArtifacts();
        } else if (robot.intake.detectorMode == Intake.DETECTION_MODE.LOADED) {
            robot.intake.detectLoadedArtifactsV2();
            robot.intake.signalArtifacts();
        } else {
            robot.intake.setRGBSignals(Intake.ARTIFACT.NONE, Intake.ARTIFACT.NONE, Intake.ARTIFACT.NONE);
        }

        telemetry.addLine("Loaded: L: " + robot.intake.leftLoad + " M: " + robot.intake.middleLoad + " R:" + robot.intake.rightLoad);
        telemetry.addLine("Intake: Num: " + robot.intake.intakeNum + " L: " + robot.intake.leftIntake + " M: " + robot.intake.middleIntake + " R: " + robot.intake.rightIntake);
        LLStatus llstatus = robot.limelight.getStatus();
        if (llstatus == null) {
            telemetry.addLine("LimeLight: NO Status");
        } else {
            telemetry.addLine(String.format("LimeLight Pipeline: %d FPS: %.1f Temp: %.1f PipeImgCount: %d",llstatus.getPipelineIndex(),llstatus.getFps(),llstatus.getTemp(), llstatus.getPipeImgCount()));
        }


        if (gamepad1.dpadUpWasReleased()) { // fix it
            robot.intake.resetIntakeDetector();
            //robot.intake.stopIntakeDetector();
            //teamUtil.pause(LLRESET_PAUSE);
            //robot.intake.startIntakeDetector();

            //robot.intake.startIntakeDetector();
            //robot.startLimeLightPipeline(Robot.PIPELINE_INTAKE); // This test first, then fix modes and integrate into Intake
        }

        if (gamepad1.dpadLeftWasReleased()) { // toggle always on mode
            Intake.KEEP_INTAKE_DETECTOR_SNAPSCRIPT_RUNNING = ! Intake.KEEP_INTAKE_DETECTOR_SNAPSCRIPT_RUNNING;
        }
        if (gamepad1.dpadRightWasReleased()) { // break it
            robot.intake.stopDetector();
            robot.intake.startDetector();
        }
        if (gamepad1.dpadLeftWasReleased()) {
            robot.startLimeLightPipeline(Robot.PIPELINE_VIEW);
        }
        if (gamepad1.bWasReleased()) {
            robot.intake.calibrateElevators();
        }
        if (gamepad1.yWasReleased()) {
            robot.intake.elevatorToFlippersV2(false, true);
            //if (robot.limeLightActive()) {
            //    robot.intake.detectorMode = Intake.DETECTION_MODE.LOADED;
            //}
            teamUtil.log("Detectors at Top: L: " + robot.intake.leftLoad + " M: " + robot.intake.middleLoad + " R: " + robot.intake.rightLoad);
        }
        if (gamepad1.aWasReleased()) {
            robot.intake.elevatorToGroundV2();
        }
        if (gamepad1.rightBumperWasReleased()) {
            robot.intake.intakeStop();
        }
        if (gamepad1.leftBumperWasReleased()) {
            robot.intake.getReadyToIntake();
            //if (robot.limeLightActive()) {
            //    robot.intake.detectorMode = Intake.DETECTION_MODE.INTAKE;
            //}
        }
        if (gamepad1.xWasReleased()) {
            teamUtil.log(teamUtil.robot.limelight.getStatus().toString());
            teamUtil.log("Time since last update: " + robot.limelight.getTimeSinceLastUpdate());
            double inputs[] = {1,2,2,2,2,2,2,2};
            teamUtil.robot.limelight.updatePythonInputs(inputs); // This should put it into intake mode
            teamUtil.pause(500);
            LLResult result = teamUtil.robot.limelight.getLatestResult();
            teamUtil.log("result: " + result);
            if (result != null) {
                teamUtil.log("Valid: " + result.isValid());
                double[] llOutput = result.getPythonOutput();

                if (llOutput != null) {
                    /*
                    double output0 = llOutput[0];
                    double output1 = llOutput[1];
                    double output2 = llOutput[2];
                    double output3 = llOutput[3];
                    teamUtil.log("llOutput: " + llOutput.length + " Values: " + output0 + ", "+ output1 + ", "+ output2 + ", "+ output3);
                    */
                    teamUtil.log("Mode: " + llOutput[0] + " Detections: "+ llOutput[1] + " L: "+ llOutput[2] + " M: "+ llOutput[3] + " R: "+ llOutput[4]);
                }
            }
        }


    }


    public void testElevator() {
        if (gamepad1.dpadUpWasReleased()) {
            robot.intake.elevator.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.intake.elevator.setPower(Intake.ELEVATOR_UP_POWER);
        }
        if (gamepad1.dpadDownWasReleased()) {
            robot.intake.elevator.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.intake.elevator.setPower(Intake.ELEVATOR_DOWN_POWER);
        }
        if (gamepad1.dpadLeftWasReleased()) {
            robot.intake.elevator.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.intake.elevator.setPower(0);
        }
        if (gamepad1.bWasReleased()) {
            robot.intake.calibrateElevators();
        }
        if (gamepad1.yWasReleased()) {
            robot.intake.elevatorToShooterFastNoWait(false);
        }
        if (gamepad1.aWasReleased()) {
            robot.intake.elevatorToGroundV2();
        }
        if (gamepad1.rightBumperWasReleased()) {
            robot.intake.intakeStop();
        }
        if (gamepad1.leftBumperWasReleased()) {
            robot.intake.getReadyToIntake();
        }
        telemetry.addData("Elevator Velocity", robot.intake.elevator.getVelocity());
        telemetry.addLine("Elevator Position: " + robot.intake.elevator.getCurrentPosition());
        telemetry.addLine("Elevator Velocity: " + robot.intake.elevator.getVelocity());
    }

    public void testIntake() {
        robot.intake.intakeTelemetry();
        //teamUtil.log("MIDDLE: " + robot.intake.middleLoad);
        //telemetry.addLine("Elevator Tolerance: " + robot.intake.elevator.getPIDFCoefficients(DcMotor.RunMode.RUN_TO_POSITION));

        if (gamepad1.rightBumperWasReleased()) {
            robot.intake.calibrate();
            robot.intake.elevator.setVelocity(0);

        }
        if(gamepad1.dpadDownWasReleased()){
            robot.intake.getReadyToIntakeNoWait();
        }
        if(gamepad1.dpadRightWasReleased()){
            robot.intake.middle_flipper.setPosition(FLIPPER_TEST_VAL);
            robot.intake.left_flipper.setPosition(FLIPPER_TEST_VAL);
            robot.intake.right_flipper.setPosition(FLIPPER_TEST_VAL);
        }
        if(gamepad1.bWasReleased()){
            robot.intake.intakeStop();
        }
        if(gamepad1.yWasReleased()){
            robot.intake.intakeStart();
        }
        if(gamepad1.xWasReleased()){
            robot.intake.intakeOut();
        }
    }

    public void testFoot() {
        robot.outputFootSensor();
        if(gamepad1.dpadUpWasReleased()){
            robot.setFootPos(Robot.FOOT_EXTENDED_POS);
        }if(gamepad1.dpadDownWasReleased()){
            robot.setFootPos(Robot.FOOT_CALIBRATE_POS);
        }
    }

    public static double SHOOTER_VELOCITY = 800;
    public static teamUtil.Pattern TEST_PATTERN = teamUtil.Pattern.PPG;
    public static Intake.ARTIFACT LEFT_LOAD = Intake.ARTIFACT.NONE;
    public static Intake.ARTIFACT MIDDLE_LOAD = Intake.ARTIFACT.NONE;
    public static Intake.ARTIFACT RIGHT_LOAD = Intake.ARTIFACT.NONE;

    public void testShooter(){
        robot.drive.loop(); // get updated localizer data
        telemetry.addData("AdjustShootMode: " , adjustShootMode);
        double distance = robot.drive.robotGoalDistance() ;
        double velocity = robot.shooter.calculateVelocityV2(distance);
        double pitch = robot.shooter.calculatePitchV2(distance);
        telemetry.addLine(String.format("CURRENT Distance: %.0f " , distance ));
        telemetry.addLine(String.format("IDEAL Velocity: %.0f IDEAL Pitch: %.3f" , velocity , pitch));
        telemetry.addLine("-------------------------------------");
        telemetry.addLine("ACTUAL Velocity Left: " + robot.shooter.leftFlywheel.getVelocity() + " Right: "+ robot.shooter.rightFlywheel.getVelocity());
        telemetry.addLine("ACTUAL Pitch: " + robot.shooter.currentAim());
        telemetry.addData("FlywheelSpeed OK?: " , robot.shooterFlyWheelsReady(distance));
        telemetry.addLine("Heading OK? Teleop: " + robot.shooterHeadingReady() + " Auto: " + robot.autoShooterHeadingReady());
        telemetry.addLine("Can Shoot? : " + robot.canShoot(distance));
        telemetry.addLine("-------------------------------------");
        robot.shooter.outputTelemetry();
        robot.drive.driveMotorTelemetry();
        telemetry.addData("Reported Left Velocity: " , robot.shooter.leftFlywheel.getVelocity());
        telemetry.addData("Reported Right Velocity: " , robot.shooter.rightFlywheel.getVelocity());
        telemetry.addData("Target ", SHOOTER_VELOCITY);
        telemetry.addLine("Aimer Position: "+robot.shooter.currentAim());

        if(gamepad1.startWasReleased()){
            robot.shooter.adjustShooterV4(robot.drive.robotGoalDistance());
            SHOOTER_VELOCITY = Shooter.VELOCITY_COMMANDED;
        }
        /*
        if (adjustShootMode && robot.shooter.flywheelSpeedOK(robot.drive.robotGoalDistance(),robot.shooter.rightFlywheel.getVelocity())) {
            //robot.shooter.adjustShooterV2(robot.drive.robotGoalDistance());
            robot.shooter.changeAim(robot.drive.robotGoalDistance(),robot.shooter.rightFlywheel.getVelocity());
        }

         */
        if(gamepad1.dpadLeftWasPressed()){
            robot.shooter.aim(robot.shooter.currentAim()-.01);
        }
        if(gamepad1.dpadRightWasPressed()){
            robot.shooter.aim(robot.shooter.currentAim()+.01);
        }
        if (gamepad1.rightBumperWasReleased()){
            SHOOTER_VELOCITY = SHOOTER_VELOCITY - 20;
            robot.shooter.setShootSpeed(SHOOTER_VELOCITY);
        }
        if (gamepad1.leftBumperWasReleased()){
            SHOOTER_VELOCITY = SHOOTER_VELOCITY + 20;
            robot.shooter.setShootSpeed(SHOOTER_VELOCITY);
        }
        if(gamepad1.dpadUpWasReleased()){
            robot.shooter.setShootSpeed(SHOOTER_VELOCITY);
        }
        if(gamepad1.dpadDownWasReleased()){
            robot.shooter.stopShooter();
        }


        if(gamepad1.aWasReleased()){
            robot.shooter.sidePushersHold();
            teamUtil.pause(500);
            double storePitch = robot.shooter.currentAim();
            robot.shooter.shoot3SuperFast(true, true, true,SHOOT_3_AUTO,robot.drive.robotGoalDistance());
            teamUtil.pause(500);
            robot.shooter.aim(storePitch); // undo any changes to the pitch made by shoot3
            robot.shooter.setShootSpeed(SHOOTER_VELOCITY); // undo any changes to the velocity made by shoot3
        }

        if(gamepad1.yWasPressed()){
            //robot.shootAllArtifacts();
            robot.shooter.pusher.pushNNoWait(1, AxonPusher.RTP_MAX_VELOCITY, 1500);
            teamUtil.pause(300);
            robot.shooter.pusher.reset(true);
        }
        if(gamepad1.bWasPressed()){ // Time pusher
            if (robot.shooter.leftFlywheel.getVelocity() > 400) { // make sure flywheels are running
                long startTime = System.currentTimeMillis();
                robot.shooter.pusher.pushNNoWait(1, AxonPusher.RTP_MAX_VELOCITY, 1500);
                double detectVelocity = robot.shooter.leftFlywheel.getVelocity() - 60;
                long timeOutTime = System.currentTimeMillis() + 1000;
                while (robot.shooter.leftFlywheel.getVelocity() > detectVelocity && teamUtil.keepGoing(timeOutTime)) {
                } // detect contact between flywheels and ball
                teamUtil.log("Push Time: " + (System.currentTimeMillis() - startTime));
            }
        }

        if (gamepad1.xWasReleased()) {
            robot.intake.flippersToTransfer();
        }
        if (gamepad1.left_trigger > .5) {
            while (gamepad1.left_trigger > .5) {}
            //robot.intake.setIntakeArtifacts(TEST_PATTERN);
            robot.intake.setIntakeArtifacts(LEFT_LOAD, MIDDLE_LOAD, RIGHT_LOAD);
            robot.autoTransferAndLoadNoWait(0, false, 5000);
            //robot.autoShootFastPreloadV2();
        }
        if (gamepad1.right_trigger > .5) {
            while (gamepad1.right_trigger > .5) {}
            robot.autoShootPattern(true, 5000);
            robot.intake.intakeStop();
            //robot.autoShootFastV2(true,3000);
        }
    }
    public boolean locked = false;
    public static double SHOOTER_OVERSHOOT = 200;
    public static long SHOOTER_RAMP_PAUSE = 200;
    public static double PIDF_P_Left = Shooter.shooterLeftP;
    public static double PIDF_I_Left = Shooter.shooterLeftI;
    public static double PIDF_D_Left = Shooter.shooterLeftD;
    public static double PIDF_F_Left = Shooter.shooterLeftF;
    public static double PIDF_P_Right = Shooter.shooterRightP;
    public static double PIDF_I_Right = Shooter.shooterRightI;
    public static double PIDF_D_Right = Shooter.shooterRightD;
    public static double PIDF_F_Right = Shooter.shooterRightF;
    public void shooterPIDF(){
        if (gamepad1.startWasReleased()) {
            robot.shooter.leftFlywheel.setVelocityPIDFCoefficients(PIDF_P_Left, PIDF_I_Left, PIDF_D_Left, PIDF_F_Left);
            robot.shooter.rightFlywheel.setVelocityPIDFCoefficients(PIDF_P_Right, PIDF_I_Right, PIDF_D_Right, PIDF_F_Right);
            teamUtil.log("Left: " + robot.shooter.leftFlywheel.getPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER));
            teamUtil.log("Right: " + robot.shooter.rightFlywheel.getPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER));
        }
        //robot.shooter.leftFlywheel.setPower(CurrentPower);
        //rightFlywheel.setPower(-CurrentPower);

        if(gamepad1.dpadUpWasReleased()){
            //robot.shooter.setShootSpeed(SHOOTER_VELOCITY+SHOOTER_OVERSHOOT);
            //teamUtil.pause(SHOOTER_RAMP_PAUSE); // TODO: Or wait until reported velocity is near target?
            robot.shooter.setShootSpeed(SHOOTER_VELOCITY);

        }if(gamepad1.dpadDownWasReleased()){
            robot.shooter.stopShooter();
        }
        if(gamepad1.yWasPressed()) {
            robot.shooter.pusher.pushNNoWait(1, AxonPusher.RTP_MAX_VELOCITY, 1500);
            //teamUtil.pause(300);
            //robot.shooter.pusher.reset(true);
        }
        /*
        if(gamepad1.yWasPressed()){
            if(!locked){
                robot.shooter.setShootSpeed(robot.shooter.leftFlywheel.getVelocity());
                locked = true;
            }
            else{
                locked = false;
            }
        }

        if(!locked){
            robot.shooter.adjustShooterV4(robot.drive.robotGoalDistance());
        }
*/
        if(gamepad1.aWasReleased()){
            //robot.shooter.pusher.pushNNoWait(3,AxonPusher.RTP_MAX_VELOCITY, 1500);
            robot.shooter.sidePushersHold();
            while (!gamepad1.aWasReleased()) {teamUtil.pause(50);}
            robot.shooter.shootSuperFastNoWait(true, true, true,SHOOT_3_AUTO,robot.drive.robotGoalDistance());
        }
        if(gamepad1.xWasPressed()){
            robot.intake.elevatorToFlippersV2(true, true);
        }

        if(gamepad1.leftBumperWasPressed()){
            robot.shooter.aim(robot.shooter.currentAim()+.005);
        }
        if(gamepad1.rightBumperWasPressed()){
            robot.shooter.aim(robot.shooter.currentAim()-.005);
        }
        telemetry.addLine("currentVelocity: " + currentRVelocity + ", " + currentLVelocity);
        telemetry.addLine("ReportedVelocity: " + robot.shooter.leftFlywheel.getVelocity()+", "+robot.shooter.rightFlywheel.getVelocity());
        telemetry.addData("Reported Left Velocity: " , robot.shooter.leftFlywheel.getVelocity());
        telemetry.addData("Reported Right Velocity: " , robot.shooter.rightFlywheel.getVelocity());
        telemetry.addData("Target ", SHOOTER_VELOCITY);
        telemetry.addData("Over Shoot ", SHOOTER_OVERSHOOT);
        telemetry.addLine("Aimer Position: "+robot.shooter.currentAim());

        robot.drive.loop();

    }
}
