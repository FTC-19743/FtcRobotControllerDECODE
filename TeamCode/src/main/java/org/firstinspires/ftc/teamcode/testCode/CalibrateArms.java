package org.firstinspires.ftc.teamcode.testCode;



import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;


import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.CameraName;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.assemblies.AxonPusher;
import org.firstinspires.ftc.teamcode.assemblies.Intake;
import org.firstinspires.ftc.teamcode.assemblies.Robot;
import org.firstinspires.ftc.teamcode.assemblies.Shooter;

import org.firstinspires.ftc.teamcode.libs.teamUtil;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;

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

    double aimerPosition = Shooter.AIMER_CALIBRATE;
    
    public enum Ops {
        Test_Intake,
        Test_Intake_Detector,
        Test_Elevator,
        Test_Shooter,
        Test_CV,
        Test_Foot,
        Test_PIDF,
        Teleport
    };
    public static Ops AA_Operation = Ops.Test_PIDF;
    public static boolean useCV = false;

    @Override
    public void runOpMode() throws InterruptedException {
        FtcDashboard dashboard = FtcDashboard.getInstance();
        //FtcDashboard.setDrawDefaultField(false); // enable to eliminate field drawing
        telemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry()); // write telemetry to Driver Station and Dashboard
        teamUtil.init(this);
        
        robot = new Robot();
        robot.initialize(false);
        robot.drive.setHeading(0);
        teamUtil.justRanAuto = false;
        teamUtil.justRanCalibrateRobot = false;

        robot.calibrate();
        if (useCV) {
            telemetry.addLine("Initializing CV");
            telemetry.update();
            initCV();
            telemetry.addLine("starting LiveView");
            telemetry.update();

            visionPortal.resumeLiveView();
        }

        telemetry.addLine("Ready to start");
        telemetry.addLine("ALLIANCE : " + teamUtil.alliance);
        telemetry.addLine("SIDE : " + teamUtil.SIDE);
        telemetry.update();
        
        waitForStart();

        if (isStopRequested()) {
            return;
        }

        while (opModeIsActive()) {
           
            telemetry.addLine("ALLIANCE : " + teamUtil.alliance + " SIDE : " + teamUtil.SIDE + "PATTERN: " + teamUtil.pattern);

            if (gamepad1.left_stick_button) {
                teamUtil.logSystemHealth();
            }
            switch (AA_Operation) {
                case Test_Intake : testIntake();break;
                case Test_Intake_Detector: testIntakeDetector();break;
                case Test_Elevator: testElevator();break;
                case Test_Shooter : testShooter();break;
                case Test_CV: testCV();break;
                case Test_Foot : testFoot();break;
                case Test_PIDF: shooterPIDF();break;
                case Teleport : teleport();break;
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

    }
    private AprilTagProcessor aprilTag;
    private VisionPortal visionPortal;
    private WebcamName webcamL, webcamF, webcamR;

    public void initCV () {
        teamUtil.log("Initializing multi cam vision port for Calibrate Arms");
        aprilTag = new AprilTagProcessor.Builder().build();
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
    }

    public void testCV () {
        robot.testDetectPattern(aprilTag);


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

    public void testIntakeDetector() {
        robot.intake.intakeTelemetry();
        if(gamepad1.dpadUpWasReleased()){
            robot.intake.startDetector(DETECTOR_COLORS);
        }
        if(gamepad1.dpadLeftWasReleased()){
            DETECTOR_COLORS = !DETECTOR_COLORS;
            robot.intake.stopDetector();
            teamUtil.pause(100);
            robot.intake.startDetector(DETECTOR_COLORS);
        }
        if(gamepad1.dpadDownWasReleased()){
            robot.intake.stopDetector();
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
            robot.intake.elevatorToFlippersV2();
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
        if(gamepad1.circleWasReleased()){
            robot.intake.intakeStop();
        }
        if(gamepad1.triangleWasReleased()){
            robot.intake.intakeIn();
        }
        if(gamepad1.squareWasReleased()){
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
    public void testShooter(){
        robot.drive.loop(); // get updated localizer data
        telemetry.addData("AdjustShootMode: " , adjustShootMode);
        telemetry.addData("Reported Left Velocity: " , robot.shooter.leftFlywheel.getVelocity());
        telemetry.addData("Reported Right Velocity: " , robot.shooter.rightFlywheel.getVelocity());
        robot.shooter.outputTelemetry();
        robot.drive.driveMotorTelemetry();

        if(gamepad1.startWasReleased()){
            adjustShootMode= !adjustShootMode;
        }
        if (adjustShootMode) {
            robot.shooter.adjustShooterV2(robot.drive.robotGoalDistance());
        }
        if(gamepad1.dpadUpWasReleased()){
            robot.shooter.setShootSpeed(SHOOTER_VELOCITY);
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
        if(gamepad1.yWasPressed()){
            robot.shootAllArtifacts();
        }
        if(gamepad1.bWasPressed()){ // Time pusher
            robot.shootAllArtifacts();
            long startTime = System.currentTimeMillis();
            robot.shooter.pusher.pushNNoWait(1, AxonPusher.RTP_MAX_VELOCITY, 1500);
            double detectVelocity = robot.shooter.leftFlywheel.getVelocity()-100;
            while (robot.shooter.leftFlywheel.getVelocity() > detectVelocity) {} // detect contact between flywheels and ball
            teamUtil.log("Push Time: " + (System.currentTimeMillis() - startTime));
        }

    }

    public static double SHOOTER_OVERSHOOT = 200;
    public static long SHOOTER_RAMP_PAUSE = 200;
    public void shooterPIDF(){
        if (gamepad1.startWasReleased()) {
            robot.shooter.leftFlywheel.setVelocityPIDFCoefficients(Shooter.shooterP, Shooter.shooterI, Shooter.shooterD, Shooter.shooterF);
            robot.shooter.rightFlywheel.setVelocityPIDFCoefficients(Shooter.shooterP, Shooter.shooterI, Shooter.shooterD, Shooter.shooterF);
        }
        //robot.shooter.leftFlywheel.setPower(CurrentPower);
        //rightFlywheel.setPower(-CurrentPower);

        if(gamepad1.dpadUpWasReleased()){
            robot.shooter.setShootSpeed(SHOOTER_VELOCITY+SHOOTER_OVERSHOOT);
            teamUtil.pause(SHOOTER_RAMP_PAUSE); // TODO: Or wait until reported velocity is near target?
            robot.shooter.setShootSpeed(SHOOTER_VELOCITY);

        }if(gamepad1.dpadDownWasReleased()){
            robot.shooter.stopShooter();
        }
        if(gamepad1.aWasReleased()){
            robot.shooter.pusher.pushNNoWait(3,AxonPusher.RTP_MAX_VELOCITY, 1500);
        }
        if(gamepad1.xWasPressed()){
            robot.intake.elevatorToFlippersV2();
        }
        if(gamepad1.bWasPressed()) {
            robot.shooter.pusher.pushNNoWait(1, AxonPusher.RTP_MAX_VELOCITY, 1000);
        }if(gamepad1.leftBumperWasPressed()){
            robot.shooter.aim(robot.shooter.currentAim()+.005);
        }if(gamepad1.rightBumperWasPressed()){
            robot.shooter.aim(robot.shooter.currentAim()-.005);
        }
        telemetry.addLine("currentVelocity: " + currentRVelocity + ", " + currentLVelocity);
        telemetry.addLine("ReportedVelocity: " + robot.shooter.leftFlywheel.getVelocity()+", "+robot.shooter.rightFlywheel.getVelocity());
        telemetry.addData("Reported Left Velocity: " , robot.shooter.leftFlywheel.getVelocity());
        telemetry.addData("Reported Right Velocity: " , robot.shooter.rightFlywheel.getVelocity());
        telemetry.addData("Target ", SHOOTER_VELOCITY);
        telemetry.addData("Over Shoot ", SHOOTER_OVERSHOOT);
        telemetry.addLine("Aimer Position: "+robot.shooter.currentAim());
    }
}
