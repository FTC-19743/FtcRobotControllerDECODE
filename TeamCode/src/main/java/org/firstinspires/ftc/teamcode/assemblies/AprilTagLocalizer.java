package org.firstinspires.ftc.teamcode.assemblies;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.teamcode.libs.teamUtil;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;
import java.util.Locale;

public class AprilTagLocalizer {
    HardwareMap hardwareMap;
    Telemetry telemetry;
    /**
     * Variables to store the position and orientation of the camera on the robot. Setting these
     * values requires a definition of the axes of the camera and robot:
     *
     * Camera axes:
     * Origin location: Center of the lens
     * Axes orientation: +x right, +y down, +z forward (from camera's perspective)
     *
     * Robot axes (this is typical, but you can define this however you want):
     * Origin location: Center of the robot at field height
     * Axes orientation: +x right, +y forward, +z upward
     *
     * Position:
     * If all values are zero (no translation), that implies the camera is at the center of the
     * robot. Suppose your camera is positioned 5 inches to the left, 7 inches forward, and 12
     * inches above the ground - you would need to set the position to (-5, 7, 12).
     *
     * Orientation:
     * If all values are zero (no rotation), that implies the camera is pointing straight up. In
     * most cases, you'll need to set the pitch to -90 degrees (rotation about the x-axis), meaning
     * the camera is horizontal. Use a yaw of 0 if the camera is pointing forwards, +90 degrees if
     * it's pointing straight left, -90 degrees for straight right, etc. You can also set the roll
     * to +/-90 degrees if it's vertical, or 180 degrees if it's upside-down.
     */
    private Position cameraPosition = new Position(DistanceUnit.INCH,
            0, 7, 3, 0);
    private YawPitchRollAngles cameraOrientation = new YawPitchRollAngles(AngleUnit.DEGREES,
            0, -70, 0, 0);

    /**
     * The variable to store our instance of the AprilTag processor.
     */
    private AprilTagProcessor aprilTag;

    /**
     * The variable to store our instance of the vision portal.
     */
    private VisionPortal visionPortal;

    public AprilTagLocalizer() {
        telemetry = teamUtil.theOpMode.telemetry;
        hardwareMap = teamUtil.theOpMode.hardwareMap;
    }

    public void stopCV() {
        if (visionPortal != null) {
            visionPortal.close();
        }
    }

    public void initCV() {

        // Create the AprilTag processor.
        aprilTag = new AprilTagProcessor.Builder()
                // The following default settings are available to un-comment and edit as needed.
                //.setDrawAxes(false)
                //.setDrawCubeProjection(false)
                //.setDrawTagOutline(true)
                //.setTagFamily(AprilTagProcessor.TagFamily.TAG_36h11)
                //.setTagLibrary(AprilTagGameDatabase.getCenterStageTagLibrary())
                //.setOutputUnits(DistanceUnit.INCH, AngleUnit.DEGREES)
                .setCameraPose(cameraPosition, cameraOrientation)

                // == CAMERA CALIBRATION ==
                // If you do not manually specify calibration parameters, the SDK will attempt
                // to load a predefined calibration for your camera.
                //.setLensIntrinsics(578.272, 578.272, 402.145, 221.506)
                // ... these parameters are fx, fy, cx, cy.

                .build();

        // Adjust Image Decimation to trade-off detection-range for detection-rate.
        // eg: Some typical detection data using a Logitech C920 WebCam
        // Decimation = 1 ..  Detect 2" Tag from 10 feet away at 10 Frames per second
        // Decimation = 2 ..  Detect 2" Tag from 6  feet away at 22 Frames per second
        // Decimation = 3 ..  Detect 2" Tag from 4  feet away at 30 Frames Per Second (default)
        // Decimation = 3 ..  Detect 5" Tag from 10 feet away at 30 Frames Per Second (default)
        // Note: Decimation can be changed on-the-fly to adapt during a match.
        //aprilTag.setDecimation(3);

        // Create the vision portal by using a builder.
        VisionPortal.Builder builder = new VisionPortal.Builder();

        // Set the camera (webcam vs. built-in RC phone camera).
        builder.setCamera(hardwareMap.get(WebcamName.class, "webcamfront"));


        // Choose a camera resolution. Not all cameras support all resolutions.
        //builder.setCameraResolution(new Size(1920, 1080));

        // Enable the RC preview (LiveView).  Set "false" to omit camera monitoring.
        //builder.enableLiveView(true);

        // Set the stream format; MJPEG uses less bandwidth than default YUY2.
        //builder.setStreamFormat(VisionPortal.StreamFormat.YUY2);
        builder.setStreamFormat(VisionPortal.StreamFormat.MJPEG);

        // Choose whether or not LiveView stops if no processors are enabled.
        // If set "true", monitor shows solid orange screen if no processors enabled.
        // If set "false", monitor shows camera view without annotations.
        //builder.setAutoStopLiveView(false);

        // Set and enable the processor.
        builder.addProcessor(aprilTag);

        // Build the Vision Portal, using the above settings.
        visionPortal = builder.build();

        // Disable or re-enable the aprilTag processor at any time.
        //visionPortal.setProcessorEnabled(aprilTag, true);
    }

    public Pose3D getFreshRobotPose() {
        if (aprilTag != null) {
            List<AprilTagDetection> currentDetections = aprilTag.getFreshDetections();
            if (currentDetections != null) {
                // Step through the list of detections
                for (AprilTagDetection detection : currentDetections) {
                    if (detection.metadata != null) {
                        if (detection.id == 20 || detection.id == 24) {
                            return detection.robotPose; // return the first one we find, very hard for the robot to see both at the same time.
                        }
                    }
                }
            }
        }
        return null;
    }

    public static long SAMPLE_TIME = 500;
    public boolean localize(long timeOut) {
        teamUtil.log("Localize starting");
        long startTime = System.currentTimeMillis();
        long timeOutTime = startTime + timeOut;
        initCV(); // set up vision portal just in time
        while (visionPortal.getCameraState() != VisionPortal.CameraState.STREAMING && teamUtil.keepGoing(timeOutTime)) {
            teamUtil.pause(50);
        }
        // We are either streaming or timed out
        if (visionPortal.getCameraState() != VisionPortal.CameraState.STREAMING) {
            teamUtil.log("localize TIMED OUT waiting for streaming to start");
            visionPortal.close();
            return false;
        }
        teamUtil.log("Streaming Started in " + (System.currentTimeMillis() - startTime));
        double sumX = 0;
        double sumY = 0;
        double sumH = 0;
        int count = 0;
        long sampleStopTime = System.currentTimeMillis() + SAMPLE_TIME;
        while (System.currentTimeMillis() < sampleStopTime && teamUtil.keepGoing(timeOutTime)) {
            Pose3D pose = getFreshRobotPose();
            if (pose != null) {
                sumX += pose.getPosition().x * -25.4;
                sumY += pose.getPosition().y * -25.4;
                sumH += pose.getOrientation().getYaw(AngleUnit.DEGREES)-90;
                count++;
            }
            teamUtil.pause(35); // give time to AprilTagProcessor (based on 30fps)
        }
        if (count == 0) {
            teamUtil.log("localize failed to get any samples");
            visionPortal.close();
            return false;
        }
        if (System.currentTimeMillis() >= timeOutTime) {
            teamUtil.log("localize TIMED OUT collecting samples");
            visionPortal.close();
            return false;
        }
        visionPortal.close();
        teamUtil.robot.drive.loop(); // make sure we have current robot position
        double newX = sumX/count ;
        double newY = sumY/count;
        double newH = sumH/count;

        teamUtil.log("Localize finished in " +((System.currentTimeMillis() - startTime)));
        teamUtil.log(String.format("Camera: X: %.0f Y: %.0f H: %.1f Count: %d", newX, newY, newH, count));
        String data = String.format(Locale.US, "OQ X: %.0f, Y: %.0f, H: %.1f", (float) teamUtil.robot.drive.oQlocalizer.posX_mm, (float) teamUtil.robot.drive.oQlocalizer.posY_mm, Math.toDegrees(teamUtil.robot.drive.oQlocalizer.heading_rad));
        teamUtil.log(data);
        data = String.format(Locale.US, "Diff X: %.0f, Y: %.0f, H: %.1f", (float) teamUtil.robot.drive.oQlocalizer.posX_mm-newX, (float) teamUtil.robot.drive.oQlocalizer.posY_mm-newY, Math.toDegrees(teamUtil.robot.drive.oQlocalizer.heading_rad) - newH);
        teamUtil.log(data);
        teamUtil.robot.drive.loop();
        teamUtil.robot.drive.setRobotPosition((int)newX, (int)newY, Math.toDegrees(teamUtil.robot.drive.oQlocalizer.heading_rad));
        return true;
    }
}
