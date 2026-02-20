package org.firstinspires.ftc.teamcode.assemblies;

import android.util.Size;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.teamcode.libs.Blinkin;
import org.firstinspires.ftc.teamcode.libs.teamUtil;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;
import java.util.Locale;
import java.util.concurrent.atomic.AtomicBoolean;

@Config
public class AprilTagLocalizer {
    HardwareMap hardwareMap;
    Telemetry telemetry;
    static public boolean details = false;
    public static boolean MJPEG = true;
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

    public class Pose {
        public double x,y, heading;

        Pose(double x, double y, double heading) {
            this.x = x; this.y = y; this.heading = heading;
        }
    }

    public class IDPose {
        public Pose3D pose;
        public int ID;
        IDPose(Pose3D pose, int ID) {
            this.pose = pose;
            this.ID = ID;
        }
    }
    private double normalizeDegrees(double degrees) {
        double normalized = degrees % 360;
        if (normalized > 180) normalized -= 360;
        if (normalized <= -180) normalized += 360;
        return normalized;
    }

    private static final double INV_SQRT2 = 1.0 / Math.sqrt(2.0);
    // Tag IDs
    private static final int TAG_20 = 20;
    private static final int TAG_24 = 24;

    // ------------------------------
    // Tag 24 coefficients
    // Model: [1, cx, cy, u, u^2, u^3], u=(cx+cy)/sqrt(2)
    // Fit excludes Actual point (671, -374)
    // ------------------------------
    private static final double[] TAG24_COEF_X = {
            9.746643398020256,
            0.9965886628140453,
            -0.002330464554402064,
            -0.026621497596707726,
            3.312963741071282E-5,
            -2.010540646581675E-8
    };
    private static final double[] TAG24_COEF_Y = {
            -6.668747907984121,
            0.002217882472099814,
            0.9988935626603818,
            -0.01913586101479955,
            2.467847221533122E-5,
            -1.545269427506511E-8
    };

    // ------------------------------
    // Tag 20 coefficients
    // Model: [1, cx, cy, u, u^2, u^3], u=(cx-cy)/sqrt(2)
    // Fit excludes Actual points (698, 513) and (116, -97)
    // ------------------------------
    private static final double[] TAG20_COEF_X = {
            -70.1536666,
            0.854499894,
            0.237955766,
            0.435962549,
            0.000793772426,
            -0.00000839521848
    };
    private static final double[] TAG20_COEF_Y = {
            22.9004585,
            0.189072143,
            0.807931501,
            -0.437599653,
            -0.00105351854,
            0.0000107291149
    };

    public Pose camToActualV4(Pose3D cameraPose, int tagID) {
        if (cameraPose == null) {
            teamUtil.log("WARNING: camToActualV4 called with null cameraPose");
            return null;
        }
        double headingOffset = tagID == 24 ? 1.954 : -1.124;
        double actualHeading = normalizeDegrees(cameraPose.getOrientation().getYaw(AngleUnit.DEGREES)-90 + headingOffset);

        double cx = cameraPose.getPosition().x*-1*25.4;
        double cy = cameraPose.getPosition().y*-1*25.4;
        final double[] bx;
        final double[] by;
        final double u;
        if (tagID == TAG_24) {
            // u = (cx + cy)/sqrt(2)
            u = (cx + cy) * INV_SQRT2;
            bx = TAG24_COEF_X;
            by = TAG24_COEF_Y;
        } else if (tagID == TAG_20) {
            // u = (cx - cy)/sqrt(2)
            u = (cx - cy) * INV_SQRT2;
            bx = TAG20_COEF_X;
            by = TAG20_COEF_Y;
        } else {
            return null;
        }

        final double u2 = u * u;
        final double u3 = u2 * u;
        // Basis = [1, cx, cy, u, u^2, u^3]
        final double outX = bx[0] + bx[1] * cx + bx[2] * cy + bx[3] * u + bx[4] * u2 + bx[5] * u3;
        final double outY = by[0] + by[1] * cx + by[2] * cy + by[3] * u + by[4] * u2 + by[5] * u3;
        return new Pose(outX, outY, actualHeading);
    }

    public AprilTagLocalizer() {
        telemetry = teamUtil.theOpMode.telemetry;
        hardwareMap = teamUtil.theOpMode.hardwareMap;
    }

    public void stopCV() {
        if (visionPortal != null) {
            visionPortal.close();
        }
        visionPortalRunning = false;
    }

    public void initCV() {
        teamUtil.log("AprilTag Localizer initCV starting");
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
        builder.setCameraResolution(new Size(1920, 1080));

        // Enable the RC preview (LiveView).  Set "false" to omit camera monitoring.
        //builder.enableLiveView(true);

        // Set the stream format; MJPEG uses less bandwidth than default YUY2.
        if (MJPEG) {
            builder.setStreamFormat(VisionPortal.StreamFormat.MJPEG);
        } else {
            builder.setStreamFormat(VisionPortal.StreamFormat.YUY2);
        }

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

        visionPortalRunning = true;
        teamUtil.log("AprilTag Localizer initCV Finished");

    }

    // Do NOT call if vision portal is not already set up
    public void startStreaming() {
        if (visionPortal != null) {
            visionPortal.resumeStreaming();
        }
    }

    // Do NOT call if vision portal is not already set up
    public void stopStreaming() {
        if (visionPortal != null) {
            visionPortal.stopStreaming();
        }
    }

    public boolean isStreaming() {
        if (visionPortal != null) {
            return visionPortal.getCameraState() == VisionPortal.CameraState.STREAMING;
        }
        return false;
    }

    // Do NOT call if vision portal is not already set up
    public void startProcessing() {
        if (visionPortal != null) {
            visionPortal.setProcessorEnabled(aprilTag, true);
        }
    }

    // Do NOT call if vision portal is not already set up
    public void stopProcessing() {
        if (visionPortal != null) {
            visionPortal.setProcessorEnabled(aprilTag, false);
        }
    }

    public boolean isProcessing() {
        if (visionPortal != null) {
            return visionPortal.getProcessorEnabled(aprilTag);
        }
        return false;
    }

    public IDPose getFreshRobotPose() {
        if (aprilTag != null) {
            List<AprilTagDetection> currentDetections = aprilTag.getFreshDetections();
            if (currentDetections != null) {
                // Step through the list of detections
                for (AprilTagDetection detection : currentDetections) {
                    if (detection.metadata != null) {
                        if (detection.id == 20 || detection.id == 24) {
                            return new IDPose(detection.robotPose, detection.id); // return the first one we find, very hard for the robot to see both at the same time...
                        }
                    }
                }
            }
        }
        return null;
    }

    AtomicBoolean localizing = new AtomicBoolean();
    public static long SAMPLE_TIME = 500;
    public static long MAX_SAMPLES = 3;
    public static double ADJUST_RED_X = 10;
    public static double ADJUST_RED_Y = 0;
    public static double ADJUST_RED_H = 3;
    public static double ADJUST_BLUE_X = -30;
    public static double ADJUST_BLUE_Y = 20;
    public static double ADJUST_BLUE_H = 1;

    public void localizeNoWait(long timeout){
        if (localizing.get()) {
            teamUtil.log("WARNING: Attempt to localizeNoWait while localizing. Ignored.");
            return;
        }
        localizing.set(true);
        teamUtil.log("Launching Thread to localize");
        Thread thread = new Thread(new Runnable() {
            @Override
            public void run() {
                localize(timeout);
            }
        });
        thread.start();
    }

    public boolean visionPortalRunning = false;
    public boolean startedJIT = false;
    public boolean startedStreaming = false;
    public boolean enabledProcessor = false;

    public boolean localize(long timeOut) {
        teamUtil.log("Localize starting");
        long startTime = System.currentTimeMillis();
        long timeOutTime = startTime + timeOut;
        teamUtil.robot.blinkin.setSignal(Blinkin.Signals.COLORWAVESFORESTPALETTE);
        if (!visionPortalRunning) {
            teamUtil.log("Setting up Vision Portal");
            initCV(); // set up vision portal just in time
            startedJIT = true;
            startedStreaming = true;
        } else if (visionPortal.getCameraState() != VisionPortal.CameraState.STREAMING) {
            teamUtil.log("Resuming Streaming");
            visionPortal.resumeStreaming();
            startedJIT = false;
            startedStreaming = true;
        }
        while (visionPortal.getCameraState() != VisionPortal.CameraState.STREAMING && teamUtil.keepGoing(timeOutTime)) {
            teamUtil.pause(50);
        }
        // We are either streaming or timed out
        if (visionPortal.getCameraState() != VisionPortal.CameraState.STREAMING) {
            teamUtil.log("localize TIMED OUT waiting for streaming to start");
            if (startedJIT) visionPortal.close(); else if (startedStreaming) visionPortal.stopStreaming();
            teamUtil.robot.blinkin.setSignal(Blinkin.Signals.OFF);
            localizing.set(false);
            return false;
        }
        //teamUtil.log("Streaming Started in " + (System.currentTimeMillis() - startTime));
        if (!visionPortal.getProcessorEnabled(aprilTag)) {
            teamUtil.log("Enabling Processor");
            visionPortal.setProcessorEnabled(aprilTag, true);
            enabledProcessor = true;
        } else {
            enabledProcessor = false;
        }
        double sumX = 0;
        double sumY = 0;
        double sumH = 0;
        int count = 0;
        long sampleStopTime = System.currentTimeMillis() + SAMPLE_TIME;
        while (count < MAX_SAMPLES && System.currentTimeMillis() < sampleStopTime && teamUtil.keepGoing(timeOutTime)) {
            AprilTagLocalizer.IDPose idPose = getFreshRobotPose();
            Pose3D pose = idPose != null ? idPose.pose : null;
            if (pose != null) {
                Pose actualPose = camToActualV4(pose, idPose.ID);
                if (details) teamUtil.log(String.format("Localization: X: %.0f  Y: %.0f  H: %.1f", actualPose.x, actualPose.y, actualPose.heading));
                sumX += actualPose.x;
                sumY += actualPose.y;
                sumH += actualPose.heading;
                count++;
            }
            teamUtil.pause(35); // give time to AprilTagProcessor (based on 30fps)
        }
        if (count == 0) {
            teamUtil.log("localize failed to get any samples");
            if (startedJIT) visionPortal.close(); else if (startedStreaming) visionPortal.stopStreaming();
            teamUtil.robot.blinkin.setSignal(Blinkin.Signals.OFF);
            localizing.set(false);
            return false;
        }
        if (System.currentTimeMillis() >= timeOutTime) {
            teamUtil.log("localize TIMED OUT collecting samples");
            if (startedJIT) visionPortal.close(); else if (startedStreaming) visionPortal.stopStreaming();
            teamUtil.robot.blinkin.setSignal(Blinkin.Signals.OFF);
            localizing.set(false);
            return false;
        }
        // wrap up
        if (enabledProcessor) {
            teamUtil.log("Stopping Processor");
            visionPortal.setProcessorEnabled(aprilTag, false);
        }
        if (startedJIT) {
            teamUtil.log("Closing Portal");
            visionPortal.close();
        } else if (startedStreaming) {
            teamUtil.log("Stopping Streaming");
            visionPortal.stopStreaming();
        }
        teamUtil.robot.drive.loop(); // make sure we have current robot position
        double newX = sumX/count ;
        double newY = sumY/count;
        double newH = sumH/count;

        /*
        if (teamUtil.alliance== teamUtil.Alliance.RED) {
            newX = newX + ADJUST_RED_X;
            newY = newY + ADJUST_RED_Y;
            newH = newH + ADJUST_RED_H;
        } else if (teamUtil.alliance== teamUtil.Alliance.BLUE) {
            newX = newX + ADJUST_BLUE_X;
            newY = newY + ADJUST_BLUE_Y;
            newH = newH + ADJUST_BLUE_H;
        }
        */

        teamUtil.log("Localize finished in " +((System.currentTimeMillis() - startTime)));
        teamUtil.log(String.format("Camera: X: %.0f Y: %.0f H: %.1f Count: %d", newX, newY, newH, count));
        String data = String.format(Locale.US, "OQ X: %.0f, Y: %.0f, H: %.1f", (float) teamUtil.robot.drive.oQlocalizer.posX_mm, (float) teamUtil.robot.drive.oQlocalizer.posY_mm, Math.toDegrees(teamUtil.robot.drive.oQlocalizer.heading_rad));
        teamUtil.log(data);
        data = String.format(Locale.US, "Diff X: %.0f, Y: %.0f, H: %.1f", (float) teamUtil.robot.drive.oQlocalizer.posX_mm-newX, (float) teamUtil.robot.drive.oQlocalizer.posY_mm-newY, Math.toDegrees(teamUtil.robot.drive.oQlocalizer.heading_rad) - newH);
        teamUtil.log(data);
        teamUtil.robot.drive.loop();
        // Set the x,y and heading
        teamUtil.robot.drive.setRobotPosition((int)newX, (int)newY, newH);
        teamUtil.robot.blinkin.setSignal(Blinkin.Signals.OFF);
        localizing.set(false);
        return true;
    }
}
