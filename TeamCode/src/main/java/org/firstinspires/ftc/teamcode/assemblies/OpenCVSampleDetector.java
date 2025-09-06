package org.firstinspires.ftc.teamcode.assemblies;

import android.graphics.Bitmap;
import android.graphics.Canvas;
import android.graphics.Color;
import android.graphics.Paint;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.internal.camera.calibration.CameraCalibration;
import org.firstinspires.ftc.teamcode.libs.teamUtil;
import org.firstinspires.ftc.vision.VisionPortal;
import org.opencv.android.Utils;
import org.opencv.calib3d.Calib3d;
import org.opencv.core.Core;
import org.opencv.core.CvType;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.MatOfPoint2f;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.RotatedRect;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;

import java.util.ArrayList;
import java.util.List;
import java.util.concurrent.ConcurrentLinkedQueue;
import java.util.concurrent.atomic.AtomicBoolean;
import java.util.concurrent.atomic.AtomicInteger;
import java.util.concurrent.atomic.AtomicLong;

import com.acmerobotics.dashboard.config.Config;

@Config
public class OpenCVSampleDetector extends OpenCVProcessor {
    HardwareMap hardwareMap;
    Telemetry telemetry;



    Scalar BLACK = new Scalar(0, 0, 0);
    Scalar GREEN = new Scalar(0, 255, 0);


    static public final int WIDTH = 640;
    static public final int HEIGHT = 480;
    static public int OBSCURE_HEIGHT = 460;
    Rect cropRect = new Rect(0, 0, WIDTH, HEIGHT);

    static public int TARGET_X = 336;
    static public int TARGET_Y = 176;
    static public double CAMERA_OFFSET_Y = TARGET_Y-HEIGHT/2f;
    static public double CAMERA_OFFSET_X = TARGET_X-WIDTH/2f;
    static public int MIN_AREA_THRESHOLD = 2000;
    static public int MAX_AREA_THRESHOLD = 15000; // was 13000 for meet #2 build
    static public int GOLDILOCKS_ZONE_RADIUS = 160;

    public static float CAM_ANGLE_ADJUST_NEG_Y = .82f;
    public static float CAM_ANGLE_ADJUST_POS_Y = .84f;
    public static float CAM_ANGLE_ADJUST_POS_Y_B = -10f;
    public static float CAM_ANGLE_ADJUST_MID_Y = .85f;

    public static float CAM_ANGLE_ADJUST_NEG_X = .82f;
    public static float CAM_ANGLE_ADJUST_NEG_X_B = 0f;
    public static float CAM_ANGLE_ADJUST_POS_X = .84f;
    public static float CAM_ANGLE_ADJUST_MID_X = .85f;

    double[][] cameraMatrixData = {
            { 478.3939243528238, 0.0, 333.6082048642555 },
            { 0.0, 470.9276030162481, 297.451893471244 },
            { 0.0, 0.0, 1.0 }
    };

    // Define your distortion coefficients constants
    double[] distCoeffsData = { -0.3219047808733794, 0.09609610660662812, -0.04785830392770931, -0.0050045386058039, -0.05072346571900022 };


    Mat cameraMatrix = new Mat(3, 3, CvType.CV_64F);
    Mat distCoeffs = new Mat(1, 5, CvType.CV_64F);


    private final double CMS_PER_PIXEL_X = 0; //Set (Will most likely not be linear)
    private final double CMS_PER_PIXEL_Y = 0; //Set (Will most likely not be linear)

    static public boolean AFOCUS = true;
    static public boolean APEXPOSURE = false;
    static public boolean AEPRIORITY = false;
    static public boolean WHITEBALANCEAUTO = true;
    static public int FOCUSLENGTH =125;
    static public int GAIN = 150;
    static public int EXPOSURE = 4; // With Adafruit 12 led ring at full white (no RGB)
    static public int TEMPERATURE = 2800;

    static public int blurFactor = 10;
    static public boolean undistort = true;


    // Adafruit Ring max white (no rgb) at manual exposure 4
    static public int yellowLowH = 10, yellowLowS = 100, yellowLowV = 170;
    static public int yellowHighH = 35, yellowHighS = 255, yellowHighV = 255;
    static public int yellowErosionFactor = 20;
    static public int blueLowH = 90, blueLowS = 80, blueLowV = 90; // lowv was 70 meet #2 build
    static public int blueHighH = 130, blueHighS = 255, blueHighV = 255;
    static public int blueErosionFactor = 20;
    static public int rbyLowH = -1, rbyLowS = 150, rbyLowV = 125;
    static public int rbyHighH = 180, rbyHighS = 255, rbyHighV = 255;
    static public int redErosionFactor = 20;
    static public int redDilutionFactor = 10;


    //static public int CLOSEFACTOR = 20;

    public int sampleX = TARGET_X;
    public int sampleY = TARGET_Y;
    public static int FOUND_ONE_RIGHT_THRESHOLD = 620;
    public static int FOUND_ONE_LEFT_THRESHOLD = 20;

    static public int SAMPLE_SIZE = 1;
    //public double[] samplePixel = new double[3];
    Rect sampleRect;
    public Scalar sampleAvgs = new Scalar(0, 0, 0); // Average HSV values in sample rectangle
    int targetIndex = 0;



    private Mat HSVMat  = new Mat();
    private Mat blurredMat = new Mat();
    private Mat thresholdMat = new Mat();
    private Mat thresholdMatAll = new Mat();
    private Mat undistortedMat = new Mat();
    private Mat thresholdMatYellow = new Mat();
    private Mat thresholdMatBlue = new Mat();
    private Mat thresholdMatYB = new Mat();
    private Mat invertedMat = new Mat();
    private Mat floodFillMask = new Mat();
    private Mat erodedMat = new Mat();
    private Mat edgesMat = new Mat();
    private Mat hierarchyMat = new Mat();


    public enum TargetColor {
        YELLOW,
        RED,
        BLUE
    }
    static public TargetColor targetColor = TargetColor.YELLOW;
    public void setTargetColor(TargetColor newTargetColor){
        targetColor = newTargetColor;
    }

    public AtomicBoolean foundOne = new AtomicBoolean(false);
    public AtomicBoolean processedFrame = new AtomicBoolean(false);
    public AtomicLong processTime = new AtomicLong(0);
    private long startProcessingTime = 0;

    public AtomicBoolean outsideUseableCameraRange = new AtomicBoolean(false);
    // Data about the located Sample
    //old atomic integers
    public AtomicInteger rectAngle = new AtomicInteger(-1);
    public AtomicInteger rectCenterXOffset = new AtomicInteger(0);
    public AtomicInteger rectCenterYOffset = new AtomicInteger(0);
    public AtomicInteger rectArea = new AtomicInteger(0);



    public static class FrameData {
        public int rectAngle =-1;
        public int rectCenterXOffset = 0;
        public int rectCenterYOffset = 0;
        public int adjRectCenterXOffset = 0;
        public int adjRectCenterYOffset = 0;
        public int rectArea = 0;
        public RotatedRect rrect;
        public int lowestPixelX = 0;
        public int lowestPixelY = 0;

        public int closestPixelX = 0;
        public int closestPixelY = 0;

        public long captureTime = 0;
        public long processingTime = 0;
        public long processingTime2 = 0;

    }
    public ConcurrentLinkedQueue<FrameData> frameDataQueue = new ConcurrentLinkedQueue<FrameData>();


    public boolean viewingPipeline = false;
    enum Stage {
        RAW_IMAGE,
        UNDISTORTED,
        HSV,
        BLURRED,
        THRESHOLD,
        ERODED,
        EDGES,
    }
    private Stage stageToRenderToViewport = Stage.UNDISTORTED;
    private Stage[] stages = Stage.values();

    private VisionPortal myPortal;

    class Context {
        RotatedRect[] rots;
        List<MatOfPoint> contours;
    }

    public OpenCVSampleDetector() {
        hardwareMap = teamUtil.theOpMode.hardwareMap;
        telemetry = teamUtil.telemetry;
    }
    @Override
    public void init(int width, int height, CameraCalibration calibration) {
        teamUtil.log("Initializing OpenCVSampleDetector processor");
        for (int i=0;i<3;i++)
            cameraMatrix.put(i,0, cameraMatrixData[i]); // preload calibration data
        distCoeffs.put(0, 0, distCoeffsData);

        teamUtil.log("Initializing OpenCVSampleDetector processor - FINISHED");
    }
    public void setVisionPortal(VisionPortal portal) {
        myPortal = portal;

    }

    public String frameString(FrameData frame) {
        if (frame == null) {
            return "No Detection";
        } else {
            return "Detection x,y: " + frame.rectCenterXOffset + ", " + frame.rectCenterYOffset
                    + " Angle: " + frame.rectAngle
                    + " Area: " + frame.rectArea
                    + " Processing Time: " + frame.processingTime
                    + " Adjusted Target: " + frame.adjRectCenterXOffset + "," + frame.adjRectCenterYOffset
                    + " Lowest Pixel: " + frame.lowestPixelX + "," + frame.lowestPixelY
                    + " Closest Pixel: " + frame.closestPixelX + "," + frame.closestPixelY
                    //+ " Capture Time: " + frame.captureTime
                    //+ " Processing Time2: " + frame.processingTime2
                    ;
        }
    }

    public void outputTelemetry () {
        telemetry.addLine("Processed Frame: " + processedFrame.get());
        //telemetry.addLine("queue size: "+ frameDataQueue.size());
        telemetry.addLine(frameString(frameDataQueue.peek()));
        //teamUtil.log(frameString(frameDataQueue.peek()));
    }


    public void sampleUp(int step) {
        sampleY = sampleY - step;
        if (sampleY < 0) sampleY = 0;
    }
    public void sampleLeft(int step) {
        sampleX = sampleX- step;
        if (sampleX< 0) sampleX = 0;
    }
    public void sampleDown(int step) {
        sampleY = sampleY + step;
        if (sampleY > HEIGHT-SAMPLE_SIZE-1) sampleY = HEIGHT-SAMPLE_SIZE-1;
    }
    public void sampleRight(int step) {
        sampleX = sampleX + step;
        if (sampleX> WIDTH-SAMPLE_SIZE-1) sampleX= WIDTH-SAMPLE_SIZE-1;
    }
    public void nextView() {

        int currentStageNum = stageToRenderToViewport.ordinal();
        int nextStageNum = currentStageNum + 1;
        if (nextStageNum >= stages.length) {
            nextStageNum = 0;
        }
        stageToRenderToViewport = stages[nextStageNum];
    }

    List<MatOfPoint> contours = new ArrayList<>();
    double largestArea;


    public void reset() {
        frameDataQueue.clear(); // remove any existing frame data from queue
        foundOne.set(false);
        processedFrame.set(false);
    }

    public FrameData processNextFrame(boolean waitForDetection, boolean turnOffProcessor, boolean captureImages, long timeOut){
        boolean details = true;
        teamUtil.log("ProcessNextFrame");
        long timeoutTime = System.currentTimeMillis() + timeOut;
        myPortal.setProcessorEnabled(this, true);
        startProcessingTime = System.currentTimeMillis();
        reset();

        FrameData imgData = null;
        while( !(waitForDetection ? (imgData != null) : processedFrame.get()) && teamUtil.keepGoing(timeoutTime)){
            teamUtil.pause(30);
            imgData = frameDataQueue.poll();
        }
        if(System.currentTimeMillis()>=timeoutTime){
            teamUtil.log("ProcessNextFrame Timed Out; No detection.");
        }
        if (turnOffProcessor) {
            myPortal.setProcessorEnabled(this, false);
        }
        teamUtil.log("ProcessNextFrame---Finished");
        return imgData;
    }

    @Override
    public Object processFrame(Mat frame, long captureTimeNanos) {
        boolean details = false;
        if (details) teamUtil.log("Sample Detector: Process Frame");

        // Set up the various objects needed to do the image processing
        // This is done here instead of at the class level so these can be adjusted using Acme Dashboard
        Size blurFactorSize = new Size(blurFactor, blurFactor);
        Scalar yellowLowHSV = new Scalar(yellowLowH, yellowLowS, yellowLowV); // lower bound HSV for yellow
        Scalar yellowHighHSV = new Scalar(yellowHighH, yellowHighS, yellowHighV); // higher bound HSV for yellow
        Mat yellowErosionElement = Imgproc.getStructuringElement(Imgproc.MORPH_RECT, new Size(2 * yellowErosionFactor + 1, 2 * yellowErosionFactor + 1),
                new Point(yellowErosionFactor, yellowErosionFactor));
        Scalar blueLowHSV = new Scalar(blueLowH, blueLowS, blueLowV); // lower bound HSV for yellow
        Scalar blueHighHSV = new Scalar(blueHighH, blueHighS, blueHighV); // higher bound HSV for yellow
        Mat blueErosionElement = Imgproc.getStructuringElement(Imgproc.MORPH_RECT, new Size(2 * blueErosionFactor + 1, 2 * blueErosionFactor + 1),
                new Point(blueErosionFactor, blueErosionFactor));
        Scalar rbyLowHSV = new Scalar(rbyLowH, rbyLowS, rbyLowV); // lower bound HSV for yellow
        Scalar rbyHighHSV = new Scalar(rbyHighH, rbyHighS, rbyHighV); // higher bound HSV for yellow
        Mat redErosionElement = Imgproc.getStructuringElement(Imgproc.MORPH_RECT, new Size(2 * redErosionFactor + 1, 2 * redErosionFactor + 1),
                new Point(redErosionFactor, redErosionFactor));

        Mat redDilutionElement = Imgproc.getStructuringElement(Imgproc.MORPH_RECT, new Size(2 * redDilutionFactor + 1, 2 * redDilutionFactor + 1),
                new Point(redDilutionFactor, redDilutionFactor));
        Rect obscureRect = new Rect(0,OBSCURE_HEIGHT,WIDTH,HEIGHT-OBSCURE_HEIGHT);

        undistortedMat = frame.clone();


        Imgproc.rectangle(undistortedMat, obscureRect, BLACK, -1); // Cover view of robot
        Imgproc.cvtColor(undistortedMat, HSVMat, Imgproc.COLOR_RGB2HSV); // convert to HSV
        Imgproc.blur(HSVMat, blurredMat, blurFactorSize); // get rid of noise


        switch (targetColor) {
            case YELLOW:
                Core.inRange(blurredMat, yellowLowHSV, yellowHighHSV, thresholdMat);

                Imgproc.erode(thresholdMat, erodedMat, yellowErosionElement);
                break;
            case BLUE:
                Core.inRange(blurredMat, blueLowHSV, blueHighHSV, thresholdMat);

                Imgproc.erode(thresholdMat, erodedMat, blueErosionElement);
                break;
            case RED:
                Core.inRange(blurredMat, rbyLowHSV, rbyHighHSV, thresholdMatAll); // Get Red and Yellow

                Core.inRange(blurredMat, yellowLowHSV, yellowHighHSV, thresholdMatYellow);
                Core.inRange(blurredMat, blueLowHSV, blueHighHSV, thresholdMatBlue);
                Core.add(thresholdMatYellow, thresholdMatBlue,  thresholdMatYB);
                Imgproc.erode(thresholdMatYB, erodedMat, yellowErosionElement); // Get yellow and blue eroded rects
                Imgproc.Canny(erodedMat, edgesMat, 100, 300); // find edges
                //thresholdMatYB = thresholdMat.clone();
                thresholdMatYB.setTo(new Scalar(0));
                contours.clear(); // empty the list from last time
                Imgproc.findContours(edgesMat, contours, hierarchyMat, Imgproc.RETR_TREE, Imgproc.CHAIN_APPROX_SIMPLE); // find contours around yellow/blue rects
                if (!contours.isEmpty()) {
                    MatOfPoint2f[] contoursPoly = new MatOfPoint2f[contours.size()];
                    RotatedRect[] boundRect = new RotatedRect[contours.size()];
                    for (int i = 0; i < contours.size(); i++) {  // for each yellow rect, draw a white rectangle inside it
                        contoursPoly[i] = new MatOfPoint2f();
                        Imgproc.approxPolyDP(new MatOfPoint2f(contours.get(i).toArray()), contoursPoly[i], 3, true);
                        boundRect[i] = Imgproc.minAreaRect(contoursPoly[i]);
                        Point[] points = new Point[4];
                        boundRect[i].points(points);
                        MatOfPoint r = new MatOfPoint(points);
                        Imgproc.fillConvexPoly(thresholdMatYB, r, new Scalar(255));
                    }
                }
                Core.subtract(thresholdMatAll, thresholdMatYB,  thresholdMat);

                Imgproc.erode(thresholdMat, erodedMat, redErosionElement);

                /* Previous algorithm for subtracting yellow and blue threshold mats from combined threshold mat--left too many small holes that were tough to fill
                Core.inRange(blurredMat, rbyLowHSV, rbyHighHSV, thresholdMatAll); // Get Red and Yellow
                Core.inRange(blurredMat, yellowLowHSV, yellowHighHSV, thresholdMatYellow);
                Core.inRange(blurredMat, blueLowHSV, blueHighHSV, thresholdMatBlue);
                Core.add(thresholdMatBlue, thresholdMatYellow, thresholdMatYB); // combine yellow and blue threshold mat
                Core.subtract(thresholdMatAll, thresholdMatYB, thresholdMat); // Then subtract from all to get red

                Core.bitwise_not(thresholdMat,invertedMat);
                Imgproc.rectangle(invertedMat, cropRect, new Scalar(255), 2); // white frame to provide seed and propagation for fill.
                floodFillMask = new Mat();
                Imgproc.floodFill(invertedMat, floodFillMask, new Point(0,0),BLACK);
                //Imgproc.Canny(dilutedMat, edgesMat, 100, 300); // find edges
                List<MatOfPoint> contours = new ArrayList<>();
                contours.clear(); // empty the list from last time
                Imgproc.findContours(invertedMat, contours, hierarchyMat, Imgproc.RETR_TREE, Imgproc.CHAIN_APPROX_SIMPLE); // find contours around holes we need to fill
                //for (int i = 0; i < contours.size(); i++) {
                Imgproc.drawContours(thresholdMat,contours,-1,BLACK,-1); // fill the holes back on the original threshold mat

                Imgproc.erode(thresholdMat, erodedMat, redErosionElement);

                 */
                break;

        }
        Imgproc.rectangle(erodedMat, cropRect, BLACK, 2); // black frame to help with edges and boundaries.

        Imgproc.Canny(erodedMat, edgesMat, 100, 300); // find edges


        List<MatOfPoint> contours = new ArrayList<>();
        contours.clear(); // empty the list from last time
        Imgproc.findContours(edgesMat, contours, hierarchyMat, Imgproc.RETR_TREE, Imgproc.CHAIN_APPROX_SIMPLE); // find contours around white areas

        if (contours.isEmpty()) {
            if (details) teamUtil.log("No Contours");
            foundOne.set(false);
            processedFrame.set(true);
            return null;
        }

        // We found at least one blob of the right color
        MatOfPoint2f[] contoursPoly = new MatOfPoint2f[contours.size()];
        RotatedRect[] rotatedRect = new RotatedRect[contours.size()];
        double closestAreaSelection = 1000; // TODO: What is this?
        int closestAreaSelectionNum = -1;
        double xDistCenter;
        double yDistCenter;

        //Create Rotated Rectangles from contours and identify the one closest to the target that meets the threshold size
        for (int i = 0; i < contours.size(); i++) {
            contoursPoly[i] = new MatOfPoint2f();
            Imgproc.approxPolyDP(new MatOfPoint2f(contours.get(i).toArray()), contoursPoly[i], 3, true);
            rotatedRect[i] = Imgproc.minAreaRect(contoursPoly[i]);

            // Is the closest one to the target we have seen so far?
            xDistCenter = Math.abs(rotatedRect[i].center.x - TARGET_X);
            yDistCenter = Math.abs(rotatedRect[i].center.y - TARGET_Y);
            if (Math.hypot(xDistCenter, yDistCenter) < closestAreaSelection && rotatedRect[i].size.area() > MIN_AREA_THRESHOLD&&rotatedRect[i].size.area() < MAX_AREA_THRESHOLD) {
                closestAreaSelection = Math.hypot(xDistCenter, yDistCenter);
                closestAreaSelectionNum = i;
            }
        }

        if (closestAreaSelectionNum == -1) { // nothing big enough
            if (details) teamUtil.log("Saw blobs but nothing big enough");
            foundOne.set(false);
            processedFrame.set(true);
            return null;
        }
        /*
        if(rotatedRect[closestAreaSelectionNum].size.area()>5){
            if (details) teamUtil.log("Saw Too Big Of A Blob");
            foundOne.set(false);
            return null;
        }

         */
        rectArea.set((int)rotatedRect[closestAreaSelectionNum].size.area());

        //Find the point with the lowest y coordinate
        Point vertices1[] = new Point[4];
        rotatedRect[closestAreaSelectionNum].points(vertices1); // get the 4 corners of the rotated rectangle
        int lowestPixel = 0;
        for (int j = 1; j < 4; j++) {
            if (vertices1[j].y > vertices1[lowestPixel].y) {
                lowestPixel = j;
            }
        }

        //Find the point closest to lowest point
        int closestPixel = 0;
        double closestDist = 1000;
        for (int k = 0; k < 4; k++) {
            if (k == lowestPixel) {
            } else {
                double xDiff = Math.abs(vertices1[lowestPixel].x - vertices1[k].x);
                double yDiff = Math.abs(vertices1[lowestPixel].x - vertices1[k].y);
                if (Math.hypot(xDiff, yDiff) < closestDist) {
                    closestDist = Math.hypot(xDiff, yDiff);
                    closestPixel = k;
                }
            }

        }

        double xDist = Math.abs(vertices1[closestPixel].x - vertices1[lowestPixel].x);
        double yDist = Math.abs(vertices1[closestPixel].y - vertices1[lowestPixel].y);
        double realAngle = Math.toDegrees(Math.atan(yDist / xDist));
        if (vertices1[closestPixel].x < vertices1[lowestPixel].x) {
            realAngle += 90;
        } else {
            realAngle = 90 - realAngle;
        }
        /*
        //TODO POssible change later
        if (Math.abs((int) realAngle - rectAngle.get()) > 2) {
            rectAngle.set((int) realAngle);
        }

        if (Math.abs((int) rotatedRect[closestAreaSelectionNum].center.y - rectCenterYOffset.get()) > 2) {
            rectCenterYOffset.set(-1 * ((int) rotatedRect[closestAreaSelectionNum].center.y - TARGET_Y));
        }
        if (Math.abs((int) rotatedRect[closestAreaSelectionNum].center.x - rectCenterXOffset.get()) > 2) {
            rectCenterXOffset.set((int) rotatedRect[closestAreaSelectionNum].center.x - TARGET_X);
        }


         */
        if(rotatedRect[closestAreaSelectionNum].center.x<FOUND_ONE_LEFT_THRESHOLD||rotatedRect[closestAreaSelectionNum].center.x>FOUND_ONE_RIGHT_THRESHOLD){
            if (details) {
                teamUtil.log("OPEN CV SAMPLE DETECTOR FOUND BLOCK BUT CENTER X VALUE WAS OUTSIDE MECHANICAL RANGE");
                teamUtil.log("X of Block Center: " + rotatedRect[closestAreaSelectionNum].center.x);
                teamUtil.log("Y of Block Center: " + rotatedRect[closestAreaSelectionNum].center.y);
            }
            outsideUseableCameraRange.set(true);
            processedFrame.set(true);
            foundOne.set(false);
        }
        else{
            outsideUseableCameraRange.set(false);
            processedFrame.set(true);

            FrameData imgData = new FrameData();
            imgData.rrect = rotatedRect[closestAreaSelectionNum];
            imgData.rectAngle = (int) realAngle;
            imgData.rectCenterXOffset = (int) rotatedRect[closestAreaSelectionNum].center.x - TARGET_X;
            imgData.rectCenterYOffset = -1 * ((int) rotatedRect[closestAreaSelectionNum].center.y - TARGET_Y);
            imgData.adjRectCenterXOffset=  adjustXCenter((int)rotatedRect[closestAreaSelectionNum].center.x) - TARGET_X;
            imgData.adjRectCenterYOffset= -1 * (adjustYCenter((int)rotatedRect[closestAreaSelectionNum].center.y) - TARGET_Y);
            imgData.rectArea = (int) rotatedRect[closestAreaSelectionNum].size.area();
            imgData.captureTime = captureTimeNanos/1000000;
            imgData.processingTime = System.currentTimeMillis() - startProcessingTime;
            imgData.processingTime2 = System.currentTimeMillis() - imgData.captureTime;

            imgData.lowestPixelX = (int) vertices1[lowestPixel].x;
            imgData.lowestPixelY = (int) vertices1[lowestPixel].y;
            imgData.closestPixelX = (int) vertices1[closestPixel].x;
            imgData.closestPixelY = (int) vertices1[closestPixel].y;

            frameDataQueue.clear(); // Only one object in the queue at a time for now.
            frameDataQueue.add(imgData);
            if (details) teamUtil.log("adding detection to queue. size: " + frameDataQueue.size());

            // old data setting
            rectCenterYOffset.set(-1 * ((int) rotatedRect[closestAreaSelectionNum].center.y - TARGET_Y));
            rectCenterXOffset.set((int) rotatedRect[closestAreaSelectionNum].center.x - TARGET_X);
            rectAngle.set((int) realAngle);
            rectArea.set((int) rotatedRect[closestAreaSelectionNum].size.area());
            foundOne.set(true);

            targetIndex = closestAreaSelectionNum;
            if (details) teamUtil.log("Real Angle" + realAngle + "Lowest: " + vertices1[lowestPixel].x + "," + vertices1[lowestPixel].y+"Closest: " + vertices1[closestPixel].x+ "," +vertices1[closestPixel].y);
        }

        Context context = new Context();
        context.rots = rotatedRect;
        context.contours = contours;
        return context;
    }




    public int adjustYCenter(int y) {
        if (TARGET_Y < HEIGHT/2) { // uniform distortion to the top, but varying to the bottom as we cross the center
            if (y < TARGET_Y) {
                return (int) (TARGET_Y - (TARGET_Y-y) * CAM_ANGLE_ADJUST_NEG_Y); // assumes linear from target but it really is linear from center of cam (might need a "b" coeffecient)
            } else if (y > TARGET_Y) {
                if (y < HEIGHT/2) { // between target and center of camera (as we move away from target we are approaching the center of the camera so distances get less distorted)
                    return (int) ((y-TARGET_Y) * CAM_ANGLE_ADJUST_MID_Y + TARGET_Y);
                } else { // below target and below center of camera (as we move away from center of camera, distances get more distorted
                    return (int) ((y-HEIGHT/2) * CAM_ANGLE_ADJUST_POS_Y + HEIGHT/2+ CAM_ANGLE_ADJUST_POS_Y_B); // linear from center of cam, so needed a "b" coefficient to deal with distance from target to center of cam
                }
            } else { // already over the target
                return y;
            }
        } else {
            teamUtil.log("adjustYTarget Case Not Implemented!");
            return y;
        }
    }

    public int adjustXCenter(int x) {
        if (TARGET_X > WIDTH/2) { // // uniform distortion to the right, but varying to the left as we cross the center
            if (x > TARGET_X) {
                return (int) (TARGET_X + (x-TARGET_X) * CAM_ANGLE_ADJUST_POS_X); // assumes linear from target but it really is linear from center of cam (might need a "b" coeffecient)
            } else if (x < TARGET_X) {
                if (x > WIDTH/2) { // between target and center of camera (as we move away from target we are approaching the center of the camera so distances get less distorted)
                    return (int) (TARGET_X - (TARGET_X-x) * CAM_ANGLE_ADJUST_MID_X);
                } else { // left of target and left of center of camera (as we move away from center of camera, distances get more distorted
                    return (int) ( WIDTH/2 - (WIDTH/2-x) * CAM_ANGLE_ADJUST_NEG_X + CAM_ANGLE_ADJUST_NEG_X_B); // linear from center of cam, so needed a "b" coefficient to deal with distance from target to center of cam
                }
            } else { // already over the target
                return x;
            }
        } else {
            teamUtil.log("adjustXTarget Case Not Implemented!");
            return x;
        }
    }


    @Override
    public void onDrawFrame(Canvas canvas, int onscreenWidth, int onscreenHeight, float scaleBmpPxToCanvasPx, float scaleCanvasDensity, Object userContext) {

        //samplePixel = blurredMat.get(sampleX,sampleY);
        sampleRect = new Rect(sampleX-SAMPLE_SIZE/2,sampleY-SAMPLE_SIZE/2,SAMPLE_SIZE,SAMPLE_SIZE);
        sampleAvgs = getAverages(blurredMat,sampleRect);

        // Use the appropriate background if we are viewing the pipeline
        if (viewingPipeline) {
            Bitmap bmp = Bitmap.createBitmap(HSVMat.cols(), HSVMat.rows(), Bitmap.Config.ARGB_8888);
            switch (stageToRenderToViewport) {
                case UNDISTORTED: { Utils.matToBitmap(undistortedMat,bmp); break;}
                case HSV: { Utils.matToBitmap(HSVMat, bmp); break; }
                case BLURRED: { Utils.matToBitmap(blurredMat, bmp); break;}
                case THRESHOLD: { Utils.matToBitmap(thresholdMat, bmp); break;}
                case ERODED: { Utils.matToBitmap(erodedMat, bmp); break;}
                case EDGES: { Utils.matToBitmap(edgesMat, bmp); break;}
                default: {/*RAW image is already on canvas*/}
            }
            Bitmap resizedBitmap = Bitmap.createScaledBitmap(bmp, (int)(WIDTH*scaleBmpPxToCanvasPx), (int)(HEIGHT*scaleBmpPxToCanvasPx), false);
            canvas.drawBitmap(resizedBitmap, 0,0,null);
        }


        Paint samplePaint = new Paint();
        samplePaint.setColor(Color.GREEN);
        samplePaint.setStyle(Paint.Style.STROKE);
        samplePaint.setStrokeWidth(scaleCanvasDensity * 4);
        canvas.drawCircle(WIDTH/2*scaleBmpPxToCanvasPx, HEIGHT/2*scaleBmpPxToCanvasPx, GOLDILOCKS_ZONE_RADIUS,samplePaint);


        canvas.drawCircle((float)TARGET_X*scaleBmpPxToCanvasPx, (float)TARGET_Y*scaleBmpPxToCanvasPx, 10,samplePaint);
        canvas.drawRect((float)sampleRect.tl().x*scaleBmpPxToCanvasPx, (float)sampleRect.tl().y*scaleBmpPxToCanvasPx, (float)sampleRect.br().x*scaleBmpPxToCanvasPx, (float)sampleRect.br().y*scaleBmpPxToCanvasPx,samplePaint);

        if (userContext != null) {
            RotatedRect[] rotatedRect = ((Context) userContext).rots;
            if (rotatedRect != null) {
                Paint rectPaint = new Paint();
                rectPaint.setColor(Color.MAGENTA);
                rectPaint.setStyle(Paint.Style.STROKE);
                rectPaint.setStrokeWidth(scaleCanvasDensity * 6);
                Paint anglePaint = new Paint();
                anglePaint.setColor(Color.CYAN);
                anglePaint.setStyle(Paint.Style.STROKE);
                anglePaint.setTextSize(20);
                anglePaint.setStrokeWidth(scaleCanvasDensity * 6);
                Paint centerPaint = new Paint();
                centerPaint.setColor(Color.BLACK);
                centerPaint.setStyle(Paint.Style.STROKE);
                centerPaint.setStrokeWidth(scaleCanvasDensity * 6);

                for (int i = 0; i < rotatedRect.length; i++) {

                    // Draw rotated Rectangle
                    Point vertices[] = new Point[4];
                    rotatedRect[i].points(vertices);
                    for (int j = 0; j < 4; j++) {
                        canvas.drawLine((float)vertices[j].x*scaleBmpPxToCanvasPx,(float)vertices[j].y*scaleBmpPxToCanvasPx,(float)vertices[(j+1)%4].x*scaleBmpPxToCanvasPx,(float)vertices[(j+1)%4].y*scaleBmpPxToCanvasPx,rectPaint);
                    }
                    if (targetIndex == i) {
                        // Draw Center
                        canvas.drawCircle((float)rotatedRect[i].center.x*scaleBmpPxToCanvasPx, (float)rotatedRect[i].center.y*scaleBmpPxToCanvasPx, 5,centerPaint);

                        // Draw angle vector
                        int endX = (int) (rotatedRect[i].center.x + 20 * Math.cos(rectAngle.get() * 3.14 / 180.0));
                        int endY =  (int) (rotatedRect[i].center.y + 20 * Math.sin(rectAngle.get() * 3.14 / 180.0));
                        canvas.drawLine((float)rotatedRect[i].center.x*scaleBmpPxToCanvasPx,(float)rotatedRect[i].center.y*scaleBmpPxToCanvasPx,(float)endX*scaleBmpPxToCanvasPx,(float)endY*scaleBmpPxToCanvasPx,rectPaint);
                        //Imgproc.putText(matImgDst, String.valueOf((int)boundRect[i].angle),boundRect[i].center,0,1,PASTEL_GREEN);
                        canvas.drawText(Integer.toString(rectAngle.get()),(float)rotatedRect[i].center.x*scaleBmpPxToCanvasPx,(float)rotatedRect[i].center.y*scaleBmpPxToCanvasPx,anglePaint);
                    }

                }
            }
            List<MatOfPoint> drawContours = ((Context) userContext).contours;
            if (drawContours != null) {
                if (!drawContours.isEmpty()) {
                    Paint polyPaint = new Paint();
                    polyPaint.setColor(Color.WHITE);
                    polyPaint.setStyle(Paint.Style.STROKE);
                    polyPaint.setStrokeWidth(scaleCanvasDensity * 6);

                    List<MatOfPoint> polygons = new ArrayList<>();
                    for (MatOfPoint contour : drawContours) {
                        MatOfPoint2f contour2f = new MatOfPoint2f(contour.toArray());
                        MatOfPoint2f polygon2f = new MatOfPoint2f();
                        //Imgproc.approxPolyDP(contour2f, polygon2f, 0.02 * Imgproc.arcLength(contour2f, true), true);
                        Imgproc.approxPolyDP(contour2f, polygon2f, 3, true);
                        Point[] points = polygon2f.toArray();
                        for (int i = 0; i < points.length - 1; i++) {
                            canvas.drawLine((float) points[i].x * scaleBmpPxToCanvasPx, (float) points[i].y * scaleBmpPxToCanvasPx, (float) points[i + 1].x * scaleBmpPxToCanvasPx, (float) points[i + 1].y * scaleBmpPxToCanvasPx, polyPaint);
                        }
                        canvas.drawLine((float) points[0].x * scaleBmpPxToCanvasPx, (float) points[0].y * scaleBmpPxToCanvasPx, (float) points[points.length-1].x * scaleBmpPxToCanvasPx, (float) points[points.length-1].y * scaleBmpPxToCanvasPx, polyPaint);
                    }
                }
            }

        }
        // TODO  figure out how to save a processed frame (Android canvas) to the local storage (SDCard)
        // TODO: set up an Atomic boolean to trigger this behavior for a single frame?
    }


}