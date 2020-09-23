package org.firstinspires.ftc.teamcode.drive;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.base;
import org.firstinspires.ftc.teamcode.vision.ultimategoalwebcampipeline;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvInternalCamera;
import org.openftc.easyopencv.OpenCvPipeline;
import org.openftc.easyopencv.OpenCvWebcam;

// FOR MECANUM DRIVETRAINS
// RECOMMENDED USE FOR OPENCV AND NO OTHER EXTERNAL REPOSITORIES

@Autonomous(name="16633: Auto Red Build Wall", group="16633")
//@Disabled
public class ultimategoalwebcamauto extends LinearOpMode {

    /* Declare OpMode members. */
    base robot   = new base();
    private ElapsedTime runtime = new ElapsedTime();

    // Variables to control Speed
    double velocity = 0.5; // Default velocity

    // OpenCV Init
    private OpenCvCamera webcam;
    ultimategoalwebcampipeline.SkystoneDeterminationPipeline pipeline;

    @Override
    public void runOpMode() {
        // Pipeline Init
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        // OpenCVWebcam instantiation not fully debugged
        webcam = new OpenCvWebcam(hardwareMap.get(WebcamName.class, "Webcam"), cameraMonitorViewId);
        pipeline = new ultimategoalwebcampipeline.SkystoneDeterminationPipeline();
        webcam.setPipeline(pipeline);

        // We set the viewport policy to optimized view so the preview doesn't appear 90 deg
        // out when the RC activity is in portrait. We do our actual image processing assuming
        // landscape orientation, though.
        webcam.setViewportRenderingPolicy(OpenCvCamera.ViewportRenderingPolicy.OPTIMIZE_VIEW);

        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
        {
            @Override
            public void onOpened()
            {
                // Make sure @width and @height fit in your webcam's resolution
                webcam.startStreaming(320,240, OpenCvCameraRotation.SIDEWAYS_LEFT);
            }
        });

        /*
         * Initialize the drive system variables.
         * The init() method of the hardware class does all the work here
         */
        robot.init(hardwareMap);

        // Send telemetry message to signify robot waiting;
        telemetry.addData("Status", "Ready to run");    //
        telemetry.update();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        telemetry.addData("Analysis", pipeline.getAnalysis());
        telemetry.addData("Position", pipeline.position);
        telemetry.update();

        public static class SkystoneDeterminationPipeline extends OpenCvPipeline {
            /*
             * An enum to define the skystone position
             */
            public enum RingPosition {
                FOUR,
                ONE,
                NONE
            }

            /*
             * Some color constants
             */
            static final Scalar BLUE = new Scalar(0, 0, 255);
            static final Scalar GREEN = new Scalar(0, 255, 0);

            /*
             * The core values which define the location and size of the sample regions
             * ADJUST TO MATCH CAMERA
             */
            static final Point REGION1_TOPLEFT_ANCHOR_POINT = new Point(181, 98);

            static final int REGION_WIDTH = 35;
            static final int REGION_HEIGHT = 25;

            final int FOUR_RING_THRESHOLD = 150;
            final int ONE_RING_THRESHOLD = 135;

            Point region1_pointA = new Point(
                    REGION1_TOPLEFT_ANCHOR_POINT.x,
                    REGION1_TOPLEFT_ANCHOR_POINT.y);
            Point region1_pointB = new Point(
                    REGION1_TOPLEFT_ANCHOR_POINT.x + REGION_WIDTH,
                    REGION1_TOPLEFT_ANCHOR_POINT.y + REGION_HEIGHT);

            /*
             * Working variables
             */
            Mat region1_Cb;
            Mat YCrCb = new Mat();
            Mat Cb = new Mat();
            int avg1;

            // Volatile since accessed by OpMode thread w/o synchronization
            private volatile ultimategoalwebcampipeline.SkystoneDeterminationPipeline.RingPosition position = EasyOpenCVExample.SkystoneDeterminationPipeline.RingPosition.FOUR;

            /*
             * This function takes the RGB frame, converts to YCrCb,
             * and extracts the Cb channel to the 'Cb' variable
             */
            void inputToCb(Mat input) {
                Imgproc.cvtColor(input, YCrCb, Imgproc.COLOR_RGB2YCrCb);
                Core.extractChannel(YCrCb, Cb, 1);
            }

            @Override
            public void init(Mat firstFrame) {
                inputToCb(firstFrame);

                region1_Cb = Cb.submat(new Rect(region1_pointA, region1_pointB));
            }

            @Override
            public Mat processFrame(Mat input) {
                inputToCb(input);

                avg1 = (int) Core.mean(region1_Cb).val[0];

                Imgproc.rectangle(
                        input, // Buffer to draw on
                        region1_pointA, // First point which defines the rectangle
                        region1_pointB, // Second point which defines the rectangle
                        BLUE, // The color the rectangle is drawn in
                        2); // Thickness of the rectangle lines

                position = ultimategoalwebcampipeline.SkystoneDeterminationPipeline.RingPosition.FOUR; // Record our analysis
                if (avg1 > FOUR_RING_THRESHOLD) {
                    position = ultimategoalwebcampipeline.SkystoneDeterminationPipeline.RingPosition.FOUR;
                } else if (avg1 > ONE_RING_THRESHOLD) {
                    position = ultimategoalwebcampipeline.SkystoneDeterminationPipeline.RingPosition.ONE;
                } else {
                    position = ultimategoalwebcampipeline.SkystoneDeterminationPipeline.RingPosition.NONE;
                }

                Imgproc.rectangle(
                        input, // Buffer to draw on
                        region1_pointA, // First point which defines the rectangle
                        region1_pointB, // Second point which defines the rectangle
                        GREEN, // The color the rectangle is drawn in
                        -1); // Negative thickness means solid fill

                return input;
            }

            public int getAnalysis() {
                return avg1;
            }
        }
        // Perform the steps of Autonomous One Step at a Time.
        // Add Code here!
        // Available Calls (SPECIFIC TO BASE AND CONFIGURATION): forward(inches),delay(seconds),right(degrees),left(degrees)
        // robot.leftHand.setPosition(), robot.rightHand.setPosition()
        //robot.strafeInches(0.8, 12, 5);  // Speed, inches, timeout seconds

        // Demonstration of OpenCV position; take out telemetry for function of what your robot will do
        if (position == ultimategoalwebcampipeline.SkystoneDeterminationPipeline.RingPosition.FOUR) {
            telemetry.addData("Target Zone", 'C');
            telemetry.update();
        } else if (position == ultimategoalwebcampipeline.SkystoneDeterminationPipeline.RingPosition.ONE) {
            telemetry.addData("Target Zone", 'B');
            telemetry.update();
        } else {
            telemetry.addData("Target Zone", 'A');
            telemetry.update();
        }

        // Random sample mecanum code
        /*
        robot.strafeInches(0.8, -10, 1);
        forward(-38);
        delay(0.5);
        robot.foundationL.setPosition(0);
        robot.foundationR.setPosition(1);
        delay(1);
        forward(27);
        right(140);
        robot.strafeInches(0.8, 10, 1);
        forward(-50);
        robot.foundationL.setPosition(1);
        delay(1);
        robot.foundationR.setPosition(0);
        delay(1);
        robot.strafeInches(0.8, -20, 1);
        forward(45);
        // Autonomous Finished
        telemetry.addData("Path", "Complete");
        telemetry.update();
        //sleep(1000);
        */
    }

    // Functions for REACH 2019 based on Python Turtles
    public void forward(double inches)
    {
        robot.driveStraightInches(velocity, inches, 10);
    }

    public void right(double degrees)
    {
        robot.pointTurnDegrees(velocity, degrees, 10);
    }

    public void left(double degrees)
    {
        degrees = degrees * -1;
        robot.pointTurnDegrees(velocity, degrees, 10);
    }

    public void speed(int speed)
    {
        double newSpeed = (double)speed / 10.0;
        velocity = newSpeed;
    }

    // Sample Delay Code
    public void delay(double t) { // Imitates the Arduino delay function
        runtime.reset();
        while (opModeIsActive() && (runtime.seconds() < t)) {
            telemetry.addData("Path", "Leg 1: %2.5f S Elapsed", runtime.seconds());
            telemetry.update();
        }
    }

    // REACH: Add code to function below to map Robot Mazes
    public void driveRectangle()
    {
        forward(24);
        delay(1);
        right(270);
        delay(1);

    }

    public void driveMazeA()
    {

    }

    public void driveMazeB()
    {

    }

    public void driveAdvanced()
    {

    }

}
