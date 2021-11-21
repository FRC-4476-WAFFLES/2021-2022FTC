package org.firstinspires.ftc.teamcode.Camera;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Subsystems.ElevatorSubsystem;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.opencv.core.Rect;
import org.opencv.core.Point;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvPipeline;

public class FreightDetector extends OpenCvPipeline {
    Telemetry telemetry;
    private OpenCvCamera webcam;

    Mat image = new Mat();

    private final int THRESHOLD = 100;

    private int leftAverage;
    private int centerAverage;
    private int rightAverage;

    private ElevatorSubsystem.Levels targetLevel;

    public Rect leftROI = new Rect(
            new Point(175, 0),
            new Point(375, 300)
    );

    public Rect centerROI = new Rect(
            new Point(550, 0),
            new Point(750, 300)
    );

    public Rect rightROI = new Rect(
            new Point(900, 0),
            new Point(1100, 300)
    );

    public FreightDetector(Telemetry telemetry) {
        this.targetLevel = ElevatorSubsystem.Levels.CONSTANT;
        this.telemetry = telemetry;
    }

    @Override
    public Mat processFrame(Mat frame) {
        // Convert frame to a preferable format for shipping element detection
        Imgproc.cvtColor(frame, image, Imgproc.COLOR_RGB2HSV);
        Core.extractChannel(image, image, 1);
        Imgproc.threshold(image, image, 96, 255, Imgproc.THRESH_BINARY);

        // Average the values of all pixels in the different regions of interest
        leftAverage = (int) Core.mean(image.submat(leftROI)).val[0];
        centerAverage = (int) Core.mean(image.submat(centerROI)).val[0];
        rightAverage = (int) Core.mean(image.submat(rightROI)).val[0];

        // Check for which region meets the threshold
        if (leftAverage > THRESHOLD) {
            targetLevel = ElevatorSubsystem.Levels.L1;
        } else if (centerAverage > THRESHOLD) {
            targetLevel = ElevatorSubsystem.Levels.L2;
        } else if (rightAverage > THRESHOLD) {
            targetLevel = ElevatorSubsystem.Levels.L3;
        }

        // Colors for boxes around each ROI
        Scalar shippingColour = new Scalar(255, 0, 0);
        Scalar emptyColour = new Scalar(0, 255, 0);

        // Create boxes around each ROI to highlight shipping element ROI
        Imgproc.rectangle(frame, leftROI, (targetLevel == ElevatorSubsystem.Levels.L1) ? shippingColour : emptyColour, 10);
        Imgproc.rectangle(frame, centerROI, (targetLevel == ElevatorSubsystem.Levels.L2) ? shippingColour : emptyColour, 10);
        Imgproc.rectangle(frame, rightROI, (targetLevel == ElevatorSubsystem.Levels.L3) ? shippingColour : emptyColour, 10);

        telemetry.addData("Target Level in Process Frame", targetLevel);

        return frame;
    }

    public ElevatorSubsystem.Levels getTargetLevel() {
        return this.targetLevel;
    }
}
