package org.firstinspires.ftc.teamcode.Camera;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.opencv.core.Rect;
import org.opencv.core.Point;
import org.openftc.easyopencv.OpenCvPipeline;

public class FreightDetector extends OpenCvPipeline {
    Telemetry telemetry;

    Mat image = new Mat();

    private final int THRESHOLD = 100;

    private int leftAverage;
    private int centerAverage;
    private int rightAverage;

    int targetLevel;

    public Rect leftROI = new Rect(
            new Point(0, 300),
            new Point(200, 700)
    );

    public Rect centerROI = new Rect(
            new Point(540, 300),
            new Point(740, 700)
    );

    public Rect rightROI = new Rect(
            new Point(1080, 300),
            new Point(1280, 700)
    );

    public FreightDetector(Telemetry t) {
        telemetry = t;
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

        telemetry.addData("leftAverage:", leftAverage);
        telemetry.addData("centerAverage:", centerAverage);
        telemetry.addData("rightAverage:", rightAverage);
        telemetry.update();

        // Check for which region meets the threshold
        if (leftAverage > THRESHOLD) {
            targetLevel = 1;
        } else if (centerAverage > THRESHOLD) {
            targetLevel = 2;
        } else {
            targetLevel = 3;
        }

        // Colors for boxes around each ROI
        Scalar shippingColour = new Scalar(255, 0, 0);
        Scalar emptyColour = new Scalar(0, 255, 0);

        // Create boxes around each ROI to highlight shipping element ROI
        Imgproc.rectangle(frame, leftROI, (targetLevel == 1) ? shippingColour : emptyColour, 10);
        Imgproc.rectangle(frame, centerROI, (targetLevel == 2) ? shippingColour : emptyColour, 10);
        Imgproc.rectangle(frame, rightROI, (targetLevel == 3) ? shippingColour : emptyColour, 10);

        return frame;
    }

    public int getTargetLevel() {
        return targetLevel;
    }
}
