package org.firstinspires.ftc.teamcode.Camera;

import android.icu.text.Transliterator;

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

    public enum barcodePosition {
        LEFT, CENTER, RIGHT
    }

    private barcodePosition position;

    public Rect leftROI = new Rect(
            new Point(20, 20),
            new Point(40, 40)
    );

    public Rect centerROI = new Rect(
            new Point(50, 50),
            new Point(100, 100)
    );

    public Rect rightROI = new Rect(
            new Point(150, 150),
            new Point(200, 200)
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
            position = barcodePosition.LEFT;
        } else if (centerAverage > THRESHOLD) {
            position = barcodePosition.CENTER;
        } else {
            position = barcodePosition.RIGHT;
        }

        // Colors for boxes around each ROI
        Scalar shippingColour = new Scalar(255, 0, 0);
        Scalar emptyColour = new Scalar(0, 255, 0);

        // Create boxes around each ROI to highlight shipping element ROI
        Imgproc.rectangle(frame, leftROI, (position == barcodePosition.LEFT) ? shippingColour : emptyColour);
        Imgproc.rectangle(frame, centerROI, (position == barcodePosition.CENTER) ? shippingColour : emptyColour);
        Imgproc.rectangle(frame, rightROI, (position == barcodePosition.RIGHT) ? shippingColour : emptyColour);

        return frame;
    }

    public barcodePosition getPosition() {
        return position;
    }
}
