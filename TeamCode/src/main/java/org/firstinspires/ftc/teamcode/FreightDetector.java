package org.firstinspires.ftc.teamcode;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.opencv.core.Mat;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

public class FreightDetector extends OpenCvPipeline {
    Telemetry telemetry;
    Mat image = new Mat();

    public FreightDetector(Telemetry t) {
        telemetry = t;
    }

    @Override
    public Mat processFrame(Mat frame) {
        Imgproc.cvtColor(frame, image, Imgproc.COLOR_RGB2HSV);

        return frame;
    }
}
