package org.firstinspires.ftc.teamcode.Camera;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

@Autonomous(name="Freight Camera Testing")
public class FreightTest extends LinearOpMode {

    @Override
    public void runOpMode() {
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        OpenCvCamera webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);

        webcam.setPipeline(new FreightDetector(telemetry));

        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                // Start streaming from the camera
                webcam.startStreaming(1280, 720, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode) {
                // Camera fails to be opened
                telemetry.addData("Failed to open webcam. Error code:", errorCode);
                telemetry.update();
            }
        });

        telemetry.addLine("Waiting for start...");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            telemetry.addData("Frame Count", webcam.getFrameCount());
            telemetry.addData("FPS", webcam.getFps());
            telemetry.addData("Total frame time (ms)", webcam.getTotalFrameTimeMs());
            telemetry.addData("Pipeline time (ms)", webcam.getPipelineTimeMs());
            telemetry.addData("Overhead time (ms)", webcam.getOverheadTimeMs());
            telemetry.addData("Current Pipeline Max FPS", webcam.getCurrentPipelineMaxFps());
            telemetry.update();

            sleep(100);
        }
    }
}
