package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;
import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

@TeleOp(name = "Vision Test")
public class CameraSthuff extends LinearOpMode {

    OpenCvCamera camera;
    GameElementDetector detector;

    @Override
    public void runOpMode() {

        int cameraMonitorViewId = hardwareMap.appContext.getResources()
                .getIdentifier("cameraMonitorViewId", "id",
                        hardwareMap.appContext.getPackageName());

        camera = OpenCvCameraFactory.getInstance().createWebcam(
                hardwareMap.get(WebcamName.class, "Webcam 1"),
                cameraMonitorViewId);

        detector = new GameElementDetector();
        camera.setPipeline(detector);

        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                camera.startStreaming(640, 480, OpenCvCameraRotation.UPRIGHT);
            }
            @Override
            public void onError(int errorCode) {
                telemetry.addData("Camera error", errorCode);
            }
        });

        waitForStart();

        while (opModeIsActive()) {
            telemetry.addData("Element detected", detector.elementDetected);
            telemetry.addData("Color", detector.lastColor);
            telemetry.addData("Hue", detector.lastHue);
            telemetry.addData("Aspect ratio", detector.lastAspectRatio);
            telemetry.addData("Solidity", detector.lastSolidity);
            telemetry.update();
        }

        camera.stopStreaming();
    }
}