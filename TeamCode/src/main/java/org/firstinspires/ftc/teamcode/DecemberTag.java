package org.firstinspires.ftc.teamcode;

import android.graphics.Bitmap;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.canvas.Canvas;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.function.Continuation;
import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.stream.CameraStreamSource;
import org.firstinspires.ftc.robotcore.internal.camera.calibration.CameraCalibration;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.VisionProcessor;
import org.opencv.android.Utils;
import org.opencv.core.Mat;
import java.util.concurrent.atomic.AtomicReference;

public class DecemberTag {
    @Autonomous
    public static class opModeIsActive extends LinearOpMode {
        public static class CameraStreamProcessor implements VisionProcessor, CameraStreamSource, org.firstinspires.ftc.teamcode.CameraStreamProcessor {
            private final AtomicReference<Bitmap> lastFrame =
                    new AtomicReference<>(Bitmap.createBitmap(1, 1, Bitmap.Config.RGB_565));

            @Override
            public void init(int width, int height, CameraCalibration calibration) {
                lastFrame.set(Bitmap.createBitmap(width, height, Bitmap.Config.RGB_565));
            }

            @Override
            public Object processFrame(Mat frame, long captureTimeNanos) {
                Bitmap b = Bitmap.createBitmap(frame.width(), frame.height(), Bitmap.Config.RGB_565);
                Utils.matToBitmap(frame, b);
                lastFrame.set(b);
                return null;
            }

            @Override
            public void onDrawFrame(android.graphics.Canvas canvas, int onscreenWidth, int onscreenHeight, float scaleBmpPxToCanvasPx, float scaleCanvasDensity, Object userContext) {

            }

            @Override
            public void onDrawFrame(Canvas canvas, int onscreenWidth, int onscreenHeight,
                                    float scaleBmpPxToCanvasPx, float scaleCanvasDensity,
                                    Object userContext) {
                // do nothing
            }

            @Override
            public void getFrameBitmap(Continuation<? extends org.firstinspires.ftc.robotcore.external.function.Consumer<Bitmap>> continuation) {

            }
        }

        @Override
        public void runOpMode() throws InterruptedException {
            final CameraStreamProcessor processor = new CameraStreamProcessor();

            new VisionPortal.Builder()
                    .addProcessor(processor)
                    .setCamera(BuiltinCameraDirection.BACK)
                    .build();

            FtcDashboard.getInstance().startCameraStream(processor, 0);

            waitForStart();

            while (opModeIsActive()) {
                sleep(100L);
            }
        }
    }
}