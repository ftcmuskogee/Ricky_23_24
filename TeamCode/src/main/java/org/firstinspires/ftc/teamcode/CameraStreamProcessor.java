package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.canvas.Canvas;

public interface CameraStreamProcessor {
    void onDrawFrame(Canvas canvas, int onscreenWidth, int onscreenHeight,
                     float scaleBmpPxToCanvasPx, float scaleCanvasDensity,
                     Object userContext);
}
