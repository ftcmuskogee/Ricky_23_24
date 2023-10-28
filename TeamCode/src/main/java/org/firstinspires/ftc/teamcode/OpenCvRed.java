package org.firstinspires.ftc.teamcode;

import static com.sun.tools.doclint.Entity.and;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.opencv.core.Core;
import org.opencv.core.Rect;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvPipeline;
import org.openftc.easyopencv.OpenCvWebcam;

import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
class examplePipline extends OpenCvPipeline{
    Mat YCbCr = new Mat();
    Mat leftcrop;
    Mat rightcrop;
    Mat midcrop;
    double leftavgfin;
    double rightavgfin;
    double midavgfin;
    Mat outPut = new Mat();
    Scalar rectColor = new Scalar(255.0,0.0,0.0);


    public Mat processFrame(Mat input){

        Imgproc.cvtColor(input,YCbCr,Imgproc.COLOR_RGB2YCrCb);
        //telemetry.addLine("pipeline running");

        Rect leftRect = new Rect(1,1,158,359);
        Rect midRect = new Rect(160,1,318,359);
        Rect rightRect = new Rect(480,1,158,359);
        input.copyTo(outPut);
        Imgproc.rectangle(outPut,leftRect,rectColor,2);
        Imgproc.rectangle(outPut,midRect,rectColor,2);
        Imgproc.rectangle(outPut,rightRect,rectColor,2);

        leftcrop = YCbCr.submat(leftRect);
        midcrop = YCbCr.submat(midRect);
        rightcrop = YCbCr.submat(rightRect);

        Core.extractChannel(leftcrop,leftcrop,1);
        Core.extractChannel(rightcrop,rightcrop,1);
        Core.extractChannel(midcrop,midcrop,1);

        Scalar leftavg = Core.mean(leftcrop);
        Scalar rightavg = Core.mean(rightcrop);
        Scalar midavg = Core.mean(midcrop);

        leftavgfin = leftavg.val[0];
        rightavgfin = rightavg.val[0];
        midavgfin = midavg.val[0];

        if ((rightavgfin > leftavgfin ) && (rightavgfin > midavgfin)){
            //telemetry.addLine("Right");
        }
        if ((leftavgfin > rightavgfin) && (leftavgfin > midavgfin)){
            //telemetry.addLine("Left");
        }
        if ((midavgfin > rightavgfin) && (midavgfin > leftavgfin)){
            //telemetry.addLine("Middle");
        }

        return(outPut);}}
