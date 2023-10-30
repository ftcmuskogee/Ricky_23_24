package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvWebcam;


@Autonomous (name = "Red Pixel Side", group = "Autonomous Main")
public class RedPixelS extends LinearOpMode {
    Camera camera = new Camera(hardwareMap);
    @Override
    public void runOpMode(){
        camera.getPipelineOutput();
        telemetry.addLine("press play");
        waitForStart();

        while (opModeIsActive()){
            telemetry.addLine(camera.getPipelineOutput());
            telemetry.update();
            if (camera.getPipelineOutput() == "Right"){
                telemetry.addLine("right");
            }
            if (camera.getPipelineOutput() == "Left"){
                telemetry.addLine("left");

            }
            if (camera.getPipelineOutput() == "Middle"){
                telemetry.addLine("middle");

            }

        }


    }

}