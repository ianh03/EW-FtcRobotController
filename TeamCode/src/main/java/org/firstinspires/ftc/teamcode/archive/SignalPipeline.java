package org.firstinspires.ftc.teamcode.archive;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.opencv.core.Mat;
import org.openftc.easyopencv.OpenCvPipeline;
import org.opencv.core.Core;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;

@Disabled
public class SignalPipeline extends OpenCvPipeline {
    Telemetry telemetry;
    Mat mat = new Mat();
    Mat ycrcb = new Mat();

    public String detectedColor = "NONE";

    static final Rect MIDDLE = new Rect(
            new Point(135, 35),
            new Point(160, 85));

    public SignalPipeline(Telemetry telemetry) {
        this.telemetry = telemetry;
    }

    @Override
    public Mat processFrame(Mat input) {
        Imgproc.cvtColor(input, ycrcb, Imgproc.COLOR_RGB2YCrCb);

        Imgproc.rectangle(input, MIDDLE, new Scalar(255, 255, 255), 3);

        Core.extractChannel(ycrcb, mat, 0); // Channel 0 Extraction
        Core.inRange(mat, new Scalar(160), new Scalar(190), mat);
        double countY = Core.mean(mat.submat(MIDDLE)).val[0];
        telemetry.addData("CountY", countY);

        Core.extractChannel(ycrcb, mat, 1); // Channel 1 Extraction
        Core.inRange(mat, new Scalar(130), new Scalar(200), mat);
        double countCr = Core.mean(mat.submat(MIDDLE)).val[0];
        telemetry.addData("countCr", countCr);

        Core.extractChannel(ycrcb, mat, 2); // Chanel 2 Extraction
        Core.inRange(mat, new Scalar(130), new Scalar(170), mat);
        double countCb = Core.mean(mat.submat(MIDDLE)).val[0];
        telemetry.addData("countCb", countCb);

        // Comparing Values for Yellow Detection
        if (countY >= 130 && countCr >= 200 && countCb <= 45) {
            telemetry.addLine("Yellow Suspected");
            detectedColor = "YELLOW";
            Imgproc.rectangle(input, MIDDLE, new Scalar(255, 255, 0), 3);
        }
        // Comparing Values for Blue Detection
        else if (countY <= 40 && countCr <= 15 && countCb >= 215) {
            telemetry.addLine("Blue Suspected");
            detectedColor = "BLUE";
            Imgproc.rectangle(input, MIDDLE, new Scalar(0, 0, 255), 3);
        }
        // Comparing Values for Green Detection
        else if (countY <= 45 && countCr <= 8 && countCb <= 50) {
            telemetry.addLine("Green Suspected");
            detectedColor = "GREEN";
            Imgproc.rectangle(input, MIDDLE, new Scalar(0, 255, 0), 3);
        }

        telemetry.update();

        return input;
    }

    public String getAnalysis() {
        if (detectedColor != null) {
            return detectedColor;
        }
        return null;
    }
}