


package net.frogbots.skystone.opmodes.util;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvPipeline;

@Autonomous

public class Blue_nonAundienceVPark extends LinearOpMode {

    OpenCvCamera WebCam;
    Thing myThing;
    public DcMotor leftFront = null;
    public DcMotor rightFront = null;
    public DcMotor leftBack = null;
    public DcMotor rightBack = null;
    //public MaxSonarI2CXL sonarfront;
    public MaxSonarI2CXL sonarleft;
    public MaxSonarI2CXL sonarright;
    public MaxSonarI2CXL sonarback;
    static int pos = 0;

    public void runOpMode() {
        telemetry.addData("Status", "Initialized");

        leftFront = hardwareMap.get(DcMotor.class, "motor00");
        rightFront = hardwareMap.get(DcMotor.class, "motor01");
        leftBack = hardwareMap.get(DcMotor.class, "motor02");
        rightBack = hardwareMap.get(DcMotor.class, "motor03");
        sonarleft = hardwareMap.get(MaxSonarI2CXL.class, "sonarleft");
        sonarback = hardwareMap.get(MaxSonarI2CXL.class, "sonarback");
        sonarright = hardwareMap.get(MaxSonarI2CXL.class, "sonarright");
        leftFront.setDirection(DcMotor.Direction.FORWARD);
        rightFront.setDirection(DcMotor.Direction.REVERSE);
        leftBack.setDirection(DcMotor.Direction.REVERSE);
        rightBack.setDirection(DcMotor.Direction.REVERSE);

        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        WebCam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
        myThing = new Thing();
        WebCam.setPipeline(myThing);
        WebCam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {

        @Override
            public void onOpened() {
                WebCam.startStreaming(320, 240, OpenCvCameraRotation.UPRIGHT);
                }
        @Override
            public void onError(int errorCode) {}
            });
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        Pose2d startPose = new Pose2d(8, 64, Math.toRadians(-90));
        drive.setPoseEstimate(startPose);
        waitForStart();
        sleep(3000);
        telemetry.addData("Analysis", myThing.getAnalysis());
        telemetry.addData("position", myThing.position);
        telemetry.addData("R",sonarright.getDistanceAsync());
        telemetry.update();

        if ((myThing.position == Thing.CapPosition.C)){
            XYgo('L',115);
            XYgo('B',60);
            sleep(2000000);

        }
        else if ((myThing.position == Thing.CapPosition.A)){
            XYgo('L',70);
            XYgo('B',55);
            XYgo('L',15);
            sleep(2000000);
        }
        else if ((myThing.position == Thing.CapPosition.B)){
            XYgo('L',68);
            XYgo('B',60);
            sleep(2000000);
        }
       else{
           telemetry.addData("error occured",0);
        }
        sleep(20000);

    }

    public void XYgo(char sensor, int tdist){
        double FBspeed = 0.65;
        double LRspeed = 0.65;
        MaxSonarI2CXL sonarcurrent= sonarback;
        //if (sensor == 'F') sonarcurrent= sonarfront;
        if (sensor == 'B') sonarcurrent = sonarback;
        if (sensor == 'R') sonarcurrent = sonarright;
        if (sensor == 'L') sonarcurrent = sonarleft;
        double LF= 0;
        double RF= 0;
        double LB= 0;
        double RB= 0;

        if ((sensor == 'F' && tdist >= sonarcurrent.getDistanceSync()) || (sensor == 'B'  && tdist <= sonarcurrent.getDistanceSync()) ) {
            LF=(FBspeed*-1);
            RF=(FBspeed*-1);
            LB=(FBspeed*-1);
            RB=(FBspeed*-1);
        }

        if ((sensor == 'B' && tdist >= sonarcurrent.getDistanceSync()) || (sensor ==  'F' && tdist <= sonarcurrent.getDistanceSync())){
            LF = (FBspeed * 1);
            RF = (FBspeed * 1);
            LB = (FBspeed * 1);
            RB = (FBspeed * 1);

        }

        if ((sensor == 'L' && tdist >= sonarcurrent.getDistanceSync()) || (sensor == 'R' && tdist <= sonarcurrent.getDistanceSync())){
            LF=(LRspeed*1);
            RF=(LRspeed*-1);
            LB=(LRspeed*-1);
            RB=(LRspeed*1);
        }


        if ((sensor == 'R' && tdist >= sonarcurrent.getDistanceSync())|| (sensor == 'L' && tdist <= sonarcurrent.getDistanceSync())){
            LF=(LRspeed*-1);
            RF=(LRspeed*1) ;
            LB=(LRspeed*1);
            RB=(LRspeed*-1);
        }
        move(LF, RF, LB, RB, sonarcurrent, tdist);
    }
    public void move(double LF, double RF,double LB,double RB, MaxSonarI2CXL sensor, int tdist) {
        int X = 0;  //Used to designate if we are traveling towards sensor or away from sensor direction
        double empval;
        if ((sensor == sonarleft)|| (sensor == sonarright)){
            empval= 0.03;
        }
        else empval = 0.014;

        if (sensor.getDistanceSync() > tdist) X = 1;
        while ((X == 1 && sensor.getDistanceSync() > tdist) || (X == 0 && sensor.getDistanceSync() < tdist)) {
            double SV;
            SV = Math.abs((tdist - sensor.getDistanceSync()) * (empval));
            //.2 - min
            SV = Math.min(Math.max(SV, 0.4), 1);

            double Factor = Math.max(Math.max(Math.max(LF*SV, LB*SV), Math.max(RF*SV, RB*SV)) / 20, 0.01);

            double LCorr = 0.03;
            double RCorr = 0.03;
            //find out if we are in need of correction - has our angle changed?

            leftFront.setPower((LF * SV) + (LCorr));
            rightFront.setPower((RF * SV) + (RCorr));
            leftBack.setPower((LB * SV) + (LCorr));
            rightBack.setPower((RB * SV)+ (RCorr));

            telemetry.addData("dist", sensor.getDistanceSync());
            telemetry.update();

        }
        leftFront.setPower(0);
        rightFront.setPower(0);
        leftBack.setPower(0);
        rightBack.setPower(0);

    }
    public static class Thing extends OpenCvPipeline {
        public enum CapPosition {
            A,
            B,
            C
        }

        static final Scalar BLUE = new Scalar(0, 0, 255);
        static final Scalar GREEN = new Scalar(0, 255, 0);
        static final Point TopLeftPoint = new Point(40, 45);
        static final int Region_width = 50;
        static final int Region_height = 70;
        Point region1_pointA = new Point(TopLeftPoint.x, TopLeftPoint.y);
        Point region1_pointB = new Point(TopLeftPoint.x + Region_width, TopLeftPoint.y + Region_height);
        Mat region1_Cr;
        Mat YCrCb = new Mat();
        Mat Cr = new Mat();
        Mat Cb = new Mat();
        int avg1;

        int iscap = 145;
        private volatile CapPosition position = CapPosition.A;
        void inputToCb(Mat input){
            Imgproc.cvtColor(input, YCrCb, Imgproc.COLOR_RGB2YCrCb);
            Core.extractChannel(YCrCb,Cr,1);
        }
        @Override
        public void init (Mat firstFrame){
           inputToCb(firstFrame);
            region1_Cr= Cr.submat(new Rect(region1_pointA, region1_pointB));
        }

        @Override
        public Mat processFrame(Mat input) {
            inputToCb(input);
            avg1 = (int)Core.mean(region1_Cr).val[0];
            Imgproc.rectangle(input, region1_pointA, region1_pointB, BLUE, 2);


            position = CapPosition.A;
            if (avg1 < 118){
                position = CapPosition.A;
            }
            else if ((avg1<135)&&(avg1>118)){
                position = CapPosition.B;
            }
            else if (avg1>=135){
            position = CapPosition.C;
            }
            Imgproc.rectangle(input,region1_pointA,region1_pointB, GREEN, 1);
            return input;
        }
        public int getAnalysis(){
            return avg1;
        }
    }


}




