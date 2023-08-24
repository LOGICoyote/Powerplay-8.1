package net.frogbots.skystone.opmodes.util;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
//import net.frogbots.skystone.drivers.MaxSonarI2CXL;

@TeleOp


public class MaxbotixTest extends LinearOpMode
{
    MaxSonarI2CXL sonar;
    MaxSonarI2CXL sonar2;
    MaxSonarI2CXL sonar3;

    @Override
    public void runOpMode() throws InterruptedException
    {
       sonar = hardwareMap.get(MaxSonarI2CXL.class, "sonarback");
        sonar2 = hardwareMap.get(MaxSonarI2CXL.class, "sonarright");
        sonar3 = hardwareMap.get(MaxSonarI2CXL.class, "sonarleft");

        telemetry.setMsTransmissionInterval(20);

        waitForStart();

        while (opModeIsActive())
        {
            telemetry.addData("back Dist CM", sonar.getDistanceSync());
            telemetry.addData("right Dist CM", sonar2.getDistanceSync());
            telemetry.addData("left Dist CM", sonar3.getDistanceSync());

            telemetry.update();
        }
    }
}