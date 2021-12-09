package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

//@Disabled
@TeleOp(name="imu test", group="Linear Opmode")
public class imutest extends LinearOpMode {

    @Override
    public void runOpMode() {
        NyxTheRobot nyx = new NyxTheRobot(this);
        nyx.initialize();
        waitForStart();

        double target = nyx.GetCurrentZAngle() + 90;
        double scale = Math.abs(target - nyx.GetCurrentZAngle());
        double position = target - nyx.GetCurrentZAngle();

        while (opModeIsActive()){
            telemetry.addData("swag", nyx.GetCurrentZAngle());
            telemetry.addData("swag2", position);
            telemetry.update();
        }

    }
}
