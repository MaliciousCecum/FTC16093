package org.firstinspires.ftc.teamcode;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.localization.Localizer;
import com.acmerobotics.roadrunner.util.NanoClock;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.util.Encoder;

import java.util.Arrays;
import java.util.List;


@Config
public class XCYLocalizer implements Localizer {
    public static final double TICKS_PER_REV = 4096;
    public static final double WHEEL_RADIUS = 25.4; // mm
    public static double Y_OFFSET = 68;
    public static double X_OFFSET = -28.5;

    private final Encoder xEncoder, yEncoder;
    private final BNO055IMU imu;

    private Pose2d poseEstimate = new Pose2d(0, 0, 0);
    private Pose2d poseVelocity = new Pose2d(0, 0, 0);

    private int last_x_pos, last_y_pos;
    private double heading_correction=0;
    private final NanoClock time;
    private double last_time, last_rotation;

    public XCYLocalizer(HardwareMap hardwareMap, BNO055IMU imu) {
        this.imu = imu;
        xEncoder = new Encoder(hardwareMap.get(DcMotorEx.class, "rb"));
        yEncoder = new Encoder(hardwareMap.get(DcMotorEx.class, "rf"));

        xEncoder.setDirection(Encoder.Direction.REVERSE);
//        yEncoder.setDirection();
        time = NanoClock.system();

        last_x_pos = xEncoder.getCurrentPosition();
        last_y_pos = yEncoder.getCurrentPosition();
        last_time = time.seconds();
        last_rotation = imu.getAngularOrientation().firstAngle;
    }

    public static double encoderTicksToMM(double ticks) {
        return WHEEL_RADIUS * 2 * Math.PI * ticks / TICKS_PER_REV;
    }

    @NonNull
    public List<Double> getWheelPositions() {
        return Arrays.asList(
                encoderTicksToMM(xEncoder.getCurrentPosition()),
                encoderTicksToMM(yEncoder.getCurrentPosition())
        );
    }

    @NonNull
    public List<Double> getWheelVelocities() {
        return Arrays.asList(
                encoderTicksToMM(xEncoder.getCorrectedVelocity()),
                encoderTicksToMM(yEncoder.getCorrectedVelocity())
        );
    }

    @NonNull
    @Override
    public Pose2d getPoseEstimate() {
        return poseEstimate;
    }

    @NonNull
    @Override
    public Pose2d getPoseVelocity() {
        return poseVelocity;
    }
    private double heading_rad_correct = 0;
    private int rev_num = 0;

    @Override
    public void update() {
        int current_right = xEncoder.getCurrentPosition();
        int current_front = yEncoder.getCurrentPosition();
        double current_time = time.seconds();
        double rotation = imu.getAngularOrientation().firstAngle - heading_rad_correct;
        double corrected_rotation = rotation + Math.PI * 2 * rev_num;
        if (corrected_rotation - last_rotation > Math.PI) {
            rev_num--;
        } else if (corrected_rotation - last_rotation < -Math.PI) {
            rev_num++;
        }
        corrected_rotation = rotation + Math.PI * 2 * rev_num;

        double rotation_avg = AngleUnit.normalizeRadians((last_rotation+corrected_rotation)/2);

        double d_time = last_time - current_time;
        double d_rotation = (corrected_rotation - last_rotation);

        double d_x = encoderTicksToMM(current_right - last_x_pos) - d_rotation * X_OFFSET;
        double d_y = encoderTicksToMM(current_front - last_y_pos) - d_rotation * Y_OFFSET;
        Vector2d d_pos = (new Vector2d(d_x, d_y)).rotated(corrected_rotation);

        poseEstimate = new Pose2d(poseEstimate.vec().plus(d_pos), rotation_avg);
        poseVelocity = new Pose2d(d_pos.div(d_time), imu.getAngularVelocity().zRotationRate);

        last_x_pos = current_right;
        last_y_pos = current_front;
        last_time = current_time;
        last_rotation = corrected_rotation;
    }

    public void setPoseEstimate(@NonNull Pose2d poseEstimate) {
        heading_rad_correct = imu.getAngularOrientation().firstAngle - poseEstimate.getHeading();
        this.poseEstimate = poseEstimate;
        last_rotation = poseEstimate.getHeading();
    }

    public void setPoseVelocity(@NonNull Pose2d poseVelocity) {
        this.poseVelocity = poseVelocity;
    }
}
