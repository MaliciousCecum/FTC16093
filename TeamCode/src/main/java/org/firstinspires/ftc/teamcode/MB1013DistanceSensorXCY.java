package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.hardware.AnalogInputController;
import com.qualcomm.robotcore.hardware.HardwareDevice;
import com.qualcomm.robotcore.hardware.configuration.annotations.AnalogSensorType;
import com.qualcomm.robotcore.hardware.configuration.annotations.DeviceProperties;


@AnalogSensorType()
@DeviceProperties(name = "MB1013DistanceSensor XCY test", description = "MB1013 driver test", xmlTag = "MB1013_XCY")
public class MB1013DistanceSensorXCY implements HardwareDevice {

   private final AnalogInputController controller;
   private final int channel;
   private DistanceUnit unit;

   enum DistanceUnit {
      METER, MILLIMETER
   }

   /**
    * Constructor
    *
    * @param controller AnalogInput controller this channel is attached to
    * @param channel    channel on the analog input controller
    */
   public MB1013DistanceSensorXCY(AnalogInputController controller, int channel) {
      this.controller = controller;
      this.channel = channel;
      unit = DistanceUnit.MILLIMETER;
   }

   public void setDistanceUnit(DistanceUnit distanceUnit) {
      unit = distanceUnit;
   }

   public double getDistance() {
      if (unit == DistanceUnit.MILLIMETER)
         return (controller.getAnalogInputVoltage(channel) / (3.3 / 1024)) * 6 - 300;
      else
         return ((controller.getAnalogInputVoltage(channel) / (3.3 / 1024)) * 6 - 300) / 1000;
   }

   @Override
   public Manufacturer getManufacturer() {
      return Manufacturer.Adafruit;
   }

   @Override
   public String getDeviceName() {
      return "MB1013DistanceSensor-XCY";
   }

   @Override
   public String getConnectionInfo() {
      return controller.getConnectionInfo() + "; analog port " + channel;
   }

   @Override
   public int getVersion() {
      return 0;
   }

   @Override
   public void resetDeviceConfigurationForOpMode() {

   }

   @Override
   public void close() {

   }
}