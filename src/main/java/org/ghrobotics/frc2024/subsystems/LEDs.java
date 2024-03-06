package org.ghrobotics.frc2022.subsystems;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LED extends SubsystemBase {

  //led
  private final AddressableLED led_;
  //led buffer
  private final AddressableLEDBuffer led_buffer_;

  private int led_rainbowFirstPixelHue = 0;

  
  public LED() {
    // Initialize
    led_buffer_ = new AddressableLEDBuffer(Constants.kBufferSize);
    led_ = new AddressableLED(Constants.kPortId);
    led_.setLength(led_buffer_.getLength());
    led_.start();
    led_.setData(led_buffer_);

  }

  @Override
  public void periodic(){
    // Fill the buffer with a rainbow
    rainbow();
    // Set the LEDs
    led_.setData(led_buffer_);
  }
  
  
  private void rainbow() {
    // For every pixel
    for (var i = 0; i < led_buffer_.getLength(); i++) {
      final var hue = (led_rainbowFirstPixelHue + (i * 180 / led_buffer_.getLength())) % 180;
      // Set the value
      led_buffer_.setHSV(i, hue, 255, 128);
    }
   
    led_rainbowFirstPixelHue += 3;
    
    led_rainbowFirstPixelHue %= 180;
  }
  public void end() {
    led_.stop();
    // is it .stop?
  }
  public static class Constants {
    public static final int kPortId = 9;
    public static final int kBufferSize = 30;
  }

}