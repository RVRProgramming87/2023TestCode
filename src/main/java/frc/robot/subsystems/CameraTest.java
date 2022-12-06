// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.awt.image.*;
import java.io.IOException;
import java.awt.*;
import javax.swing.*;
import org.opencv.videoio.*;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Scalar;
import org.opencv.highgui.HighGui;
import org.opencv.imgproc.Imgproc;


import edu.wpi.first.wpilibj2.command.SubsystemBase;


public class CameraTest {
  /** Creates a new ExampleSubsystem. */
  public CameraTest() {
    displayImage(Mat2BufferedImage(null));
  }

  public static void videoFace() throws IOException {
    VideoCapture capture = new VideoCapture(0);
    Mat image = new Mat();
    int index = 0;
    if (capture.isOpened()) {
      while (true) {
        capture.read(image);
        HighGui.imshow("Titties", image);
        index = HighGui.waitKey(1);
        if (index == 27) {
          break;
        }
      }
    }
  }



  //More helios garbage
  BufferedImage Mat2BufferedImage(Mat m){
    int bufferSize = m.channels()*m.cols()*m.rows();
    byte [] b = new byte[bufferSize];
    m.get(0,0,b); // get all the pixels
    BufferedImage image = new BufferedImage(m.cols(),m.rows(), BufferedImage.TYPE_INT_RGB);
    final byte[] targetPixels = ((DataBufferByte) image.getRaster().getDataBuffer()).getData();
    System.arraycopy(b, 0, targetPixels, 0, b.length);
    return image;
  }

  void displayImage(Image img2) {
      ImageIcon icon = new ImageIcon(img2);
      JFrame frame = new JFrame();
      frame.setLayout(new FlowLayout());
      frame.setSize(img2.getWidth(null)+50, img2.getHeight(null)+50);
      JLabel lbl = new JLabel();
      lbl.setIcon(icon);
      frame.add(lbl);
      frame.setVisible(true);
      frame.setDefaultCloseOperation(JFrame.EXIT_ON_CLOSE);
  }
}
