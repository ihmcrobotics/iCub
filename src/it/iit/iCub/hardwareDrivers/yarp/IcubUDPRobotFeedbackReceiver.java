package it.iit.iCub.hardwareDrivers.yarp;

import it.iit.iCub.messages.it.iit.yarp.RobotFeedback;
import it.iit.iCub.messages.it.iit.yarp.RobotFeedbackPubSubType;
import us.ihmc.idl.CDR;
import us.ihmc.pubsub.common.SerializedPayload;

import java.io.IOException;
import java.net.InetSocketAddress;
import java.net.StandardProtocolFamily;
import java.net.StandardSocketOptions;
import java.nio.ByteBuffer;
import java.nio.channels.DatagramChannel;

/**
 * @author Doug Stephen <a href="mailto:dstephen@ihmc.us">(dstephen@ihmc.us)</a>
 */
public class IcubUDPRobotFeedbackReceiver
{
   public static final String DEFAULT_YARP_ROBOT_FEEDBACK_IP = "192.168.96.1";
   public static final int DEFAULT_YARP_ROBOT_FEEDBACK_PORT = 9970;

   private final RobotFeedbackPubSubType robotFeedbackPubSubType = new RobotFeedbackPubSubType();
   private final SerializedPayload payload = new SerializedPayload(robotFeedbackPubSubType.getTypeSize());
   private final String yarpHostOrIP;
   private final int yarpUDPPort;
   private final RobotFeedback iCubRobotFeedbackToUpdate;

   private DatagramChannel receiveChannel = null;

   public IcubUDPRobotFeedbackReceiver(String yarpHostOrIP, int yarpUDPPort, RobotFeedback iCubRobotFeedbackToUpdate)
   {
      this.yarpHostOrIP = yarpHostOrIP;
      this.yarpUDPPort = yarpUDPPort;
      this.iCubRobotFeedbackToUpdate = iCubRobotFeedbackToUpdate;
   }

   public synchronized long receive()
   {
      ByteBuffer payloadData = payload.getData();

      payloadData.clear();

      CDR.writeEncapsulation(payload);

      try
      {
         receiveChannel.receive(payloadData);
      }
      catch (IOException e)
      {
         e.printStackTrace();
         return -1;
      }

      payloadData.flip();

      long currentTime = System.nanoTime();
      try
      {
         robotFeedbackPubSubType.deserialize(payload, iCubRobotFeedbackToUpdate);
      }
      catch (IOException e)
      {
         e.printStackTrace();
         return -1;
      }

      return currentTime;
   }

   public void connect() throws IOException
   {
      try
      {
         receiveChannel = DatagramChannel.open(StandardProtocolFamily.INET).setOption(StandardSocketOptions.SO_REUSEADDR, true)
                                        .bind(new InetSocketAddress(yarpHostOrIP, yarpUDPPort));
         receiveChannel.socket().setReceiveBufferSize(65535);
         receiveChannel.socket().setSoTimeout(1000);

//         System.out.println("Local address: " + receiveChannel.getLocalAddress());
//         System.out.println("Remote address: " + receiveChannel.getRemoteAddress());
//         System.out.println("Is connected: " + receiveChannel.isConnected());
//         System.out.println("Is blocking: " + receiveChannel.isBlocking());
      }
      catch (IOException e)
      {
         receiveChannel.disconnect();
         receiveChannel.close();
         receiveChannel = null;
         e.printStackTrace();
      }
   }

   public void disconnect()
   {
      if (receiveChannel != null)
      {
         try
         {
            receiveChannel.disconnect();
            receiveChannel.close();
            receiveChannel = null;
         }
         catch (IOException e)
         {
            e.printStackTrace();
         }
      }
   }
}
