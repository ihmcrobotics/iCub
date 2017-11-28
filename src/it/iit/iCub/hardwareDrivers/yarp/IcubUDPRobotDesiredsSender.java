package it.iit.iCub.hardwareDrivers.yarp;

import it.iit.iCub.messages.it.iit.yarp.RobotDesireds;
import it.iit.iCub.messages.it.iit.yarp.RobotDesiredsPubSubType;
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
public class IcubUDPRobotDesiredsSender
{
   public static final String DEFAULT_YARP_ROBOT_DESIRED_IP = "192.168.96.130";
   public static final int DEFAULT_YARP_ROBOT_DESIRED_PORT = 9980;

   private final RobotDesiredsPubSubType robotDesiredsPubSubType = new RobotDesiredsPubSubType();
   private final SerializedPayload payload = new SerializedPayload(robotDesiredsPubSubType.getTypeSize());

   private final String yarpHostOrIP;
   private final int yarpUDPPort;

   private final RobotDesireds robotDesiredsToSend;

   private DatagramChannel sendChannel = null;

   public IcubUDPRobotDesiredsSender(String yarpHostOrIP, int yarpUDPPort, RobotDesireds robotDesiredsToSend)
   {
      this.yarpHostOrIP = yarpHostOrIP;
      this.yarpUDPPort = yarpUDPPort;
      this.robotDesiredsToSend = robotDesiredsToSend;
   }

   public synchronized void send()
   {
      try
      {
         payload.getData().clear();
         robotDesiredsPubSubType.serialize(robotDesiredsToSend, payload);

         CDR.readEncapsulation(payload); // strip out the encapsulation, YARP bridge doesn't read/write it.

         sendChannel.write(payload.getData());
      }
      catch (IOException e)
      {
         e.printStackTrace();
      }
   }

   public void connect() throws IOException
   {
      try
      {
         sendChannel = DatagramChannel.open(StandardProtocolFamily.INET).setOption(StandardSocketOptions.SO_REUSEADDR, true)
                                      .connect(new InetSocketAddress(yarpHostOrIP, yarpUDPPort));
      }
      catch (IOException e)
      {
         sendChannel.disconnect();
         sendChannel.close();
         sendChannel = null;
         e.printStackTrace();
      }

   }
}
