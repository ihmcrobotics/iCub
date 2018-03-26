package it.iit.iCub.messages.it.iit.yarp;

/**
 * Topic data type of the struct "RobotFeedback" defined in "robotFeedback.idl". Use this class to provide the TopicDataType to a Participant.
 *
 * This file was automatically generated from robotFeedback.idl by us.ihmc.idl.generator.IDLGenerator.
 * Do not update this file directly, edit robotFeedback.idl instead.
 */
public class RobotFeedbackPubSubType implements us.ihmc.pubsub.TopicDataType<it.iit.iCub.messages.it.iit.yarp.RobotFeedback>
{
   public static final java.lang.String name = "it::iit::yarp::RobotFeedback";
   private final us.ihmc.idl.CDR serializeCDR = new us.ihmc.idl.CDR();
   private final us.ihmc.idl.CDR deserializeCDR = new us.ihmc.idl.CDR();

   public RobotFeedbackPubSubType()
   {

   }

   public static int getMaxCdrSerializedSize()
   {
      return getMaxCdrSerializedSize(0);
   }

   public static int getMaxCdrSerializedSize(int current_alignment)
   {
      int initial_alignment = current_alignment;

      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);
      for (int i0 = 0; i0 < 100; ++i0)
      {
         current_alignment += it.iit.iCub.messages.it.iit.yarp.JointStatePubSubType.getMaxCdrSerializedSize(current_alignment);
      }

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);
      for (int i0 = 0; i0 < 100; ++i0)
      {
         current_alignment += it.iit.iCub.messages.it.iit.yarp.IMUStatePubSubType.getMaxCdrSerializedSize(current_alignment);
      }

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);
      for (int i0 = 0; i0 < 100; ++i0)
      {
         current_alignment += it.iit.iCub.messages.it.iit.yarp.ForceSensorPubSubType.getMaxCdrSerializedSize(current_alignment);
      }

      return current_alignment - initial_alignment;
   }

   public final static int getCdrSerializedSize(it.iit.iCub.messages.it.iit.yarp.RobotFeedback data)
   {
      return getCdrSerializedSize(data, 0);
   }

   public final static int getCdrSerializedSize(it.iit.iCub.messages.it.iit.yarp.RobotFeedback data, int current_alignment)
   {
      int initial_alignment = current_alignment;

      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);
      for (int i0 = 0; i0 < data.getJointStates().size(); ++i0)
      {
         current_alignment += it.iit.iCub.messages.it.iit.yarp.JointStatePubSubType.getCdrSerializedSize(data.getJointStates().get(i0), current_alignment);
      }

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);
      for (int i0 = 0; i0 < data.getImuStates().size(); ++i0)
      {
         current_alignment += it.iit.iCub.messages.it.iit.yarp.IMUStatePubSubType.getCdrSerializedSize(data.getImuStates().get(i0), current_alignment);
      }

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);
      for (int i0 = 0; i0 < data.getForceSensors().size(); ++i0)
      {
         current_alignment += it.iit.iCub.messages.it.iit.yarp.ForceSensorPubSubType.getCdrSerializedSize(data.getForceSensors().get(i0), current_alignment);
      }

      return current_alignment - initial_alignment;
   }

   public static void write(it.iit.iCub.messages.it.iit.yarp.RobotFeedback data, us.ihmc.idl.CDR cdr)
   {

      cdr.write_type_11(data.getTimestampInNanoseconds());

      if (data.getJointStates().size() <= 100)
         cdr.write_type_e(data.getJointStates());
      else
         throw new RuntimeException("jointStates field exceeds the maximum length");

      if (data.getImuStates().size() <= 100)
         cdr.write_type_e(data.getImuStates());
      else
         throw new RuntimeException("imuStates field exceeds the maximum length");

      if (data.getForceSensors().size() <= 100)
         cdr.write_type_e(data.getForceSensors());
      else
         throw new RuntimeException("forceSensors field exceeds the maximum length");
   }

   public static void read(it.iit.iCub.messages.it.iit.yarp.RobotFeedback data, us.ihmc.idl.CDR cdr)
   {

      data.setTimestampInNanoseconds(cdr.read_type_11());

      cdr.read_type_e(data.getJointStates());

      cdr.read_type_e(data.getImuStates());

      cdr.read_type_e(data.getForceSensors());
   }

   public static void staticCopy(it.iit.iCub.messages.it.iit.yarp.RobotFeedback src, it.iit.iCub.messages.it.iit.yarp.RobotFeedback dest)
   {
      dest.set(src);
   }

   @Override
   public void serialize(it.iit.iCub.messages.it.iit.yarp.RobotFeedback data, us.ihmc.pubsub.common.SerializedPayload serializedPayload)
         throws java.io.IOException
   {
      serializeCDR.serialize(serializedPayload);
      write(data, serializeCDR);
      serializeCDR.finishSerialize();
   }

   @Override
   public void deserialize(us.ihmc.pubsub.common.SerializedPayload serializedPayload, it.iit.iCub.messages.it.iit.yarp.RobotFeedback data)
         throws java.io.IOException
   {
      deserializeCDR.deserialize(serializedPayload);
      read(data, deserializeCDR);
      deserializeCDR.finishDeserialize();
   }

   @Override
   public final void serialize(it.iit.iCub.messages.it.iit.yarp.RobotFeedback data, us.ihmc.idl.InterchangeSerializer ser)
   {
      ser.write_type_11("timestampInNanoseconds", data.getTimestampInNanoseconds());

      ser.write_type_e("jointStates", data.getJointStates());

      ser.write_type_e("imuStates", data.getImuStates());

      ser.write_type_e("forceSensors", data.getForceSensors());
   }

   @Override
   public final void deserialize(us.ihmc.idl.InterchangeSerializer ser, it.iit.iCub.messages.it.iit.yarp.RobotFeedback data)
   {
      data.setTimestampInNanoseconds(ser.read_type_11("timestampInNanoseconds"));

      ser.read_type_e("jointStates", data.getJointStates());

      ser.read_type_e("imuStates", data.getImuStates());

      ser.read_type_e("forceSensors", data.getForceSensors());
   }

   @Override
   public it.iit.iCub.messages.it.iit.yarp.RobotFeedback createData()
   {
      return new it.iit.iCub.messages.it.iit.yarp.RobotFeedback();
   }

   @Override
   public int getTypeSize()
   {
      return us.ihmc.idl.CDR.getTypeSize(getMaxCdrSerializedSize());
   }

   @Override
   public java.lang.String getName()
   {
      return name;
   }

   public void serialize(it.iit.iCub.messages.it.iit.yarp.RobotFeedback data, us.ihmc.idl.CDR cdr)
   {
      write(data, cdr);
   }

   public void deserialize(it.iit.iCub.messages.it.iit.yarp.RobotFeedback data, us.ihmc.idl.CDR cdr)
   {
      read(data, cdr);
   }

   public void copy(it.iit.iCub.messages.it.iit.yarp.RobotFeedback src, it.iit.iCub.messages.it.iit.yarp.RobotFeedback dest)
   {
      staticCopy(src, dest);
   }

   @Override
   public RobotFeedbackPubSubType newInstance()
   {
      return new RobotFeedbackPubSubType();
   }
}