package it.iit.iCub.messages.it.iit.yarp;

/**
 * Topic data type of the struct "JointDesired" defined in "robotDesired.idl". Use this class to provide the TopicDataType to a Participant.
 *
 * This file was automatically generated from robotDesired.idl by us.ihmc.idl.generator.IDLGenerator.
 * Do not update this file directly, edit robotDesired.idl instead.
 */
public class JointDesiredPubSubType implements us.ihmc.pubsub.TopicDataType<it.iit.iCub.messages.it.iit.yarp.JointDesired>
{
   public static final java.lang.String name = "it::iit::yarp::JointDesired";
   private final us.ihmc.idl.CDR serializeCDR = new us.ihmc.idl.CDR();
   private final us.ihmc.idl.CDR deserializeCDR = new us.ihmc.idl.CDR();

   public JointDesiredPubSubType()
   {

   }

   public static int getMaxCdrSerializedSize()
   {
      return getMaxCdrSerializedSize(0);
   }

   public static int getMaxCdrSerializedSize(int current_alignment)
   {
      int initial_alignment = current_alignment;

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);

      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);

      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);

      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);

      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);

      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);

      return current_alignment - initial_alignment;
   }

   public final static int getCdrSerializedSize(it.iit.iCub.messages.it.iit.yarp.JointDesired data)
   {
      return getCdrSerializedSize(data, 0);
   }

   public final static int getCdrSerializedSize(it.iit.iCub.messages.it.iit.yarp.JointDesired data, int current_alignment)
   {
      int initial_alignment = current_alignment;

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);

      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);

      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);

      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);

      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);

      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);

      return current_alignment - initial_alignment;
   }

   public static void write(it.iit.iCub.messages.it.iit.yarp.JointDesired data, us.ihmc.idl.CDR cdr)
   {

      cdr.write_type_c(data.getControlMode().ordinal());

      cdr.write_type_6(data.getTau());

      cdr.write_type_6(data.getKp());

      cdr.write_type_6(data.getKd());

      cdr.write_type_6(data.getQDesired());

      cdr.write_type_6(data.getQdDesired());
   }

   public static void read(it.iit.iCub.messages.it.iit.yarp.JointDesired data, us.ihmc.idl.CDR cdr)
   {

      data.setControlMode(it.iit.iCub.messages.it.iit.yarp.ORSControlMode.values[cdr.read_type_c()]);

      data.setTau(cdr.read_type_6());

      data.setKp(cdr.read_type_6());

      data.setKd(cdr.read_type_6());

      data.setQDesired(cdr.read_type_6());

      data.setQdDesired(cdr.read_type_6());
   }

   public static void staticCopy(it.iit.iCub.messages.it.iit.yarp.JointDesired src, it.iit.iCub.messages.it.iit.yarp.JointDesired dest)
   {
      dest.set(src);
   }

   @Override
   public void serialize(it.iit.iCub.messages.it.iit.yarp.JointDesired data, us.ihmc.pubsub.common.SerializedPayload serializedPayload)
         throws java.io.IOException
   {
      serializeCDR.serialize(serializedPayload);
      write(data, serializeCDR);
      serializeCDR.finishSerialize();
   }

   @Override
   public void deserialize(us.ihmc.pubsub.common.SerializedPayload serializedPayload, it.iit.iCub.messages.it.iit.yarp.JointDesired data)
         throws java.io.IOException
   {
      deserializeCDR.deserialize(serializedPayload);
      read(data, deserializeCDR);
      deserializeCDR.finishDeserialize();
   }

   @Override
   public final void serialize(it.iit.iCub.messages.it.iit.yarp.JointDesired data, us.ihmc.idl.InterchangeSerializer ser)
   {
      ser.write_type_c("controlMode", data.getControlMode());

      ser.write_type_6("tau", data.getTau());

      ser.write_type_6("kp", data.getKp());

      ser.write_type_6("kd", data.getKd());

      ser.write_type_6("qDesired", data.getQDesired());

      ser.write_type_6("qdDesired", data.getQdDesired());
   }

   @Override
   public final void deserialize(us.ihmc.idl.InterchangeSerializer ser, it.iit.iCub.messages.it.iit.yarp.JointDesired data)
   {
      data.setControlMode(
            (it.iit.iCub.messages.it.iit.yarp.ORSControlMode) ser.read_type_c("controlMode", it.iit.iCub.messages.it.iit.yarp.ORSControlMode.class));

      data.setTau(ser.read_type_6("tau"));

      data.setKp(ser.read_type_6("kp"));

      data.setKd(ser.read_type_6("kd"));

      data.setQDesired(ser.read_type_6("qDesired"));

      data.setQdDesired(ser.read_type_6("qdDesired"));
   }

   @Override
   public it.iit.iCub.messages.it.iit.yarp.JointDesired createData()
   {
      return new it.iit.iCub.messages.it.iit.yarp.JointDesired();
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

   public void serialize(it.iit.iCub.messages.it.iit.yarp.JointDesired data, us.ihmc.idl.CDR cdr)
   {
      write(data, cdr);
   }

   public void deserialize(it.iit.iCub.messages.it.iit.yarp.JointDesired data, us.ihmc.idl.CDR cdr)
   {
      read(data, cdr);
   }

   public void copy(it.iit.iCub.messages.it.iit.yarp.JointDesired src, it.iit.iCub.messages.it.iit.yarp.JointDesired dest)
   {
      staticCopy(src, dest);
   }

   @Override
   public JointDesiredPubSubType newInstance()
   {
      return new JointDesiredPubSubType();
   }
}