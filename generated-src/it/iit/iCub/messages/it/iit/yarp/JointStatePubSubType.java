package it.iit.iCub.messages.it.iit.yarp;

/**
 * Topic data type of the struct "JointState" defined in "robotFeedback.idl". Use this class to provide the TopicDataType to a Participant.
 *
 * This file was automatically generated from robotFeedback.idl by us.ihmc.idl.generator.IDLGenerator.
 * Do not update this file directly, edit robotFeedback.idl instead.
 */
public class JointStatePubSubType implements us.ihmc.pubsub.TopicDataType<it.iit.iCub.messages.it.iit.yarp.JointState>
{
   public static final java.lang.String name = "it::iit::yarp::JointState";
   private final us.ihmc.idl.CDR serializeCDR = new us.ihmc.idl.CDR();
   private final us.ihmc.idl.CDR deserializeCDR = new us.ihmc.idl.CDR();

   public JointStatePubSubType()
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

      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);

      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);

      return current_alignment - initial_alignment;
   }

   public final static int getCdrSerializedSize(it.iit.iCub.messages.it.iit.yarp.JointState data)
   {
      return getCdrSerializedSize(data, 0);
   }

   public final static int getCdrSerializedSize(it.iit.iCub.messages.it.iit.yarp.JointState data, int current_alignment)
   {
      int initial_alignment = current_alignment;

      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);

      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);

      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);

      return current_alignment - initial_alignment;
   }

   public static void write(it.iit.iCub.messages.it.iit.yarp.JointState data, us.ihmc.idl.CDR cdr)
   {

      cdr.write_type_6(data.getTau());

      cdr.write_type_6(data.getQ());

      cdr.write_type_6(data.getQd());
   }

   public static void read(it.iit.iCub.messages.it.iit.yarp.JointState data, us.ihmc.idl.CDR cdr)
   {

      data.setTau(cdr.read_type_6());

      data.setQ(cdr.read_type_6());

      data.setQd(cdr.read_type_6());
   }

   public static void staticCopy(it.iit.iCub.messages.it.iit.yarp.JointState src, it.iit.iCub.messages.it.iit.yarp.JointState dest)
   {
      dest.set(src);
   }

   @Override
   public void serialize(it.iit.iCub.messages.it.iit.yarp.JointState data, us.ihmc.pubsub.common.SerializedPayload serializedPayload) throws java.io.IOException
   {
      serializeCDR.serialize(serializedPayload);
      write(data, serializeCDR);
      serializeCDR.finishSerialize();
   }

   @Override
   public void deserialize(us.ihmc.pubsub.common.SerializedPayload serializedPayload, it.iit.iCub.messages.it.iit.yarp.JointState data)
         throws java.io.IOException
   {
      deserializeCDR.deserialize(serializedPayload);
      read(data, deserializeCDR);
      deserializeCDR.finishDeserialize();
   }

   @Override
   public final void serialize(it.iit.iCub.messages.it.iit.yarp.JointState data, us.ihmc.idl.InterchangeSerializer ser)
   {
      ser.write_type_6("tau", data.getTau());

      ser.write_type_6("q", data.getQ());

      ser.write_type_6("qd", data.getQd());
   }

   @Override
   public final void deserialize(us.ihmc.idl.InterchangeSerializer ser, it.iit.iCub.messages.it.iit.yarp.JointState data)
   {
      data.setTau(ser.read_type_6("tau"));

      data.setQ(ser.read_type_6("q"));

      data.setQd(ser.read_type_6("qd"));
   }

   @Override
   public it.iit.iCub.messages.it.iit.yarp.JointState createData()
   {
      return new it.iit.iCub.messages.it.iit.yarp.JointState();
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

   public void serialize(it.iit.iCub.messages.it.iit.yarp.JointState data, us.ihmc.idl.CDR cdr)
   {
      write(data, cdr);
   }

   public void deserialize(it.iit.iCub.messages.it.iit.yarp.JointState data, us.ihmc.idl.CDR cdr)
   {
      read(data, cdr);
   }

   public void copy(it.iit.iCub.messages.it.iit.yarp.JointState src, it.iit.iCub.messages.it.iit.yarp.JointState dest)
   {
      staticCopy(src, dest);
   }

   @Override
   public JointStatePubSubType newInstance()
   {
      return new JointStatePubSubType();
   }
}