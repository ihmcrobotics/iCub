package it.iit.iCub.messages.it.iit.yarp;

/**
 * Topic data type of the struct "IMUState" defined in "robotFeedback.idl". Use this class to provide the TopicDataType to a Participant.
 *
 * This file was automatically generated from robotFeedback.idl by us.ihmc.idl.generator.IDLGenerator.
 * Do not update this file directly, edit robotFeedback.idl instead.
 */
public class IMUStatePubSubType implements us.ihmc.pubsub.TopicDataType<it.iit.iCub.messages.it.iit.yarp.IMUState>
{
   public static final java.lang.String name = "it::iit::yarp::IMUState";
   private final us.ihmc.idl.CDR serializeCDR = new us.ihmc.idl.CDR();
   private final us.ihmc.idl.CDR deserializeCDR = new us.ihmc.idl.CDR();

   public IMUStatePubSubType()
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

      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);

      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);

      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);

      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);

      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);

      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);

      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);

      return current_alignment - initial_alignment;
   }

   public final static int getCdrSerializedSize(it.iit.iCub.messages.it.iit.yarp.IMUState data)
   {
      return getCdrSerializedSize(data, 0);
   }

   public final static int getCdrSerializedSize(it.iit.iCub.messages.it.iit.yarp.IMUState data, int current_alignment)
   {
      int initial_alignment = current_alignment;

      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);

      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);

      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);

      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);

      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);

      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);

      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);

      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);

      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);

      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);

      return current_alignment - initial_alignment;
   }

   public static void write(it.iit.iCub.messages.it.iit.yarp.IMUState data, us.ihmc.idl.CDR cdr)
   {

      cdr.write_type_6(data.getXdd());

      cdr.write_type_6(data.getYdd());

      cdr.write_type_6(data.getZdd());

      cdr.write_type_6(data.getWx());

      cdr.write_type_6(data.getWy());

      cdr.write_type_6(data.getWz());

      cdr.write_type_6(data.getQs());

      cdr.write_type_6(data.getQx());

      cdr.write_type_6(data.getQy());

      cdr.write_type_6(data.getQz());
   }

   public static void read(it.iit.iCub.messages.it.iit.yarp.IMUState data, us.ihmc.idl.CDR cdr)
   {

      data.setXdd(cdr.read_type_6());

      data.setYdd(cdr.read_type_6());

      data.setZdd(cdr.read_type_6());

      data.setWx(cdr.read_type_6());

      data.setWy(cdr.read_type_6());

      data.setWz(cdr.read_type_6());

      data.setQs(cdr.read_type_6());

      data.setQx(cdr.read_type_6());

      data.setQy(cdr.read_type_6());

      data.setQz(cdr.read_type_6());
   }

   public static void staticCopy(it.iit.iCub.messages.it.iit.yarp.IMUState src, it.iit.iCub.messages.it.iit.yarp.IMUState dest)
   {
      dest.set(src);
   }

   @Override
   public void serialize(it.iit.iCub.messages.it.iit.yarp.IMUState data, us.ihmc.pubsub.common.SerializedPayload serializedPayload) throws java.io.IOException
   {
      serializeCDR.serialize(serializedPayload);
      write(data, serializeCDR);
      serializeCDR.finishSerialize();
   }

   @Override
   public void deserialize(us.ihmc.pubsub.common.SerializedPayload serializedPayload, it.iit.iCub.messages.it.iit.yarp.IMUState data) throws java.io.IOException
   {
      deserializeCDR.deserialize(serializedPayload);
      read(data, deserializeCDR);
      deserializeCDR.finishDeserialize();
   }

   @Override
   public final void serialize(it.iit.iCub.messages.it.iit.yarp.IMUState data, us.ihmc.idl.InterchangeSerializer ser)
   {
      ser.write_type_6("xdd", data.getXdd());

      ser.write_type_6("ydd", data.getYdd());

      ser.write_type_6("zdd", data.getZdd());

      ser.write_type_6("wx", data.getWx());

      ser.write_type_6("wy", data.getWy());

      ser.write_type_6("wz", data.getWz());

      ser.write_type_6("qs", data.getQs());

      ser.write_type_6("qx", data.getQx());

      ser.write_type_6("qy", data.getQy());

      ser.write_type_6("qz", data.getQz());
   }

   @Override
   public final void deserialize(us.ihmc.idl.InterchangeSerializer ser, it.iit.iCub.messages.it.iit.yarp.IMUState data)
   {
      data.setXdd(ser.read_type_6("xdd"));

      data.setYdd(ser.read_type_6("ydd"));

      data.setZdd(ser.read_type_6("zdd"));

      data.setWx(ser.read_type_6("wx"));

      data.setWy(ser.read_type_6("wy"));

      data.setWz(ser.read_type_6("wz"));

      data.setQs(ser.read_type_6("qs"));

      data.setQx(ser.read_type_6("qx"));

      data.setQy(ser.read_type_6("qy"));

      data.setQz(ser.read_type_6("qz"));
   }

   @Override
   public it.iit.iCub.messages.it.iit.yarp.IMUState createData()
   {
      return new it.iit.iCub.messages.it.iit.yarp.IMUState();
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

   public void serialize(it.iit.iCub.messages.it.iit.yarp.IMUState data, us.ihmc.idl.CDR cdr)
   {
      write(data, cdr);
   }

   public void deserialize(it.iit.iCub.messages.it.iit.yarp.IMUState data, us.ihmc.idl.CDR cdr)
   {
      read(data, cdr);
   }

   public void copy(it.iit.iCub.messages.it.iit.yarp.IMUState src, it.iit.iCub.messages.it.iit.yarp.IMUState dest)
   {
      staticCopy(src, dest);
   }

   @Override
   public IMUStatePubSubType newInstance()
   {
      return new IMUStatePubSubType();
   }
}