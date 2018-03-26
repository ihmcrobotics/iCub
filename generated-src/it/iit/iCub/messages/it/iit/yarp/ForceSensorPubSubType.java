package it.iit.iCub.messages.it.iit.yarp;

/**
 * Topic data type of the struct "ForceSensor" defined in "robotFeedback.idl". Use this class to provide the TopicDataType to a Participant.
 *
 * This file was automatically generated from robotFeedback.idl by us.ihmc.idl.generator.IDLGenerator.
 * Do not update this file directly, edit robotFeedback.idl instead.
 */
public class ForceSensorPubSubType implements us.ihmc.pubsub.TopicDataType<it.iit.iCub.messages.it.iit.yarp.ForceSensor>
{
   public static final java.lang.String name = "it::iit::yarp::ForceSensor";
   private final us.ihmc.idl.CDR serializeCDR = new us.ihmc.idl.CDR();
   private final us.ihmc.idl.CDR deserializeCDR = new us.ihmc.idl.CDR();

   public ForceSensorPubSubType()
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

      return current_alignment - initial_alignment;
   }

   public final static int getCdrSerializedSize(it.iit.iCub.messages.it.iit.yarp.ForceSensor data)
   {
      return getCdrSerializedSize(data, 0);
   }

   public final static int getCdrSerializedSize(it.iit.iCub.messages.it.iit.yarp.ForceSensor data, int current_alignment)
   {
      int initial_alignment = current_alignment;

      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);

      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);

      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);

      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);

      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);

      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);

      return current_alignment - initial_alignment;
   }

   public static void write(it.iit.iCub.messages.it.iit.yarp.ForceSensor data, us.ihmc.idl.CDR cdr)
   {

      cdr.write_type_6(data.getFx());

      cdr.write_type_6(data.getFy());

      cdr.write_type_6(data.getFz());

      cdr.write_type_6(data.getTauX());

      cdr.write_type_6(data.getTauY());

      cdr.write_type_6(data.getTauZ());
   }

   public static void read(it.iit.iCub.messages.it.iit.yarp.ForceSensor data, us.ihmc.idl.CDR cdr)
   {

      data.setFx(cdr.read_type_6());

      data.setFy(cdr.read_type_6());

      data.setFz(cdr.read_type_6());

      data.setTauX(cdr.read_type_6());

      data.setTauY(cdr.read_type_6());

      data.setTauZ(cdr.read_type_6());
   }

   public static void staticCopy(it.iit.iCub.messages.it.iit.yarp.ForceSensor src, it.iit.iCub.messages.it.iit.yarp.ForceSensor dest)
   {
      dest.set(src);
   }

   @Override
   public void serialize(it.iit.iCub.messages.it.iit.yarp.ForceSensor data, us.ihmc.pubsub.common.SerializedPayload serializedPayload)
         throws java.io.IOException
   {
      serializeCDR.serialize(serializedPayload);
      write(data, serializeCDR);
      serializeCDR.finishSerialize();
   }

   @Override
   public void deserialize(us.ihmc.pubsub.common.SerializedPayload serializedPayload, it.iit.iCub.messages.it.iit.yarp.ForceSensor data)
         throws java.io.IOException
   {
      deserializeCDR.deserialize(serializedPayload);
      read(data, deserializeCDR);
      deserializeCDR.finishDeserialize();
   }

   @Override
   public final void serialize(it.iit.iCub.messages.it.iit.yarp.ForceSensor data, us.ihmc.idl.InterchangeSerializer ser)
   {
      ser.write_type_6("Fx", data.getFx());

      ser.write_type_6("Fy", data.getFy());

      ser.write_type_6("Fz", data.getFz());

      ser.write_type_6("tauX", data.getTauX());

      ser.write_type_6("tauY", data.getTauY());

      ser.write_type_6("tauZ", data.getTauZ());
   }

   @Override
   public final void deserialize(us.ihmc.idl.InterchangeSerializer ser, it.iit.iCub.messages.it.iit.yarp.ForceSensor data)
   {
      data.setFx(ser.read_type_6("Fx"));

      data.setFy(ser.read_type_6("Fy"));

      data.setFz(ser.read_type_6("Fz"));

      data.setTauX(ser.read_type_6("tauX"));

      data.setTauY(ser.read_type_6("tauY"));

      data.setTauZ(ser.read_type_6("tauZ"));
   }

   @Override
   public it.iit.iCub.messages.it.iit.yarp.ForceSensor createData()
   {
      return new it.iit.iCub.messages.it.iit.yarp.ForceSensor();
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

   public void serialize(it.iit.iCub.messages.it.iit.yarp.ForceSensor data, us.ihmc.idl.CDR cdr)
   {
      write(data, cdr);
   }

   public void deserialize(it.iit.iCub.messages.it.iit.yarp.ForceSensor data, us.ihmc.idl.CDR cdr)
   {
      read(data, cdr);
   }

   public void copy(it.iit.iCub.messages.it.iit.yarp.ForceSensor src, it.iit.iCub.messages.it.iit.yarp.ForceSensor dest)
   {
      staticCopy(src, dest);
   }

   @Override
   public ForceSensorPubSubType newInstance()
   {
      return new ForceSensorPubSubType();
   }
}