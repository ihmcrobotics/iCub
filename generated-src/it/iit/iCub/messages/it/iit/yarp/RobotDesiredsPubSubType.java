package it.iit.iCub.messages.it.iit.yarp;

/**
* 
* Topic data type of the struct "RobotDesireds" defined in "robotDesired.idl". Use this class to provide the TopicDataType to a Participant. 
*
* This file was automatically generated from robotDesired.idl by us.ihmc.idl.generator.IDLGenerator. 
* Do not update this file directly, edit robotDesired.idl instead.
*
*/
public class RobotDesiredsPubSubType implements us.ihmc.pubsub.TopicDataType<it.iit.iCub.messages.it.iit.yarp.RobotDesireds>
{
	public static final java.lang.String name = "it::iit::yarp::RobotDesireds";
	
	
	
    public RobotDesiredsPubSubType()
    {
        
    }

	private final us.ihmc.idl.CDR serializeCDR = new us.ihmc.idl.CDR();
	private final us.ihmc.idl.CDR deserializeCDR = new us.ihmc.idl.CDR();

    
    @Override
   public void serialize(it.iit.iCub.messages.it.iit.yarp.RobotDesireds data, us.ihmc.pubsub.common.SerializedPayload serializedPayload) throws java.io.IOException
   {
      serializeCDR.serialize(serializedPayload);
      write(data, serializeCDR);
      serializeCDR.finishSerialize();
   }
   @Override
   public void deserialize(us.ihmc.pubsub.common.SerializedPayload serializedPayload, it.iit.iCub.messages.it.iit.yarp.RobotDesireds data) throws java.io.IOException
   {
      deserializeCDR.deserialize(serializedPayload);
      read(data, deserializeCDR);
      deserializeCDR.finishDeserialize();
   }
   
	public static int getMaxCdrSerializedSize()
	{
		return getMaxCdrSerializedSize(0);
	}

	public static int getMaxCdrSerializedSize(int current_alignment)
	{
	    int initial_alignment = current_alignment;
	            
	    current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);
	    for(int a = 0; a < 100; ++a)
	    {
	        current_alignment += it.iit.iCub.messages.it.iit.yarp.JointDesiredPubSubType.getMaxCdrSerializedSize(current_alignment);}
	
	    return current_alignment - initial_alignment;
	}


	public final static int getCdrSerializedSize(it.iit.iCub.messages.it.iit.yarp.RobotDesireds data)
	{
		return getCdrSerializedSize(data, 0);
	}

	public final static int getCdrSerializedSize(it.iit.iCub.messages.it.iit.yarp.RobotDesireds data, int current_alignment)
	{
	    int initial_alignment = current_alignment;
	            
	    current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);
	    for(int a = 0; a < data.getJointDesireds().size(); ++a)
	    {
	        current_alignment += it.iit.iCub.messages.it.iit.yarp.JointDesiredPubSubType.getCdrSerializedSize(data.getJointDesireds().get(a), current_alignment);}
	
	    return current_alignment - initial_alignment;
	}
	
   public static void write(it.iit.iCub.messages.it.iit.yarp.RobotDesireds data, us.ihmc.idl.CDR cdr)
   {

	    if(data.getJointDesireds().size() <= 100)
	    cdr.write_type_e(data.getJointDesireds());else
	        throw new RuntimeException("jointDesireds field exceeds the maximum length");
   }

   public static void read(it.iit.iCub.messages.it.iit.yarp.RobotDesireds data, us.ihmc.idl.CDR cdr)
   {

	    	cdr.read_type_e(data.getJointDesireds());	
   }
   
	@Override
	public final void serialize(it.iit.iCub.messages.it.iit.yarp.RobotDesireds data, us.ihmc.idl.InterchangeSerializer ser)
	{
			    ser.write_type_e("jointDesireds", data.getJointDesireds());
			    
	}
	
	@Override
	public final void deserialize(us.ihmc.idl.InterchangeSerializer ser, it.iit.iCub.messages.it.iit.yarp.RobotDesireds data)
	{
	    			ser.read_type_e("jointDesireds", data.getJointDesireds());	
	    	    
	}

   public static void staticCopy(it.iit.iCub.messages.it.iit.yarp.RobotDesireds src, it.iit.iCub.messages.it.iit.yarp.RobotDesireds dest)
   {
      dest.set(src);
   }
   
   
   @Override
   public it.iit.iCub.messages.it.iit.yarp.RobotDesireds createData()
   {
      return new it.iit.iCub.messages.it.iit.yarp.RobotDesireds();
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
   
   public void serialize(it.iit.iCub.messages.it.iit.yarp.RobotDesireds data, us.ihmc.idl.CDR cdr)
	{
		write(data, cdr);
	}

   public void deserialize(it.iit.iCub.messages.it.iit.yarp.RobotDesireds data, us.ihmc.idl.CDR cdr)
   {
        read(data, cdr);
   }
   
   public void copy(it.iit.iCub.messages.it.iit.yarp.RobotDesireds src, it.iit.iCub.messages.it.iit.yarp.RobotDesireds dest)
   {
      staticCopy(src, dest);
   }	

   
   @Override
   public RobotDesiredsPubSubType newInstance()
   {
   	  return new RobotDesiredsPubSubType();
   }
}