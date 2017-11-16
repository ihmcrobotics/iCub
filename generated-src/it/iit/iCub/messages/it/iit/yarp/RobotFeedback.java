package it.iit.iCub.messages.it.iit.yarp;
/**
* 
* Definition of the class "RobotFeedback" defined in robotFeedback.idl. 
*
* This file was automatically generated from robotFeedback.idl by us.ihmc.idl.generator.IDLGenerator. 
* Do not update this file directly, edit robotFeedback.idl instead.
*
*/
public class RobotFeedback
{
    public RobotFeedback()
    {
        	jointStates_ = new us.ihmc.idl.IDLSequence.Object<it.iit.iCub.messages.it.iit.yarp.JointState> (100, it.iit.iCub.messages.it.iit.yarp.JointState.class, new it.iit.iCub.messages.it.iit.yarp.JointStatePubSubType());

        	imuStates_ = new us.ihmc.idl.IDLSequence.Object<it.iit.iCub.messages.it.iit.yarp.IMUState> (100, it.iit.iCub.messages.it.iit.yarp.IMUState.class, new it.iit.iCub.messages.it.iit.yarp.IMUStatePubSubType());

        	forceSensors_ = new us.ihmc.idl.IDLSequence.Object<it.iit.iCub.messages.it.iit.yarp.ForceSensor> (100, it.iit.iCub.messages.it.iit.yarp.ForceSensor.class, new it.iit.iCub.messages.it.iit.yarp.ForceSensorPubSubType());


        
        
    }

    public void set(RobotFeedback other)
    {
        	timestampInNanoseconds_ = other.timestampInNanoseconds_;
            jointStates_.set(other.jointStates_);	imuStates_.set(other.imuStates_);	forceSensors_.set(other.forceSensors_);	
    }

    public void setTimestampInNanoseconds(long timestampInNanoseconds)
    {
        timestampInNanoseconds_ = timestampInNanoseconds;
    }

    public long getTimestampInNanoseconds()
    {
        return timestampInNanoseconds_;
    }

        

    public us.ihmc.idl.IDLSequence.Object<it.iit.iCub.messages.it.iit.yarp.JointState>  getJointStates()
    {
        return jointStates_;
    }

        

    public us.ihmc.idl.IDLSequence.Object<it.iit.iCub.messages.it.iit.yarp.IMUState>  getImuStates()
    {
        return imuStates_;
    }

        

    public us.ihmc.idl.IDLSequence.Object<it.iit.iCub.messages.it.iit.yarp.ForceSensor>  getForceSensors()
    {
        return forceSensors_;
    }

        




    @Override
    public boolean equals(java.lang.Object other)
    {
        if(other == null) return false;
        if(other == this) return true;
        if(!(other instanceof RobotFeedback)) return false;
        RobotFeedback otherMyClass = (RobotFeedback)other;
        boolean returnedValue = true;

        returnedValue &= this.timestampInNanoseconds_ == otherMyClass.timestampInNanoseconds_;

                
        returnedValue &= this.jointStates_.equals(otherMyClass.jointStates_);
                
        returnedValue &= this.imuStates_.equals(otherMyClass.imuStates_);
                
        returnedValue &= this.forceSensors_.equals(otherMyClass.forceSensors_);
                

        return returnedValue;
    }
    
     @Override
    public java.lang.String toString()
    {
		StringBuilder builder = new StringBuilder();
		
      	builder.append("RobotFeedback {");
        builder.append("timestampInNanoseconds=");
        builder.append(this.timestampInNanoseconds_);

                builder.append(", ");
        builder.append("jointStates=");
        builder.append(this.jointStates_);

                builder.append(", ");
        builder.append("imuStates=");
        builder.append(this.imuStates_);

                builder.append(", ");
        builder.append("forceSensors=");
        builder.append(this.forceSensors_);

                
        builder.append("}");
		return builder.toString();
    }

    private long timestampInNanoseconds_; 
    private us.ihmc.idl.IDLSequence.Object<it.iit.iCub.messages.it.iit.yarp.JointState>  jointStates_; 
    private us.ihmc.idl.IDLSequence.Object<it.iit.iCub.messages.it.iit.yarp.IMUState>  imuStates_; 
    private us.ihmc.idl.IDLSequence.Object<it.iit.iCub.messages.it.iit.yarp.ForceSensor>  forceSensors_; 

}