package it.iit.iCub.messages.it.iit.yarp;
/**
* 
* Definition of the class "RobotDesireds" defined in robotDesired.idl. 
*
* This file was automatically generated from robotDesired.idl by us.ihmc.idl.generator.IDLGenerator. 
* Do not update this file directly, edit robotDesired.idl instead.
*
*/
public class RobotDesireds
{
    public RobotDesireds()
    {
        	jointDesireds_ = new us.ihmc.idl.IDLSequence.Object<it.iit.iCub.messages.it.iit.yarp.JointDesired> (100, it.iit.iCub.messages.it.iit.yarp.JointDesired.class, new it.iit.iCub.messages.it.iit.yarp.JointDesiredPubSubType());


        
        
    }

    public void set(RobotDesireds other)
    {
            jointDesireds_.set(other.jointDesireds_);	
    }


    public us.ihmc.idl.IDLSequence.Object<it.iit.iCub.messages.it.iit.yarp.JointDesired>  getJointDesireds()
    {
        return jointDesireds_;
    }

        




    @Override
    public boolean equals(java.lang.Object other)
    {
        if(other == null) return false;
        if(other == this) return true;
        if(!(other instanceof RobotDesireds)) return false;
        RobotDesireds otherMyClass = (RobotDesireds)other;
        boolean returnedValue = true;

        returnedValue &= this.jointDesireds_.equals(otherMyClass.jointDesireds_);
                

        return returnedValue;
    }
    
     @Override
    public java.lang.String toString()
    {
		StringBuilder builder = new StringBuilder();
		
      	builder.append("RobotDesireds {");
        builder.append("jointDesireds=");
        builder.append(this.jointDesireds_);

                
        builder.append("}");
		return builder.toString();
    }

    private us.ihmc.idl.IDLSequence.Object<it.iit.iCub.messages.it.iit.yarp.JointDesired>  jointDesireds_; 

}