package it.iit.iCub.messages.it.iit.yarp;
/**
* 
* Definition of the class "JointDesired" defined in robotDesired.idl. 
*
* This file was automatically generated from robotDesired.idl by us.ihmc.idl.generator.IDLGenerator. 
* Do not update this file directly, edit robotDesired.idl instead.
*
*/
public class JointDesired
{
    public JointDesired()
    {
        
        
    }

    public void set(JointDesired other)
    {
        	controlMode_ = other.controlMode_;
        	tau_ = other.tau_;
        	kp_ = other.kp_;
        	kd_ = other.kd_;
        	qDesired_ = other.qDesired_;
        	qdDesired_ = other.qdDesired_;

    }

    public void setControlMode(it.iit.iCub.messages.it.iit.yarp.ORSControlMode controlMode)
    {
        controlMode_ = controlMode;
    }

    public it.iit.iCub.messages.it.iit.yarp.ORSControlMode getControlMode()
    {
        return controlMode_;
    }

        
    public void setTau(double tau)
    {
        tau_ = tau;
    }

    public double getTau()
    {
        return tau_;
    }

        
    public void setKp(double kp)
    {
        kp_ = kp;
    }

    public double getKp()
    {
        return kp_;
    }

        
    public void setKd(double kd)
    {
        kd_ = kd;
    }

    public double getKd()
    {
        return kd_;
    }

        
    public void setQDesired(double qDesired)
    {
        qDesired_ = qDesired;
    }

    public double getQDesired()
    {
        return qDesired_;
    }

        
    public void setQdDesired(double qdDesired)
    {
        qdDesired_ = qdDesired;
    }

    public double getQdDesired()
    {
        return qdDesired_;
    }

        




    @Override
    public boolean equals(java.lang.Object other)
    {
        if(other == null) return false;
        if(other == this) return true;
        if(!(other instanceof JointDesired)) return false;
        JointDesired otherMyClass = (JointDesired)other;
        boolean returnedValue = true;

        returnedValue &= this.controlMode_ == otherMyClass.controlMode_;

                
        returnedValue &= this.tau_ == otherMyClass.tau_;

                
        returnedValue &= this.kp_ == otherMyClass.kp_;

                
        returnedValue &= this.kd_ == otherMyClass.kd_;

                
        returnedValue &= this.qDesired_ == otherMyClass.qDesired_;

                
        returnedValue &= this.qdDesired_ == otherMyClass.qdDesired_;

                

        return returnedValue;
    }
    
     @Override
    public java.lang.String toString()
    {
		StringBuilder builder = new StringBuilder();
		
      	builder.append("JointDesired {");
        builder.append("controlMode=");
        builder.append(this.controlMode_);

                builder.append(", ");
        builder.append("tau=");
        builder.append(this.tau_);

                builder.append(", ");
        builder.append("kp=");
        builder.append(this.kp_);

                builder.append(", ");
        builder.append("kd=");
        builder.append(this.kd_);

                builder.append(", ");
        builder.append("qDesired=");
        builder.append(this.qDesired_);

                builder.append(", ");
        builder.append("qdDesired=");
        builder.append(this.qdDesired_);

                
        builder.append("}");
		return builder.toString();
    }

    private it.iit.iCub.messages.it.iit.yarp.ORSControlMode controlMode_; 
    private double tau_; 
    private double kp_; 
    private double kd_; 
    private double qDesired_; 
    private double qdDesired_; 

}