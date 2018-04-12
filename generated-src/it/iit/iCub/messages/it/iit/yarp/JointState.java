package it.iit.iCub.messages.it.iit.yarp;
/**
* 
* Definition of the class "JointState" defined in robotFeedback.idl. 
*
* This file was automatically generated from robotFeedback.idl by us.ihmc.idl.generator.IDLGenerator. 
* Do not update this file directly, edit robotFeedback.idl instead.
*
*/
public class JointState
{
    public JointState()
    {
        
        
    }

    public void set(JointState other)
    {
        	tau_ = other.tau_;
        	q_ = other.q_;
        	qd_ = other.qd_;

    }

    public void setTau(double tau)
    {
        tau_ = tau;
    }

    public double getTau()
    {
        return tau_;
    }

        
    public void setQ(double q)
    {
        q_ = q;
    }

    public double getQ()
    {
        return q_;
    }

        
    public void setQd(double qd)
    {
        qd_ = qd;
    }

    public double getQd()
    {
        return qd_;
    }

        




    @Override
    public boolean equals(java.lang.Object other)
    {
        if(other == null) return false;
        if(other == this) return true;
        if(!(other instanceof JointState)) return false;
        JointState otherMyClass = (JointState)other;
        boolean returnedValue = true;

        returnedValue &= this.tau_ == otherMyClass.tau_;

                
        returnedValue &= this.q_ == otherMyClass.q_;

                
        returnedValue &= this.qd_ == otherMyClass.qd_;

                

        return returnedValue;
    }
    
     @Override
    public java.lang.String toString()
    {
		StringBuilder builder = new StringBuilder();
		
      	builder.append("JointState {");
        builder.append("tau=");
        builder.append(this.tau_);

                builder.append(", ");
        builder.append("q=");
        builder.append(this.q_);

                builder.append(", ");
        builder.append("qd=");
        builder.append(this.qd_);

                
        builder.append("}");
		return builder.toString();
    }

    private double tau_; 
    private double q_; 
    private double qd_; 

}