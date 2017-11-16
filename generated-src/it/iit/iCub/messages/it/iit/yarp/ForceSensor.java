package it.iit.iCub.messages.it.iit.yarp;
/**
* 
* Definition of the class "ForceSensor" defined in robotFeedback.idl. 
*
* This file was automatically generated from robotFeedback.idl by us.ihmc.idl.generator.IDLGenerator. 
* Do not update this file directly, edit robotFeedback.idl instead.
*
*/
public class ForceSensor
{
    public ForceSensor()
    {
        
        
    }

    public void set(ForceSensor other)
    {
        	Fx_ = other.Fx_;
        	Fy_ = other.Fy_;
        	Fz_ = other.Fz_;
        	tauX_ = other.tauX_;
        	tauY_ = other.tauY_;
        	tauZ_ = other.tauZ_;

    }

    public void setFx(double Fx)
    {
        Fx_ = Fx;
    }

    public double getFx()
    {
        return Fx_;
    }

        
    public void setFy(double Fy)
    {
        Fy_ = Fy;
    }

    public double getFy()
    {
        return Fy_;
    }

        
    public void setFz(double Fz)
    {
        Fz_ = Fz;
    }

    public double getFz()
    {
        return Fz_;
    }

        
    public void setTauX(double tauX)
    {
        tauX_ = tauX;
    }

    public double getTauX()
    {
        return tauX_;
    }

        
    public void setTauY(double tauY)
    {
        tauY_ = tauY;
    }

    public double getTauY()
    {
        return tauY_;
    }

        
    public void setTauZ(double tauZ)
    {
        tauZ_ = tauZ;
    }

    public double getTauZ()
    {
        return tauZ_;
    }

        




    @Override
    public boolean equals(java.lang.Object other)
    {
        if(other == null) return false;
        if(other == this) return true;
        if(!(other instanceof ForceSensor)) return false;
        ForceSensor otherMyClass = (ForceSensor)other;
        boolean returnedValue = true;

        returnedValue &= this.Fx_ == otherMyClass.Fx_;

                
        returnedValue &= this.Fy_ == otherMyClass.Fy_;

                
        returnedValue &= this.Fz_ == otherMyClass.Fz_;

                
        returnedValue &= this.tauX_ == otherMyClass.tauX_;

                
        returnedValue &= this.tauY_ == otherMyClass.tauY_;

                
        returnedValue &= this.tauZ_ == otherMyClass.tauZ_;

                

        return returnedValue;
    }
    
     @Override
    public java.lang.String toString()
    {
		StringBuilder builder = new StringBuilder();
		
      	builder.append("ForceSensor {");
        builder.append("Fx=");
        builder.append(this.Fx_);

                builder.append(", ");
        builder.append("Fy=");
        builder.append(this.Fy_);

                builder.append(", ");
        builder.append("Fz=");
        builder.append(this.Fz_);

                builder.append(", ");
        builder.append("tauX=");
        builder.append(this.tauX_);

                builder.append(", ");
        builder.append("tauY=");
        builder.append(this.tauY_);

                builder.append(", ");
        builder.append("tauZ=");
        builder.append(this.tauZ_);

                
        builder.append("}");
		return builder.toString();
    }

    private double Fx_; 
    private double Fy_; 
    private double Fz_; 
    private double tauX_; 
    private double tauY_; 
    private double tauZ_; 

}