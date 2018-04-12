package it.iit.iCub.messages.it.iit.yarp;
/**
* 
* Definition of the class "IMUState" defined in robotFeedback.idl. 
*
* This file was automatically generated from robotFeedback.idl by us.ihmc.idl.generator.IDLGenerator. 
* Do not update this file directly, edit robotFeedback.idl instead.
*
*/
public class IMUState
{
    public IMUState()
    {
        
        
    }

    public void set(IMUState other)
    {
        	xdd_ = other.xdd_;
        	ydd_ = other.ydd_;
        	zdd_ = other.zdd_;
        	wx_ = other.wx_;
        	wy_ = other.wy_;
        	wz_ = other.wz_;
        	qs_ = other.qs_;
        	qx_ = other.qx_;
        	qy_ = other.qy_;
        	qz_ = other.qz_;

    }

    public void setXdd(double xdd)
    {
        xdd_ = xdd;
    }

    public double getXdd()
    {
        return xdd_;
    }

        
    public void setYdd(double ydd)
    {
        ydd_ = ydd;
    }

    public double getYdd()
    {
        return ydd_;
    }

        
    public void setZdd(double zdd)
    {
        zdd_ = zdd;
    }

    public double getZdd()
    {
        return zdd_;
    }

        
    public void setWx(double wx)
    {
        wx_ = wx;
    }

    public double getWx()
    {
        return wx_;
    }

        
    public void setWy(double wy)
    {
        wy_ = wy;
    }

    public double getWy()
    {
        return wy_;
    }

        
    public void setWz(double wz)
    {
        wz_ = wz;
    }

    public double getWz()
    {
        return wz_;
    }

        
    public void setQs(double qs)
    {
        qs_ = qs;
    }

    public double getQs()
    {
        return qs_;
    }

        
    public void setQx(double qx)
    {
        qx_ = qx;
    }

    public double getQx()
    {
        return qx_;
    }

        
    public void setQy(double qy)
    {
        qy_ = qy;
    }

    public double getQy()
    {
        return qy_;
    }

        
    public void setQz(double qz)
    {
        qz_ = qz;
    }

    public double getQz()
    {
        return qz_;
    }

        




    @Override
    public boolean equals(java.lang.Object other)
    {
        if(other == null) return false;
        if(other == this) return true;
        if(!(other instanceof IMUState)) return false;
        IMUState otherMyClass = (IMUState)other;
        boolean returnedValue = true;

        returnedValue &= this.xdd_ == otherMyClass.xdd_;

                
        returnedValue &= this.ydd_ == otherMyClass.ydd_;

                
        returnedValue &= this.zdd_ == otherMyClass.zdd_;

                
        returnedValue &= this.wx_ == otherMyClass.wx_;

                
        returnedValue &= this.wy_ == otherMyClass.wy_;

                
        returnedValue &= this.wz_ == otherMyClass.wz_;

                
        returnedValue &= this.qs_ == otherMyClass.qs_;

                
        returnedValue &= this.qx_ == otherMyClass.qx_;

                
        returnedValue &= this.qy_ == otherMyClass.qy_;

                
        returnedValue &= this.qz_ == otherMyClass.qz_;

                

        return returnedValue;
    }
    
     @Override
    public java.lang.String toString()
    {
		StringBuilder builder = new StringBuilder();
		
      	builder.append("IMUState {");
        builder.append("xdd=");
        builder.append(this.xdd_);

                builder.append(", ");
        builder.append("ydd=");
        builder.append(this.ydd_);

                builder.append(", ");
        builder.append("zdd=");
        builder.append(this.zdd_);

                builder.append(", ");
        builder.append("wx=");
        builder.append(this.wx_);

                builder.append(", ");
        builder.append("wy=");
        builder.append(this.wy_);

                builder.append(", ");
        builder.append("wz=");
        builder.append(this.wz_);

                builder.append(", ");
        builder.append("qs=");
        builder.append(this.qs_);

                builder.append(", ");
        builder.append("qx=");
        builder.append(this.qx_);

                builder.append(", ");
        builder.append("qy=");
        builder.append(this.qy_);

                builder.append(", ");
        builder.append("qz=");
        builder.append(this.qz_);

                
        builder.append("}");
		return builder.toString();
    }

    private double xdd_; 
    private double ydd_; 
    private double zdd_; 
    private double wx_; 
    private double wy_; 
    private double wz_; 
    private double qs_; 
    private double qx_; 
    private double qy_; 
    private double qz_; 

}