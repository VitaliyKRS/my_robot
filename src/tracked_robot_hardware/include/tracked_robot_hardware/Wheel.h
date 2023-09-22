#include <string>
#include <cmath>

class Wheel
{
private:
    std::string mJointName = "";
    double mEncoderVal;
    double mCommand = 0;
    double mPosition = 0;
    double mVelocity = 0;
    double mRadsPerCount;
    bool   mCmdSign;
public:
    Wheel() = default;

    void setup(const std::string& wheel_name, int counts_per_rev)
    {
        mJointName = wheel_name;
        mRadsPerCount = (2*M_PI)/counts_per_rev;
    }

    void setEncoderValue(double value) {
        mEncoderVal = value;
    }

    void setCommandSign(bool value) {
        mCmdSign = value;
    }

    void setCommand(double&& command) {
        mCommand = command;
    }
    void setPosition(double&& position) {
        mPosition = position;
    }
    void setVelocity(double&& velocity) {
        mVelocity = velocity;
    }

    double& getCommand() {return mCommand;}
    bool& getCommandSign()  {return mCmdSign;}
    double& getPosition() {return mPosition;}
    double& getVelocity() {return mVelocity;}
    double& getEncoderValue() {return mEncoderVal;}
    std::string getJointName() {return mJointName;}
    double getRadsPerCount() {return mRadsPerCount;}
    void calcEncAngle() {
         mPosition = mEncoderVal * mRadsPerCount;
    }
};
