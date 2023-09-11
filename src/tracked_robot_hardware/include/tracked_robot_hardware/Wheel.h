#include <cmath>
#include <string>

class Wheel {
private:
    std::string mJointName = "";
    int mEncoderVal;
    double mCommand = 0;
    double mPosition = 0;
    double mVelocity = 0;
    double mRadsPerCount;

public:
    Wheel() = default;

    void setup(const std::string& wheel_name, int counts_per_rev)
    {
        mJointName = wheel_name;
        mRadsPerCount = (2 * M_PI) / counts_per_rev;
    }

    void setEncoderValue(int value) { mEncoderVal = value; }

    void setCommand(double&& command) { mCommand = command; }
    void setPosition(double&& position) { mPosition = position; }
    void setVelocity(double&& velocity) { mVelocity = velocity; }

    double& getCommand() { return mCommand; }
    double& getPosition() { return mPosition; }
    double& getVelocity() { return mVelocity; }
    int& getEncoderValue() { return mEncoderVal; }
    std::string getJointName() { return mJointName; }
    double getRadsPerCount() { return mRadsPerCount; }
    void calcEncAngle() { mPosition = mEncoderVal * mRadsPerCount; }
};