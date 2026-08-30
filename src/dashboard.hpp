#include <GLFW/glfw3.h>

#include <networktables/DoubleTopic.h>
#include <networktables/NetworkTable.h>
#include <networktables/NetworkTableInstance.h>
#include <networktables/StringArrayTopic.h>
#include <networktables/StringTopic.h>

class Dashboard
{
    public:
        Dashboard();
        void run();

    private:
        void init();
        void mainLoop();
        void cleanUp();

        GLuint LoadTextureFromFile(const char* filename, int* out_width, int* out_height);

    private:
        GLFWwindow* window;

        GLuint fieldImg;

    private:
        nt::NetworkTableInstance inst;
        std::shared_ptr<nt::NetworkTable> table;

        nt::DoubleSubscriber xSub;
        nt::DoubleSubscriber ySub;
        nt::DoubleSubscriber dxSub;
        nt::DoubleSubscriber dySub;
        nt::DoubleSubscriber headingSub;

        nt::DoubleSubscriber intakeLiftPosSub;
        nt::DoubleSubscriber intakeLiftTargetPosSub;
        nt::DoubleSubscriber intakeFeedPowerSub;

        nt::DoubleSubscriber feedPowerSub;

        nt::DoubleSubscriber kickerPowerSub;

        nt::DoubleSubscriber shooterLeftRPMSub;
        nt::DoubleSubscriber shooterLeftTargetRPMSub;
        nt::DoubleSubscriber shooterRightRPMSub;
        nt::DoubleSubscriber shooterRightTargetRPMSub;

        nt::DoubleSubscriber vSub;

        nt::StringArraySubscriber autoListSub;
        nt::StringPublisher autoPub;
};