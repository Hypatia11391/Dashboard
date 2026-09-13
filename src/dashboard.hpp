#include <GLFW/glfw3.h>

#include <networktables/DoubleArrayTopic.h>
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

        int current_item_idx = 0;

    private:
        nt::NetworkTableInstance inst;
        std::shared_ptr<nt::NetworkTable> table;

        std::shared_ptr<nt::NetworkTable> fieldTable;
        nt::DoubleArraySubscriber robotPoseSub;
        nt::DoubleArraySubscriber trajPoseSub;
        nt::DoubleArraySubscriber robotVelocitySub;

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

        nt::DoublePublisher kpPub;
        nt::DoublePublisher kiPub;
        nt::DoublePublisher kdPub;
};