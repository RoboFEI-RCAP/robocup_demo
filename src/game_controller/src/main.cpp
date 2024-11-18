#include "game_controller_node.h"

using namespace std;


int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);

    // 创建 Node，init, spin
    auto node = make_shared<GameControllerNode>("game_controller_node");

    node->init();
    node->spin();

    rclcpp::shutdown();

    return 0;
}