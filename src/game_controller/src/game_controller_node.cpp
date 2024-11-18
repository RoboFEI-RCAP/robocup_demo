#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <unistd.h>
#include <cerrno>
#include <cstring>

#include "game_controller_node.h"

GameControllerNode::GameControllerNode(string name) : rclcpp::Node(name)
{
    _socket = -1;

    // 声明 Ros2 参数，注意在配置文件中新加的参数需要在这里显示声明
    declare_parameter<int>("port", 3838);
    declare_parameter<bool>("enable_ip_white_list", false);
    declare_parameter<vector<string>>("ip_white_list", vector<string>{});

    // 从配置中读取参数，注意把读取到的参数打印到日志中方便查问题
    get_parameter("port", _port);
    RCLCPP_INFO(get_logger(), "[get_parameter] port: %d", _port);
    get_parameter("enable_ip_white_list", _enable_ip_white_list);
    RCLCPP_INFO(get_logger(), "[get_parameter] enable_ip_white_list: %d", _enable_ip_white_list);
    get_parameter("ip_white_list", _ip_white_list);
    RCLCPP_INFO(get_logger(), "[get_parameter] ip_white_list(len=%ld)", _ip_white_list.size());
    for (size_t i = 0; i < _ip_white_list.size(); i++)
    {
        RCLCPP_INFO(get_logger(), "[get_parameter]     --[%ld]: %s", i, _ip_white_list[i].c_str());
    }

    // 创建 publisher，发布到 /game_state
    _publisher = create_publisher<game_controller::msg::GameControl>("/game_state", 10);
}

GameControllerNode::~GameControllerNode()
{
    if (_socket >= 0)
    {
        // 关闭打开的文件描述符是个好习惯
        close(_socket);
    }
}

void GameControllerNode::init()
{
    // 创建 socket，失败了直接抛异常
    _socket = socket(AF_INET, SOCK_DGRAM, 0);
    if (_socket < 0)
    {
        RCLCPP_ERROR(get_logger(), "socket failed: %s", strerror(errno));
        throw runtime_error(strerror(errno));
    }

    // 初始化地址
    sockaddr_in addr;
    addr.sin_family = AF_INET;
    // INADDR_ANY 将监听本机所有网络接口，默认情况这样就可以
    addr.sin_addr.s_addr = htonl(INADDR_ANY);
    addr.sin_port = htons(_port);

    // 绑定地址，失败了就抛异常
    if (bind(_socket, (sockaddr *)&addr, sizeof(addr)) < 0)
    {
        RCLCPP_ERROR(get_logger(), "bind failed: %s (port=%d)", strerror(errno), _port);
        throw runtime_error(strerror(errno));
    }

    // bind 成功后就可以开始从 socket 里接收数据了
    RCLCPP_INFO(get_logger(), "Listening for UDP broadcast on 0.0.0.0:%d", _port);
}

void GameControllerNode::spin()
{
    // 用来获取远程地址
    sockaddr_in remote_addr;
    socklen_t remote_addr_len = sizeof(remote_addr);

    // RoboCupGameControlData
    RoboCupGameControlData data;
    game_controller::msg::GameControl msg;

    // 进入循环
    while (rclcpp::ok())
    {
        // 从 socket 中接收数据包，期望的是接收完整的数据包
        ssize_t ret = recvfrom(_socket, &data, sizeof(data), 0, (sockaddr *)&remote_addr, &remote_addr_len);
        if (ret < 0)
        {
            RCLCPP_ERROR(get_logger(), "receiving UDP message failed: %s", strerror(errno));
            continue;
        }

        // 获取远端 IP
        string remote_ip = inet_ntoa(remote_addr.sin_addr);
        RCLCPP_INFO(get_logger(), "received packet from %s length=%ld", remote_ip.c_str(), ret);

        // 接收到不完整的包或其它非法的包，忽略掉
        if (ret != sizeof(data))
        {
            RCLCPP_INFO(get_logger(), "packet from %s invalid length=%ld", remote_ip.c_str(), ret);
            continue;
        }

        // 过滤 IP 白名单
        if (!check_ip_white_list(remote_ip))
        {
            RCLCPP_INFO(get_logger(), "%s not in ip_white_list, ignore packet from it", remote_ip.c_str());
            continue;
        }

        // 处理逻辑
        if (!handle_packet(data, msg))
        {
            // 非法的包
            RCLCPP_INFO(get_logger(), "packet from %s invalid", remote_ip.c_str());
            continue;
        }

        // 将消息发布到 Topic 中
        _publisher->publish(msg);
    }
}

bool GameControllerNode::check_ip_white_list(string ip)
{
    // 没有开启或在白名单内，返回 true
    if (!_enable_ip_white_list)
    {
        return true;
    }
    for (size_t i = 0; i < _ip_white_list.size(); i++)
    {
        if (ip == _ip_white_list[i])
        {
            return true;
        }
    }
    return false;
}

bool GameControllerNode::handle_packet(RoboCupGameControlData &data, game_controller::msg::GameControl &msg)
{
    if (data.version != GAMECONTROLLER_STRUCT_VERSION)
    {
        RCLCPP_INFO(get_logger(), "invalid packet version: %d", data.version);
        return false;
    }

    // FAKE DATA:!!! 注意这里：playerOnCourt_A-D 在原来的代码里没有初始化，应该初始化成什么值？(0,1,2,3?)
    int myTeamID = 29;
    int playerOnCourt_A = 0;
    int playerOnCourt_B = 1;
    int playerOnCourt_C = 2;
    int playerOnCourt_D = 3;
    // FAKE DATA

    TeamInfo myTeamInfo;
    TeamInfo oppTeamInfo;

    RobotInfo myRobot_A;
    RobotInfo myRobot_B;
    RobotInfo myRobot_C;
    RobotInfo myRobot_D;

    RobotInfo oppRobot_A;
    RobotInfo oppRobot_B;
    RobotInfo oppRobot_C;
    RobotInfo oppRobot_D;

    if (data.teams[0].teamNumber == myTeamID)
    {
        RCLCPP_INFO(get_logger(), "-----------------------0---------------------");
        myTeamInfo = data.teams[0];
        oppTeamInfo = data.teams[1];
    }
    else if (data.teams[1].teamNumber == myTeamID)
    {
        RCLCPP_INFO(get_logger(), "-----------------------1---------------------");
        myTeamInfo = data.teams[1];
        oppTeamInfo = data.teams[0];
    }
    myRobot_A = myTeamInfo.players[playerOnCourt_A];
    myRobot_B = myTeamInfo.players[playerOnCourt_B];
    myRobot_C = myTeamInfo.players[playerOnCourt_C];
    myRobot_D = myTeamInfo.players[playerOnCourt_D];

    oppRobot_A = oppTeamInfo.players[playerOnCourt_A];
    oppRobot_B = oppTeamInfo.players[playerOnCourt_B];
    oppRobot_C = oppTeamInfo.players[playerOnCourt_C];
    oppRobot_D = oppTeamInfo.players[playerOnCourt_D];

    // valid packet!
    // data
    msg.game_type = data.gameType;
    msg.state = data.state;
    msg.first_half = data.firstHalf;
    msg.kick_off_team = data.kickOffTeam;
    msg.secondary_state = data.secondaryState;
    msg.secondary_state_team = data.secondaryStateInfo[0]; // Extra Secondary State Info: Freekick and Penaltykick etc.
    msg.secondary_state_info = data.secondaryStateInfo[1];
    msg.drop_in_team = data.dropInTeam;
    msg.drop_in_time = data.dropInTime;
    msg.secs_remaining = data.secsRemaining;
    msg.secondary_time = data.secondaryTime;

    // TeamInfo
    msg.score = myTeamInfo.score;
    msg.penalty_shot = myTeamInfo.penaltyShot;
    msg.single_shots = myTeamInfo.singleShots;

    // oppTeamInfo
    msg.opp_score = oppTeamInfo.score;
    msg.opp_penalty_shot = oppTeamInfo.penaltyShot;
    msg.opp_single_shots = oppTeamInfo.singleShots;

    // RobotInfo
    msg.a_penalty = myRobot_A.penalty;
    msg.a_secs_till_unpenalised = myRobot_A.secsTillUnpenalised;
    msg.a_yellow_card_count = myRobot_A.yellowCardCount;
    msg.a_red_card_count = myRobot_A.redCardCount;

    msg.b_penalty = myRobot_B.penalty;
    msg.b_secs_till_unpenalised = myRobot_B.secsTillUnpenalised;
    msg.b_yellow_card_count = myRobot_B.yellowCardCount;
    msg.b_red_card_count = myRobot_B.redCardCount;

    msg.c_penalty = myRobot_C.penalty;
    msg.c_secs_till_unpenalised = myRobot_C.secsTillUnpenalised;
    msg.c_yellow_card_count = myRobot_C.yellowCardCount;
    msg.c_red_card_count = myRobot_C.redCardCount;

    msg.d_penalty = myRobot_D.penalty;
    msg.d_secs_till_unpenalised = myRobot_D.secsTillUnpenalised;
    msg.d_yellow_card_count = myRobot_D.yellowCardCount;
    msg.d_red_card_count = myRobot_D.redCardCount;

    msg.opp_a_penalty = oppRobot_A.penalty;
    msg.opp_b_penalty = oppRobot_B.penalty;
    msg.opp_c_penalty = oppRobot_C.penalty;
    msg.opp_d_penalty = oppRobot_D.penalty;

    if (msg.kick_off_team == myTeamID)
    {
        msg.is_kick_off = true;
    }
    else
    {
        msg.is_kick_off = false;
    }

    switch (msg.game_type)
    {
    case GAME_ROUNDROBIN:
        RCLCPP_INFO(get_logger(), "Game Type: Roundrobin");
        break;
    case GAME_PLAYOFF:
        RCLCPP_INFO(get_logger(), "Game Type: Playoff");
        break;
    case GAME_DROPIN:
        RCLCPP_INFO(get_logger(), "Game Type: Drop");
        break;
    default:
        RCLCPP_INFO(get_logger(), "Game Type: Unknown");
        break;
    }
    switch (msg.state)
    {
    case STATE_INITIAL:
        msg.is_ready = false;
        msg.is_set = false;
        msg.is_start = false;
        msg.is_end = false;
        RCLCPP_INFO(get_logger(), "Game State: Initial");
        break;
    case STATE_READY:
        msg.is_ready = true;
        msg.is_set = false;
        msg.is_start = false;
        msg.is_end = false;
        RCLCPP_INFO(get_logger(), "Game State: Ready");
        break;
    case STATE_SET:
        msg.is_ready = false;
        msg.is_set = true;
        msg.is_start = false;
        msg.is_end = false;
        RCLCPP_INFO(get_logger(), "Game State: Set");
        break;
    case STATE_PLAYING:
        msg.is_ready = false;
        msg.is_set = false;
        msg.is_start = true;
        msg.is_end = false;
        RCLCPP_INFO(get_logger(), "Game State: Playing");
        break;
    case STATE_FINISHED:
        msg.is_ready = false;
        msg.is_set = false;
        msg.is_start = false;
        msg.is_end = true;
        RCLCPP_INFO(get_logger(), "Game State: Finished");
        break;
    default:
        msg.is_ready = false;
        msg.is_set = false;
        msg.is_start = false;
        msg.is_end = false;
        RCLCPP_INFO(get_logger(), "Game State: Unknown");
        break;
    }
    switch (msg.first_half)
    {
    case 1:
        RCLCPP_INFO(get_logger(), "First half");
        break;
    default:
        RCLCPP_INFO(get_logger(), "Second half");
        break;
    }
    RCLCPP_INFO(get_logger(), "Kick Off Team: %d", msg.kick_off_team);
    switch (msg.secondary_state)
    {
    case STATE2_NORMAL:
        RCLCPP_INFO(get_logger(), "Secondary State: Normal");
        break;
    case STATE2_PENALTYSHOOT: // 空门
        RCLCPP_INFO(get_logger(), "Secondary State: Penalty Shoot");
        RCLCPP_INFO(get_logger(), "Penalty Shoot Team: %d", msg.secondary_state_team);
        if (msg.secondary_state_info == 0)
            RCLCPP_INFO(get_logger(), "Ball is being positioned.");
        else if (msg.secondary_state_info == 1)
            RCLCPP_INFO(get_logger(), "Prepared.");
        break;
    case STATE2_OVERTIME:
        RCLCPP_INFO(get_logger(), "Secondary State: Overtime");
        break;
    case STATE2_TIMEOUT:
        RCLCPP_INFO(get_logger(), "Secondary State: Time Out");
        break;
    case STATE2_DIRECT_FREEKICK: // 直接任意球
        RCLCPP_INFO(get_logger(), "Secondary State: Direct Freekick");
        RCLCPP_INFO(get_logger(), "Direct Freekick Team: %d", msg.secondary_state_team);
        if (msg.secondary_state_info == 0)
            RCLCPP_INFO(get_logger(), "Prepare");
        else if (msg.secondary_state_info == 1)
            RCLCPP_INFO(get_logger(), "Freeze");
        break;
    case STATE2_INDIRECT_FREEKICK: // 间接任意球
        RCLCPP_INFO(get_logger(), "Secondary State: Indirect Freekick");
        RCLCPP_INFO(get_logger(), "Indirect Freekick Team: %d", msg.secondary_state_team);
        if (msg.secondary_state_info == 0)
            RCLCPP_INFO(get_logger(), "Prepare");
        else if (msg.secondary_state_info == 1)
            RCLCPP_INFO(get_logger(), "Freeze");
        break;
    case STATE2_PENALTYKICK: // 点球
        RCLCPP_INFO(get_logger(), "Secondary State: Penalty Kick");
        RCLCPP_INFO(get_logger(), "Penalty Kick Team: %d", msg.secondary_state_team);
        if (msg.secondary_state_info == 0)
            RCLCPP_INFO(get_logger(), "Ball is being positioned.");
        else if (msg.secondary_state_info == 1)
            RCLCPP_INFO(get_logger(), "Prepared.");
        break;
    default:
        RCLCPP_INFO(get_logger(), "Secondary State: Unknown");
        break;
    }
    RCLCPP_INFO(get_logger(), "Drop In Team: %d", msg.drop_in_team);
    RCLCPP_INFO(get_logger(), "Drop In Time: %d s", msg.drop_in_time);
    RCLCPP_INFO(get_logger(), "Remaining Time: %d s", msg.secs_remaining);
    RCLCPP_INFO(get_logger(), "Secondary Time: %d s", msg.secondary_time);

    RCLCPP_INFO(get_logger(), "Score: THU %d : %d XXX ", msg.score, msg.opp_score);
    RCLCPP_INFO(get_logger(), "Penalty Shot: THU %d : %d XXX ", msg.penalty_shot, msg.opp_penalty_shot);
    RCLCPP_INFO(get_logger(), "Single Shots: THU %d : %d XXX\n", msg.single_shots, msg.opp_single_shots);

    switch (msg.a_penalty)
    {
    case HL_BALL_MANIPULATION:
        RCLCPP_INFO(get_logger(), "A_Penalty: Ball Manipulation");
        break;
    case HL_PHYSICAL_CONTACT:
        RCLCPP_INFO(get_logger(), "A_Penalty: Pushing");
        break;
    case HL_ILLEGAL_ATTACK:
        RCLCPP_INFO(get_logger(), "A_Penalty: Illegal Attack");
        break;
    case HL_ILLEGAL_DEFENSE:
        RCLCPP_INFO(get_logger(), "A_Penalty: Illegal Defense");
        break;
    case HL_PICKUP_OR_INCAPABLE:
        RCLCPP_INFO(get_logger(), "A_Penalty: Pickup or Incapable");
        break;
    case HL_SERVICE:
        RCLCPP_INFO(get_logger(), "A_Penalty: Service");
        break;
    case PENALTY_NONE:
        RCLCPP_INFO(get_logger(), "A_Penalty: None");
        break;
    case SUBSTITUTE: // Substitute: Substitute Player
        RCLCPP_INFO(get_logger(), "A_Penalty: Substitute");
        break;
    case MANUAL: // Manual: Coach
        RCLCPP_INFO(get_logger(), "A_Penalty: Manual");
        break;
    default:
        RCLCPP_INFO(get_logger(), "A_Penalty: Unknown");
        break;
    }

    RCLCPP_INFO(get_logger(), "A_Penalty: %d", msg.a_penalty);
    RCLCPP_INFO(get_logger(), "A_Time Until Unpenalized: %d s", msg.a_secs_till_unpenalised);
    RCLCPP_INFO(get_logger(), "A_Yellow Card: %d", msg.a_yellow_card_count);
    RCLCPP_INFO(get_logger(), "A_Red Card: %d\n", msg.a_red_card_count);

    switch (msg.b_penalty)
    {
    case HL_BALL_MANIPULATION:
        RCLCPP_INFO(get_logger(), "B_Penalty: Ball Manipulation");
        break;
    case HL_PHYSICAL_CONTACT:
        RCLCPP_INFO(get_logger(), "B_Penalty: Pushing");
        break;
    case HL_ILLEGAL_ATTACK:
        RCLCPP_INFO(get_logger(), "B_Penalty: Illegal Attack");
        break;
    case HL_ILLEGAL_DEFENSE:
        RCLCPP_INFO(get_logger(), "B_Penalty: Illegal Defense");
        break;
    case HL_PICKUP_OR_INCAPABLE:
        RCLCPP_INFO(get_logger(), "B_Penalty: Pickup or Incapable");
        break;
    case HL_SERVICE:
        RCLCPP_INFO(get_logger(), "B_Penalty: Service");
        break;
    case PENALTY_NONE:
        RCLCPP_INFO(get_logger(), "B_Penalty: None");
        break;
    case SUBSTITUTE: // Substitute: Substitute Player
        RCLCPP_INFO(get_logger(), "B_Penalty: Substitute");
        break;
    case MANUAL: // Manual: Coach
        RCLCPP_INFO(get_logger(), "B_Penalty: Manual");
        break;
    default:
        RCLCPP_INFO(get_logger(), "B_Penalty: Unknown");
        break;
    }
    RCLCPP_INFO(get_logger(), "B_Penalty: %d", msg.b_penalty);
    RCLCPP_INFO(get_logger(), "B_Time Until Unpenalized: %d s", msg.b_secs_till_unpenalised);
    RCLCPP_INFO(get_logger(), "B_Yellow Card: %d", msg.b_yellow_card_count);
    RCLCPP_INFO(get_logger(), "B_Red Card: %d", msg.b_red_card_count);
    RCLCPP_INFO(get_logger(), "****************************************\n");

    return true;
}