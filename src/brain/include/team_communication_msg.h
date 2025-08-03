#pragma once

#include "types.h"

#define VALIDATION_COMMUNICATION 77722
#define VALIDATION_DISCOVERY 88811

#define MAX_ROBOT_DETECTION 20

using namespace std;

struct BallInfo {
    bool detected;
    int playerId;
    float range;
    Pose2D fieldPos;
};

struct StrategyData {
    int strickerId;
    // TODO: Dados da jogada a ser realizada: passe, finalização...
};

struct FieldPerception {
    Pose2D ballPos;
};

struct DetectedRobots {
    bool isAlly;
    int playerId;
    Pose2D pos;
};

struct TeamCommunicationMsg
{
    int validation = VALIDATION_COMMUNICATION; // validate msg, to determine if it's sent by us.
    int communicationId;
    int teamId;
    int playerId;
    Pose2D selfPos;

    BallInfo ballInfo;
    vector<DetectedRobots> opponentsInfo;

    StrategyData strategy;
};

struct TeamDiscoveryMsg
{
    int validation = VALIDATION_DISCOVERY; // validate msg, to determine if it's sent by us.
    int communicationId;
    int teamId;
    int playerId;
};
