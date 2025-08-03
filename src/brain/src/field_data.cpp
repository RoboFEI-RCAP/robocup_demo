#include "field_data.h"

FieldData::FieldData() {
  int teamSize = 5;
  allyBallData.reserve(teamSize);
  allyData.reserve(teamSize);
  opponentData.reserve(teamSize);

  BallInfo ball;
  ball.detected = false;
  ball.range = -1;
  ball.playerId = -1;

  DetectedRobots robot;
  robot.playerId = -1;
  robot.isAlly = false;

  vector<DetectedRobots> robotArray = {};

  for (int n = 0; n < teamSize; n++) {
    allyBallData.push_back(ball);
    allyData.push_back(robot);
    opponentData.push_back(robotArray);
  }
}

FieldData::~FieldData() {}

void FieldData::updateFromMsg(TeamCommunicationMsg &_msg) {
  allyBallData.at(_msg.playerId) = _msg.ballInfo;

  DetectedRobots ally;
  ally.playerId = _msg.playerId;
  ally.pos = _msg.selfPos;
  ally.isAlly = true;

  allyData.at(_msg.playerId) = ally;

  opponentData.at(_msg.playerId) = _msg.opponentsInfo;
}
