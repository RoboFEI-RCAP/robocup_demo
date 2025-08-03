#pragma once

#include <string>
#include <mutex>

#include "team_communication_msg.h"

using namespace std;

class FieldData
{
public:
  FieldData();
  ~FieldData();

  void updateFromMsg(TeamCommunicationMsg &_msg);

  vector<BallInfo> allyBallData = {}; // Latest message received of each ally about ball detection in the field
  vector<DetectedRobots> allyData = {}; // Latest message about ally positioning in the field
  vector<vector<DetectedRobots>> opponentData = {}; // Latest message about opponent positioning in the field
  StrategyData currentStrategy; // Current strategy decision by the coach robot
};
