/**
 * @file RLKickEngine.h
 *
 * Kick motion engine that drives the K1 legs (12 DOF) with an ONNX policy
 * trained in IsaacLab.  Provides KickGenerator, mirroring the keyframe
 * KickEngine so the Nao path is unaffected and the K1 only needs a
 * threads.cfg swap to activate it.  Upper body, waist, head and arms keep
 * flowing from their existing engines while this phase is active.
 *
 * @author James Petri
 */

#pragma once

#include "Framework/Module.h"
#include "Framework/Settings.h"
#include "Representations/Configuration/JointLimits.h"
#include "Representations/Configuration/KickInfo.h"
#include "Representations/Infrastructure/FrameInfo.h"
#include "Representations/Infrastructure/JointAngles.h"
#include "Representations/Infrastructure/JointRequest.h"
#include "Representations/MotionControl/KickGenerator.h"
#include "Representations/MotionControl/KickRequest.h"
#include "Representations/MotionControl/MotionInfo.h"
#include "Representations/MotionControl/MotionRequest.h"
#include "Representations/MotionControl/WalkGenerator.h"
#include "Representations/Sensing/InertialData.h"
#include "Tools/Motion/MotionPhase.h"

#include "Platform/File.h"
#include <CompiledNN2ONNX/CompiledNN.h>

using namespace NeuralNetworkONNX;

MODULE(RLKickEngine,
{,
  REQUIRES(FrameInfo),
  REQUIRES(InertialData),
  REQUIRES(JointAngles),
  REQUIRES(JointLimits),
  USES(JointRequest),
  REQUIRES(KickInfo),
  REQUIRES(MotionRequest),
  REQUIRES(WalkGenerator),
  PROVIDES(KickGenerator),
  LOADS_PARAMETERS(
  {,
    (std::string) policyName,
    (unsigned) numInput,        /**< Observation dimensionality (must match exported ONNX). */
    (unsigned) numOutput,       /**< Action dimensionality: 12 leg joints. */
    (Rangef) clipActions,       /**< Per-joint action clipping (rad, matches training). */
    (float) maxKickTime,        /**< Total kick duration (ms) before the phase finishes. */
    (float) speedFactor,        /**< Multiplier on executedTime; >1 plays the kick faster. */
    (float) maxKickLength,      /**< Normalization for kickLength observation (mm). */
    (Angle) abortTorsoTilt,     /**< Safety cap: abort the kick if torso pitch/roll exceeds this. */
  }),
});

class RLKickEngine : public RLKickEngineBase
{
public:
  RLKickEngine();

  CompiledNN policy;                            /**< Compiled ONNX policy. */
  JointAngles offset;                           /**< Default joint pose; matches IsaacLab init_state. */
  std::vector<Joints::Joint> boosterLegJoints;  /**< 12-joint ordering, IsaacLab order. */

  unsigned int lastFrameInfo = 0;
  JointAngles lastMeasurement;

private:
  void update(KickGenerator& kickGenerator) override;
  void compile();

  const std::string modelPath = std::string(File::getBHDir()) + "/Config/NeuralNets/RLKickEngine/";
};

struct RLKickPhase : MotionPhase
{
  RLKickPhase(RLKickEngine& engine,
              const KickRequest& kickRequest,
              const Vector2f& ballInRobot,
              const Vector2f& ballVelInRobot,
              Angle targetDirection,
              float kickLength,
              bool isLeftKick);

  const bool isLeftKick;  /**< Public so KickGenerator::wasLeftPhase can read it. */

private:
  bool isDone(const MotionRequest& motionRequest) const override;
  void calcJoints(const MotionRequest& motionRequest, JointRequest& jointRequest,
                  Pose2f& odometryOffset, MotionInfo& motionInfo) override;
  std::unique_ptr<MotionPhase> createNextPhase(const MotionPhase& defaultNextPhase) const override;
  unsigned freeLimbs() const override;
  void update() override;

  void executePolicy(JointRequest& target);

  RLKickEngine& engine;

  unsigned startTime;
  unsigned lastInference = 0;
  float executedTime = 0.f;
  bool done = false;

  // Snapshotted command at phase construction so the policy sees a stable target.
  Vector2f ballInRobot;
  Vector2f ballVelInRobot;
  Angle targetDirection;
  float kickLength;

  JointRequest nextRequest;
};
