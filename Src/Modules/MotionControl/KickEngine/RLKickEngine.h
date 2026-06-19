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
#include "Representations/MotionControl/OdometryData.h"
#include "Representations/MotionControl/WalkGenerator.h"
#include "Representations/Sensing/InertialData.h"
#include "Representations/Sensing/RobotModel.h"
#include "Representations/Sensing/TorsoMatrix.h"
#include "Tools/Motion/MotionPhase.h"

#include "Platform/File.h"
#include <CompiledNN2ONNX/CompiledNN.h>
#include <array>

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
  REQUIRES(OdometryDataPreview),
  REQUIRES(RobotModel),
  REQUIRES(TorsoMatrix),
  REQUIRES(WalkGenerator),
  PROVIDES(KickGenerator),
  LOADS_PARAMETERS(
  {,
    (std::string) policyName,
    (unsigned) numInput,        /**< Observation dimensionality (must match exported ONNX). */
    (unsigned) numOutput,       /**< Action dimensionality: 12 leg joints. */
    (Rangef) clipActions,       /**< Per-joint action clipping (rad, matches training). */
    (float) maxKickTime,        /**< Kick duration (ms); phase_progress reaches 1 here. */
    (float) postKickSettleTime, /**< Extra time (ms) the phase keeps running after maxKickTime with phase_progress pinned at 1, so the policy stabilizes the robot before the walk handoff. Scaled by speedFactor like maxKickTime. */
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
  Pose2f supportSolePose() const;

  RLKickEngine& engine;

  unsigned startTime;
  unsigned lastInference = 0;
  float executedTime = 0.f;
  bool done = false;

  // Kick command snapshotted at phase construction. The ball estimate is read live from
  // theMotionRequest each inference instead; the target direction is committed here so a
  // mid-kick behavior change cannot retarget the policy, and is re-expressed in the current
  // robot frame each inference via the odometry it was created in.
  Angle targetDirection;
  float kickLength;
  Pose2f requestOdometry; /**< theMotionRequest.odometryData at phase construction. */

  // Support-sole pose in the ground frame from the previous frame, for odometry translation.
  Pose2f lastSupportSole;

  // Raw network output of the previous inference (pre-offset, pre-clip), matching
  // Isaac Lab's last_action observation, which is zeroed at episode reset.
  std::array<float, 12> lastAction;

  JointRequest nextRequest;
};
