/**
 * @file RERLWalkingEngine.h
 *
 * @Author Philip Reichenberg
 */

#pragma once

#include "Framework/Module.h"
#include "Framework/Settings.h"
#include "Representations/Configuration/JointLimits.h"
#include "Representations/Configuration/MassCalibration.h"
#include "Representations/Configuration/RobotDimensions.h"
#include "Representations/Infrastructure/FrameInfo.h"
#include "Representations/Infrastructure/GameState.h"
#include "Representations/Infrastructure/JointAngles.h"
#include "Representations/Infrastructure/JointRequest.h"
#include "Representations/Infrastructure/StiffnessData.h"
#include "Representations/MotionControl/MotionInfo.h"
#include "Representations/MotionControl/OdometryData.h"
#include "Representations/MotionControl/StandGenerator.h"
#include "Representations/MotionControl/WalkGenerator.h"
#include "Representations/MotionControl/WalkStepData.h"
#include "Representations/MotionControl/WalkingEngineOutput.h"
#include "Representations/Sensing/GroundContactState.h"
#include "Representations/Sensing/FootSupport.h"
#include "Representations/Sensing/InertialData.h"
#include "Representations/Sensing/RobotModel.h"
#include "Representations/Sensing/SolePressureState.h"
#include "Tools/Motion/MotionPhase.h"
#include "Tools/RECommon/IntervalTrigger.h"

#include "Platform/File.h"
#include <CompiledNN2ONNX/CompiledNN.h>

using namespace NeuralNetworkONNX;

STREAMABLE(BoosterArmParameters,
{,
  (Angle) armShoulderRoll, /**< Arm shoulder angle in radians. */
  (float) armShoulderRollIncreaseFactor, /**< Factor between sideways step size (in m) and additional arm roll angles. */
  (float) armShoulderPitchFactor, /**< Factor between forward foot position (in m) and arm pitch angles. */
  (Angle) armShoulderPitch, /**< Arm shoulder pitch angle. */
  (Angle) armElbowYaw, /**< Arm elbow yaw angle. */
});

STREAMABLE(BoosterCommonSpeedParameters,
{,
  (Vector2f) maxAcceleration, /**< Maximum acceleration of forward and sideways speed at each leg change to ratchet up/down in (mm/s/step). */
  (Vector2f) maxDeceleration, /**< (Positive) maximum deceleration of forward and sideways speed at each leg change to ratchet up/down in (mm/s/step). */
});

STREAMABLE(CommandRampParameters,
{,
  (bool)(true) active, /**< Enable motion-level ramping (rate limiting) of the body velocity command. */
  (Vector2f)(Vector2f(3.f, 3.f)) maxAcceleration, /**< Max increase of the linear velocity command magnitude, in (m/s)/s. */
  (Vector2f)(Vector2f(4.f, 4.f)) maxDeceleration, /**< Max (positive) decrease of the linear velocity command toward zero / on reversal, in (m/s)/s. */
  (Angle)(360_deg) maxRotationAcceleration, /**< Max increase of the angular velocity command magnitude, in (rad/s)/s. */
  (Angle)(540_deg) maxRotationDeceleration, /**< Max (positive) decrease of the angular velocity command toward zero / on reversal, in (rad/s)/s. */
  (float)(0.06f) maxRampTimeStep, /**< Upper clamp on the ramp integration time step [s], guards against large gaps (e.g. resuming from stand). */
});

STREAMABLE(BoosterStepSizeParameters,
{,
  (float) insideTurnRatio, /**< The ratio of turn that can be done in the inner step of a double-step. */
  (Vector2a) reduceTranslationFromRotation, /**< The rotation from which the translation per step will be reduced. */
  (Vector2a) noTranslationFromRotation, /**< The rotation from which no translation is possible at all. */
  (Angle) noTranslationYFromRotationFastInner, /**< When walking fast with a rotation outwards, the rotation from which no translation is possible at all is higher for the y translation. */
  (Angle) noTranslationYFromRotationFastOuter, /**< When walking fast with a rotation inwards, the rotation from which no translation is possible at all is higher for the y translation. */
  (Angle) reduceTranslationYFromRotationFast, /**< When walking fast, the rotation from which the translation per step will be reduced is higher for the y translation. */
  (float) minXTranslationStep, /**< The forward and backward step size has a minimum. */
  (float) minXForwardTranslationFast, /**< The forward step size has a minimum for the fast translation polygon. */
  (Rangef) minXBackwardTranslationFastRange,/**< The backward step size has a minimum for the fast translation polygon. */
  (Rangef) translationPolygonSafeRange, /**< For the calculation of the translation polygon, the feet are adjusted by this amount in x translation,
                                             to ensure a possible pose even when the arms are on the back or the feet are balanced. */
  (int) noFastTranslationPolygonStepsNumber, /**< Number of steps before big steps near the target are allowed. */
});

STREAMABLE(BoosterConfiguredParameters,
{,
  (Pose2f) maxSpeed, /**< Maximum speeds in mm/s and degrees/s. */
  (float) maxSpeedBackwards, /**< Maximum backwards speed. Positive, in mm/s. */
  (Pose2f) thresholdStopStandTransition, /**< Threshold to be able to switch from walk state stopping to standing. */
  (float) baseWalkPeriod,
});

STREAMABLE(FrequencyParameters,
{,
  (float) base, /**< Base frequency. */
  (Rangef) clipRange, /**< Clip frequency offset from neuronal network. */
  (bool) active, /**< Does the neuronal network output a frequency offset? */
});

STREAMABLE(BallNetworkParameters,
{,
  (bool) active,
});

STREAMABLE(ReboostWalkNNParameters,
{,
  (std::string) modelName, /** Neural Network name. Must be .onnx or .hdf5. */
  (float) velocityFactor, /**< Factor for the joint velocities. */
  (Rangef) actionClipRange, /**< Clip action output. */
  (bool) useWaist, /**< Is the waist also part of the action output. */
  (unsigned) numInput, /**< DEPRECATED (no point having it here) Neural Network input size. */
  (unsigned) numOutput, /**< DEPRECATED (no point having it here) Neural Network output size. */
  (bool) enablePhaseInputs, /**< Allows you to disable the phase inputs to the model */
  (int) walkToStandSlowdownMs, /**< To allow standing, the robot must walk slow for this time. */
  (Pose2f) slowWalkStepSpeed, /**< Speed values to classify slow walking. This is based on the step size. */
  (float) zeroCmdThreshold, /**< Command norm (m/s, rad/s) below which we are standing: phase obs zeroed, gait clock frozen. Mirrors training zero_cmd_threshold. */
});

MODULE(RERLWalkingEngine,
{,
  REQUIRES(FootSupport),
  REQUIRES(FrameInfo),
  REQUIRES(GameState),
  REQUIRES(GroundContactState),
  REQUIRES(InertialData),
  REQUIRES(JointAngles),
  USES(JointRequest),
  REQUIRES(MassCalibration),
  REQUIRES(OdometryDataPreview),
  REQUIRES(RobotDimensions),
  REQUIRES(RobotModel),
  REQUIRES(SolePressureState),
  PROVIDES(WalkStepData),
  USES(WalkStepData),
  PROVIDES(WalkGenerator),
  REQUIRES(WalkGenerator),
  PROVIDES(StandGenerator),
  PROVIDES(WalkingEngineOutput),
  LOADS_PARAMETERS(
  {,
    (BoosterConfiguredParameters) configuredParameters,   /**< Speed parameters */
    (BoosterStepSizeParameters) stepSizeParameters,       /**< Step size parameters. */
    (BoosterCommonSpeedParameters) commonSpeedParameters, /**< Acceleration parameters. */
    (BoosterArmParameters) armParameters,                 /**< Arm parameters. */
    (ReboostWalkNNParameters) reboostWalkNNParameters,    /**< Parameters of the walk neuroal network. */
    (FrequencyParameters) frequencyParameters,            /**< Frequency parameters. */
    (BallNetworkParameters) ballNetworkParameters,        /**< Ball parameters for the network. */
    (BrakingModelParameters) brakingModelParameters,      /**< Empirical braking-distance model (forwarded to WalkingEngineOutput). */
    (CommandRampParameters) commandRampParameters,        /**< Motion-level velocity-command ramping (rate limiting). */
  }),
});

class RERLWalkingEngine : public RERLWalkingEngineBase
{
public:
  /** Constructor */
  RERLWalkingEngine();
  const float motionCycleTime = Global::getSettings().motionCycleTime;
  RingBuffer<JointAngles, 10> lastMeasurements;
  CompiledNN network; /**< The compiled neural network. */
  unsigned networkInputSize = 0; // will be populated in ctor based on real code usage
  unsigned networkOutputSize = 0; // will be populated in ctor based on real code usage
  JointAngles offset;

  std::vector<Joints::Joint> boosterJoints;
  std::vector<Joints::Joint> boosterWaistJoints;
  std::vector<Joints::Joint> naoJoints;

  Pose2f lastRampedStep;     /**< Last ramped step target. */
  unsigned lastRampTime = 0; /**< Frame time [ms] the ramp was last advanced. */

  Pose2f rampCommand(const Pose2f& current, const Pose2f& target, const float dt) const;
private:
  float rampValue(const float current, const float target, const float maxAccelDelta, const float maxDecelDelta) const;

  void update(WalkStepData& walkStepData) override;
  void update(StandGenerator& standGenerator) override;
  void update(WalkGenerator& walkGenerator) override;
  void update(WalkingEngineOutput& walkingEngineOutput) override;

  Rangea getMaxRotationToStepFactor(const bool isFastWalk, const bool isLeftPhase, const Angle customReduceOffset, const Vector2f& stepRatio);
  Vector2f getStepSizeFactor(const Angle rotation, const bool isFastWalk, const bool isLeftPhase, const Angle customReduceOffset, const Vector2f& walkSpeedRatio);
  void filterTranslationPolygon(std::vector<Vector2f>& polygonOut, std::vector<Vector2f>& polygonIn, const std::vector<Vector2f>& polygonOriginal);
  void generateTranslationPolygon(std::vector<Vector2f>& polygon, const Vector2f& backRight, const Vector2f& frontLeft, const bool useMaxPossibleStepSize);

  /**
   * Compile the model.
   * @param output Whether to output information.
   */
  void compile(bool output);

  std::vector<Vector2f> translationPolygon; /**< The polygon that defines the max allowed translation for the step size. */

  const std::string modelPath = std::string(File::getBHDir()) + "/Config/NeuralNets/BoosterWalk/";
  IntervalTriggerMs logInterval;
};

// ==========================================================================
struct DummyPhase : MotionPhase
{
  using MotionPhase::MotionPhase;

  bool isDone(const MotionRequest&) const override { return true; }
  void calcJoints(const MotionRequest&, JointRequest&, Pose2f&, MotionInfo&) override {}
};

// ==========================================================================

struct RERLWalkPhaseDummy : MotionPhase
{
  RERLWalkPhaseDummy(RERLWalkingEngine& engine, Pose2f stepTarget, const MotionPhase& lastPhase, const bool isDelay = false,
              const WalkGenerator::CreateNextPhaseCallback& createNextPhaseCallback = WalkGenerator::CreateNextPhaseCallback(),
              const WalkKickStep& walkKickStep = WalkKickStep());

public:
  bool isLeftPhase = false;
  Pose2f step;
  WalkKickStep walkKickStep;

  float tBase = 0.f;
  float tWalk = 0.f;
  float frequency = 0.f;
  float rawFrequency = 0.f;
  bool standing = false; ///< command below zeroCmdThreshold -> stand (phase obs zeroed, gait clock frozen)
private:
  bool isDone(const MotionRequest& motionRequest) const override;
  void calcJoints(const MotionRequest& motionRequest, JointRequest& jointRequest, Pose2f& odometryOffset, MotionInfo& motionInfo) override;
  std::unique_ptr<MotionPhase> createNextPhase(const MotionPhase& defaultNextPhase) const override;
  void update() override;

  void getNextTargetRequest(JointAngles& target);
  std::vector<Joints::Joint> getBoosterLegJointSequence();

  friend class WalkingEngine;

  RERLWalkingEngine& engine; /**< A reference to the running motion engine. */
  unsigned lastModelRequest = 0;
  JointAngles nextTarget;
  JointAngles lastTarget;
  // JointAngles requestAtStandStart;
  JointAngles standTarget;
  WalkGenerator::CreateNextPhaseCallback callBack;
  IntervalTriggerMs logInterval;

  unsigned lastTimeWalking = 0; ///< the last time we were actually walking
  unsigned slowingDownToStandStartTime = 0;

  // Interpolation to our stand from something else (e.g. Booster PREP mode)
  unsigned interpStartTime = 0; ///< zero means no interpolation active, otherwise time when interpolation started
  unsigned interpDurationMs = 0; ///< how long to interpolate for
  JointAngles interpStartAngles; ///< the starting joint values that we'll blend from until eventually 100% model output

protected:
  unsigned freeLimbs() const override
  {
    return bit(MotionPhase::head);
  }
};
