/**
 * @file TRTMultiPerceptor.h
 *
 * Declares a module that runs a multi-class object detector on the full camera
 * image and emits BallSpots and ObstaclesImagePercept for downstream processing
 * by the existing pipeline (BallAndPenaltyMarkPerceptor, JerseyClassifierProvider,
 * RobotDetector field projection, etc.).
 *
 * Two execution backends, selected at compile time:
 *   BUILD_BOOSTER  — TensorRT FP16 engine on Jetson Orin (hardware)
 *   otherwise      — ONNX Runtime on CPU (desktop / SimRobot)
 *
 * Model I/O contract (both backends):
 *   Input  "images"  — [1, 3, H, W]     float32 BGR CHW, normalised [0, 1]
 *   Output "output0" — [1, max_det, 6]  float32 post-NMS xyxy rows:
 *                      [x1, y1, x2, y2, conf, cls_id]
 *
 * Class IDs and confidence thresholds are in TRTMultiPerceptor.cfg.
 */

#pragma once

#include "Representations/Infrastructure/CameraImage.h"
#include "Representations/Infrastructure/CameraInfo.h"
#include "Representations/Perception/BallPercepts/BallSpots.h"
#include "Representations/Perception/ObstaclesPercepts/ObstaclesImagePercept.h"
#include "Framework/Module.h"

#ifdef BUILD_BOOSTER
#include "Tools/NeuralNetworks/TensorRTModel.h"
#else
#include "CompiledNN2ONNX/CompiledNN.h"
#endif

MODULE(TRTMultiPerceptor,
{,
  REQUIRES(CameraInfo),
  REQUIRES(CameraImage),
  PROVIDES(BallSpots),
  PROVIDES(ObstaclesImagePercept),
  LOADS_PARAMETERS(
  {,
    (std::string)("NeuralNets/TRTVision/model.engine") enginePath,        /**< TRT engine path relative to Config/. BUILD_BOOSTER only. */
    (std::string)("NeuralNets/TRTVision/model.onnx")   onnxModelPath,     /**< ONNX model path relative to Config/. Non-Booster only. */
    (int)(544)    inputWidth,                                              /**< Must match model input width. */
    (int)(448)    inputHeight,                                             /**< Must match model input height. */
    (float)(0.3f) ballConfidenceThreshold,                                /**< Minimum detection confidence for ball. */
    (float)(0.4f) robotConfidenceThreshold,                               /**< Minimum detection confidence for robot/human. */
    (int)(0)      ballClassId,                                            /**< YOLO class index for ball. Must match training labels. */
    (int)(2)      robotClassId,                                           /**< YOLO class index for upright robot. Must match training labels. */
    (int)(-1)     humanClassId,                                           /**< YOLO class index for fallen player. -1 = not present in current model. */
  }),
});

class TRTMultiPerceptor : public TRTMultiPerceptorBase
{
public:
  TRTMultiPerceptor();

private:
  /**
   * Run inference on the current image (guarded by lastPrediction timestamp).
   * @return true if a valid result is available.
   */
  bool apply();

  void update(BallSpots& ballSpots) override;
  void update(ObstaclesImagePercept& obstaclesImagePercept) override;

  /** Convert YUYV camera image to normalised BGR CHW float and fill the input buffer. */
  void preprocess();

#ifdef BUILD_BOOSTER
  TensorRTModel trt;
#else
  NeuralNetworkONNX::CompiledNN network; ///< ONNX Runtime session (desktop / SimRobot)
#endif

  unsigned lastPrediction = 0; ///< timestamp of the last processed frame
};
