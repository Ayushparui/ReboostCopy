/**
 * @file TRTMultiPerceptor.cpp
 * @see TRTMultiPerceptor.h
 */

#include "TRTMultiPerceptor.h"
#include "Platform/BHAssert.h"
#include "Platform/File.h"
#include "Debugging/Debugging.h"
#include "Debugging/Stopwatch.h"
#include "Framework/Settings.h"
#include "ImageProcessing/ColorModelConversions.h"
#include "TextLogging/TextLogging.h"

DECL_TLOGGER(tlogger, "TRTMultiPerceptor", TextLogging::DEBUG);

#ifndef BUILD_BOOSTER
#include <memory>
#endif

#include <algorithm>
#include <cmath>

MAKE_MODULE(TRTMultiPerceptor);

TRTMultiPerceptor::TRTMultiPerceptor()
{
#ifdef BUILD_BOOSTER
  if(!trt.load(enginePath))
  {
    TLOGE(tlogger, "TRTMultiPerceptor: failed to load engine from Config/{}", enginePath);
    return;
  }

  if(!trt.hasTensor("images"))
    TLOGE(tlogger, "TRTMultiPerceptor: tensor 'images' not found in engine");
  if(!trt.hasTensor("output0"))
    TLOGE(tlogger, "TRTMultiPerceptor: tensor 'output0' not found in engine");
#else
  auto model = std::make_unique<NeuralNetworkONNX::Model>(
      std::string(File::getBHDir()) + "/Config/" + onnxModelPath);
  NeuralNetworkONNX::CompilationSettings settings;
  network.compile(*model, settings);
  if(!network.valid())
    TLOGE(tlogger, "TRTMultiPerceptor: failed to load ONNX model from {}", onnxModelPath);
  else
    TLOGI(tlogger, "TRTMultiPerceptor: loaded ONNX model '{}'", onnxModelPath);
#endif
}

/* Converts YUYV camera image to BGR CHW float32 normalised to [0,1].
   Layout: [B_plane | G_plane | R_plane], each H×W values. */
void TRTMultiPerceptor::preprocess()
{
  const int W = inputWidth;
  const int H = inputHeight;

#ifdef BUILD_BOOSTER
  std::vector<float>& buf = trt.hostBuffer("images");
  float* planeB = buf.data();
  float* planeG = buf.data() + H * W;
  float* planeR = buf.data() + 2 * H * W;
#else
  // NeuralNetworkONNX::CompiledNN::Tensor is a span whose data() points directly
  // into the pre-allocated ORT input tensor — safe to write before apply().
  auto inTensor = network.input(0);
  float* planeB = inTensor.data();
  float* planeG = inTensor.data() + H * W;
  float* planeR = inTensor.data() + 2 * H * W;
#endif

  for(int y = 0; y < H; ++y)
  {
    for(int x = 0; x < W; ++x)
    {
      // Each YUYVPixel covers two horizontal pixels (x and x+1).
      const PixelTypes::YUYVPixel& yuyv = theCameraImage[y][x / 2];
      const unsigned char Y = (x & 1) ? yuyv.y1 : yuyv.y0;
      const unsigned char U = yuyv.u;
      const unsigned char V = yuyv.v;

      unsigned char R, G, B;
      ColorModelConversions::fromYUVToRGB(Y, U, V, R, G, B);

      const int idx = y * W + x;
      planeB[idx] = static_cast<float>(B) / 255.0f;
      planeG[idx] = static_cast<float>(G) / 255.0f;
      planeR[idx] = static_cast<float>(R) / 255.0f;
    }
  }
}

bool TRTMultiPerceptor::apply()
{
#ifdef BUILD_BOOSTER
  if(!trt.isLoaded())
    return false;
#else
  if(!network.valid())
    return false;
#endif

  if(theCameraInfo.width != inputWidth || theCameraInfo.height != inputHeight)
  {
    TLOGE(tlogger, "TRTMultiPerceptor: camera {}x{} does not match model input {}x{}",
          theCameraInfo.width, theCameraInfo.height, inputWidth, inputHeight);
    return false;
  }

  if(lastPrediction == theCameraImage.timestamp)
    return true;

  TLOGV(tlogger, "apply: new frame {}", theCameraImage.timestamp);

  STOPWATCH("module:TRTMultiPerceptor:preprocess")
    preprocess();

  STOPWATCH("module:TRTMultiPerceptor:infer")
  {
#ifdef BUILD_BOOSTER
    if(!trt.infer())
    {
      TLOGE(tlogger, "TRTMultiPerceptor: inference failed");
      return false;
    }
#else
    network.apply();
#endif
  }

  TLOGV(tlogger, "infer ok: frame {}", theCameraImage.timestamp);
  lastPrediction = theCameraImage.timestamp;
  return true;
}

void TRTMultiPerceptor::update(BallSpots& ballSpots)
{
  ballSpots.ballSpots.clear();
  ballSpots.firstSpotIsPredicted = false;

  if(!apply())
    return;

  // Output shape: [1, max_det, 6] — each row: [x1, y1, x2, y2, conf, cls_id, fallen?, [Prob. dist. ]]
  static constexpr int stride = 6;
#ifdef BUILD_BOOSTER
  const std::vector<float>& outBuf = trt.hostBuffer("output0");
  const float* out = outBuf.data();
  const int maxDet = static_cast<int>(outBuf.size()) / stride;
#else
  const auto outTensor = network.output(0);
  const float* out = outTensor.data();
  const int maxDet = static_cast<int>(outTensor.size()) / stride;
#endif

  for(int i = 0; i < maxDet; ++i)
  {
    const float conf = out[i * stride + 4];
    const int   cls  = static_cast<int>(std::round(out[i * stride + 5]));

    if(cls != ballClassId || conf < ballConfidenceThreshold)
      continue;

    const float x1 = out[i * stride + 0];
    const float y1 = out[i * stride + 1];
    const float x2 = out[i * stride + 2];
    const float y2 = out[i * stride + 3];
    
    const int cx = static_cast<int>((x1 + x2) * 0.5f);
    const int cy = static_cast<int>((y1 + y2) * 0.5f);

    ballSpots.addBallSpot(cx, cy);

    DEBUG_DRAWING("representation:BallSpots:image", "drawingOnImage")
    {
      const int x1_px = static_cast<int>(x1);
      const int y1_px = static_cast<int>(y1);
      const int x2_px = static_cast<int>(x2);
      const int y2_px = static_cast<int>(y2);
      RECTANGLE("representation:BallSpots:image", x1_px, y1_px, x2_px, y2_px, 2,
                Drawings::solidPen, ColorRGBA::cyan);
      CIRCLE("representation:BallSpots:image", cx, cy, 15, 2, Drawings::solidPen, ColorRGBA::yellow,
             Drawings::noBrush, ColorRGBA::yellow);
      LINE("representation:BallSpots:image", cx - 5, cy, cx + 5, cy, 1, Drawings::solidPen, ColorRGBA::red);
      LINE("representation:BallSpots:image", cx, cy - 5, cx, cy + 5, 1, Drawings::solidPen, ColorRGBA::red);
    }
  }
  TLOGI(tlogger, "BallSpots: {} detected", (int)ballSpots.ballSpots.size());
}

/* Bounding boxes in image coordinates for JerseyClassifierProvider and RobotDetector downstream. */
void TRTMultiPerceptor::update(ObstaclesImagePercept& obstaclesImagePercept)
{
  obstaclesImagePercept.obstacles.clear();

  if(!apply())
  {
    TLOGE(tlogger, "TRTMultiPerceptor: failed to apply network");
    return;
  }

  // Output shape: [1, max_det, 6] — each row: [x1, y1, x2, y2, conf, cls_id]
  static constexpr int stride = 6;
#ifdef BUILD_BOOSTER
  const std::vector<float>& outBuf = trt.hostBuffer("output0");
  const float* out = outBuf.data();
  const int maxDet = static_cast<int>(outBuf.size()) / stride;
#else
  const auto outTensor = network.output(0);
  const float* out = outTensor.data();
  const int maxDet = static_cast<int>(outTensor.size()) / stride;
#endif

  for(int i = 0; i < maxDet; ++i)
  {
    const float conf = out[i * stride + 4];
    const int   cls  = static_cast<int>(std::round(out[i * stride + 5]));

    if((cls != robotClassId && (humanClassId < 0 || cls != humanClassId)) || conf < robotConfidenceThreshold)
      continue;

    const float x1 = out[i * stride + 0];
    const float y1 = out[i * stride + 1];
    const float x2 = out[i * stride + 2];
    const float y2 = out[i * stride + 3];
    
    ObstaclesImagePercept::Obstacle obs;
    obs.left        = static_cast<int>(x1);
    obs.top         = static_cast<int>(y1);
    obs.right       = static_cast<int>(x2);
    obs.bottom      = static_cast<int>(y2);
    obs.bottomFound = true;
    obs.fallen      = (humanClassId >= 0 && cls == humanClassId);
    obs.confidence  = conf;

    obstaclesImagePercept.obstacles.push_back(obs);
  }
  TLOGI(tlogger, "ObstaclesImagePercept: {} detected", (int)obstaclesImagePercept.obstacles.size());
}
