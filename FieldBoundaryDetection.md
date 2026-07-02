# FieldBoundaryDetection

## Overview

Field boundary detection is implemented in `FieldBoundaryProvider` (`Src/Modules/Perception/ImagePreprocessors/`). It requires a neural network — no fallback classical vision path exists.

**Models loaded from:**
- Upper camera: `Config/NeuralNets/FieldBoundary/net.h5`
- Lower camera: `Config/NeuralNets/FieldBoundary/net-uncertainty.h5`

The lower camera model outputs 2 values per column (boundary y-position + uncertainty); the upper camera model outputs 1 (y-position only).

---

## Pipeline

### 1. `predictSpots()`
Scales the camera image to the network's input patch size, runs inference, and reads one boundary y-coordinate per horizontal column. Output values are normalized [0, 1] fractions of image height. Valid spots (beyond `minDistance` from the robot) are projected to field coordinates via the camera matrix.

### 2. `boundaryIsOdd()`
Validates predicted spots by checking:
- Enough points are off the top of the image (above `top` pixel threshold)
- Average uncertainty is below `uncertaintyLimit` (lower camera only)
- Points aren't too far below the image border
- Smoothness: deviation of each middle point from the line between its neighbors stays below `threshold`

### 3. Line Fitting
Controlled by `fittingMethod` parameter (default: `NoFitting`):

| Method | Behaviour |
|---|---|
| `NoFitting` | Passes raw spots through directly |
| `Ransac` | Fits a 1- or 2-segment piecewise model; the 2-segment case models a field corner using perpendicular lines in field space |
| `NotRansac` | Tries all split points, fits two lines, picks the split with angle deviation closest to 90° |

### 4. `projectPrevious()` (fallback)
When the current prediction is "odd" or the upper camera fails, reprojects the previous frame's boundary using odometry offset. The result is flagged with `extrapolated = true`.

---

## Upper vs. Lower Camera Logic

- **Upper camera**: always runs inference. Falls back to `projectPrevious` if prediction is odd, unless the previous boundary was itself extrapolated.
- **Lower camera**: only runs inference if the upper camera boundary was odd, extrapolated, or empty. Otherwise reprojects the upper camera result downward.

The upper camera drives detection; the lower camera is a fallback/refinement.

---

## Output Representation (`FieldBoundary`)

| Field | Type | Description |
|---|---|---|
| `boundaryInImage` | `vector<Vector2i>` | Boundary points in image coordinates |
| `boundaryOnField` | `vector<Vector2f>` | Boundary points in robot-relative field coordinates |
| `isValid` | `bool` | True if at least 2 boundary points were found |
| `extrapolated` | `bool` | True if boundary was projected from a previous frame |
| `odd` | `bool` | True if the prediction was flagged as unreliable |

`getBoundaryY(int x)` returns the boundary y-coordinate at a given image x.
