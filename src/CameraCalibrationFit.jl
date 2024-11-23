module CameraCalibrationFit

using CameraCalibrationMeta
using Statistics, LinearAlgebra
using PythonCall, FileIO, StaticArrays, CoordinateTransformations, Rotations, Polynomials
using ImageTransformations, Colors, ImageDraw

# using CondaPkg
# CondaPkg.add.(["numpy", "opencv"])

const cv2 = PythonCall.pynew()
const np = PythonCall.pynew()

function __init__()
    PythonCall.pycopy!(cv2, pyimport("cv2"))
    PythonCall.pycopy!(np, pyimport("numpy"))
end

include("detect_fit.jl")
include("buildcalibrations.jl")
include("plot_calibration.jl")


end # module CameraCalibrationFit
