# the following convertion functions are necessary due to some overly eager type checking in the python functions.
convert_from_py_corners(py_corners, n_corners) = reshape(RowCol.(eachslice(PyArray(py_corners); dims = 1)), n_corners)

convert_to_py_objpoints(objpoints, n) = PyList(fill(np.array(Float32.(reshape(reduce((x1, x2) -> hcat(x1, Vector(x2)), objpoints)', (1, length(objpoints), 3)))), n))

convert_to_py_imgpointss(imgpointss) = PyList([np.array(reshape(reduce((x1, x2) -> hcat(x1, Vector(x2)), imgpoints)', 1, length(imgpoints), 2)) for imgpoints in imgpointss])

"""
    get_object_points
Produce the real-world locations of the corners of the checkerboard.
"""
function get_object_points(n_corners)
    objpoints = Matrix{CameraCalibrationMeta.XYZ}(undef, n_corners)
    for i in CartesianIndices(n_corners)
        x, y = Tuple(i) .- 1
        objpoints[i] = CameraCalibrationMeta.XYZ(x, y, 0)
    end
    return reverse(objpoints, dims = 2)
end

"""
    _detect_corners
Wraps OpenCV function to auto-detect corners in an image.
"""
function _detect_corners(file, n_corners)
    img = load(file)
    sz = size(img)
    gry = reshape(collect(rawview(channelview(Gray.(img)))), 1, sz...)
    cv_n_corners = OpenCV.Size{Int32}(n_corners...)
    _cv_corners = OpenCV.Mat(Array{Float32}(undef, 2, 1, 40))
    ret, cv_corners = OpenCV.findChessboardCornersSB(gry, cv_n_corners, _cv_corners, OpenCV.CALIB_CB_NORMALIZE_IMAGE + # Normalize the image gamma with equalizeHist before detection.
                                                  OpenCV.CALIB_CB_EXHAUSTIVE + # Run an exhaustive search to improve detection rate.
                                                  OpenCV.CALIB_CB_ACCURACY # Up sample input image to improve sub-pixel accuracy due to aliasing effects.
                                                 )
    !Bool(ret) && return missing
    criteria = OpenCV.TermCriteria(OpenCV.TERM_CRITERIA_EPS + OpenCV.TERM_CRITERIA_MAX_ITER, 30, 0.001)
    ref_corners = OpenCV.cornerSubPix(gry, cv_corners, OpenCV.Size{Int32}(11,11), OpenCV.Size{Int32}(-1,-1), criteria)
    # flp_corners = np.flip(ref_corners, axis = 2)
    corners = RowCol.(eachslice(ref_corners, dims = 3))
    return (file, corners)
end

"""
    fit_model
Wraps OpenCV function to fit a camera model to given object and image points.
"""
function fit_model(sz, objpoints, imgpointss, n_corners,  with_distortion, aspect)

    fun(x::Vector{Union{Int, String}}) = x
    x = Vector{Int}(undef, 3)
    x = Union{Int, String}[i for i in 1:3]
    fun(x)

    with_distortion = false
    objectPoints = OpenCV.InputArray[reshape(Matrix{Float32}(reduce(hcat, objpoints)), 3, 1, :) for _ in 1:length(imgpointss)]
    imagePoints = OpenCV.InputArray[reshape(Matrix{Float32}(reduce(hcat, imgpoints)), 2, 1, :) for imgpoints in imgpointss]
    imageSize = OpenCV.Size{Int32}(sz...)
    cameraMatrix = OpenCV.Mat(rand(Float32, 3, 3, 1))
    distCoeffs = OpenCV.Mat(rand(Float32, 3, 1, 1))

    flags = OpenCV.CALIB_ZERO_TANGENT_DIST + OpenCV.CALIB_FIX_K3 + OpenCV.CALIB_FIX_K2 + (with_distortion ? 0 : OpenCV.CALIB_FIX_K1) + OpenCV.CALIB_FIX_ASPECT_RATIO

    r = OpenCV.InputArray[reshape(Matrix{Float32}(reduce(hcat, objpoints)), 3, 1, :) for _ in 1:length(imgpointss)]
    t = OpenCV.InputArray[reshape(Matrix{Float32}(reduce(hcat, objpoints)), 3, 1, :) for _ in 1:length(imgpointss)]

    OpenCV.calibrateCamera(objectPoints, imagePoints, imageSize, cameraMatrix, distCoeffs, r, t, flags, OpenCV.TermCriteria(1, 10, 1.0)) # this actually "works"

    # next, I need to see what I should put in all the containers for this to work optimally (i.e. compare to a naive python example).






calibrateCamera(objPoints, imgPoints, imgSize, cameraMatrix,
                                distCoeffs, r, t, calibFlags);



(::Vector{Union{OpenCV.CxxMat, AbstractArray{T, 3} where T<:Union{Float32, Float64, Int16, Int32, Int8, UInt16, UInt8}}},
 ::Vector{Union{OpenCV.CxxMat, AbstractArray{T, 3} where T<:Union{Float32, Float64, Int16, Int32, Int8, UInt16, UInt8}}},
 ::Int64, ::OpenCV.TermCriteria, ::typeof(OpenCV.calibrateCamera), ::Vector{Union{OpenCV.CxxMat, AbstractArray{T, 3} where T<:Union{Float32, Float64, Int16, Int32, Int8, UInt16, UInt8}}}, ::Vector{Union{OpenCV.CxxMat, AbstractArray{T, 3} where T<:Union{Float32, Float64, Int16, Int32, Int8, UInt16, UInt8}}}, ::OpenCV.Size{Int32}, ::Union{OpenCV.CxxMat, AbstractArray{T, 3} where T<:Union{Float32, Float64, Int16, Int32, Int8, UInt16, UInt8}}, ::Union{OpenCV.CxxMat, AbstractArray{T, 3} where T<:Union{Float32, Float64, Int16, Int32, Int8, UInt16, UInt8}})



calibrateCamera(objectPoints::Vector{Union{OpenCV.CxxMat, AbstractArray{T, 3} where T<:Union{Float32, Float64, Int16, Int32, Int8, UInt16, UInt8}}}, 
                imagePoints::Vector{Union{OpenCV.CxxMat, AbstractArray{T, 3} where T<:Union{Float32, Float64, Int16, Int32, Int8, UInt16, UInt8}}}, 
                imageSize::OpenCV.Size{Int32},
                cameraMatrix::Union{OpenCV.CxxMat, AbstractArray{T, 3} where T<:Union{Float32, Float64, Int16, Int32, Int8, UInt16, UInt8}},
                distCoeffs::Union{OpenCV.CxxMat, AbstractArray{T, 3} where T<:Union{Float32, Float64, Int16, Int32, Int8, UInt16, UInt8}};
                rvecs, tvecs, flags, criteria) @ OpenCV ~/.julia/artifacts/dcc70861ed0ffc5427898f6aeb436620f33fa02f/OpenCV/src/cv_cxx_wrap.jl:2278

calibrateCamera(objectPoints::Vector{Union{OpenCV.CxxMat, AbstractArray{T, 3} where T<:Union{Float32, Float64, Int16, Int32, Int8, UInt16, UInt8}}},
                imagePoints::Vector{Union{OpenCV.CxxMat, AbstractArray{T, 3} where T<:Union{Float32, Float64, Int16, Int32, Int8, UInt16, UInt8}}},
                imageSize::OpenCV.Size{Int32},
                cameraMatrix::Union{OpenCV.CxxMat, AbstractArray{T, 3} where T<:Union{Float32, Float64, Int16, Int32, Int8, UInt16, UInt8}}, distCoeffs::Union{OpenCV.CxxMat, AbstractArray{T, 3} where T<:Union{Float32, Float64, Int16, Int32, Int8, UInt16, UInt8}}, rvecs::Vector{Union{OpenCV.CxxMat, AbstractArray{T, 3} where T<:Union{Float32, Float64, Int16, Int32, Int8, UInt16, UInt8}}}, tvecs::Vector{Union{OpenCV.CxxMat, AbstractArray{T, 3} where T<:Union{Float32, Float64, Int16, Int32, Int8, UInt16, UInt8}}}, flags::Int64, criteria::OpenCV.TermCriteria) @ OpenCV ~/.julia/artifacts/dcc70861ed0ffc5427898f6aeb436620f33fa02f/OpenCV/src/cv_cxx_wrap.jl:2275









    OpenCV.calibrateCamera(objectPoints,
                           imagePoints,
                           imageSize,
                           cameraMatrix,
                           distCoeffs)




    OpenCV.calibrateCamera(objectPoints::Vector{Union{OpenCV.CxxMat, AbstractArray{T, 3} where T<:Union{Float32, Float64, Int16, Int32, Int8, UInt16, UInt8}}},
                           imagePoints::Vector{Union{OpenCV.CxxMat, AbstractArray{T, 3} where T<:Union{Float32, Float64, Int16, Int32, Int8, UInt16, UInt8}}},
                           imageSize::OpenCV.Size{Int32},
                           cameraMatrix::Union{OpenCV.CxxMat, AbstractArray{T, 3} where T<:Union{Float32, Float64, Int16, Int32, Int8, UInt16, UInt8}},
                           distCoeffs::Union{OpenCV.CxxMat, AbstractArray{T, 3} where T<:Union{Float32, Float64, Int16, Int32, Int8, UInt16, UInt8}})




    with_distortion = false
    aspect = 1

    flags = OpenCV.CALIB_ZERO_TANGENT_DIST + OpenCV.CALIB_FIX_K3 + OpenCV.CALIB_FIX_K2 + (with_distortion ? 0 : OpenCV.CALIB_FIX_K1) + OpenCV.CALIB_FIX_ASPECT_RATIO
    cammat = collect(I(3))
    cammat[1,:] .= aspect

    nfiles = length(files)

    objectPoints = fill(vec(Vector{Float32}.(objpoints)), length(files))

    objectPoints = fill(vec(Vector{Int32}.(objpoints)), length(files))

    OpenCV.calibrateCamera(objectPoints, [reshape(reduce(hcat, imgpoints), 2, 1, 40) for imgpoints in imgpointss])#, sz, cammat, nothing; flags)

    OpenCV.calibrateCamera(OpenCV.Mat(Int32.(reshape(reduce(hcat, objpoints), 3, 1, 40))), imgpointss, sz, cammat, nothing; flags)

    k, _ = PyArray(py_dist)
    @assert with_distortion || k == 0 "distortion was $with_distortion but k isn't zero:" k

    Rs = [vec(pyconvert(Matrix{Float64}, x)) for x in py_rvecs]

    ts = [vec(pyconvert(Matrix{Float64}, x)) for x in py_tvecs]

    mtx = Matrix(PyArray(py_mtx))
    frow = mtx[1,1]
    fcol = mtx[2,2]
    # @show frow, fcol
    crow = mtx[1,3]
    ccol = mtx[2,3]

    return (; k, Rs, ts, frow, fcol, crow, ccol)
end

function detect_fit(_files, n_corners, with_distortion, aspect)

    _files = readdir("example", join=true)
    n_corners = (5, 8)
    fi = skipmissing(_detect_corners.(_files, Ref(n_corners)))
    @assert !isempty(fi) "No checkers were detected in any of the images, perhaps try a different `n_corners`."
    files = first.(fi)
    imgpointss = last.(fi)
    objpoints = get_object_points(n_corners)
    sz = reverse(size(load(files[1])))

    k, Rs, ts, frow, fcol, crow, ccol = fit_model(sz, fill(objpoints, length(files)), imgpointss, n_corners, with_distortion, aspect)

    return (; files, objpoints, imgpointss, sz, k, Rs, ts, frow, fcol, crow, ccol)
end




# n_corners = (4,6)
#     img = CF.cv2.imread(file)
#     gry = CF.cv2.cvtColor(img, CF.cv2.COLOR_BGR2GRAY)
#
#     ret, py_corners = CF.cv2.findChessboardCornersSB(gry, n_corners, CF.cv2.CALIB_CB_ADAPTIVE_THRESH + CF.cv2.CALIB_CB_NORMALIZE_IMAGE + CF.cv2.CALIB_CB_FAST_CHECK)
