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

    with_distortion = false
    aspect = 1

    flags = OpenCV.CALIB_ZERO_TANGENT_DIST + OpenCV.CALIB_FIX_K3 + OpenCV.CALIB_FIX_K2 + (with_distortion ? 0 : OpenCV.CALIB_FIX_K1) + OpenCV.CALIB_FIX_ASPECT_RATIO
    cammat = collect(I(3))
    cammat[1,:] .= aspect

    _, py_mtx, py_dist, py_rvecs, py_tvecs = 

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
    fi = skipmissing(_detect_corners.(_files, Ref(n_corners)))
    @assert !isempty(fi) "No checkers were detected in any of the images, perhaps try a different `n_corners`."
    files = first.(fi)
    imgpointss = last.(fi)

    objpoints = get_object_points(n_corners)
    sz = reverse(size(load(files[1])))

    k, Rs, ts, frow, fcol, crow, ccol = fit_model(sz, fill(objpoints, length(files)), imgpointss, n_corners, with_distortion, aspect)

    return (; files, objpoints, imgpointss, sz, k, Rs, ts, frow, fcol, crow, ccol)
end

    _files = readdir("example", join=true)
    n_corners = (5, 8)



# n_corners = (4,6)
#     img = CF.cv2.imread(file)
#     gry = CF.cv2.cvtColor(img, CF.cv2.COLOR_BGR2GRAY)
#
#     ret, py_corners = CF.cv2.findChessboardCornersSB(gry, n_corners, CF.cv2.CALIB_CB_ADAPTIVE_THRESH + CF.cv2.CALIB_CB_NORMALIZE_IMAGE + CF.cv2.CALIB_CB_FAST_CHECK)
