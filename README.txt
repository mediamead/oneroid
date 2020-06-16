** Installation

conda create -n oneroid3 -c conda-forge opencv=4 tensorflow-gpu notebook pandas matplotlib
conda activate oneroid3
pip install pybullet

*** Usage

set PYTHONPATH=%CD%

opencv/
    view.py             - grabs frames from the camera, displays, saves (for calibration)
    calibrate.py        - processes saved frames, creates camera calibration file
    pose.py             - does real-time pose estimation (requires camera calibration file)
    dump_filecam_cal.py - dumps content of the calibration file (computed for Lifecam)
    cal-(ms|lg)-(1080)/ - one directory per camera + resolution
