## Install
1. Download the repo
    ```bash
    git clone https://github.com/leochien1110/pcl_test.git
    ```
1. Build
    ```bash
    cd pcl_test/
    mkdir build && cd build
    cmake ..
    make
    ```

## Run
1. Navigate to `build/src/`
    ```bash
    cd build/src/
    ```
* pcd_align

* pcd_filter
    ```bash
    ./pcd_filter --pc_path ../../res/room_scan1.pcd
    ```
    Field of View CropHull Filter , Horizontal: 40deg, Vertical: 20deg, Distance: 200m