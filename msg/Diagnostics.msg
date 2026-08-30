std_msgs/Header header
uint8 source
uint8 SOURCE_LIO=0
uint8 SOURCE_VIO=1

float64 lio_downsample
float64 lio_icp
float64 lio_update_voxel
float64 lio_current_total
float64 lio_average_total
int32 lio_raw_feat
int32 lio_down_feat
int32 lio_effective_feat
float64 lio_avg_residual

float64 vio_retrieve
float64 vio_jacobian_ekf
float64 vio_compute_jacobian
float64 vio_update_ekf
float64 vio_generate
float64 vio_update_points
float64 vio_update_patch
float64 vio_current_total
float64 vio_average_total
int32 vio_sparse_map
int32 vio_raw_feat
int32 vio_retrieve_n
int32 vio_append_n
int32 vio_update_n

int32 preprocess_in
int32 preprocess_out

float64 pos_std_x
float64 pos_std_y
float64 pos_std_z
float64 pos_std
float64 rot_std
float64 vel_std
