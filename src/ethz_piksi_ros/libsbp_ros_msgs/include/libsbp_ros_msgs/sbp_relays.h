// This message is automatically generated using generator.py
// PLEASE DO NOT MODIFY MANUALLY.

#ifndef LIBSBP_ROS_MSGS_SBP_RELAYS_H_
#define LIBSBP_ROS_MSGS_SBP_RELAYS_H_

#include <piksi_multi_cpp/sbp_callback_handler/sbp_callback_handler_relay/sbp_relay.h>
#include <libsbp_ros_msgs/conversion.h>

namespace piksi_multi_cpp {

// Declare all relays.
class SbpRelayMsgAcqResult
    : public SbpRelay<msg_acq_result_t,
                                     libsbp_ros_msgs::MsgAcqResult> {
  public:
    inline SbpRelayMsgAcqResult(
        const ros::NodeHandle& nh, const std::shared_ptr<sbp_state_t>& state)
        : SbpRelay(nh, SBP_MSG_ACQ_RESULT, state, "acq_result") {}
};

class SbpRelayMsgAcqSvProfile
    : public SbpRelay<msg_acq_sv_profile_t,
                                     libsbp_ros_msgs::MsgAcqSvProfile> {
  public:
    inline SbpRelayMsgAcqSvProfile(
        const ros::NodeHandle& nh, const std::shared_ptr<sbp_state_t>& state)
        : SbpRelay(nh, SBP_MSG_ACQ_SV_PROFILE, state, "acq_sv_profile") {}
};

class SbpRelayMsgBootloaderHandshakeResp
    : public SbpRelay<msg_bootloader_handshake_resp_t,
                                     libsbp_ros_msgs::MsgBootloaderHandshakeResp> {
  public:
    inline SbpRelayMsgBootloaderHandshakeResp(
        const ros::NodeHandle& nh, const std::shared_ptr<sbp_state_t>& state)
        : SbpRelay(nh, SBP_MSG_BOOTLOADER_HANDSHAKE_RESP, state, "bootloader_handshake_resp") {}
};

class SbpRelayMsgBootloaderJumpToApp
    : public SbpRelay<msg_bootloader_jump_to_app_t,
                                     libsbp_ros_msgs::MsgBootloaderJumpToApp> {
  public:
    inline SbpRelayMsgBootloaderJumpToApp(
        const ros::NodeHandle& nh, const std::shared_ptr<sbp_state_t>& state)
        : SbpRelay(nh, SBP_MSG_BOOTLOADER_JUMP_TO_APP, state, "bootloader_jump_to_app") {}
};

class SbpRelayMsgNapDeviceDnaResp
    : public SbpRelay<msg_nap_device_dna_resp_t,
                                     libsbp_ros_msgs::MsgNapDeviceDnaResp> {
  public:
    inline SbpRelayMsgNapDeviceDnaResp(
        const ros::NodeHandle& nh, const std::shared_ptr<sbp_state_t>& state)
        : SbpRelay(nh, SBP_MSG_NAP_DEVICE_DNA_RESP, state, "nap_device_dna_resp") {}
};

class SbpRelayMsgExtEvent
    : public SbpRelay<msg_ext_event_t,
                                     libsbp_ros_msgs::MsgExtEvent> {
  public:
    inline SbpRelayMsgExtEvent(
        const ros::NodeHandle& nh, const std::shared_ptr<sbp_state_t>& state)
        : SbpRelay(nh, SBP_MSG_EXT_EVENT, state, "ext_event") {}
};

class SbpRelayMsgFileioReadReq
    : public SbpRelay<msg_fileio_read_req_t,
                                     libsbp_ros_msgs::MsgFileioReadReq> {
  public:
    inline SbpRelayMsgFileioReadReq(
        const ros::NodeHandle& nh, const std::shared_ptr<sbp_state_t>& state)
        : SbpRelay(nh, SBP_MSG_FILEIO_READ_REQ, state, "fileio_read_req") {}
};

class SbpRelayMsgFileioReadResp
    : public SbpRelay<msg_fileio_read_resp_t,
                                     libsbp_ros_msgs::MsgFileioReadResp> {
  public:
    inline SbpRelayMsgFileioReadResp(
        const ros::NodeHandle& nh, const std::shared_ptr<sbp_state_t>& state)
        : SbpRelay(nh, SBP_MSG_FILEIO_READ_RESP, state, "fileio_read_resp") {}
};

class SbpRelayMsgFileioReadDirReq
    : public SbpRelay<msg_fileio_read_dir_req_t,
                                     libsbp_ros_msgs::MsgFileioReadDirReq> {
  public:
    inline SbpRelayMsgFileioReadDirReq(
        const ros::NodeHandle& nh, const std::shared_ptr<sbp_state_t>& state)
        : SbpRelay(nh, SBP_MSG_FILEIO_READ_DIR_REQ, state, "fileio_read_dir_req") {}
};

class SbpRelayMsgFileioReadDirResp
    : public SbpRelay<msg_fileio_read_dir_resp_t,
                                     libsbp_ros_msgs::MsgFileioReadDirResp> {
  public:
    inline SbpRelayMsgFileioReadDirResp(
        const ros::NodeHandle& nh, const std::shared_ptr<sbp_state_t>& state)
        : SbpRelay(nh, SBP_MSG_FILEIO_READ_DIR_RESP, state, "fileio_read_dir_resp") {}
};

class SbpRelayMsgFileioRemove
    : public SbpRelay<msg_fileio_remove_t,
                                     libsbp_ros_msgs::MsgFileioRemove> {
  public:
    inline SbpRelayMsgFileioRemove(
        const ros::NodeHandle& nh, const std::shared_ptr<sbp_state_t>& state)
        : SbpRelay(nh, SBP_MSG_FILEIO_REMOVE, state, "fileio_remove") {}
};

class SbpRelayMsgFileioWriteReq
    : public SbpRelay<msg_fileio_write_req_t,
                                     libsbp_ros_msgs::MsgFileioWriteReq> {
  public:
    inline SbpRelayMsgFileioWriteReq(
        const ros::NodeHandle& nh, const std::shared_ptr<sbp_state_t>& state)
        : SbpRelay(nh, SBP_MSG_FILEIO_WRITE_REQ, state, "fileio_write_req") {}
};

class SbpRelayMsgFileioWriteResp
    : public SbpRelay<msg_fileio_write_resp_t,
                                     libsbp_ros_msgs::MsgFileioWriteResp> {
  public:
    inline SbpRelayMsgFileioWriteResp(
        const ros::NodeHandle& nh, const std::shared_ptr<sbp_state_t>& state)
        : SbpRelay(nh, SBP_MSG_FILEIO_WRITE_RESP, state, "fileio_write_resp") {}
};

class SbpRelayMsgFileioConfigReq
    : public SbpRelay<msg_fileio_config_req_t,
                                     libsbp_ros_msgs::MsgFileioConfigReq> {
  public:
    inline SbpRelayMsgFileioConfigReq(
        const ros::NodeHandle& nh, const std::shared_ptr<sbp_state_t>& state)
        : SbpRelay(nh, SBP_MSG_FILEIO_CONFIG_REQ, state, "fileio_config_req") {}
};

class SbpRelayMsgFileioConfigResp
    : public SbpRelay<msg_fileio_config_resp_t,
                                     libsbp_ros_msgs::MsgFileioConfigResp> {
  public:
    inline SbpRelayMsgFileioConfigResp(
        const ros::NodeHandle& nh, const std::shared_ptr<sbp_state_t>& state)
        : SbpRelay(nh, SBP_MSG_FILEIO_CONFIG_RESP, state, "fileio_config_resp") {}
};

class SbpRelayMsgFlashProgram
    : public SbpRelay<msg_flash_program_t,
                                     libsbp_ros_msgs::MsgFlashProgram> {
  public:
    inline SbpRelayMsgFlashProgram(
        const ros::NodeHandle& nh, const std::shared_ptr<sbp_state_t>& state)
        : SbpRelay(nh, SBP_MSG_FLASH_PROGRAM, state, "flash_program") {}
};

class SbpRelayMsgFlashDone
    : public SbpRelay<msg_flash_done_t,
                                     libsbp_ros_msgs::MsgFlashDone> {
  public:
    inline SbpRelayMsgFlashDone(
        const ros::NodeHandle& nh, const std::shared_ptr<sbp_state_t>& state)
        : SbpRelay(nh, SBP_MSG_FLASH_DONE, state, "flash_done") {}
};

class SbpRelayMsgFlashReadReq
    : public SbpRelay<msg_flash_read_req_t,
                                     libsbp_ros_msgs::MsgFlashReadReq> {
  public:
    inline SbpRelayMsgFlashReadReq(
        const ros::NodeHandle& nh, const std::shared_ptr<sbp_state_t>& state)
        : SbpRelay(nh, SBP_MSG_FLASH_READ_REQ, state, "flash_read_req") {}
};

class SbpRelayMsgFlashReadResp
    : public SbpRelay<msg_flash_read_resp_t,
                                     libsbp_ros_msgs::MsgFlashReadResp> {
  public:
    inline SbpRelayMsgFlashReadResp(
        const ros::NodeHandle& nh, const std::shared_ptr<sbp_state_t>& state)
        : SbpRelay(nh, SBP_MSG_FLASH_READ_RESP, state, "flash_read_resp") {}
};

class SbpRelayMsgFlashErase
    : public SbpRelay<msg_flash_erase_t,
                                     libsbp_ros_msgs::MsgFlashErase> {
  public:
    inline SbpRelayMsgFlashErase(
        const ros::NodeHandle& nh, const std::shared_ptr<sbp_state_t>& state)
        : SbpRelay(nh, SBP_MSG_FLASH_ERASE, state, "flash_erase") {}
};

class SbpRelayMsgStmFlashLockSector
    : public SbpRelay<msg_stm_flash_lock_sector_t,
                                     libsbp_ros_msgs::MsgStmFlashLockSector> {
  public:
    inline SbpRelayMsgStmFlashLockSector(
        const ros::NodeHandle& nh, const std::shared_ptr<sbp_state_t>& state)
        : SbpRelay(nh, SBP_MSG_STM_FLASH_LOCK_SECTOR, state, "stm_flash_lock_sector") {}
};

class SbpRelayMsgStmFlashUnlockSector
    : public SbpRelay<msg_stm_flash_unlock_sector_t,
                                     libsbp_ros_msgs::MsgStmFlashUnlockSector> {
  public:
    inline SbpRelayMsgStmFlashUnlockSector(
        const ros::NodeHandle& nh, const std::shared_ptr<sbp_state_t>& state)
        : SbpRelay(nh, SBP_MSG_STM_FLASH_UNLOCK_SECTOR, state, "stm_flash_unlock_sector") {}
};

class SbpRelayMsgStmUniqueIdResp
    : public SbpRelay<msg_stm_unique_id_resp_t,
                                     libsbp_ros_msgs::MsgStmUniqueIdResp> {
  public:
    inline SbpRelayMsgStmUniqueIdResp(
        const ros::NodeHandle& nh, const std::shared_ptr<sbp_state_t>& state)
        : SbpRelay(nh, SBP_MSG_STM_UNIQUE_ID_RESP, state, "stm_unique_id_resp") {}
};

class SbpRelayMsgM25FlashWriteStatus
    : public SbpRelay<msg_m25_flash_write_status_t,
                                     libsbp_ros_msgs::MsgM25FlashWriteStatus> {
  public:
    inline SbpRelayMsgM25FlashWriteStatus(
        const ros::NodeHandle& nh, const std::shared_ptr<sbp_state_t>& state)
        : SbpRelay(nh, SBP_MSG_M25_FLASH_WRITE_STATUS, state, "m25_flash_write_status") {}
};

class SbpRelayMsgImuRaw
    : public SbpRelay<msg_imu_raw_t,
                                     libsbp_ros_msgs::MsgImuRaw> {
  public:
    inline SbpRelayMsgImuRaw(
        const ros::NodeHandle& nh, const std::shared_ptr<sbp_state_t>& state)
        : SbpRelay(nh, SBP_MSG_IMU_RAW, state, "imu_raw") {}
};

class SbpRelayMsgImuAux
    : public SbpRelay<msg_imu_aux_t,
                                     libsbp_ros_msgs::MsgImuAux> {
  public:
    inline SbpRelayMsgImuAux(
        const ros::NodeHandle& nh, const std::shared_ptr<sbp_state_t>& state)
        : SbpRelay(nh, SBP_MSG_IMU_AUX, state, "imu_aux") {}
};

class SbpRelayMsgLinuxCpuState
    : public SbpRelay<msg_linux_cpu_state_t,
                                     libsbp_ros_msgs::MsgLinuxCpuState> {
  public:
    inline SbpRelayMsgLinuxCpuState(
        const ros::NodeHandle& nh, const std::shared_ptr<sbp_state_t>& state)
        : SbpRelay(nh, SBP_MSG_LINUX_CPU_STATE, state, "linux_cpu_state") {}
};

class SbpRelayMsgLinuxMemState
    : public SbpRelay<msg_linux_mem_state_t,
                                     libsbp_ros_msgs::MsgLinuxMemState> {
  public:
    inline SbpRelayMsgLinuxMemState(
        const ros::NodeHandle& nh, const std::shared_ptr<sbp_state_t>& state)
        : SbpRelay(nh, SBP_MSG_LINUX_MEM_STATE, state, "linux_mem_state") {}
};

class SbpRelayMsgLinuxSysState
    : public SbpRelay<msg_linux_sys_state_t,
                                     libsbp_ros_msgs::MsgLinuxSysState> {
  public:
    inline SbpRelayMsgLinuxSysState(
        const ros::NodeHandle& nh, const std::shared_ptr<sbp_state_t>& state)
        : SbpRelay(nh, SBP_MSG_LINUX_SYS_STATE, state, "linux_sys_state") {}
};

class SbpRelayMsgLinuxProcessSocketCounts
    : public SbpRelay<msg_linux_process_socket_counts_t,
                                     libsbp_ros_msgs::MsgLinuxProcessSocketCounts> {
  public:
    inline SbpRelayMsgLinuxProcessSocketCounts(
        const ros::NodeHandle& nh, const std::shared_ptr<sbp_state_t>& state)
        : SbpRelay(nh, SBP_MSG_LINUX_PROCESS_SOCKET_COUNTS, state, "linux_process_socket_counts") {}
};

class SbpRelayMsgLinuxProcessSocketQueues
    : public SbpRelay<msg_linux_process_socket_queues_t,
                                     libsbp_ros_msgs::MsgLinuxProcessSocketQueues> {
  public:
    inline SbpRelayMsgLinuxProcessSocketQueues(
        const ros::NodeHandle& nh, const std::shared_ptr<sbp_state_t>& state)
        : SbpRelay(nh, SBP_MSG_LINUX_PROCESS_SOCKET_QUEUES, state, "linux_process_socket_queues") {}
};

class SbpRelayMsgLinuxSocketUsage
    : public SbpRelay<msg_linux_socket_usage_t,
                                     libsbp_ros_msgs::MsgLinuxSocketUsage> {
  public:
    inline SbpRelayMsgLinuxSocketUsage(
        const ros::NodeHandle& nh, const std::shared_ptr<sbp_state_t>& state)
        : SbpRelay(nh, SBP_MSG_LINUX_SOCKET_USAGE, state, "linux_socket_usage") {}
};

class SbpRelayMsgLinuxProcessFdCount
    : public SbpRelay<msg_linux_process_fd_count_t,
                                     libsbp_ros_msgs::MsgLinuxProcessFdCount> {
  public:
    inline SbpRelayMsgLinuxProcessFdCount(
        const ros::NodeHandle& nh, const std::shared_ptr<sbp_state_t>& state)
        : SbpRelay(nh, SBP_MSG_LINUX_PROCESS_FD_COUNT, state, "linux_process_fd_count") {}
};

class SbpRelayMsgLinuxProcessFdSummary
    : public SbpRelay<msg_linux_process_fd_summary_t,
                                     libsbp_ros_msgs::MsgLinuxProcessFdSummary> {
  public:
    inline SbpRelayMsgLinuxProcessFdSummary(
        const ros::NodeHandle& nh, const std::shared_ptr<sbp_state_t>& state)
        : SbpRelay(nh, SBP_MSG_LINUX_PROCESS_FD_SUMMARY, state, "linux_process_fd_summary") {}
};

class SbpRelayMsgLog
    : public SbpRelay<msg_log_t,
                                     libsbp_ros_msgs::MsgLog> {
  public:
    inline SbpRelayMsgLog(
        const ros::NodeHandle& nh, const std::shared_ptr<sbp_state_t>& state)
        : SbpRelay(nh, SBP_MSG_LOG, state, "log") {}
};

class SbpRelayMsgFwd
    : public SbpRelay<msg_fwd_t,
                                     libsbp_ros_msgs::MsgFwd> {
  public:
    inline SbpRelayMsgFwd(
        const ros::NodeHandle& nh, const std::shared_ptr<sbp_state_t>& state)
        : SbpRelay(nh, SBP_MSG_FWD, state, "fwd") {}
};

class SbpRelayMsgMagRaw
    : public SbpRelay<msg_mag_raw_t,
                                     libsbp_ros_msgs::MsgMagRaw> {
  public:
    inline SbpRelayMsgMagRaw(
        const ros::NodeHandle& nh, const std::shared_ptr<sbp_state_t>& state)
        : SbpRelay(nh, SBP_MSG_MAG_RAW, state, "mag_raw") {}
};

class SbpRelayMsgGpsTime
    : public SbpRelay<msg_gps_time_t,
                                     libsbp_ros_msgs::MsgGpsTime> {
  public:
    inline SbpRelayMsgGpsTime(
        const ros::NodeHandle& nh, const std::shared_ptr<sbp_state_t>& state)
        : SbpRelay(nh, SBP_MSG_GPS_TIME, state, "gps_time") {}
};

class SbpRelayMsgUtcTime
    : public SbpRelay<msg_utc_time_t,
                                     libsbp_ros_msgs::MsgUtcTime> {
  public:
    inline SbpRelayMsgUtcTime(
        const ros::NodeHandle& nh, const std::shared_ptr<sbp_state_t>& state)
        : SbpRelay(nh, SBP_MSG_UTC_TIME, state, "utc_time") {}
};

class SbpRelayMsgDops
    : public SbpRelay<msg_dops_t,
                                     libsbp_ros_msgs::MsgDops> {
  public:
    inline SbpRelayMsgDops(
        const ros::NodeHandle& nh, const std::shared_ptr<sbp_state_t>& state)
        : SbpRelay(nh, SBP_MSG_DOPS, state, "dops") {}
};

class SbpRelayMsgPosEcef
    : public SbpRelay<msg_pos_ecef_t,
                                     libsbp_ros_msgs::MsgPosEcef> {
  public:
    inline SbpRelayMsgPosEcef(
        const ros::NodeHandle& nh, const std::shared_ptr<sbp_state_t>& state)
        : SbpRelay(nh, SBP_MSG_POS_ECEF, state, "pos_ecef") {}
};

class SbpRelayMsgPosEcefCov
    : public SbpRelay<msg_pos_ecef_cov_t,
                                     libsbp_ros_msgs::MsgPosEcefCov> {
  public:
    inline SbpRelayMsgPosEcefCov(
        const ros::NodeHandle& nh, const std::shared_ptr<sbp_state_t>& state)
        : SbpRelay(nh, SBP_MSG_POS_ECEF_COV, state, "pos_ecef_cov") {}
};

class SbpRelayMsgPosLlh
    : public SbpRelay<msg_pos_llh_t,
                                     libsbp_ros_msgs::MsgPosLlh> {
  public:
    inline SbpRelayMsgPosLlh(
        const ros::NodeHandle& nh, const std::shared_ptr<sbp_state_t>& state)
        : SbpRelay(nh, SBP_MSG_POS_LLH, state, "pos_llh") {}
};

class SbpRelayMsgPosLlhCov
    : public SbpRelay<msg_pos_llh_cov_t,
                                     libsbp_ros_msgs::MsgPosLlhCov> {
  public:
    inline SbpRelayMsgPosLlhCov(
        const ros::NodeHandle& nh, const std::shared_ptr<sbp_state_t>& state)
        : SbpRelay(nh, SBP_MSG_POS_LLH_COV, state, "pos_llh_cov") {}
};

class SbpRelayMsgBaselineEcef
    : public SbpRelay<msg_baseline_ecef_t,
                                     libsbp_ros_msgs::MsgBaselineEcef> {
  public:
    inline SbpRelayMsgBaselineEcef(
        const ros::NodeHandle& nh, const std::shared_ptr<sbp_state_t>& state)
        : SbpRelay(nh, SBP_MSG_BASELINE_ECEF, state, "baseline_ecef") {}
};

class SbpRelayMsgBaselineNed
    : public SbpRelay<msg_baseline_ned_t,
                                     libsbp_ros_msgs::MsgBaselineNed> {
  public:
    inline SbpRelayMsgBaselineNed(
        const ros::NodeHandle& nh, const std::shared_ptr<sbp_state_t>& state)
        : SbpRelay(nh, SBP_MSG_BASELINE_NED, state, "baseline_ned") {}
};

class SbpRelayMsgVelEcef
    : public SbpRelay<msg_vel_ecef_t,
                                     libsbp_ros_msgs::MsgVelEcef> {
  public:
    inline SbpRelayMsgVelEcef(
        const ros::NodeHandle& nh, const std::shared_ptr<sbp_state_t>& state)
        : SbpRelay(nh, SBP_MSG_VEL_ECEF, state, "vel_ecef") {}
};

class SbpRelayMsgVelEcefCov
    : public SbpRelay<msg_vel_ecef_cov_t,
                                     libsbp_ros_msgs::MsgVelEcefCov> {
  public:
    inline SbpRelayMsgVelEcefCov(
        const ros::NodeHandle& nh, const std::shared_ptr<sbp_state_t>& state)
        : SbpRelay(nh, SBP_MSG_VEL_ECEF_COV, state, "vel_ecef_cov") {}
};

class SbpRelayMsgVelNed
    : public SbpRelay<msg_vel_ned_t,
                                     libsbp_ros_msgs::MsgVelNed> {
  public:
    inline SbpRelayMsgVelNed(
        const ros::NodeHandle& nh, const std::shared_ptr<sbp_state_t>& state)
        : SbpRelay(nh, SBP_MSG_VEL_NED, state, "vel_ned") {}
};

class SbpRelayMsgVelNedCov
    : public SbpRelay<msg_vel_ned_cov_t,
                                     libsbp_ros_msgs::MsgVelNedCov> {
  public:
    inline SbpRelayMsgVelNedCov(
        const ros::NodeHandle& nh, const std::shared_ptr<sbp_state_t>& state)
        : SbpRelay(nh, SBP_MSG_VEL_NED_COV, state, "vel_ned_cov") {}
};

class SbpRelayMsgPosEcefGnss
    : public SbpRelay<msg_pos_ecef_gnss_t,
                                     libsbp_ros_msgs::MsgPosEcefGnss> {
  public:
    inline SbpRelayMsgPosEcefGnss(
        const ros::NodeHandle& nh, const std::shared_ptr<sbp_state_t>& state)
        : SbpRelay(nh, SBP_MSG_POS_ECEF_GNSS, state, "pos_ecef_gnss") {}
};

class SbpRelayMsgPosEcefCovGnss
    : public SbpRelay<msg_pos_ecef_cov_gnss_t,
                                     libsbp_ros_msgs::MsgPosEcefCovGnss> {
  public:
    inline SbpRelayMsgPosEcefCovGnss(
        const ros::NodeHandle& nh, const std::shared_ptr<sbp_state_t>& state)
        : SbpRelay(nh, SBP_MSG_POS_ECEF_COV_GNSS, state, "pos_ecef_cov_gnss") {}
};

class SbpRelayMsgPosLlhGnss
    : public SbpRelay<msg_pos_llh_gnss_t,
                                     libsbp_ros_msgs::MsgPosLlhGnss> {
  public:
    inline SbpRelayMsgPosLlhGnss(
        const ros::NodeHandle& nh, const std::shared_ptr<sbp_state_t>& state)
        : SbpRelay(nh, SBP_MSG_POS_LLH_GNSS, state, "pos_llh_gnss") {}
};

class SbpRelayMsgPosLlhCovGnss
    : public SbpRelay<msg_pos_llh_cov_gnss_t,
                                     libsbp_ros_msgs::MsgPosLlhCovGnss> {
  public:
    inline SbpRelayMsgPosLlhCovGnss(
        const ros::NodeHandle& nh, const std::shared_ptr<sbp_state_t>& state)
        : SbpRelay(nh, SBP_MSG_POS_LLH_COV_GNSS, state, "pos_llh_cov_gnss") {}
};

class SbpRelayMsgVelEcefGnss
    : public SbpRelay<msg_vel_ecef_gnss_t,
                                     libsbp_ros_msgs::MsgVelEcefGnss> {
  public:
    inline SbpRelayMsgVelEcefGnss(
        const ros::NodeHandle& nh, const std::shared_ptr<sbp_state_t>& state)
        : SbpRelay(nh, SBP_MSG_VEL_ECEF_GNSS, state, "vel_ecef_gnss") {}
};

class SbpRelayMsgVelEcefCovGnss
    : public SbpRelay<msg_vel_ecef_cov_gnss_t,
                                     libsbp_ros_msgs::MsgVelEcefCovGnss> {
  public:
    inline SbpRelayMsgVelEcefCovGnss(
        const ros::NodeHandle& nh, const std::shared_ptr<sbp_state_t>& state)
        : SbpRelay(nh, SBP_MSG_VEL_ECEF_COV_GNSS, state, "vel_ecef_cov_gnss") {}
};

class SbpRelayMsgVelNedGnss
    : public SbpRelay<msg_vel_ned_gnss_t,
                                     libsbp_ros_msgs::MsgVelNedGnss> {
  public:
    inline SbpRelayMsgVelNedGnss(
        const ros::NodeHandle& nh, const std::shared_ptr<sbp_state_t>& state)
        : SbpRelay(nh, SBP_MSG_VEL_NED_GNSS, state, "vel_ned_gnss") {}
};

class SbpRelayMsgVelNedCovGnss
    : public SbpRelay<msg_vel_ned_cov_gnss_t,
                                     libsbp_ros_msgs::MsgVelNedCovGnss> {
  public:
    inline SbpRelayMsgVelNedCovGnss(
        const ros::NodeHandle& nh, const std::shared_ptr<sbp_state_t>& state)
        : SbpRelay(nh, SBP_MSG_VEL_NED_COV_GNSS, state, "vel_ned_cov_gnss") {}
};

class SbpRelayMsgVelBody
    : public SbpRelay<msg_vel_body_t,
                                     libsbp_ros_msgs::MsgVelBody> {
  public:
    inline SbpRelayMsgVelBody(
        const ros::NodeHandle& nh, const std::shared_ptr<sbp_state_t>& state)
        : SbpRelay(nh, SBP_MSG_VEL_BODY, state, "vel_body") {}
};

class SbpRelayMsgAgeCorrections
    : public SbpRelay<msg_age_corrections_t,
                                     libsbp_ros_msgs::MsgAgeCorrections> {
  public:
    inline SbpRelayMsgAgeCorrections(
        const ros::NodeHandle& nh, const std::shared_ptr<sbp_state_t>& state)
        : SbpRelay(nh, SBP_MSG_AGE_CORRECTIONS, state, "age_corrections") {}
};

class SbpRelayMsgProtectionLevel
    : public SbpRelay<msg_protection_level_t,
                                     libsbp_ros_msgs::MsgProtectionLevel> {
  public:
    inline SbpRelayMsgProtectionLevel(
        const ros::NodeHandle& nh, const std::shared_ptr<sbp_state_t>& state)
        : SbpRelay(nh, SBP_MSG_PROTECTION_LEVEL, state, "protection_level") {}
};

class SbpRelayMsgNdbEvent
    : public SbpRelay<msg_ndb_event_t,
                                     libsbp_ros_msgs::MsgNdbEvent> {
  public:
    inline SbpRelayMsgNdbEvent(
        const ros::NodeHandle& nh, const std::shared_ptr<sbp_state_t>& state)
        : SbpRelay(nh, SBP_MSG_NDB_EVENT, state, "ndb_event") {}
};

class SbpRelayMsgObs
    : public SbpRelay<msg_obs_t,
                                     libsbp_ros_msgs::MsgObs> {
  public:
    inline SbpRelayMsgObs(
        const ros::NodeHandle& nh, const std::shared_ptr<sbp_state_t>& state)
        : SbpRelay(nh, SBP_MSG_OBS, state, "obs") {}
};

class SbpRelayMsgBasePosLlh
    : public SbpRelay<msg_base_pos_llh_t,
                                     libsbp_ros_msgs::MsgBasePosLlh> {
  public:
    inline SbpRelayMsgBasePosLlh(
        const ros::NodeHandle& nh, const std::shared_ptr<sbp_state_t>& state)
        : SbpRelay(nh, SBP_MSG_BASE_POS_LLH, state, "base_pos_llh") {}
};

class SbpRelayMsgBasePosEcef
    : public SbpRelay<msg_base_pos_ecef_t,
                                     libsbp_ros_msgs::MsgBasePosEcef> {
  public:
    inline SbpRelayMsgBasePosEcef(
        const ros::NodeHandle& nh, const std::shared_ptr<sbp_state_t>& state)
        : SbpRelay(nh, SBP_MSG_BASE_POS_ECEF, state, "base_pos_ecef") {}
};

class SbpRelayMsgEphemerisGps
    : public SbpRelay<msg_ephemeris_gps_t,
                                     libsbp_ros_msgs::MsgEphemerisGps> {
  public:
    inline SbpRelayMsgEphemerisGps(
        const ros::NodeHandle& nh, const std::shared_ptr<sbp_state_t>& state)
        : SbpRelay(nh, SBP_MSG_EPHEMERIS_GPS, state, "ephemeris_gps") {}
};

class SbpRelayMsgEphemerisQzss
    : public SbpRelay<msg_ephemeris_qzss_t,
                                     libsbp_ros_msgs::MsgEphemerisQzss> {
  public:
    inline SbpRelayMsgEphemerisQzss(
        const ros::NodeHandle& nh, const std::shared_ptr<sbp_state_t>& state)
        : SbpRelay(nh, SBP_MSG_EPHEMERIS_QZSS, state, "ephemeris_qzss") {}
};

class SbpRelayMsgEphemerisBds
    : public SbpRelay<msg_ephemeris_bds_t,
                                     libsbp_ros_msgs::MsgEphemerisBds> {
  public:
    inline SbpRelayMsgEphemerisBds(
        const ros::NodeHandle& nh, const std::shared_ptr<sbp_state_t>& state)
        : SbpRelay(nh, SBP_MSG_EPHEMERIS_BDS, state, "ephemeris_bds") {}
};

class SbpRelayMsgEphemerisGal
    : public SbpRelay<msg_ephemeris_gal_t,
                                     libsbp_ros_msgs::MsgEphemerisGal> {
  public:
    inline SbpRelayMsgEphemerisGal(
        const ros::NodeHandle& nh, const std::shared_ptr<sbp_state_t>& state)
        : SbpRelay(nh, SBP_MSG_EPHEMERIS_GAL, state, "ephemeris_gal") {}
};

class SbpRelayMsgEphemerisSbas
    : public SbpRelay<msg_ephemeris_sbas_t,
                                     libsbp_ros_msgs::MsgEphemerisSbas> {
  public:
    inline SbpRelayMsgEphemerisSbas(
        const ros::NodeHandle& nh, const std::shared_ptr<sbp_state_t>& state)
        : SbpRelay(nh, SBP_MSG_EPHEMERIS_SBAS, state, "ephemeris_sbas") {}
};

class SbpRelayMsgEphemerisGlo
    : public SbpRelay<msg_ephemeris_glo_t,
                                     libsbp_ros_msgs::MsgEphemerisGlo> {
  public:
    inline SbpRelayMsgEphemerisGlo(
        const ros::NodeHandle& nh, const std::shared_ptr<sbp_state_t>& state)
        : SbpRelay(nh, SBP_MSG_EPHEMERIS_GLO, state, "ephemeris_glo") {}
};

class SbpRelayMsgIono
    : public SbpRelay<msg_iono_t,
                                     libsbp_ros_msgs::MsgIono> {
  public:
    inline SbpRelayMsgIono(
        const ros::NodeHandle& nh, const std::shared_ptr<sbp_state_t>& state)
        : SbpRelay(nh, SBP_MSG_IONO, state, "iono") {}
};

class SbpRelayMsgGnssCapb
    : public SbpRelay<msg_gnss_capb_t,
                                     libsbp_ros_msgs::MsgGnssCapb> {
  public:
    inline SbpRelayMsgGnssCapb(
        const ros::NodeHandle& nh, const std::shared_ptr<sbp_state_t>& state)
        : SbpRelay(nh, SBP_MSG_GNSS_CAPB, state, "gnss_capb") {}
};

class SbpRelayMsgGroupDelay
    : public SbpRelay<msg_group_delay_t,
                                     libsbp_ros_msgs::MsgGroupDelay> {
  public:
    inline SbpRelayMsgGroupDelay(
        const ros::NodeHandle& nh, const std::shared_ptr<sbp_state_t>& state)
        : SbpRelay(nh, SBP_MSG_GROUP_DELAY, state, "group_delay") {}
};

class SbpRelayMsgAlmanacGps
    : public SbpRelay<msg_almanac_gps_t,
                                     libsbp_ros_msgs::MsgAlmanacGps> {
  public:
    inline SbpRelayMsgAlmanacGps(
        const ros::NodeHandle& nh, const std::shared_ptr<sbp_state_t>& state)
        : SbpRelay(nh, SBP_MSG_ALMANAC_GPS, state, "almanac_gps") {}
};

class SbpRelayMsgAlmanacGlo
    : public SbpRelay<msg_almanac_glo_t,
                                     libsbp_ros_msgs::MsgAlmanacGlo> {
  public:
    inline SbpRelayMsgAlmanacGlo(
        const ros::NodeHandle& nh, const std::shared_ptr<sbp_state_t>& state)
        : SbpRelay(nh, SBP_MSG_ALMANAC_GLO, state, "almanac_glo") {}
};

class SbpRelayMsgGloBiases
    : public SbpRelay<msg_glo_biases_t,
                                     libsbp_ros_msgs::MsgGloBiases> {
  public:
    inline SbpRelayMsgGloBiases(
        const ros::NodeHandle& nh, const std::shared_ptr<sbp_state_t>& state)
        : SbpRelay(nh, SBP_MSG_GLO_BIASES, state, "glo_biases") {}
};

class SbpRelayMsgSvAzEl
    : public SbpRelay<msg_sv_az_el_t,
                                     libsbp_ros_msgs::MsgSvAzEl> {
  public:
    inline SbpRelayMsgSvAzEl(
        const ros::NodeHandle& nh, const std::shared_ptr<sbp_state_t>& state)
        : SbpRelay(nh, SBP_MSG_SV_AZ_EL, state, "sv_az_el") {}
};

class SbpRelayMsgOsr
    : public SbpRelay<msg_osr_t,
                                     libsbp_ros_msgs::MsgOsr> {
  public:
    inline SbpRelayMsgOsr(
        const ros::NodeHandle& nh, const std::shared_ptr<sbp_state_t>& state)
        : SbpRelay(nh, SBP_MSG_OSR, state, "osr") {}
};

class SbpRelayMsgBaselineHeading
    : public SbpRelay<msg_baseline_heading_t,
                                     libsbp_ros_msgs::MsgBaselineHeading> {
  public:
    inline SbpRelayMsgBaselineHeading(
        const ros::NodeHandle& nh, const std::shared_ptr<sbp_state_t>& state)
        : SbpRelay(nh, SBP_MSG_BASELINE_HEADING, state, "baseline_heading") {}
};

class SbpRelayMsgOrientQuat
    : public SbpRelay<msg_orient_quat_t,
                                     libsbp_ros_msgs::MsgOrientQuat> {
  public:
    inline SbpRelayMsgOrientQuat(
        const ros::NodeHandle& nh, const std::shared_ptr<sbp_state_t>& state)
        : SbpRelay(nh, SBP_MSG_ORIENT_QUAT, state, "orient_quat") {}
};

class SbpRelayMsgOrientEuler
    : public SbpRelay<msg_orient_euler_t,
                                     libsbp_ros_msgs::MsgOrientEuler> {
  public:
    inline SbpRelayMsgOrientEuler(
        const ros::NodeHandle& nh, const std::shared_ptr<sbp_state_t>& state)
        : SbpRelay(nh, SBP_MSG_ORIENT_EULER, state, "orient_euler") {}
};

class SbpRelayMsgAngularRate
    : public SbpRelay<msg_angular_rate_t,
                                     libsbp_ros_msgs::MsgAngularRate> {
  public:
    inline SbpRelayMsgAngularRate(
        const ros::NodeHandle& nh, const std::shared_ptr<sbp_state_t>& state)
        : SbpRelay(nh, SBP_MSG_ANGULAR_RATE, state, "angular_rate") {}
};

class SbpRelayMsgReset
    : public SbpRelay<msg_reset_t,
                                     libsbp_ros_msgs::MsgReset> {
  public:
    inline SbpRelayMsgReset(
        const ros::NodeHandle& nh, const std::shared_ptr<sbp_state_t>& state)
        : SbpRelay(nh, SBP_MSG_RESET, state, "reset") {}
};

class SbpRelayMsgResetFilters
    : public SbpRelay<msg_reset_filters_t,
                                     libsbp_ros_msgs::MsgResetFilters> {
  public:
    inline SbpRelayMsgResetFilters(
        const ros::NodeHandle& nh, const std::shared_ptr<sbp_state_t>& state)
        : SbpRelay(nh, SBP_MSG_RESET_FILTERS, state, "reset_filters") {}
};

class SbpRelayMsgThreadState
    : public SbpRelay<msg_thread_state_t,
                                     libsbp_ros_msgs::MsgThreadState> {
  public:
    inline SbpRelayMsgThreadState(
        const ros::NodeHandle& nh, const std::shared_ptr<sbp_state_t>& state)
        : SbpRelay(nh, SBP_MSG_THREAD_STATE, state, "thread_state") {}
};

class SbpRelayMsgUartState
    : public SbpRelay<msg_uart_state_t,
                                     libsbp_ros_msgs::MsgUartState> {
  public:
    inline SbpRelayMsgUartState(
        const ros::NodeHandle& nh, const std::shared_ptr<sbp_state_t>& state)
        : SbpRelay(nh, SBP_MSG_UART_STATE, state, "uart_state") {}
};

class SbpRelayMsgIarState
    : public SbpRelay<msg_iar_state_t,
                                     libsbp_ros_msgs::MsgIarState> {
  public:
    inline SbpRelayMsgIarState(
        const ros::NodeHandle& nh, const std::shared_ptr<sbp_state_t>& state)
        : SbpRelay(nh, SBP_MSG_IAR_STATE, state, "iar_state") {}
};

class SbpRelayMsgMaskSatellite
    : public SbpRelay<msg_mask_satellite_t,
                                     libsbp_ros_msgs::MsgMaskSatellite> {
  public:
    inline SbpRelayMsgMaskSatellite(
        const ros::NodeHandle& nh, const std::shared_ptr<sbp_state_t>& state)
        : SbpRelay(nh, SBP_MSG_MASK_SATELLITE, state, "mask_satellite") {}
};

class SbpRelayMsgDeviceMonitor
    : public SbpRelay<msg_device_monitor_t,
                                     libsbp_ros_msgs::MsgDeviceMonitor> {
  public:
    inline SbpRelayMsgDeviceMonitor(
        const ros::NodeHandle& nh, const std::shared_ptr<sbp_state_t>& state)
        : SbpRelay(nh, SBP_MSG_DEVICE_MONITOR, state, "device_monitor") {}
};

class SbpRelayMsgCommandReq
    : public SbpRelay<msg_command_req_t,
                                     libsbp_ros_msgs::MsgCommandReq> {
  public:
    inline SbpRelayMsgCommandReq(
        const ros::NodeHandle& nh, const std::shared_ptr<sbp_state_t>& state)
        : SbpRelay(nh, SBP_MSG_COMMAND_REQ, state, "command_req") {}
};

class SbpRelayMsgCommandResp
    : public SbpRelay<msg_command_resp_t,
                                     libsbp_ros_msgs::MsgCommandResp> {
  public:
    inline SbpRelayMsgCommandResp(
        const ros::NodeHandle& nh, const std::shared_ptr<sbp_state_t>& state)
        : SbpRelay(nh, SBP_MSG_COMMAND_RESP, state, "command_resp") {}
};

class SbpRelayMsgCommandOutput
    : public SbpRelay<msg_command_output_t,
                                     libsbp_ros_msgs::MsgCommandOutput> {
  public:
    inline SbpRelayMsgCommandOutput(
        const ros::NodeHandle& nh, const std::shared_ptr<sbp_state_t>& state)
        : SbpRelay(nh, SBP_MSG_COMMAND_OUTPUT, state, "command_output") {}
};

class SbpRelayMsgNetworkStateResp
    : public SbpRelay<msg_network_state_resp_t,
                                     libsbp_ros_msgs::MsgNetworkStateResp> {
  public:
    inline SbpRelayMsgNetworkStateResp(
        const ros::NodeHandle& nh, const std::shared_ptr<sbp_state_t>& state)
        : SbpRelay(nh, SBP_MSG_NETWORK_STATE_RESP, state, "network_state_resp") {}
};

class SbpRelayMsgNetworkBandwidthUsage
    : public SbpRelay<msg_network_bandwidth_usage_t,
                                     libsbp_ros_msgs::MsgNetworkBandwidthUsage> {
  public:
    inline SbpRelayMsgNetworkBandwidthUsage(
        const ros::NodeHandle& nh, const std::shared_ptr<sbp_state_t>& state)
        : SbpRelay(nh, SBP_MSG_NETWORK_BANDWIDTH_USAGE, state, "network_bandwidth_usage") {}
};

class SbpRelayMsgCellModemStatus
    : public SbpRelay<msg_cell_modem_status_t,
                                     libsbp_ros_msgs::MsgCellModemStatus> {
  public:
    inline SbpRelayMsgCellModemStatus(
        const ros::NodeHandle& nh, const std::shared_ptr<sbp_state_t>& state)
        : SbpRelay(nh, SBP_MSG_CELL_MODEM_STATUS, state, "cell_modem_status") {}
};

class SbpRelayMsgSpecan
    : public SbpRelay<msg_specan_t,
                                     libsbp_ros_msgs::MsgSpecan> {
  public:
    inline SbpRelayMsgSpecan(
        const ros::NodeHandle& nh, const std::shared_ptr<sbp_state_t>& state)
        : SbpRelay(nh, SBP_MSG_SPECAN, state, "specan") {}
};

class SbpRelayMsgFrontEndGain
    : public SbpRelay<msg_front_end_gain_t,
                                     libsbp_ros_msgs::MsgFrontEndGain> {
  public:
    inline SbpRelayMsgFrontEndGain(
        const ros::NodeHandle& nh, const std::shared_ptr<sbp_state_t>& state)
        : SbpRelay(nh, SBP_MSG_FRONT_END_GAIN, state, "front_end_gain") {}
};

class SbpRelayMsgSbasRaw
    : public SbpRelay<msg_sbas_raw_t,
                                     libsbp_ros_msgs::MsgSbasRaw> {
  public:
    inline SbpRelayMsgSbasRaw(
        const ros::NodeHandle& nh, const std::shared_ptr<sbp_state_t>& state)
        : SbpRelay(nh, SBP_MSG_SBAS_RAW, state, "sbas_raw") {}
};

class SbpRelayMsgSettingsWrite
    : public SbpRelay<msg_settings_write_t,
                                     libsbp_ros_msgs::MsgSettingsWrite> {
  public:
    inline SbpRelayMsgSettingsWrite(
        const ros::NodeHandle& nh, const std::shared_ptr<sbp_state_t>& state)
        : SbpRelay(nh, SBP_MSG_SETTINGS_WRITE, state, "settings_write") {}
};

class SbpRelayMsgSettingsWriteResp
    : public SbpRelay<msg_settings_write_resp_t,
                                     libsbp_ros_msgs::MsgSettingsWriteResp> {
  public:
    inline SbpRelayMsgSettingsWriteResp(
        const ros::NodeHandle& nh, const std::shared_ptr<sbp_state_t>& state)
        : SbpRelay(nh, SBP_MSG_SETTINGS_WRITE_RESP, state, "settings_write_resp") {}
};

class SbpRelayMsgSettingsReadReq
    : public SbpRelay<msg_settings_read_req_t,
                                     libsbp_ros_msgs::MsgSettingsReadReq> {
  public:
    inline SbpRelayMsgSettingsReadReq(
        const ros::NodeHandle& nh, const std::shared_ptr<sbp_state_t>& state)
        : SbpRelay(nh, SBP_MSG_SETTINGS_READ_REQ, state, "settings_read_req") {}
};

class SbpRelayMsgSettingsReadResp
    : public SbpRelay<msg_settings_read_resp_t,
                                     libsbp_ros_msgs::MsgSettingsReadResp> {
  public:
    inline SbpRelayMsgSettingsReadResp(
        const ros::NodeHandle& nh, const std::shared_ptr<sbp_state_t>& state)
        : SbpRelay(nh, SBP_MSG_SETTINGS_READ_RESP, state, "settings_read_resp") {}
};

class SbpRelayMsgSettingsReadByIndexReq
    : public SbpRelay<msg_settings_read_by_index_req_t,
                                     libsbp_ros_msgs::MsgSettingsReadByIndexReq> {
  public:
    inline SbpRelayMsgSettingsReadByIndexReq(
        const ros::NodeHandle& nh, const std::shared_ptr<sbp_state_t>& state)
        : SbpRelay(nh, SBP_MSG_SETTINGS_READ_BY_INDEX_REQ, state, "settings_read_by_index_req") {}
};

class SbpRelayMsgSettingsReadByIndexResp
    : public SbpRelay<msg_settings_read_by_index_resp_t,
                                     libsbp_ros_msgs::MsgSettingsReadByIndexResp> {
  public:
    inline SbpRelayMsgSettingsReadByIndexResp(
        const ros::NodeHandle& nh, const std::shared_ptr<sbp_state_t>& state)
        : SbpRelay(nh, SBP_MSG_SETTINGS_READ_BY_INDEX_RESP, state, "settings_read_by_index_resp") {}
};

class SbpRelayMsgSettingsRegister
    : public SbpRelay<msg_settings_register_t,
                                     libsbp_ros_msgs::MsgSettingsRegister> {
  public:
    inline SbpRelayMsgSettingsRegister(
        const ros::NodeHandle& nh, const std::shared_ptr<sbp_state_t>& state)
        : SbpRelay(nh, SBP_MSG_SETTINGS_REGISTER, state, "settings_register") {}
};

class SbpRelayMsgSettingsRegisterResp
    : public SbpRelay<msg_settings_register_resp_t,
                                     libsbp_ros_msgs::MsgSettingsRegisterResp> {
  public:
    inline SbpRelayMsgSettingsRegisterResp(
        const ros::NodeHandle& nh, const std::shared_ptr<sbp_state_t>& state)
        : SbpRelay(nh, SBP_MSG_SETTINGS_REGISTER_RESP, state, "settings_register_resp") {}
};

class SbpRelayMsgSsrOrbitClock
    : public SbpRelay<msg_ssr_orbit_clock_t,
                                     libsbp_ros_msgs::MsgSsrOrbitClock> {
  public:
    inline SbpRelayMsgSsrOrbitClock(
        const ros::NodeHandle& nh, const std::shared_ptr<sbp_state_t>& state)
        : SbpRelay(nh, SBP_MSG_SSR_ORBIT_CLOCK, state, "ssr_orbit_clock") {}
};

class SbpRelayMsgSsrCodeBiases
    : public SbpRelay<msg_ssr_code_biases_t,
                                     libsbp_ros_msgs::MsgSsrCodeBiases> {
  public:
    inline SbpRelayMsgSsrCodeBiases(
        const ros::NodeHandle& nh, const std::shared_ptr<sbp_state_t>& state)
        : SbpRelay(nh, SBP_MSG_SSR_CODE_BIASES, state, "ssr_code_biases") {}
};

class SbpRelayMsgSsrPhaseBiases
    : public SbpRelay<msg_ssr_phase_biases_t,
                                     libsbp_ros_msgs::MsgSsrPhaseBiases> {
  public:
    inline SbpRelayMsgSsrPhaseBiases(
        const ros::NodeHandle& nh, const std::shared_ptr<sbp_state_t>& state)
        : SbpRelay(nh, SBP_MSG_SSR_PHASE_BIASES, state, "ssr_phase_biases") {}
};

class SbpRelayMsgSsrStecCorrection
    : public SbpRelay<msg_ssr_stec_correction_t,
                                     libsbp_ros_msgs::MsgSsrStecCorrection> {
  public:
    inline SbpRelayMsgSsrStecCorrection(
        const ros::NodeHandle& nh, const std::shared_ptr<sbp_state_t>& state)
        : SbpRelay(nh, SBP_MSG_SSR_STEC_CORRECTION, state, "ssr_stec_correction") {}
};

class SbpRelayMsgSsrGriddedCorrectionNoStd
    : public SbpRelay<msg_ssr_gridded_correction_no_std_t,
                                     libsbp_ros_msgs::MsgSsrGriddedCorrectionNoStd> {
  public:
    inline SbpRelayMsgSsrGriddedCorrectionNoStd(
        const ros::NodeHandle& nh, const std::shared_ptr<sbp_state_t>& state)
        : SbpRelay(nh, SBP_MSG_SSR_GRIDDED_CORRECTION_NO_STD, state, "ssr_gridded_correction_no_std") {}
};

class SbpRelayMsgSsrGriddedCorrection
    : public SbpRelay<msg_ssr_gridded_correction_t,
                                     libsbp_ros_msgs::MsgSsrGriddedCorrection> {
  public:
    inline SbpRelayMsgSsrGriddedCorrection(
        const ros::NodeHandle& nh, const std::shared_ptr<sbp_state_t>& state)
        : SbpRelay(nh, SBP_MSG_SSR_GRIDDED_CORRECTION, state, "ssr_gridded_correction") {}
};

class SbpRelayMsgSsrGridDefinition
    : public SbpRelay<msg_ssr_grid_definition_t,
                                     libsbp_ros_msgs::MsgSsrGridDefinition> {
  public:
    inline SbpRelayMsgSsrGridDefinition(
        const ros::NodeHandle& nh, const std::shared_ptr<sbp_state_t>& state)
        : SbpRelay(nh, SBP_MSG_SSR_GRID_DEFINITION, state, "ssr_grid_definition") {}
};

class SbpRelayMsgStartup
    : public SbpRelay<msg_startup_t,
                                     libsbp_ros_msgs::MsgStartup> {
  public:
    inline SbpRelayMsgStartup(
        const ros::NodeHandle& nh, const std::shared_ptr<sbp_state_t>& state)
        : SbpRelay(nh, SBP_MSG_STARTUP, state, "startup") {}
};

class SbpRelayMsgDgnssStatus
    : public SbpRelay<msg_dgnss_status_t,
                                     libsbp_ros_msgs::MsgDgnssStatus> {
  public:
    inline SbpRelayMsgDgnssStatus(
        const ros::NodeHandle& nh, const std::shared_ptr<sbp_state_t>& state)
        : SbpRelay(nh, SBP_MSG_DGNSS_STATUS, state, "dgnss_status") {}
};

class SbpRelayMsgHeartbeat
    : public SbpRelay<msg_heartbeat_t,
                                     libsbp_ros_msgs::MsgHeartbeat> {
  public:
    inline SbpRelayMsgHeartbeat(
        const ros::NodeHandle& nh, const std::shared_ptr<sbp_state_t>& state)
        : SbpRelay(nh, SBP_MSG_HEARTBEAT, state, "heartbeat") {}
};

class SbpRelayMsgInsStatus
    : public SbpRelay<msg_ins_status_t,
                                     libsbp_ros_msgs::MsgInsStatus> {
  public:
    inline SbpRelayMsgInsStatus(
        const ros::NodeHandle& nh, const std::shared_ptr<sbp_state_t>& state)
        : SbpRelay(nh, SBP_MSG_INS_STATUS, state, "ins_status") {}
};

class SbpRelayMsgCsacTelemetry
    : public SbpRelay<msg_csac_telemetry_t,
                                     libsbp_ros_msgs::MsgCsacTelemetry> {
  public:
    inline SbpRelayMsgCsacTelemetry(
        const ros::NodeHandle& nh, const std::shared_ptr<sbp_state_t>& state)
        : SbpRelay(nh, SBP_MSG_CSAC_TELEMETRY, state, "csac_telemetry") {}
};

class SbpRelayMsgCsacTelemetryLabels
    : public SbpRelay<msg_csac_telemetry_labels_t,
                                     libsbp_ros_msgs::MsgCsacTelemetryLabels> {
  public:
    inline SbpRelayMsgCsacTelemetryLabels(
        const ros::NodeHandle& nh, const std::shared_ptr<sbp_state_t>& state)
        : SbpRelay(nh, SBP_MSG_CSAC_TELEMETRY_LABELS, state, "csac_telemetry_labels") {}
};

class SbpRelayMsgTrackingState
    : public SbpRelay<msg_tracking_state_t,
                                     libsbp_ros_msgs::MsgTrackingState> {
  public:
    inline SbpRelayMsgTrackingState(
        const ros::NodeHandle& nh, const std::shared_ptr<sbp_state_t>& state)
        : SbpRelay(nh, SBP_MSG_TRACKING_STATE, state, "tracking_state") {}
};

class SbpRelayMsgMeasurementState
    : public SbpRelay<msg_measurement_state_t,
                                     libsbp_ros_msgs::MsgMeasurementState> {
  public:
    inline SbpRelayMsgMeasurementState(
        const ros::NodeHandle& nh, const std::shared_ptr<sbp_state_t>& state)
        : SbpRelay(nh, SBP_MSG_MEASUREMENT_STATE, state, "measurement_state") {}
};

class SbpRelayMsgTrackingIq
    : public SbpRelay<msg_tracking_iq_t,
                                     libsbp_ros_msgs::MsgTrackingIq> {
  public:
    inline SbpRelayMsgTrackingIq(
        const ros::NodeHandle& nh, const std::shared_ptr<sbp_state_t>& state)
        : SbpRelay(nh, SBP_MSG_TRACKING_IQ, state, "tracking_iq") {}
};

class SbpRelayMsgUserData
    : public SbpRelay<msg_user_data_t,
                                     libsbp_ros_msgs::MsgUserData> {
  public:
    inline SbpRelayMsgUserData(
        const ros::NodeHandle& nh, const std::shared_ptr<sbp_state_t>& state)
        : SbpRelay(nh, SBP_MSG_USER_DATA, state, "user_data") {}
};

class SbpRelayMsgOdometry
    : public SbpRelay<msg_odometry_t,
                                     libsbp_ros_msgs::MsgOdometry> {
  public:
    inline SbpRelayMsgOdometry(
        const ros::NodeHandle& nh, const std::shared_ptr<sbp_state_t>& state)
        : SbpRelay(nh, SBP_MSG_ODOMETRY, state, "odometry") {}
};

class SbpRelayMsgWheeltick
    : public SbpRelay<msg_wheeltick_t,
                                     libsbp_ros_msgs::MsgWheeltick> {
  public:
    inline SbpRelayMsgWheeltick(
        const ros::NodeHandle& nh, const std::shared_ptr<sbp_state_t>& state)
        : SbpRelay(nh, SBP_MSG_WHEELTICK, state, "wheeltick") {}
};


// Create all relays
std::vector<SBPCallbackHandler::Ptr> createAllSbpRelays(const ros::NodeHandle& nh, const std::shared_ptr<sbp_state_t>& state) {
  std::vector<SBPCallbackHandler::Ptr> relays;

  relays.push_back(SBPCallbackHandler::Ptr(new SbpRelayMsgAcqResult(nh, state)));
  relays.push_back(SBPCallbackHandler::Ptr(new SbpRelayMsgAcqSvProfile(nh, state)));
  relays.push_back(SBPCallbackHandler::Ptr(new SbpRelayMsgBootloaderHandshakeResp(nh, state)));
  relays.push_back(SBPCallbackHandler::Ptr(new SbpRelayMsgBootloaderJumpToApp(nh, state)));
  relays.push_back(SBPCallbackHandler::Ptr(new SbpRelayMsgNapDeviceDnaResp(nh, state)));
  relays.push_back(SBPCallbackHandler::Ptr(new SbpRelayMsgExtEvent(nh, state)));
  relays.push_back(SBPCallbackHandler::Ptr(new SbpRelayMsgFileioReadReq(nh, state)));
  relays.push_back(SBPCallbackHandler::Ptr(new SbpRelayMsgFileioReadResp(nh, state)));
  relays.push_back(SBPCallbackHandler::Ptr(new SbpRelayMsgFileioReadDirReq(nh, state)));
  relays.push_back(SBPCallbackHandler::Ptr(new SbpRelayMsgFileioReadDirResp(nh, state)));
  relays.push_back(SBPCallbackHandler::Ptr(new SbpRelayMsgFileioRemove(nh, state)));
  relays.push_back(SBPCallbackHandler::Ptr(new SbpRelayMsgFileioWriteReq(nh, state)));
  relays.push_back(SBPCallbackHandler::Ptr(new SbpRelayMsgFileioWriteResp(nh, state)));
  relays.push_back(SBPCallbackHandler::Ptr(new SbpRelayMsgFileioConfigReq(nh, state)));
  relays.push_back(SBPCallbackHandler::Ptr(new SbpRelayMsgFileioConfigResp(nh, state)));
  relays.push_back(SBPCallbackHandler::Ptr(new SbpRelayMsgFlashProgram(nh, state)));
  relays.push_back(SBPCallbackHandler::Ptr(new SbpRelayMsgFlashDone(nh, state)));
  relays.push_back(SBPCallbackHandler::Ptr(new SbpRelayMsgFlashReadReq(nh, state)));
  relays.push_back(SBPCallbackHandler::Ptr(new SbpRelayMsgFlashReadResp(nh, state)));
  relays.push_back(SBPCallbackHandler::Ptr(new SbpRelayMsgFlashErase(nh, state)));
  relays.push_back(SBPCallbackHandler::Ptr(new SbpRelayMsgStmFlashLockSector(nh, state)));
  relays.push_back(SBPCallbackHandler::Ptr(new SbpRelayMsgStmFlashUnlockSector(nh, state)));
  relays.push_back(SBPCallbackHandler::Ptr(new SbpRelayMsgStmUniqueIdResp(nh, state)));
  relays.push_back(SBPCallbackHandler::Ptr(new SbpRelayMsgM25FlashWriteStatus(nh, state)));
  relays.push_back(SBPCallbackHandler::Ptr(new SbpRelayMsgImuRaw(nh, state)));
  relays.push_back(SBPCallbackHandler::Ptr(new SbpRelayMsgImuAux(nh, state)));
  relays.push_back(SBPCallbackHandler::Ptr(new SbpRelayMsgLinuxCpuState(nh, state)));
  relays.push_back(SBPCallbackHandler::Ptr(new SbpRelayMsgLinuxMemState(nh, state)));
  relays.push_back(SBPCallbackHandler::Ptr(new SbpRelayMsgLinuxSysState(nh, state)));
  relays.push_back(SBPCallbackHandler::Ptr(new SbpRelayMsgLinuxProcessSocketCounts(nh, state)));
  relays.push_back(SBPCallbackHandler::Ptr(new SbpRelayMsgLinuxProcessSocketQueues(nh, state)));
  relays.push_back(SBPCallbackHandler::Ptr(new SbpRelayMsgLinuxSocketUsage(nh, state)));
  relays.push_back(SBPCallbackHandler::Ptr(new SbpRelayMsgLinuxProcessFdCount(nh, state)));
  relays.push_back(SBPCallbackHandler::Ptr(new SbpRelayMsgLinuxProcessFdSummary(nh, state)));
  relays.push_back(SBPCallbackHandler::Ptr(new SbpRelayMsgLog(nh, state)));
  relays.push_back(SBPCallbackHandler::Ptr(new SbpRelayMsgFwd(nh, state)));
  relays.push_back(SBPCallbackHandler::Ptr(new SbpRelayMsgMagRaw(nh, state)));
  relays.push_back(SBPCallbackHandler::Ptr(new SbpRelayMsgGpsTime(nh, state)));
  relays.push_back(SBPCallbackHandler::Ptr(new SbpRelayMsgUtcTime(nh, state)));
  relays.push_back(SBPCallbackHandler::Ptr(new SbpRelayMsgDops(nh, state)));
  relays.push_back(SBPCallbackHandler::Ptr(new SbpRelayMsgPosEcef(nh, state)));
  relays.push_back(SBPCallbackHandler::Ptr(new SbpRelayMsgPosEcefCov(nh, state)));
  relays.push_back(SBPCallbackHandler::Ptr(new SbpRelayMsgPosLlh(nh, state)));
  relays.push_back(SBPCallbackHandler::Ptr(new SbpRelayMsgPosLlhCov(nh, state)));
  relays.push_back(SBPCallbackHandler::Ptr(new SbpRelayMsgBaselineEcef(nh, state)));
  relays.push_back(SBPCallbackHandler::Ptr(new SbpRelayMsgBaselineNed(nh, state)));
  relays.push_back(SBPCallbackHandler::Ptr(new SbpRelayMsgVelEcef(nh, state)));
  relays.push_back(SBPCallbackHandler::Ptr(new SbpRelayMsgVelEcefCov(nh, state)));
  relays.push_back(SBPCallbackHandler::Ptr(new SbpRelayMsgVelNed(nh, state)));
  relays.push_back(SBPCallbackHandler::Ptr(new SbpRelayMsgVelNedCov(nh, state)));
  relays.push_back(SBPCallbackHandler::Ptr(new SbpRelayMsgPosEcefGnss(nh, state)));
  relays.push_back(SBPCallbackHandler::Ptr(new SbpRelayMsgPosEcefCovGnss(nh, state)));
  relays.push_back(SBPCallbackHandler::Ptr(new SbpRelayMsgPosLlhGnss(nh, state)));
  relays.push_back(SBPCallbackHandler::Ptr(new SbpRelayMsgPosLlhCovGnss(nh, state)));
  relays.push_back(SBPCallbackHandler::Ptr(new SbpRelayMsgVelEcefGnss(nh, state)));
  relays.push_back(SBPCallbackHandler::Ptr(new SbpRelayMsgVelEcefCovGnss(nh, state)));
  relays.push_back(SBPCallbackHandler::Ptr(new SbpRelayMsgVelNedGnss(nh, state)));
  relays.push_back(SBPCallbackHandler::Ptr(new SbpRelayMsgVelNedCovGnss(nh, state)));
  relays.push_back(SBPCallbackHandler::Ptr(new SbpRelayMsgVelBody(nh, state)));
  relays.push_back(SBPCallbackHandler::Ptr(new SbpRelayMsgAgeCorrections(nh, state)));
  relays.push_back(SBPCallbackHandler::Ptr(new SbpRelayMsgProtectionLevel(nh, state)));
  relays.push_back(SBPCallbackHandler::Ptr(new SbpRelayMsgNdbEvent(nh, state)));
  relays.push_back(SBPCallbackHandler::Ptr(new SbpRelayMsgObs(nh, state)));
  relays.push_back(SBPCallbackHandler::Ptr(new SbpRelayMsgBasePosLlh(nh, state)));
  relays.push_back(SBPCallbackHandler::Ptr(new SbpRelayMsgBasePosEcef(nh, state)));
  relays.push_back(SBPCallbackHandler::Ptr(new SbpRelayMsgEphemerisGps(nh, state)));
  relays.push_back(SBPCallbackHandler::Ptr(new SbpRelayMsgEphemerisQzss(nh, state)));
  relays.push_back(SBPCallbackHandler::Ptr(new SbpRelayMsgEphemerisBds(nh, state)));
  relays.push_back(SBPCallbackHandler::Ptr(new SbpRelayMsgEphemerisGal(nh, state)));
  relays.push_back(SBPCallbackHandler::Ptr(new SbpRelayMsgEphemerisSbas(nh, state)));
  relays.push_back(SBPCallbackHandler::Ptr(new SbpRelayMsgEphemerisGlo(nh, state)));
  relays.push_back(SBPCallbackHandler::Ptr(new SbpRelayMsgIono(nh, state)));
  relays.push_back(SBPCallbackHandler::Ptr(new SbpRelayMsgGnssCapb(nh, state)));
  relays.push_back(SBPCallbackHandler::Ptr(new SbpRelayMsgGroupDelay(nh, state)));
  relays.push_back(SBPCallbackHandler::Ptr(new SbpRelayMsgAlmanacGps(nh, state)));
  relays.push_back(SBPCallbackHandler::Ptr(new SbpRelayMsgAlmanacGlo(nh, state)));
  relays.push_back(SBPCallbackHandler::Ptr(new SbpRelayMsgGloBiases(nh, state)));
  relays.push_back(SBPCallbackHandler::Ptr(new SbpRelayMsgSvAzEl(nh, state)));
  relays.push_back(SBPCallbackHandler::Ptr(new SbpRelayMsgOsr(nh, state)));
  relays.push_back(SBPCallbackHandler::Ptr(new SbpRelayMsgBaselineHeading(nh, state)));
  relays.push_back(SBPCallbackHandler::Ptr(new SbpRelayMsgOrientQuat(nh, state)));
  relays.push_back(SBPCallbackHandler::Ptr(new SbpRelayMsgOrientEuler(nh, state)));
  relays.push_back(SBPCallbackHandler::Ptr(new SbpRelayMsgAngularRate(nh, state)));
  relays.push_back(SBPCallbackHandler::Ptr(new SbpRelayMsgReset(nh, state)));
  relays.push_back(SBPCallbackHandler::Ptr(new SbpRelayMsgResetFilters(nh, state)));
  relays.push_back(SBPCallbackHandler::Ptr(new SbpRelayMsgThreadState(nh, state)));
  relays.push_back(SBPCallbackHandler::Ptr(new SbpRelayMsgUartState(nh, state)));
  relays.push_back(SBPCallbackHandler::Ptr(new SbpRelayMsgIarState(nh, state)));
  relays.push_back(SBPCallbackHandler::Ptr(new SbpRelayMsgMaskSatellite(nh, state)));
  relays.push_back(SBPCallbackHandler::Ptr(new SbpRelayMsgDeviceMonitor(nh, state)));
  relays.push_back(SBPCallbackHandler::Ptr(new SbpRelayMsgCommandReq(nh, state)));
  relays.push_back(SBPCallbackHandler::Ptr(new SbpRelayMsgCommandResp(nh, state)));
  relays.push_back(SBPCallbackHandler::Ptr(new SbpRelayMsgCommandOutput(nh, state)));
  relays.push_back(SBPCallbackHandler::Ptr(new SbpRelayMsgNetworkStateResp(nh, state)));
  relays.push_back(SBPCallbackHandler::Ptr(new SbpRelayMsgNetworkBandwidthUsage(nh, state)));
  relays.push_back(SBPCallbackHandler::Ptr(new SbpRelayMsgCellModemStatus(nh, state)));
  relays.push_back(SBPCallbackHandler::Ptr(new SbpRelayMsgSpecan(nh, state)));
  relays.push_back(SBPCallbackHandler::Ptr(new SbpRelayMsgFrontEndGain(nh, state)));
  relays.push_back(SBPCallbackHandler::Ptr(new SbpRelayMsgSbasRaw(nh, state)));
  relays.push_back(SBPCallbackHandler::Ptr(new SbpRelayMsgSettingsWrite(nh, state)));
  relays.push_back(SBPCallbackHandler::Ptr(new SbpRelayMsgSettingsWriteResp(nh, state)));
  relays.push_back(SBPCallbackHandler::Ptr(new SbpRelayMsgSettingsReadReq(nh, state)));
  relays.push_back(SBPCallbackHandler::Ptr(new SbpRelayMsgSettingsReadResp(nh, state)));
  relays.push_back(SBPCallbackHandler::Ptr(new SbpRelayMsgSettingsReadByIndexReq(nh, state)));
  relays.push_back(SBPCallbackHandler::Ptr(new SbpRelayMsgSettingsReadByIndexResp(nh, state)));
  relays.push_back(SBPCallbackHandler::Ptr(new SbpRelayMsgSettingsRegister(nh, state)));
  relays.push_back(SBPCallbackHandler::Ptr(new SbpRelayMsgSettingsRegisterResp(nh, state)));
  relays.push_back(SBPCallbackHandler::Ptr(new SbpRelayMsgSsrOrbitClock(nh, state)));
  relays.push_back(SBPCallbackHandler::Ptr(new SbpRelayMsgSsrCodeBiases(nh, state)));
  relays.push_back(SBPCallbackHandler::Ptr(new SbpRelayMsgSsrPhaseBiases(nh, state)));
  relays.push_back(SBPCallbackHandler::Ptr(new SbpRelayMsgSsrStecCorrection(nh, state)));
  relays.push_back(SBPCallbackHandler::Ptr(new SbpRelayMsgSsrGriddedCorrectionNoStd(nh, state)));
  relays.push_back(SBPCallbackHandler::Ptr(new SbpRelayMsgSsrGriddedCorrection(nh, state)));
  relays.push_back(SBPCallbackHandler::Ptr(new SbpRelayMsgSsrGridDefinition(nh, state)));
  relays.push_back(SBPCallbackHandler::Ptr(new SbpRelayMsgStartup(nh, state)));
  relays.push_back(SBPCallbackHandler::Ptr(new SbpRelayMsgDgnssStatus(nh, state)));
  relays.push_back(SBPCallbackHandler::Ptr(new SbpRelayMsgHeartbeat(nh, state)));
  relays.push_back(SBPCallbackHandler::Ptr(new SbpRelayMsgInsStatus(nh, state)));
  relays.push_back(SBPCallbackHandler::Ptr(new SbpRelayMsgCsacTelemetry(nh, state)));
  relays.push_back(SBPCallbackHandler::Ptr(new SbpRelayMsgCsacTelemetryLabels(nh, state)));
  relays.push_back(SBPCallbackHandler::Ptr(new SbpRelayMsgTrackingState(nh, state)));
  relays.push_back(SBPCallbackHandler::Ptr(new SbpRelayMsgMeasurementState(nh, state)));
  relays.push_back(SBPCallbackHandler::Ptr(new SbpRelayMsgTrackingIq(nh, state)));
  relays.push_back(SBPCallbackHandler::Ptr(new SbpRelayMsgUserData(nh, state)));
  relays.push_back(SBPCallbackHandler::Ptr(new SbpRelayMsgOdometry(nh, state)));
  relays.push_back(SBPCallbackHandler::Ptr(new SbpRelayMsgWheeltick(nh, state)));
  
  // Remove all invalid (nullptr) callbacks.
  relays.erase(std::remove_if(
  relays.begin(), relays.end(),
  [](const SBPCallbackHandler::Ptr& relay) { return relay.get() == nullptr; }));

  return relays;
}

} // namespace piksi_multi_cpp

#endif  // LIBSBP_ROS_MSGS_SBP_RELAYS_H_