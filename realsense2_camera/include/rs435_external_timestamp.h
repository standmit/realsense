#ifndef REALSENSE2_CAMERA_RS435_EXTERNAL_TIMESTAMP_H
#define REALSENSE2_CAMERA_RS435_EXTERNAL_TIMESTAMP_H

#include "ros/ros.h"
#include <std_msgs/Header.h>
#include <geometry_msgs/PointStamped.h>
#include <mutex>
#include <tuple>

namespace external_timestamp {
enum class sync_state {
    synced = 1,
    not_initalized,
    wait_for_sync,
};

enum class inter_cam_sync_mode {
    none = 0,
    master,
    slave
};

template<typename t_chanel_id, typename t_frame_buffer>
class ExternalTimestamp {
    // callback definition for processing buffered frames
    typedef boost::function<void(const t_chanel_id &channel,
                                 const ros::Time &stamp,
                                 const std::shared_ptr<t_frame_buffer> &cal)> pub_frame_fn;

    // internal representation of a buffered frame
    // t_frame_buffer is the external representation
    typedef struct {
        uint32_t seq;
        ros::Time cam_stamp;
        ros::Time arrival_stamp;
        std::shared_ptr<t_frame_buffer> frame;
        double exposure;
    } frame_buffer_type;

    typedef struct {
        uint32_t seq;
        ros::Time hw_stamp;
        ros::Time arrival_stamp;
        void reset() {
            seq = 0;
        }
    } hw_stamp_buffer_type;


 public:

    ExternalTimestamp(const std::set<t_chanel_id> &channel_set) :
            channel_set_(channel_set),
            state_(sync_state::not_initalized) {
        ROS_DEBUG_STREAM(log_prefix_ << " Initialized with " << channel_set_.size() << " channels.");
        for (t_chanel_id channel : channel_set_) {
            hw_stamp_buffer_[channel].reset();
        }
    }

    void setup(const pub_frame_fn &pub_function_ptr, int fps, 
               double static_time_offset, int inter_cam_sync_mode) {

        inter_cam_sync_mode_ = inter_cam_sync_mode;
        static_time_offset_ = static_time_offset;
        publish_frame_fn_ = pub_function_ptr;
        frame_rate_ = fps;

        hw_stamp_seq_offset_ = 0;
        state_ = sync_state::not_initalized;

        cam_imu_sub_ = nh_.subscribe("/hw_stamp", 100, &ExternalTimestamp::hwStampCallback, this);
        delay_pub_ = nh_.advertise<geometry_msgs::PointStamped>("/camera/mavros_restamping_info", 1);

        // const std::string mavros_trig_control_srv = "/mavros/cmd/trigger_control";
        // const std::string mavros_trig_interval_srv = "/mavros/cmd/trigger_interval";
        // if (inter_cam_sync_mode == inter_cam_sync_mode::slave) {
        //     // setup camera triggering on the fc
        //     if (ros::service::exists(mavros_trig_control_srv, false) && 
        //         ros::service::exists(mavros_trig_interval_srv, false)) {

        //         // disable trigger until triggering is started
        //         mavros_msgs::CommandTriggerControl req_control;
        //         req_control.request.trigger_enable = false;
        //         req_control.request.sequence_reset = true;
        //         req_control.request.trigger_pause = false;
        //         ros::service::call(mavros_trig_control_srv, req_control);

        //         // set trigger cycle time
        //         mavros_msgs::CommandTriggerInterval req_interval;
        //         req_interval.request.cycle_time = 1000.0/frame_rate_;
        //         req_interval.request.integration_time = -1.0;
        //         ros::service::call(mavros_trig_interval_srv, req_interval);

        //         ROS_INFO("Set mavros trigger interval to %f! Success? %d Result? %d",
        //                          1000.0/frame_rate_, req_interval.response.success, req_interval.response.result);
        //     } else {
        //         ROS_ERROR("Mavros service for trigger setup not available!");
        //     }
        // }
    }

    void start() {
        std::lock_guard<std::mutex> lg(mutex_);

        if (state_ != sync_state::not_initalized) {
            //already started, ignore
            return;
        }

        for (t_chanel_id channel : channel_set_) {
            // clear buffers in case start() is invoked for re-initialization
            hw_stamp_buffer_[channel].reset();
            frame_buffer_[channel].frame.reset();
        }

        // Reset sequence number and enable triggering
        // hw_stamp_seq_offset_ = 0;
        // if (inter_cam_sync_mode == inter_cam_sync_mode::slave) { 
        //     const std::string mavros_trig_control_srv = "/mavros/cmd/trigger_control";
        //     mavros_msgs::CommandTriggerControl req_enable;
        //     req_enable.request.trigger_enable = true;
        //     req_enable.request.sequence_reset = true;
        //     req_enable.request.trigger_pause = false;
        //     ros::service::call(mavros_trig_control_srv, req_enable);

        //     ROS_INFO_STREAM(log_prefix_ << " Started triggering.");
        // }

        state_ = sync_state::wait_for_sync;
    }

    bool channelValid(const t_chanel_id &channel) const {
        return channel_set_.count(channel) == 1;
    }

    void cacheFrame(const t_chanel_id &channel, const uint32_t seq, const ros::Time &cam_stamp, 
                    double exposure, const std::shared_ptr<t_frame_buffer> frame) {

        if (!channelValid(channel)) {
            ROS_WARN_STREAM_ONCE(log_prefix_ << "cacheFrame called for invalid channel.");
            return;
        }

        if(frame_buffer_[channel].frame){
            ros::spinOnce();
        }

        if (frame_buffer_[channel].frame) {
            ROS_WARN_STREAM(log_prefix_ << 
                "Overwriting image buffer! Make sure you're getting Timestamps from mavros.");
        }

        // set frame buffer
        frame_buffer_[channel].frame = frame;
        frame_buffer_[channel].seq = seq;
        frame_buffer_[channel].arrival_stamp = ros::Time::now();
        frame_buffer_[channel].cam_stamp = cam_stamp; //store stamp that was constructed by ros-realsense
        frame_buffer_[channel].exposure = exposure;
        ROS_DEBUG_STREAM(log_prefix_ << "Buffered frame, seq: " << seq);
    }

    bool syncOffset(const t_chanel_id &channel, const uint32_t seq_frame, const ros::Time &cam_stamp) {
        // no lock_guard as this function is only called within locked scopes

        // Get offset between first frame sequence and mavros
        hw_stamp_seq_offset_ = hw_stamp_buffer_[channel].seq - static_cast<int32_t>(seq_frame);

        double delay = cam_stamp.toSec() - hw_stamp_buffer_[channel].arrival_stamp.toSec();

        ROS_INFO(
                "%sNew header offset determined by channel %i: %d, from %d to %d, timestamp "
                "correction: %f seconds.",
                log_prefix_.c_str(), channel.first, hw_stamp_seq_offset_,
                hw_stamp_buffer_[channel].seq, seq_frame, delay);

        frame_buffer_[channel].frame.reset();
        hw_stamp_buffer_[channel].reset();

        state_ = sync_state::synced;
        return true;
    }

    bool lookupHardwareStamp(const t_chanel_id &channel, const uint32_t frame_seq,
                            const ros::Time &cam_stamp, double exposure, 
                            ros::Time &hw_stamp) {
        // Function to match an incoming frame to a buffered trigger
        // Takes a trigger stamp that was initialized equally as the old frame stamp
        // if a matching trigger was found the trigger_stamp is overwritten and the frame is published
        // if no matching trigger is found we return without changing trigger_stamp
        std::lock_guard<std::mutex> lg(mutex_);

        if (!channelValid(channel)) {
            return false;
        }

        ROS_INFO_STREAM(log_prefix_ << "Received frame with stamp: " <<
                        std::setprecision(15) <<
                        cam_stamp.toSec() << 
                        " rn: " << ros::Time::now().toSec() <<
                        ", for seq nr: " << frame_seq);

        if (state_ == sync_state::not_initalized) {
            return false;
        }

        if (hw_stamp_buffer_[channel].seq == 0) {
            return false;
        }

        const double kMaxStampAge = 1.0/frame_rate_;
        const double age_cached_hw_stamp = cam_stamp.toSec() - hw_stamp_buffer_[channel].arrival_stamp.toSec();
        
        if (std::fabs(age_cached_hw_stamp) > kMaxStampAge) {
            ROS_WARN_STREAM(log_prefix_ << "Delay out of bounds: "
                                            << kMaxStampAge << " seconds. Clearing hw stamp buffer...");
            frame_buffer_[channel].frame.reset();
            hw_stamp_buffer_[channel].reset();
            state_ = sync_state::wait_for_sync;
            return false;
        }

        if (state_ == sync_state::wait_for_sync) {
            syncOffset(channel, frame_seq, cam_stamp);
            return false;
        }

        uint32_t expected_hw_stamp_seq = frame_seq + hw_stamp_seq_offset_;

        // if we cannot assign a matching stamp
        if (hw_stamp_buffer_[channel].seq != expected_hw_stamp_seq) {
            // cached hw stamp is within the expected delay but does not match the expected seq nr
            // call syncOffset()
            ROS_WARN_STREAM(log_prefix_ << "Could not find hw stamp for seq: " <<  expected_hw_stamp_seq);
            syncOffset(channel, frame_seq, cam_stamp);
            return false;
        }

        hw_stamp = hw_stamp_buffer_[channel].hw_stamp;
        hw_stamp = shiftTimestampToMidExposure(hw_stamp, exposure);
        hw_stamp_buffer_[channel].reset();

        const double interval = hw_stamp.toSec() - prev_stamp_.toSec();
        prev_stamp_ = hw_stamp;
        ROS_INFO_STREAM(log_prefix_ << "Matched frame to trigger: t" << expected_hw_stamp_seq << " -> c" << frame_seq <<
                        ", t_old " <<  std::setprecision(15) << cam_stamp.toSec() << " -> t_new " << hw_stamp.toSec() 
                        << std::setprecision(7) << " ~ " << age_cached_hw_stamp);

        geometry_msgs::PointStamped msg;
        msg.header.stamp = hw_stamp;
        msg.point.x = age_cached_hw_stamp;
        msg.point.y = interval;
        delay_pub_.publish(msg);

        return true;
    }


    bool lookupFrame(const t_chanel_id channel, const uint32_t hw_stamp_seq, 
                     ros::Time &hw_stamp, const ros::Time &arrival_stamp) {
        // Function to match an incoming trigger to a buffered frame
        // Return true to publish frame, return false to buffer frame

        if (!publish_frame_fn_) {
            ROS_ERROR_STREAM(log_prefix_ << " No callback set - discarding buffered images.");
            return false;
        }

        if (!frame_buffer_[channel].frame) {
            return false;
        }

        const double kMaxFrameAge = 0e-3;
        const double age_buffered_frame = arrival_stamp.toSec() 
                                        - frame_buffer_[channel].arrival_stamp.toSec(); 
        if (std::fabs(age_buffered_frame) > kMaxFrameAge) {
            // buffered frame is too old. release buffered frame
            ROS_WARN_STREAM(log_prefix_ << "Delay out of bounds:  "
                            << kMaxFrameAge << " seconds. Releasing buffered frame...");
            return false;
        }

        if (state_ == sync_state::wait_for_sync) {
            syncOffset(channel, frame_buffer_[channel].seq, frame_buffer_[channel].cam_stamp);
            return false;
        }

        uint32_t expected_frame_seq = hw_stamp_seq - hw_stamp_seq_offset_;
        if (frame_buffer_[channel].seq != expected_frame_seq) {
            // cached frame is within the expected delay but does not match the expected seq nr
            // return false in order to call syncOffset()
            ROS_WARN_STREAM(log_prefix_ << "Could not find frame for seq: " << expected_frame_seq);
            return  false;
        }

        // successfully matched trigger to buffered frame
        hw_stamp = shiftTimestampToMidExposure(hw_stamp, frame_buffer_[channel].exposure);
        publish_frame_fn_(channel, hw_stamp, frame_buffer_[channel].frame);
        
        // calc delay between mavros stamp and frame stamp
        const double delay = frame_buffer_[channel].cam_stamp.toSec() - arrival_stamp.toSec();
        const double interval = hw_stamp.toSec() - prev_stamp_.toSec();
        ROS_INFO_STREAM(log_prefix_ << "Matched trigger to frame: t" << hw_stamp_seq << " -> c" << expected_frame_seq <<
                        ", t_old " << frame_buffer_[channel].cam_stamp.toSec() << 
                        " -> t_new " << hw_stamp.toSec() << " ~ " << delay);

        geometry_msgs::PointStamped msg;
        msg.header.stamp = hw_stamp;
        msg.point.x = delay;
        msg.point.y = interval;
        delay_pub_.publish(msg);

        frame_buffer_[channel].frame.reset();
        prev_stamp_ = hw_stamp;
        return true;
    }

    ros::Time shiftTimestampToMidExposure(const ros::Time &stamp, double exposure_us) {
        ros::Time new_stamp = stamp
                            + ros::Duration(exposure_us * 1e-6 / 2.0)
                            + ros::Duration(static_time_offset_);
        ROS_DEBUG_STREAM(log_prefix_ << "Shift timestamp: " << stamp.toSec() << " -> " << 
                         new_stamp.toSec() << " exposure: " << exposure_us * 1e-6);
        return new_stamp;
    }

    void hwStampCallback(const std_msgs::Header &header) {

        if (state_ == sync_state::not_initalized) {
            // Do nothing before triggering is setup and initialized
            return;
        }

        ros::Time arrival_stamp = ros::Time::now();
        ros::Time hw_stamp = header.stamp;
        uint32_t hw_stamp_seq  = header.seq;

        ROS_INFO_STREAM(log_prefix_ << "Received hw stamp   : " <<
                std::setprecision(15) <<
                hw_stamp.toSec() <<
                " rn: " << ros::Time::now().toSec() <<
                ", for seq nr: " << hw_stamp_seq <<
                " (synced_seq: " << hw_stamp_seq-hw_stamp_seq_offset_ << ")");

        for (auto channel : channel_set_) {

            if (!lookupFrame(channel, hw_stamp_seq, hw_stamp, arrival_stamp)) {
                // buffer hw_stamp if lookupFrame returns false
                frame_buffer_[channel].frame.reset();
                hw_stamp_buffer_[channel].seq = hw_stamp_seq;
                hw_stamp_buffer_[channel].arrival_stamp = arrival_stamp;
                hw_stamp_buffer_[channel].hw_stamp = hw_stamp;
            }
            // synced: matched, re-stamped and published frame
        }
    }

 private:
    ros::NodeHandle nh_;

    const std::set<t_chanel_id> channel_set_;
    int hw_stamp_seq_offset_ = 0;
    double static_time_offset_;
    int inter_cam_sync_mode_;
    int frame_rate_;

    ros::Time prev_stamp_;

    ros::Subscriber cam_imu_sub_;
    ros::Publisher delay_pub_;
    
    std::mutex mutex_;

    pub_frame_fn publish_frame_fn_;
    sync_state state_;

    std::map<t_chanel_id, std::string> logging_name_;
    std::map<t_chanel_id, hw_stamp_buffer_type> hw_stamp_buffer_;
    std::map<t_chanel_id, frame_buffer_type> frame_buffer_;

    const std::string log_prefix_ = "[Mavros Triggering] ";
};

}

#endif //REALSENSE2_CAMERA_RS435_EXTERNAL_TIMESTAMP_H
