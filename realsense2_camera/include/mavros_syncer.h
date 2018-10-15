#ifndef REALSENSE2_CAMERA_MAVROS_SYNCER_H
#define REALSENSE2_CAMERA_MAVROS_SYNCER_H

#include "ros/ros.h"
#include <mavros_msgs/CamIMUStamp.h>
#include <mavros_msgs/CommandTriggerControl.h>
#include <mavros_msgs/CommandTriggerInterval.h>
#include <sensor_msgs/Image.h>
#include <geometry_msgs/PointStamped.h>
#include <mutex>
#include <tuple>

// Note on multi threading:
//      To avoid any confusion and non-defined state, this class locks a mutex for every function call
//      that is not const. This is due to the fact that many callbacks can happen simultaneously, especially
//      on callback based drivers such as the realsense. If not locked properly, this can lead to weird states.


namespace mavros_syncer {
enum sync_state {
    synced = 1,
    not_initalized,
    wait_for_sync,   // MAVROS reset sent, but no image/timestamps correlated yet.
};

template<typename t_chanel_id, typename t_cache>
class MavrosSyncer {

    // callback definition for processing buffered frames (with type t_cache).
    typedef boost::function<void(const t_chanel_id &channel,
                                                             const ros::Time &new_stamp,
                                                             const std::shared_ptr<t_cache> &cal)> caching_callback;

    // internal representation of a buffered frame
    // t_cache is the external representation
    typedef struct {
        uint32_t seq;
        ros::Time old_stamp;
        std::shared_ptr<t_cache> frame;
        double exposure;
    } frame_buffer_type;

 public:

    MavrosSyncer(const std::set<t_chanel_id> &channel_set) :
            channel_set_(channel_set),
            state_(not_initalized) {
        ROS_DEBUG_STREAM(log_prefix_ << " Initialized with " << channel_set_.size() << " channels.");
        for (t_chanel_id id : channel_set_) {
            trigger_buffer_map_[id].clear();
        }
    }

    void setup(const caching_callback &callback, int fps) {
        std::lock_guard<std::mutex> lg(mutex_);

        trigger_sequence_offset_ = 0;
        delay_pub_ = nh_.advertise<geometry_msgs::PointStamped>("delay", 100);
        state_ = not_initalized;
        restamp_callback_ = callback;
        cam_imu_sub_ = nh_.subscribe("/mavros/cam_imu_sync/cam_imu_stamp", 100,
                                     &MavrosSyncer::triggerStampCallback, this);

        const std::string mavros_trig_control_srv = "/mavros/cmd/trigger_control";
        const std::string mavros_trig_interval_srv = "/mavros/cmd/trigger_interval";
        if (ros::service::exists(mavros_trig_control_srv, false) && 
                ros::service::exists(mavros_trig_interval_srv, false)) {
            
            // disable trigger until triggering is started
            mavros_msgs::CommandTriggerControl req_control;
            req_control.request.trigger_enable = false;
            req_control.request.sequence_reset = true;
            req_control.request.trigger_pause = false;
            ros::service::call(mavros_trig_control_srv, req_control);

            // set trigger cycle time
            mavros_msgs::CommandTriggerInterval req_interval;
            req_interval.request.cycle_time = 1000.0/fps;
            req_interval.request.integration_time = -1.0;
            ros::service::call(mavros_trig_interval_srv, req_interval);

            ROS_INFO("Set mavros trigger interval to %f! Success? %d Result? %d",
                             1000.0/fps, req_interval.response.success, req_interval.response.result);
        } else {
            ROS_ERROR("Mavros service not available!");
        }

        ROS_DEBUG_STREAM(log_prefix_ << " Callback set and subscribed to cam_imu_stamp");
    }

    void start() {
        std::lock_guard<std::mutex> lg(mutex_);

        if (state_ != not_initalized) {
            //already started, ignore
            return;
        }

        for (t_chanel_id id : channel_set_) {
            trigger_buffer_map_[id].clear();
            frame_buffer_[id].frame.reset();
        }

        trigger_sequence_offset_ = 0;

        // Reset sequence number and enable triggering
        const std::string mavros_trig_control_srv = "/mavros/cmd/trigger_control";
        mavros_msgs::CommandTriggerControl req_enable;
        req_enable.request.trigger_enable = true;
        req_enable.request.sequence_reset = true;
        req_enable.request.trigger_pause = false;
        ros::service::call(mavros_trig_control_srv, req_enable);
        
        ROS_INFO("Called mavros trigger service! Success? %d Result? %d",
                         req_enable.response.success, req_enable.response.result);

        ROS_INFO_STREAM(log_prefix_ << " Started triggering.");

        state_ = wait_for_sync;
    }

    bool channelValid(const t_chanel_id &channel) const {
        return channel_set_.count(channel) == 1;
    }

    void cacheFrame(const t_chanel_id &channel, const uint32_t seq, const ros::Time &original_stamp, double exposure,
                                    const std::shared_ptr<t_cache> frame) {

        if (!channelValid(channel)) {
            ROS_WARN_STREAM_ONCE(log_prefix_ << "cacheFrame called for invalid channel.");
            return;
        }

        if(frame_buffer_[channel].frame){
            ros::spinOnce();
        }

        if (frame_buffer_[channel].frame) {
            ROS_WARN_STREAM_THROTTLE(1, log_prefix_ << 
                "Overwriting image queue! Make sure you're getting Timestamps from mavros.");
            restamp_callback_(channel, frame_buffer_[channel].old_stamp, frame_buffer_[channel].frame);
        }

        // set frame buffer
        frame_buffer_[channel].frame = frame;
        frame_buffer_[channel].old_stamp = original_stamp;
        frame_buffer_[channel].seq = seq;
        frame_buffer_[channel].exposure = exposure;
        ROS_INFO_STREAM(log_prefix_ << "Buffered frame, seq: " << seq);
    }

    bool syncOffset(const t_chanel_id &channel, const uint32_t seq, const ros::Time &old_stamp) {
        // Get the first from the sequence time map.
        auto it = trigger_buffer_map_[channel].rbegin();
        int32_t mavros_sequence = it->first;

        // Get offset between first frame sequence and mavros
        trigger_sequence_offset_ = mavros_sequence - static_cast<int32_t>(seq);

        double delay = old_stamp.toSec() - it->second.toSec();


        ROS_INFO(
                "%sNew header offset determined by channel %i: %d, from %d to %d, timestamp "
                "correction: %f seconds.",
                log_prefix_.c_str(), channel.first,
                trigger_sequence_offset_, it->first, seq,
                delay);

        frame_buffer_[channel].frame.reset();
        trigger_buffer_map_[channel].clear();

        state_ = synced;
        return true;

    }

    bool lookupTriggerStamp(const t_chanel_id &channel, const uint32_t frame_seq,
                            const ros::Time &old_stamp, double exposure, 
                            ros::Time *new_stamp) {
        // Function to match an incoming frame to a buffered trigger
        // Return true to publish frame, return false to buffer frame

        std::lock_guard<std::mutex> lg(mutex_);

        if (!channelValid(channel)) {
            return true; // return and publish image with unchanged stamp if not on synced stream
        }

        ROS_INFO_STREAM(log_prefix_ << "Received frame with stamp: " <<
                        std::setprecision(16) <<
                        old_stamp.toSec() << 
                        ", for seq nr: " << frame_seq <<
                        ", syncState: " << state_);

        if (state_ == not_initalized) {
            return true; // publish directly when not initialized (no caching)
        }

        if (trigger_buffer_map_[channel].empty()) {
            return false; // do not publish and buffer frame if empty trigger buffer
        }

        const double kMaxExpectedDelay = 5e-3;
        const double age_cached_trigger = old_stamp.toSec() - trigger_buffer_map_[channel].rbegin()->second.toSec();
        if (age_cached_trigger > kMaxExpectedDelay) {
            ROS_WARN_STREAM(log_prefix_ << "Delay out of bounds: Cached trigger is older than "
                                            << kMaxExpectedDelay << " seconds. Clearing trigger buffer...");
            frame_buffer_[channel].frame.reset();
            trigger_buffer_map_[channel].clear();
            state_ = wait_for_sync;
            return false; // do not publish and buffer frame if waiting to sync all buffered triggers are to old
        }

        if (state_ == wait_for_sync) {
            syncOffset(channel, frame_seq, old_stamp);
            return true; // publish directly and wait for seq offset sync
        }

        uint32_t trigger_seq = frame_seq + trigger_sequence_offset_;
        auto it = trigger_buffer_map_[channel].find(trigger_seq);

        // if we haven't found the sequence
        if (it == trigger_buffer_map_[channel].end()) {
            // cached trigger is within the expected delay but does not match the expected seq nr
            // call syncOffset()
            ROS_WARN_STREAM(log_prefix_ << "Could not find trigger for seq: " <<  trigger_seq);
            syncOffset(channel, frame_seq, old_stamp);
            return false; // publish directly and wait for seq offset sync
        }

        *new_stamp = it->second;
        *new_stamp = shiftTimestampToMidExposure(*new_stamp, exposure);
        trigger_buffer_map_[channel].clear();

        const double delay = age_cached_trigger;
        ROS_INFO_STREAM(log_prefix_ << "Matched frame to trigger: t" << trigger_seq << " -> c" << frame_seq <<
                        ", t_old " <<  std::setprecision(15) << old_stamp.toSec() << " -> t_new " << new_stamp->toSec() 
                        << std::setprecision(7) << " ~ " << delay);

        geometry_msgs::PointStamped msg;
        msg.header.stamp = *new_stamp;
        msg.point.x = delay;
        delay_pub_.publish(msg);

        return true;
    }


    bool lookupFrame(const t_chanel_id &channel, const uint32_t trigger_seq, 
                     const ros::Time &new_stamp, const ros::Time &old_stamp) {
        // Function to match an incoming trigger to a buffered frame
        // Return true to publish frame, return false to buffer frame

        if (state_ == wait_for_sync) {
            return false; // do nothing if seq offset is not yet determined
        }

        uint32_t synced_seq = trigger_seq - trigger_sequence_offset_;
        if (frame_buffer_[channel].seq != synced_seq) {
            // cached frame is within the expected delay but does not match the expected seq nr
            // return false in order to call syncOffset()
            ROS_WARN_STREAM(log_prefix_ << "Could not find frame for seq: " << synced_seq);
            return  false;
        }

        if (!restamp_callback_) {
            ROS_WARN_STREAM_THROTTLE(10, log_prefix_ << " No callback set - discarding buffered images.");
            frame_buffer_[channel].frame.reset();
            return false;
        }

        // successfully matched frame to cached trigger
        restamp_callback_(channel, new_stamp, frame_buffer_[channel].frame);
        
        // calc delay between mavros stamp and frame stamp
        const double delay = old_stamp.toSec() - new_stamp.toSec();
        ROS_INFO_STREAM(log_prefix_ << "Matched trigger to frame: t" << trigger_seq << " -> c" << synced_seq <<
                        ", t_old " <<  std::setprecision(15) << old_stamp.toSec() << " -> t_new " << new_stamp.toSec() 
                        << std::setprecision(7) << " ~ " << delay);

        geometry_msgs::PointStamped msg;
        msg.header.stamp = new_stamp;
        msg.point.x = delay;
        delay_pub_.publish(msg);
        frame_buffer_[channel].frame.reset();

        return true;
    }

    ros::Time shiftTimestampToMidExposure(const ros::Time &stamp,double exposure_us) {
        ros::Time new_stamp = stamp
                            + ros::Duration(exposure_us * 1e-6 / 2.0)
                            + ros::Duration(kalibr_time_offset_ * 1e-3);
        return new_stamp;
    }

    void triggerStampCallback(const mavros_msgs::CamIMUStamp &cam_imu_stamp) {

        if (state_ == not_initalized) {
            // Do nothing before triggering is setup and initialized
            return;
        }

        ROS_INFO_STREAM(log_prefix_ << "Received trigger stamp   : " <<
                std::setprecision(16) <<
                cam_imu_stamp.frame_stamp.toSec() <<
                ", for seq nr: " << cam_imu_stamp.frame_seq_id <<
                " (synced_seq: " << cam_imu_stamp.frame_seq_id-trigger_sequence_offset_ << ")");

        for (auto channel : channel_set_) {

            if (!frame_buffer_[channel].frame) {
                // trigger_buffer_map_[channel].clear();
                trigger_buffer_map_[channel][cam_imu_stamp.frame_seq_id] = cam_imu_stamp.frame_stamp;
                return;
            }

            const double kMaxExpectedDelay = 5e-3;
            const double age_cached_frame = cam_imu_stamp.frame_stamp.toSec() - frame_buffer_[channel].old_stamp.toSec();
            
            if (age_cached_frame > kMaxExpectedDelay) {
                // buffered frame is too old. release buffered frame
                ROS_WARN_STREAM(log_prefix_ << "Delay out of bounds: Cached frame is older than "
                                << kMaxExpectedDelay << " seconds. Releasing buffered frame...");
                
                restamp_callback_(channel, frame_buffer_[channel].old_stamp, frame_buffer_[channel].frame);

                frame_buffer_[channel].frame.reset();
                trigger_buffer_map_[channel].clear();
                trigger_buffer_map_[channel][cam_imu_stamp.frame_seq_id] = cam_imu_stamp.frame_stamp; 
                return;
            }

            if (!lookupFrame(channel, cam_imu_stamp.frame_seq_id, 
                             cam_imu_stamp.frame_stamp, frame_buffer_[channel].old_stamp)) {
                // waiting for sync or
                // OR 
                // seq numbers did not match: sync offsets
                trigger_buffer_map_[channel][cam_imu_stamp.frame_seq_id] = cam_imu_stamp.frame_stamp;
                syncOffset(channel, frame_buffer_[channel].seq, frame_buffer_[channel].old_stamp);
                return;
            }

            // synced: matched, re-stamped and published frame
        }
        return;
    }

 private:
    ros::NodeHandle nh_;

    const std::set<t_chanel_id> channel_set_;
    int trigger_sequence_offset_ = 0;

    ros::Subscriber cam_imu_sub_;
    ros::Publisher delay_pub_;
    std::map<t_chanel_id, std::map<uint32_t, ros::Time>> trigger_buffer_map_;
    std::mutex mutex_;

    caching_callback restamp_callback_;
    sync_state state_;

    std::map<t_chanel_id, std::string> logging_name_;
    std::map<t_chanel_id, frame_buffer_type> frame_buffer_;

    const std::string log_prefix_ = "[Mavros Triggering] ";
};

}

#endif //REALSENSE2_CAMERA_MAVROS_SYNCER_H
