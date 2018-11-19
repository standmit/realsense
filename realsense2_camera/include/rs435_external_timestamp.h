#ifndef REALSENSE2_CAMERA_RS435_EXTERNAL_TIMESTAMP_H
#define REALSENSE2_CAMERA_RS435_EXTERNAL_TIMESTAMP_H

#include "ros/ros.h"
#include <std_msgs/Header.h>
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
            _channel_set(channel_set),
            _state(sync_state::not_initalized) {
        ROS_DEBUG_STREAM(_log_prefix << " Initialized with " << _channel_set.size() << " channels.");
        for (t_chanel_id channel : _channel_set) {
            _hw_stamp_buffer[channel].reset();
        }
    }

    void setup(const pub_frame_fn &pub_function_ptr, int fps, 
               double static_time_offset, int inter_cam_sync_mode) {

        _inter_cam_sync_mode = inter_cam_sync_mode;
        _static_time_offset = static_time_offset;
        _publish_frame_fn = pub_function_ptr;
        _frame_rate = fps;

        _hw_stamp_seq_offset = 0;
        _state = sync_state::not_initalized;

        _cam_imu_sub = nh_.subscribe("/hw_stamp", 100, &ExternalTimestamp::hwStampCallback, this);
    }

    void start() {
        std::lock_guard<std::mutex> lg(_mutex);

        if (_state != sync_state::not_initalized) {
            //already started, ignore
            return;
        }

        for (t_chanel_id channel : _channel_set) {
            // clear buffers in case start() is invoked for re-initialization
            _hw_stamp_buffer[channel].reset();
            _frame_buffer[channel].frame.reset();
        }

        _state = sync_state::wait_for_sync;
    }

    bool channelValid(const t_chanel_id &channel) const {
        return _channel_set.count(channel) == 1;
    }

    void bufferFrame(const t_chanel_id &channel, const uint32_t seq, const ros::Time &cam_stamp, 
                    double exposure, const std::shared_ptr<t_frame_buffer> frame) {

        if (!channelValid(channel)) {
            ROS_WARN_STREAM_ONCE(_log_prefix << "bufferFrame called for invalid channel.");
            return;
        }

        if(_frame_buffer[channel].frame){
            ros::spinOnce();
        }

        if (_frame_buffer[channel].frame) {
            ROS_WARN_STREAM(_log_prefix << 
                "Overwriting image buffer! Make sure you're getting Timestamps from mavros.");
        }

        // set frame buffer
        _frame_buffer[channel].frame = frame;
        _frame_buffer[channel].seq = seq;
        _frame_buffer[channel].arrival_stamp = ros::Time::now();
        _frame_buffer[channel].cam_stamp = cam_stamp; //store stamp that was constructed by ros-realsense
        _frame_buffer[channel].exposure = exposure;
        ROS_DEBUG_STREAM(_log_prefix << "Buffered frame, seq: " << seq);
    }

    bool syncOffset(const t_chanel_id &channel, const uint32_t seq_frame, const double &delay) {
        // no lock_guard as this function is only called within locked scopes

        // Get offset between first frame sequence and mavros
        _hw_stamp_seq_offset = _hw_stamp_buffer[channel].seq - static_cast<int32_t>(seq_frame);

        ROS_INFO(
                "%sNew header offset determined by channel %i: %d, from %d to %d, timestamp "
                "correction: %f seconds.",
                _log_prefix.c_str(), channel.first, _hw_stamp_seq_offset,
                _hw_stamp_buffer[channel].seq, seq_frame, delay);

        _frame_buffer[channel].frame.reset();
        _hw_stamp_buffer[channel].reset();

        _state = sync_state::synced;
        return true;
    }

    bool lookupHardwareStamp(const t_chanel_id &channel, const uint32_t frame_seq,
                            const ros::Time &cam_stamp, double exposure, std::shared_ptr<t_frame_buffer> frame) {
        // Function to match an incoming frame to a buffered trigger
        // Takes a trigger stamp that was initialized equally as the old frame stamp
        // if a matching trigger was found the trigger_stamp is overwritten and the frame is published
        // if no matching trigger is found we return without changing trigger_stamp
        std::lock_guard<std::mutex> lg(_mutex);

        if (!channelValid(channel)) {
            return false;
        }

        ROS_INFO_STREAM(_log_prefix << "Received frame with stamp: " <<
                        std::setprecision(15) <<
                        cam_stamp.toSec() << 
                        " rn: " << ros::Time::now().toSec() <<
                        ", for seq nr: " << frame_seq);

        if (_state == sync_state::not_initalized) {
            return false;
        }

        if (_hw_stamp_buffer[channel].seq == 0) {
            return false;
        }

        const double kMaxStampAge = 1.0/_frame_rate;
        const double age_buffered_hw_stamp = cam_stamp.toSec() - _hw_stamp_buffer[channel].arrival_stamp.toSec();
        
        if (std::fabs(age_buffered_hw_stamp) > kMaxStampAge) {
            ROS_WARN_STREAM(_log_prefix << "Delay out of bounds: " << 
                            kMaxStampAge << " seconds. Clearing hw stamp buffer...");
            _frame_buffer[channel].frame.reset();
            _hw_stamp_buffer[channel].reset();
            _state = sync_state::wait_for_sync;
            return false;
        }

        if (_state == sync_state::wait_for_sync) {
            syncOffset(channel, frame_seq, age_buffered_hw_stamp);
            return false;
        }

        uint32_t expected_hw_stamp_seq = frame_seq + _hw_stamp_seq_offset;

        // if we cannot assign a matching stamp
        if (_hw_stamp_buffer[channel].seq != expected_hw_stamp_seq) {
            // cached hw stamp is within the expected delay but does not match the expected seq nr
            // call syncOffset()
            ROS_WARN_STREAM(_log_prefix << "Could not find hw stamp for seq: " <<  expected_hw_stamp_seq);
            syncOffset(channel, frame_seq, age_buffered_hw_stamp);
            return false;
        }

        // successful match: shift stamp and publish frame
        ros::Time hw_stamp = _hw_stamp_buffer[channel].hw_stamp;
        hw_stamp = shiftTimestampToMidExposure(hw_stamp, exposure);
        _publish_frame_fn(channel, hw_stamp, frame);

        ROS_INFO_STREAM(_log_prefix << "Matched frame to trigger: t" << expected_hw_stamp_seq << " -> c" << frame_seq <<
                        ", t_old " <<  std::setprecision(15) << cam_stamp.toSec() << " -> t_new " << hw_stamp.toSec() 
                        << std::setprecision(7) << " ~ " << age_buffered_hw_stamp);
        _hw_stamp_buffer[channel].reset();

        return true;
    }


    bool lookupFrame(const t_chanel_id channel, const uint32_t hw_stamp_seq, 
                     ros::Time &hw_stamp, const ros::Time &arrival_stamp) {
        // Function to match an incoming trigger to a buffered frame
        // Return true to publish frame, return false to buffer frame

        if (!_publish_frame_fn) {
            ROS_ERROR_STREAM(_log_prefix << " No callback set - discarding buffered images.");
            return false;
        }

        if (!_frame_buffer[channel].frame) {
            return false;
        }

        const double kMaxFrameAge = 0e-3;
        const double age_buffered_frame = arrival_stamp.toSec() 
                                        - _frame_buffer[channel].arrival_stamp.toSec(); 
        if (std::fabs(age_buffered_frame) > kMaxFrameAge) {
            // buffered frame is too old. release buffered frame
            ROS_WARN_STREAM(_log_prefix << "Delay out of bounds:  "
                            << kMaxFrameAge << " seconds. Releasing buffered frame...");
            return false;
        }

        if (_state == sync_state::wait_for_sync) {
            syncOffset(channel, _frame_buffer[channel].seq, age_buffered_frame);
            return false;
        }

        uint32_t expected_frame_seq = hw_stamp_seq - _hw_stamp_seq_offset;
        if (_frame_buffer[channel].seq != expected_frame_seq) {
            // cached frame is within the expected delay but does not match the expected seq nr
            // return false in order to call syncOffset()
            ROS_WARN_STREAM(_log_prefix << "Could not find frame for seq: " << expected_frame_seq);
            return  false;
        }

        // successful match: shift stamp and publish frame
        hw_stamp = shiftTimestampToMidExposure(hw_stamp, _frame_buffer[channel].exposure);
        _publish_frame_fn(channel, hw_stamp, _frame_buffer[channel].frame);
        
        ROS_INFO_STREAM(_log_prefix << "Matched trigger to frame: t" << hw_stamp_seq << " -> c" << expected_frame_seq <<
                        ", t_old " << _frame_buffer[channel].cam_stamp.toSec() << 
                        " -> t_new " << hw_stamp.toSec() << " ~ " << age_buffered_frame);
        _frame_buffer[channel].frame.reset();

        return true;
    }

    ros::Time shiftTimestampToMidExposure(const ros::Time &stamp, double exposure_us) {
        ros::Time new_stamp = stamp
                            + ros::Duration(exposure_us * 1e-6 / 2.0)
                            + ros::Duration(_static_time_offset);
        ROS_DEBUG_STREAM(_log_prefix << "Shift timestamp: " << stamp.toSec() << " -> " << 
                         new_stamp.toSec() << " exposure: " << exposure_us * 1e-6);
        return new_stamp;
    }

    void hwStampCallback(const std_msgs::Header &header) {

        if (_state == sync_state::not_initalized) {
            // Do nothing before triggering is setup and initialized
            return;
        }

        ros::Time arrival_stamp = ros::Time::now();
        ros::Time hw_stamp = header.stamp;
        uint32_t hw_stamp_seq  = header.seq;

        ROS_INFO_STREAM(_log_prefix << "Received hw stamp   : " <<
                std::setprecision(15) <<
                hw_stamp.toSec() <<
                " rn: " << ros::Time::now().toSec() <<
                ", for seq nr: " << hw_stamp_seq <<
                " (synced_seq: " << hw_stamp_seq-_hw_stamp_seq_offset << ")");

        for (auto channel : _channel_set) {

            if (!lookupFrame(channel, hw_stamp_seq, hw_stamp, arrival_stamp)) {
                // buffer hw_stamp if lookupFrame returns false
                _frame_buffer[channel].frame.reset();
                _hw_stamp_buffer[channel].seq = hw_stamp_seq;
                _hw_stamp_buffer[channel].arrival_stamp = arrival_stamp;
                _hw_stamp_buffer[channel].hw_stamp = hw_stamp;
            }
            // synced: matched, re-stamped and published frame
        }
    }

 private:
    ros::NodeHandle nh_;

    const std::set<t_chanel_id> _channel_set;
    int _hw_stamp_seq_offset = 0;
    double _static_time_offset;
    int _inter_cam_sync_mode;
    int _frame_rate;

    ros::Subscriber _cam_imu_sub;
    
    std::mutex _mutex;

    pub_frame_fn _publish_frame_fn;
    sync_state _state;

    const std::string _log_prefix = "[HW timesync] ";
    std::map<t_chanel_id, hw_stamp_buffer_type> _hw_stamp_buffer;
    std::map<t_chanel_id, frame_buffer_type> _frame_buffer;

};

}

#endif //REALSENSE2_CAMERA_RS435_EXTERNAL_TIMESTAMP_H
