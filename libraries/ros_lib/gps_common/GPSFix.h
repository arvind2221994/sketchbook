#ifndef _ROS_gps_common_GPSFix_h
#define _ROS_gps_common_GPSFix_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "std_msgs/Header.h"
#include "gps_common/GPSStatus.h"

namespace gps_common
{

  class GPSFix : public ros::Msg
  {
    public:
      std_msgs::Header header;
      gps_common::GPSStatus status;
      float latitude;
      float longitude;
      float altitude;
      float track;
      float speed;
      float climb;
      float pitch;
      float roll;
      float dip;
      float time;
      float gdop;
      float pdop;
      float hdop;
      float vdop;
      float tdop;
      float err;
      float err_horz;
      float err_vert;
      float err_track;
      float err_speed;
      float err_climb;
      float err_time;
      float err_pitch;
      float err_roll;
      float err_dip;
      float position_covariance[9];
      uint8_t position_covariance_type;
      enum { COVARIANCE_TYPE_UNKNOWN =  0 };
      enum { COVARIANCE_TYPE_APPROXIMATED =  1 };
      enum { COVARIANCE_TYPE_DIAGONAL_KNOWN =  2 };
      enum { COVARIANCE_TYPE_KNOWN =  3 };

<<<<<<< HEAD
    GPSFix():
      header(),
      status(),
      latitude(0),
      longitude(0),
      altitude(0),
      track(0),
      speed(0),
      climb(0),
      pitch(0),
      roll(0),
      dip(0),
      time(0),
      gdop(0),
      pdop(0),
      hdop(0),
      vdop(0),
      tdop(0),
      err(0),
      err_horz(0),
      err_vert(0),
      err_track(0),
      err_speed(0),
      err_climb(0),
      err_time(0),
      err_pitch(0),
      err_roll(0),
      err_dip(0),
      position_covariance(),
      position_covariance_type(0)
    {
    }

=======
>>>>>>> 114c33badc7e157f5d09deb42c0831bdbf295d3c
    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      offset += this->header.serialize(outbuffer + offset);
      offset += this->status.serialize(outbuffer + offset);
<<<<<<< HEAD
      offset += serializeAvrFloat64(outbuffer + offset, this->latitude);
      offset += serializeAvrFloat64(outbuffer + offset, this->longitude);
      offset += serializeAvrFloat64(outbuffer + offset, this->altitude);
      offset += serializeAvrFloat64(outbuffer + offset, this->track);
      offset += serializeAvrFloat64(outbuffer + offset, this->speed);
      offset += serializeAvrFloat64(outbuffer + offset, this->climb);
      offset += serializeAvrFloat64(outbuffer + offset, this->pitch);
      offset += serializeAvrFloat64(outbuffer + offset, this->roll);
      offset += serializeAvrFloat64(outbuffer + offset, this->dip);
      offset += serializeAvrFloat64(outbuffer + offset, this->time);
      offset += serializeAvrFloat64(outbuffer + offset, this->gdop);
      offset += serializeAvrFloat64(outbuffer + offset, this->pdop);
      offset += serializeAvrFloat64(outbuffer + offset, this->hdop);
      offset += serializeAvrFloat64(outbuffer + offset, this->vdop);
      offset += serializeAvrFloat64(outbuffer + offset, this->tdop);
      offset += serializeAvrFloat64(outbuffer + offset, this->err);
      offset += serializeAvrFloat64(outbuffer + offset, this->err_horz);
      offset += serializeAvrFloat64(outbuffer + offset, this->err_vert);
      offset += serializeAvrFloat64(outbuffer + offset, this->err_track);
      offset += serializeAvrFloat64(outbuffer + offset, this->err_speed);
      offset += serializeAvrFloat64(outbuffer + offset, this->err_climb);
      offset += serializeAvrFloat64(outbuffer + offset, this->err_time);
      offset += serializeAvrFloat64(outbuffer + offset, this->err_pitch);
      offset += serializeAvrFloat64(outbuffer + offset, this->err_roll);
      offset += serializeAvrFloat64(outbuffer + offset, this->err_dip);
      for( uint8_t i = 0; i < 9; i++){
      offset += serializeAvrFloat64(outbuffer + offset, this->position_covariance[i]);
=======
      int32_t * val_latitude = (int32_t *) &(this->latitude);
      int32_t exp_latitude = (((*val_latitude)>>23)&255);
      if(exp_latitude != 0)
        exp_latitude += 1023-127;
      int32_t sig_latitude = *val_latitude;
      *(outbuffer + offset++) = 0;
      *(outbuffer + offset++) = 0;
      *(outbuffer + offset++) = 0;
      *(outbuffer + offset++) = (sig_latitude<<5) & 0xff;
      *(outbuffer + offset++) = (sig_latitude>>3) & 0xff;
      *(outbuffer + offset++) = (sig_latitude>>11) & 0xff;
      *(outbuffer + offset++) = ((exp_latitude<<4) & 0xF0) | ((sig_latitude>>19)&0x0F);
      *(outbuffer + offset++) = (exp_latitude>>4) & 0x7F;
      if(this->latitude < 0) *(outbuffer + offset -1) |= 0x80;
      int32_t * val_longitude = (int32_t *) &(this->longitude);
      int32_t exp_longitude = (((*val_longitude)>>23)&255);
      if(exp_longitude != 0)
        exp_longitude += 1023-127;
      int32_t sig_longitude = *val_longitude;
      *(outbuffer + offset++) = 0;
      *(outbuffer + offset++) = 0;
      *(outbuffer + offset++) = 0;
      *(outbuffer + offset++) = (sig_longitude<<5) & 0xff;
      *(outbuffer + offset++) = (sig_longitude>>3) & 0xff;
      *(outbuffer + offset++) = (sig_longitude>>11) & 0xff;
      *(outbuffer + offset++) = ((exp_longitude<<4) & 0xF0) | ((sig_longitude>>19)&0x0F);
      *(outbuffer + offset++) = (exp_longitude>>4) & 0x7F;
      if(this->longitude < 0) *(outbuffer + offset -1) |= 0x80;
      int32_t * val_altitude = (int32_t *) &(this->altitude);
      int32_t exp_altitude = (((*val_altitude)>>23)&255);
      if(exp_altitude != 0)
        exp_altitude += 1023-127;
      int32_t sig_altitude = *val_altitude;
      *(outbuffer + offset++) = 0;
      *(outbuffer + offset++) = 0;
      *(outbuffer + offset++) = 0;
      *(outbuffer + offset++) = (sig_altitude<<5) & 0xff;
      *(outbuffer + offset++) = (sig_altitude>>3) & 0xff;
      *(outbuffer + offset++) = (sig_altitude>>11) & 0xff;
      *(outbuffer + offset++) = ((exp_altitude<<4) & 0xF0) | ((sig_altitude>>19)&0x0F);
      *(outbuffer + offset++) = (exp_altitude>>4) & 0x7F;
      if(this->altitude < 0) *(outbuffer + offset -1) |= 0x80;
      int32_t * val_track = (int32_t *) &(this->track);
      int32_t exp_track = (((*val_track)>>23)&255);
      if(exp_track != 0)
        exp_track += 1023-127;
      int32_t sig_track = *val_track;
      *(outbuffer + offset++) = 0;
      *(outbuffer + offset++) = 0;
      *(outbuffer + offset++) = 0;
      *(outbuffer + offset++) = (sig_track<<5) & 0xff;
      *(outbuffer + offset++) = (sig_track>>3) & 0xff;
      *(outbuffer + offset++) = (sig_track>>11) & 0xff;
      *(outbuffer + offset++) = ((exp_track<<4) & 0xF0) | ((sig_track>>19)&0x0F);
      *(outbuffer + offset++) = (exp_track>>4) & 0x7F;
      if(this->track < 0) *(outbuffer + offset -1) |= 0x80;
      int32_t * val_speed = (int32_t *) &(this->speed);
      int32_t exp_speed = (((*val_speed)>>23)&255);
      if(exp_speed != 0)
        exp_speed += 1023-127;
      int32_t sig_speed = *val_speed;
      *(outbuffer + offset++) = 0;
      *(outbuffer + offset++) = 0;
      *(outbuffer + offset++) = 0;
      *(outbuffer + offset++) = (sig_speed<<5) & 0xff;
      *(outbuffer + offset++) = (sig_speed>>3) & 0xff;
      *(outbuffer + offset++) = (sig_speed>>11) & 0xff;
      *(outbuffer + offset++) = ((exp_speed<<4) & 0xF0) | ((sig_speed>>19)&0x0F);
      *(outbuffer + offset++) = (exp_speed>>4) & 0x7F;
      if(this->speed < 0) *(outbuffer + offset -1) |= 0x80;
      int32_t * val_climb = (int32_t *) &(this->climb);
      int32_t exp_climb = (((*val_climb)>>23)&255);
      if(exp_climb != 0)
        exp_climb += 1023-127;
      int32_t sig_climb = *val_climb;
      *(outbuffer + offset++) = 0;
      *(outbuffer + offset++) = 0;
      *(outbuffer + offset++) = 0;
      *(outbuffer + offset++) = (sig_climb<<5) & 0xff;
      *(outbuffer + offset++) = (sig_climb>>3) & 0xff;
      *(outbuffer + offset++) = (sig_climb>>11) & 0xff;
      *(outbuffer + offset++) = ((exp_climb<<4) & 0xF0) | ((sig_climb>>19)&0x0F);
      *(outbuffer + offset++) = (exp_climb>>4) & 0x7F;
      if(this->climb < 0) *(outbuffer + offset -1) |= 0x80;
      int32_t * val_pitch = (int32_t *) &(this->pitch);
      int32_t exp_pitch = (((*val_pitch)>>23)&255);
      if(exp_pitch != 0)
        exp_pitch += 1023-127;
      int32_t sig_pitch = *val_pitch;
      *(outbuffer + offset++) = 0;
      *(outbuffer + offset++) = 0;
      *(outbuffer + offset++) = 0;
      *(outbuffer + offset++) = (sig_pitch<<5) & 0xff;
      *(outbuffer + offset++) = (sig_pitch>>3) & 0xff;
      *(outbuffer + offset++) = (sig_pitch>>11) & 0xff;
      *(outbuffer + offset++) = ((exp_pitch<<4) & 0xF0) | ((sig_pitch>>19)&0x0F);
      *(outbuffer + offset++) = (exp_pitch>>4) & 0x7F;
      if(this->pitch < 0) *(outbuffer + offset -1) |= 0x80;
      int32_t * val_roll = (int32_t *) &(this->roll);
      int32_t exp_roll = (((*val_roll)>>23)&255);
      if(exp_roll != 0)
        exp_roll += 1023-127;
      int32_t sig_roll = *val_roll;
      *(outbuffer + offset++) = 0;
      *(outbuffer + offset++) = 0;
      *(outbuffer + offset++) = 0;
      *(outbuffer + offset++) = (sig_roll<<5) & 0xff;
      *(outbuffer + offset++) = (sig_roll>>3) & 0xff;
      *(outbuffer + offset++) = (sig_roll>>11) & 0xff;
      *(outbuffer + offset++) = ((exp_roll<<4) & 0xF0) | ((sig_roll>>19)&0x0F);
      *(outbuffer + offset++) = (exp_roll>>4) & 0x7F;
      if(this->roll < 0) *(outbuffer + offset -1) |= 0x80;
      int32_t * val_dip = (int32_t *) &(this->dip);
      int32_t exp_dip = (((*val_dip)>>23)&255);
      if(exp_dip != 0)
        exp_dip += 1023-127;
      int32_t sig_dip = *val_dip;
      *(outbuffer + offset++) = 0;
      *(outbuffer + offset++) = 0;
      *(outbuffer + offset++) = 0;
      *(outbuffer + offset++) = (sig_dip<<5) & 0xff;
      *(outbuffer + offset++) = (sig_dip>>3) & 0xff;
      *(outbuffer + offset++) = (sig_dip>>11) & 0xff;
      *(outbuffer + offset++) = ((exp_dip<<4) & 0xF0) | ((sig_dip>>19)&0x0F);
      *(outbuffer + offset++) = (exp_dip>>4) & 0x7F;
      if(this->dip < 0) *(outbuffer + offset -1) |= 0x80;
      int32_t * val_time = (int32_t *) &(this->time);
      int32_t exp_time = (((*val_time)>>23)&255);
      if(exp_time != 0)
        exp_time += 1023-127;
      int32_t sig_time = *val_time;
      *(outbuffer + offset++) = 0;
      *(outbuffer + offset++) = 0;
      *(outbuffer + offset++) = 0;
      *(outbuffer + offset++) = (sig_time<<5) & 0xff;
      *(outbuffer + offset++) = (sig_time>>3) & 0xff;
      *(outbuffer + offset++) = (sig_time>>11) & 0xff;
      *(outbuffer + offset++) = ((exp_time<<4) & 0xF0) | ((sig_time>>19)&0x0F);
      *(outbuffer + offset++) = (exp_time>>4) & 0x7F;
      if(this->time < 0) *(outbuffer + offset -1) |= 0x80;
      int32_t * val_gdop = (int32_t *) &(this->gdop);
      int32_t exp_gdop = (((*val_gdop)>>23)&255);
      if(exp_gdop != 0)
        exp_gdop += 1023-127;
      int32_t sig_gdop = *val_gdop;
      *(outbuffer + offset++) = 0;
      *(outbuffer + offset++) = 0;
      *(outbuffer + offset++) = 0;
      *(outbuffer + offset++) = (sig_gdop<<5) & 0xff;
      *(outbuffer + offset++) = (sig_gdop>>3) & 0xff;
      *(outbuffer + offset++) = (sig_gdop>>11) & 0xff;
      *(outbuffer + offset++) = ((exp_gdop<<4) & 0xF0) | ((sig_gdop>>19)&0x0F);
      *(outbuffer + offset++) = (exp_gdop>>4) & 0x7F;
      if(this->gdop < 0) *(outbuffer + offset -1) |= 0x80;
      int32_t * val_pdop = (int32_t *) &(this->pdop);
      int32_t exp_pdop = (((*val_pdop)>>23)&255);
      if(exp_pdop != 0)
        exp_pdop += 1023-127;
      int32_t sig_pdop = *val_pdop;
      *(outbuffer + offset++) = 0;
      *(outbuffer + offset++) = 0;
      *(outbuffer + offset++) = 0;
      *(outbuffer + offset++) = (sig_pdop<<5) & 0xff;
      *(outbuffer + offset++) = (sig_pdop>>3) & 0xff;
      *(outbuffer + offset++) = (sig_pdop>>11) & 0xff;
      *(outbuffer + offset++) = ((exp_pdop<<4) & 0xF0) | ((sig_pdop>>19)&0x0F);
      *(outbuffer + offset++) = (exp_pdop>>4) & 0x7F;
      if(this->pdop < 0) *(outbuffer + offset -1) |= 0x80;
      int32_t * val_hdop = (int32_t *) &(this->hdop);
      int32_t exp_hdop = (((*val_hdop)>>23)&255);
      if(exp_hdop != 0)
        exp_hdop += 1023-127;
      int32_t sig_hdop = *val_hdop;
      *(outbuffer + offset++) = 0;
      *(outbuffer + offset++) = 0;
      *(outbuffer + offset++) = 0;
      *(outbuffer + offset++) = (sig_hdop<<5) & 0xff;
      *(outbuffer + offset++) = (sig_hdop>>3) & 0xff;
      *(outbuffer + offset++) = (sig_hdop>>11) & 0xff;
      *(outbuffer + offset++) = ((exp_hdop<<4) & 0xF0) | ((sig_hdop>>19)&0x0F);
      *(outbuffer + offset++) = (exp_hdop>>4) & 0x7F;
      if(this->hdop < 0) *(outbuffer + offset -1) |= 0x80;
      int32_t * val_vdop = (int32_t *) &(this->vdop);
      int32_t exp_vdop = (((*val_vdop)>>23)&255);
      if(exp_vdop != 0)
        exp_vdop += 1023-127;
      int32_t sig_vdop = *val_vdop;
      *(outbuffer + offset++) = 0;
      *(outbuffer + offset++) = 0;
      *(outbuffer + offset++) = 0;
      *(outbuffer + offset++) = (sig_vdop<<5) & 0xff;
      *(outbuffer + offset++) = (sig_vdop>>3) & 0xff;
      *(outbuffer + offset++) = (sig_vdop>>11) & 0xff;
      *(outbuffer + offset++) = ((exp_vdop<<4) & 0xF0) | ((sig_vdop>>19)&0x0F);
      *(outbuffer + offset++) = (exp_vdop>>4) & 0x7F;
      if(this->vdop < 0) *(outbuffer + offset -1) |= 0x80;
      int32_t * val_tdop = (int32_t *) &(this->tdop);
      int32_t exp_tdop = (((*val_tdop)>>23)&255);
      if(exp_tdop != 0)
        exp_tdop += 1023-127;
      int32_t sig_tdop = *val_tdop;
      *(outbuffer + offset++) = 0;
      *(outbuffer + offset++) = 0;
      *(outbuffer + offset++) = 0;
      *(outbuffer + offset++) = (sig_tdop<<5) & 0xff;
      *(outbuffer + offset++) = (sig_tdop>>3) & 0xff;
      *(outbuffer + offset++) = (sig_tdop>>11) & 0xff;
      *(outbuffer + offset++) = ((exp_tdop<<4) & 0xF0) | ((sig_tdop>>19)&0x0F);
      *(outbuffer + offset++) = (exp_tdop>>4) & 0x7F;
      if(this->tdop < 0) *(outbuffer + offset -1) |= 0x80;
      int32_t * val_err = (int32_t *) &(this->err);
      int32_t exp_err = (((*val_err)>>23)&255);
      if(exp_err != 0)
        exp_err += 1023-127;
      int32_t sig_err = *val_err;
      *(outbuffer + offset++) = 0;
      *(outbuffer + offset++) = 0;
      *(outbuffer + offset++) = 0;
      *(outbuffer + offset++) = (sig_err<<5) & 0xff;
      *(outbuffer + offset++) = (sig_err>>3) & 0xff;
      *(outbuffer + offset++) = (sig_err>>11) & 0xff;
      *(outbuffer + offset++) = ((exp_err<<4) & 0xF0) | ((sig_err>>19)&0x0F);
      *(outbuffer + offset++) = (exp_err>>4) & 0x7F;
      if(this->err < 0) *(outbuffer + offset -1) |= 0x80;
      int32_t * val_err_horz = (int32_t *) &(this->err_horz);
      int32_t exp_err_horz = (((*val_err_horz)>>23)&255);
      if(exp_err_horz != 0)
        exp_err_horz += 1023-127;
      int32_t sig_err_horz = *val_err_horz;
      *(outbuffer + offset++) = 0;
      *(outbuffer + offset++) = 0;
      *(outbuffer + offset++) = 0;
      *(outbuffer + offset++) = (sig_err_horz<<5) & 0xff;
      *(outbuffer + offset++) = (sig_err_horz>>3) & 0xff;
      *(outbuffer + offset++) = (sig_err_horz>>11) & 0xff;
      *(outbuffer + offset++) = ((exp_err_horz<<4) & 0xF0) | ((sig_err_horz>>19)&0x0F);
      *(outbuffer + offset++) = (exp_err_horz>>4) & 0x7F;
      if(this->err_horz < 0) *(outbuffer + offset -1) |= 0x80;
      int32_t * val_err_vert = (int32_t *) &(this->err_vert);
      int32_t exp_err_vert = (((*val_err_vert)>>23)&255);
      if(exp_err_vert != 0)
        exp_err_vert += 1023-127;
      int32_t sig_err_vert = *val_err_vert;
      *(outbuffer + offset++) = 0;
      *(outbuffer + offset++) = 0;
      *(outbuffer + offset++) = 0;
      *(outbuffer + offset++) = (sig_err_vert<<5) & 0xff;
      *(outbuffer + offset++) = (sig_err_vert>>3) & 0xff;
      *(outbuffer + offset++) = (sig_err_vert>>11) & 0xff;
      *(outbuffer + offset++) = ((exp_err_vert<<4) & 0xF0) | ((sig_err_vert>>19)&0x0F);
      *(outbuffer + offset++) = (exp_err_vert>>4) & 0x7F;
      if(this->err_vert < 0) *(outbuffer + offset -1) |= 0x80;
      int32_t * val_err_track = (int32_t *) &(this->err_track);
      int32_t exp_err_track = (((*val_err_track)>>23)&255);
      if(exp_err_track != 0)
        exp_err_track += 1023-127;
      int32_t sig_err_track = *val_err_track;
      *(outbuffer + offset++) = 0;
      *(outbuffer + offset++) = 0;
      *(outbuffer + offset++) = 0;
      *(outbuffer + offset++) = (sig_err_track<<5) & 0xff;
      *(outbuffer + offset++) = (sig_err_track>>3) & 0xff;
      *(outbuffer + offset++) = (sig_err_track>>11) & 0xff;
      *(outbuffer + offset++) = ((exp_err_track<<4) & 0xF0) | ((sig_err_track>>19)&0x0F);
      *(outbuffer + offset++) = (exp_err_track>>4) & 0x7F;
      if(this->err_track < 0) *(outbuffer + offset -1) |= 0x80;
      int32_t * val_err_speed = (int32_t *) &(this->err_speed);
      int32_t exp_err_speed = (((*val_err_speed)>>23)&255);
      if(exp_err_speed != 0)
        exp_err_speed += 1023-127;
      int32_t sig_err_speed = *val_err_speed;
      *(outbuffer + offset++) = 0;
      *(outbuffer + offset++) = 0;
      *(outbuffer + offset++) = 0;
      *(outbuffer + offset++) = (sig_err_speed<<5) & 0xff;
      *(outbuffer + offset++) = (sig_err_speed>>3) & 0xff;
      *(outbuffer + offset++) = (sig_err_speed>>11) & 0xff;
      *(outbuffer + offset++) = ((exp_err_speed<<4) & 0xF0) | ((sig_err_speed>>19)&0x0F);
      *(outbuffer + offset++) = (exp_err_speed>>4) & 0x7F;
      if(this->err_speed < 0) *(outbuffer + offset -1) |= 0x80;
      int32_t * val_err_climb = (int32_t *) &(this->err_climb);
      int32_t exp_err_climb = (((*val_err_climb)>>23)&255);
      if(exp_err_climb != 0)
        exp_err_climb += 1023-127;
      int32_t sig_err_climb = *val_err_climb;
      *(outbuffer + offset++) = 0;
      *(outbuffer + offset++) = 0;
      *(outbuffer + offset++) = 0;
      *(outbuffer + offset++) = (sig_err_climb<<5) & 0xff;
      *(outbuffer + offset++) = (sig_err_climb>>3) & 0xff;
      *(outbuffer + offset++) = (sig_err_climb>>11) & 0xff;
      *(outbuffer + offset++) = ((exp_err_climb<<4) & 0xF0) | ((sig_err_climb>>19)&0x0F);
      *(outbuffer + offset++) = (exp_err_climb>>4) & 0x7F;
      if(this->err_climb < 0) *(outbuffer + offset -1) |= 0x80;
      int32_t * val_err_time = (int32_t *) &(this->err_time);
      int32_t exp_err_time = (((*val_err_time)>>23)&255);
      if(exp_err_time != 0)
        exp_err_time += 1023-127;
      int32_t sig_err_time = *val_err_time;
      *(outbuffer + offset++) = 0;
      *(outbuffer + offset++) = 0;
      *(outbuffer + offset++) = 0;
      *(outbuffer + offset++) = (sig_err_time<<5) & 0xff;
      *(outbuffer + offset++) = (sig_err_time>>3) & 0xff;
      *(outbuffer + offset++) = (sig_err_time>>11) & 0xff;
      *(outbuffer + offset++) = ((exp_err_time<<4) & 0xF0) | ((sig_err_time>>19)&0x0F);
      *(outbuffer + offset++) = (exp_err_time>>4) & 0x7F;
      if(this->err_time < 0) *(outbuffer + offset -1) |= 0x80;
      int32_t * val_err_pitch = (int32_t *) &(this->err_pitch);
      int32_t exp_err_pitch = (((*val_err_pitch)>>23)&255);
      if(exp_err_pitch != 0)
        exp_err_pitch += 1023-127;
      int32_t sig_err_pitch = *val_err_pitch;
      *(outbuffer + offset++) = 0;
      *(outbuffer + offset++) = 0;
      *(outbuffer + offset++) = 0;
      *(outbuffer + offset++) = (sig_err_pitch<<5) & 0xff;
      *(outbuffer + offset++) = (sig_err_pitch>>3) & 0xff;
      *(outbuffer + offset++) = (sig_err_pitch>>11) & 0xff;
      *(outbuffer + offset++) = ((exp_err_pitch<<4) & 0xF0) | ((sig_err_pitch>>19)&0x0F);
      *(outbuffer + offset++) = (exp_err_pitch>>4) & 0x7F;
      if(this->err_pitch < 0) *(outbuffer + offset -1) |= 0x80;
      int32_t * val_err_roll = (int32_t *) &(this->err_roll);
      int32_t exp_err_roll = (((*val_err_roll)>>23)&255);
      if(exp_err_roll != 0)
        exp_err_roll += 1023-127;
      int32_t sig_err_roll = *val_err_roll;
      *(outbuffer + offset++) = 0;
      *(outbuffer + offset++) = 0;
      *(outbuffer + offset++) = 0;
      *(outbuffer + offset++) = (sig_err_roll<<5) & 0xff;
      *(outbuffer + offset++) = (sig_err_roll>>3) & 0xff;
      *(outbuffer + offset++) = (sig_err_roll>>11) & 0xff;
      *(outbuffer + offset++) = ((exp_err_roll<<4) & 0xF0) | ((sig_err_roll>>19)&0x0F);
      *(outbuffer + offset++) = (exp_err_roll>>4) & 0x7F;
      if(this->err_roll < 0) *(outbuffer + offset -1) |= 0x80;
      int32_t * val_err_dip = (int32_t *) &(this->err_dip);
      int32_t exp_err_dip = (((*val_err_dip)>>23)&255);
      if(exp_err_dip != 0)
        exp_err_dip += 1023-127;
      int32_t sig_err_dip = *val_err_dip;
      *(outbuffer + offset++) = 0;
      *(outbuffer + offset++) = 0;
      *(outbuffer + offset++) = 0;
      *(outbuffer + offset++) = (sig_err_dip<<5) & 0xff;
      *(outbuffer + offset++) = (sig_err_dip>>3) & 0xff;
      *(outbuffer + offset++) = (sig_err_dip>>11) & 0xff;
      *(outbuffer + offset++) = ((exp_err_dip<<4) & 0xF0) | ((sig_err_dip>>19)&0x0F);
      *(outbuffer + offset++) = (exp_err_dip>>4) & 0x7F;
      if(this->err_dip < 0) *(outbuffer + offset -1) |= 0x80;
      unsigned char * position_covariance_val = (unsigned char *) this->position_covariance;
      for( uint8_t i = 0; i < 9; i++){
      int32_t * val_position_covariancei = (int32_t *) &(this->position_covariance[i]);
      int32_t exp_position_covariancei = (((*val_position_covariancei)>>23)&255);
      if(exp_position_covariancei != 0)
        exp_position_covariancei += 1023-127;
      int32_t sig_position_covariancei = *val_position_covariancei;
      *(outbuffer + offset++) = 0;
      *(outbuffer + offset++) = 0;
      *(outbuffer + offset++) = 0;
      *(outbuffer + offset++) = (sig_position_covariancei<<5) & 0xff;
      *(outbuffer + offset++) = (sig_position_covariancei>>3) & 0xff;
      *(outbuffer + offset++) = (sig_position_covariancei>>11) & 0xff;
      *(outbuffer + offset++) = ((exp_position_covariancei<<4) & 0xF0) | ((sig_position_covariancei>>19)&0x0F);
      *(outbuffer + offset++) = (exp_position_covariancei>>4) & 0x7F;
      if(this->position_covariance[i] < 0) *(outbuffer + offset -1) |= 0x80;
>>>>>>> 114c33badc7e157f5d09deb42c0831bdbf295d3c
      }
      *(outbuffer + offset + 0) = (this->position_covariance_type >> (8 * 0)) & 0xFF;
      offset += sizeof(this->position_covariance_type);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      offset += this->header.deserialize(inbuffer + offset);
      offset += this->status.deserialize(inbuffer + offset);
<<<<<<< HEAD
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->latitude));
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->longitude));
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->altitude));
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->track));
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->speed));
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->climb));
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->pitch));
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->roll));
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->dip));
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->time));
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->gdop));
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->pdop));
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->hdop));
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->vdop));
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->tdop));
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->err));
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->err_horz));
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->err_vert));
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->err_track));
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->err_speed));
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->err_climb));
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->err_time));
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->err_pitch));
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->err_roll));
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->err_dip));
      for( uint8_t i = 0; i < 9; i++){
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->position_covariance[i]));
=======
      uint32_t * val_latitude = (uint32_t*) &(this->latitude);
      offset += 3;
      *val_latitude = ((uint32_t)(*(inbuffer + offset++))>>5 & 0x07);
      *val_latitude |= ((uint32_t)(*(inbuffer + offset++)) & 0xff)<<3;
      *val_latitude |= ((uint32_t)(*(inbuffer + offset++)) & 0xff)<<11;
      *val_latitude |= ((uint32_t)(*(inbuffer + offset)) & 0x0f)<<19;
      uint32_t exp_latitude = ((uint32_t)(*(inbuffer + offset++))&0xf0)>>4;
      exp_latitude |= ((uint32_t)(*(inbuffer + offset)) & 0x7f)<<4;
      if(exp_latitude !=0)
        *val_latitude |= ((exp_latitude)-1023+127)<<23;
      if( ((*(inbuffer+offset++)) & 0x80) > 0) this->latitude = -this->latitude;
      uint32_t * val_longitude = (uint32_t*) &(this->longitude);
      offset += 3;
      *val_longitude = ((uint32_t)(*(inbuffer + offset++))>>5 & 0x07);
      *val_longitude |= ((uint32_t)(*(inbuffer + offset++)) & 0xff)<<3;
      *val_longitude |= ((uint32_t)(*(inbuffer + offset++)) & 0xff)<<11;
      *val_longitude |= ((uint32_t)(*(inbuffer + offset)) & 0x0f)<<19;
      uint32_t exp_longitude = ((uint32_t)(*(inbuffer + offset++))&0xf0)>>4;
      exp_longitude |= ((uint32_t)(*(inbuffer + offset)) & 0x7f)<<4;
      if(exp_longitude !=0)
        *val_longitude |= ((exp_longitude)-1023+127)<<23;
      if( ((*(inbuffer+offset++)) & 0x80) > 0) this->longitude = -this->longitude;
      uint32_t * val_altitude = (uint32_t*) &(this->altitude);
      offset += 3;
      *val_altitude = ((uint32_t)(*(inbuffer + offset++))>>5 & 0x07);
      *val_altitude |= ((uint32_t)(*(inbuffer + offset++)) & 0xff)<<3;
      *val_altitude |= ((uint32_t)(*(inbuffer + offset++)) & 0xff)<<11;
      *val_altitude |= ((uint32_t)(*(inbuffer + offset)) & 0x0f)<<19;
      uint32_t exp_altitude = ((uint32_t)(*(inbuffer + offset++))&0xf0)>>4;
      exp_altitude |= ((uint32_t)(*(inbuffer + offset)) & 0x7f)<<4;
      if(exp_altitude !=0)
        *val_altitude |= ((exp_altitude)-1023+127)<<23;
      if( ((*(inbuffer+offset++)) & 0x80) > 0) this->altitude = -this->altitude;
      uint32_t * val_track = (uint32_t*) &(this->track);
      offset += 3;
      *val_track = ((uint32_t)(*(inbuffer + offset++))>>5 & 0x07);
      *val_track |= ((uint32_t)(*(inbuffer + offset++)) & 0xff)<<3;
      *val_track |= ((uint32_t)(*(inbuffer + offset++)) & 0xff)<<11;
      *val_track |= ((uint32_t)(*(inbuffer + offset)) & 0x0f)<<19;
      uint32_t exp_track = ((uint32_t)(*(inbuffer + offset++))&0xf0)>>4;
      exp_track |= ((uint32_t)(*(inbuffer + offset)) & 0x7f)<<4;
      if(exp_track !=0)
        *val_track |= ((exp_track)-1023+127)<<23;
      if( ((*(inbuffer+offset++)) & 0x80) > 0) this->track = -this->track;
      uint32_t * val_speed = (uint32_t*) &(this->speed);
      offset += 3;
      *val_speed = ((uint32_t)(*(inbuffer + offset++))>>5 & 0x07);
      *val_speed |= ((uint32_t)(*(inbuffer + offset++)) & 0xff)<<3;
      *val_speed |= ((uint32_t)(*(inbuffer + offset++)) & 0xff)<<11;
      *val_speed |= ((uint32_t)(*(inbuffer + offset)) & 0x0f)<<19;
      uint32_t exp_speed = ((uint32_t)(*(inbuffer + offset++))&0xf0)>>4;
      exp_speed |= ((uint32_t)(*(inbuffer + offset)) & 0x7f)<<4;
      if(exp_speed !=0)
        *val_speed |= ((exp_speed)-1023+127)<<23;
      if( ((*(inbuffer+offset++)) & 0x80) > 0) this->speed = -this->speed;
      uint32_t * val_climb = (uint32_t*) &(this->climb);
      offset += 3;
      *val_climb = ((uint32_t)(*(inbuffer + offset++))>>5 & 0x07);
      *val_climb |= ((uint32_t)(*(inbuffer + offset++)) & 0xff)<<3;
      *val_climb |= ((uint32_t)(*(inbuffer + offset++)) & 0xff)<<11;
      *val_climb |= ((uint32_t)(*(inbuffer + offset)) & 0x0f)<<19;
      uint32_t exp_climb = ((uint32_t)(*(inbuffer + offset++))&0xf0)>>4;
      exp_climb |= ((uint32_t)(*(inbuffer + offset)) & 0x7f)<<4;
      if(exp_climb !=0)
        *val_climb |= ((exp_climb)-1023+127)<<23;
      if( ((*(inbuffer+offset++)) & 0x80) > 0) this->climb = -this->climb;
      uint32_t * val_pitch = (uint32_t*) &(this->pitch);
      offset += 3;
      *val_pitch = ((uint32_t)(*(inbuffer + offset++))>>5 & 0x07);
      *val_pitch |= ((uint32_t)(*(inbuffer + offset++)) & 0xff)<<3;
      *val_pitch |= ((uint32_t)(*(inbuffer + offset++)) & 0xff)<<11;
      *val_pitch |= ((uint32_t)(*(inbuffer + offset)) & 0x0f)<<19;
      uint32_t exp_pitch = ((uint32_t)(*(inbuffer + offset++))&0xf0)>>4;
      exp_pitch |= ((uint32_t)(*(inbuffer + offset)) & 0x7f)<<4;
      if(exp_pitch !=0)
        *val_pitch |= ((exp_pitch)-1023+127)<<23;
      if( ((*(inbuffer+offset++)) & 0x80) > 0) this->pitch = -this->pitch;
      uint32_t * val_roll = (uint32_t*) &(this->roll);
      offset += 3;
      *val_roll = ((uint32_t)(*(inbuffer + offset++))>>5 & 0x07);
      *val_roll |= ((uint32_t)(*(inbuffer + offset++)) & 0xff)<<3;
      *val_roll |= ((uint32_t)(*(inbuffer + offset++)) & 0xff)<<11;
      *val_roll |= ((uint32_t)(*(inbuffer + offset)) & 0x0f)<<19;
      uint32_t exp_roll = ((uint32_t)(*(inbuffer + offset++))&0xf0)>>4;
      exp_roll |= ((uint32_t)(*(inbuffer + offset)) & 0x7f)<<4;
      if(exp_roll !=0)
        *val_roll |= ((exp_roll)-1023+127)<<23;
      if( ((*(inbuffer+offset++)) & 0x80) > 0) this->roll = -this->roll;
      uint32_t * val_dip = (uint32_t*) &(this->dip);
      offset += 3;
      *val_dip = ((uint32_t)(*(inbuffer + offset++))>>5 & 0x07);
      *val_dip |= ((uint32_t)(*(inbuffer + offset++)) & 0xff)<<3;
      *val_dip |= ((uint32_t)(*(inbuffer + offset++)) & 0xff)<<11;
      *val_dip |= ((uint32_t)(*(inbuffer + offset)) & 0x0f)<<19;
      uint32_t exp_dip = ((uint32_t)(*(inbuffer + offset++))&0xf0)>>4;
      exp_dip |= ((uint32_t)(*(inbuffer + offset)) & 0x7f)<<4;
      if(exp_dip !=0)
        *val_dip |= ((exp_dip)-1023+127)<<23;
      if( ((*(inbuffer+offset++)) & 0x80) > 0) this->dip = -this->dip;
      uint32_t * val_time = (uint32_t*) &(this->time);
      offset += 3;
      *val_time = ((uint32_t)(*(inbuffer + offset++))>>5 & 0x07);
      *val_time |= ((uint32_t)(*(inbuffer + offset++)) & 0xff)<<3;
      *val_time |= ((uint32_t)(*(inbuffer + offset++)) & 0xff)<<11;
      *val_time |= ((uint32_t)(*(inbuffer + offset)) & 0x0f)<<19;
      uint32_t exp_time = ((uint32_t)(*(inbuffer + offset++))&0xf0)>>4;
      exp_time |= ((uint32_t)(*(inbuffer + offset)) & 0x7f)<<4;
      if(exp_time !=0)
        *val_time |= ((exp_time)-1023+127)<<23;
      if( ((*(inbuffer+offset++)) & 0x80) > 0) this->time = -this->time;
      uint32_t * val_gdop = (uint32_t*) &(this->gdop);
      offset += 3;
      *val_gdop = ((uint32_t)(*(inbuffer + offset++))>>5 & 0x07);
      *val_gdop |= ((uint32_t)(*(inbuffer + offset++)) & 0xff)<<3;
      *val_gdop |= ((uint32_t)(*(inbuffer + offset++)) & 0xff)<<11;
      *val_gdop |= ((uint32_t)(*(inbuffer + offset)) & 0x0f)<<19;
      uint32_t exp_gdop = ((uint32_t)(*(inbuffer + offset++))&0xf0)>>4;
      exp_gdop |= ((uint32_t)(*(inbuffer + offset)) & 0x7f)<<4;
      if(exp_gdop !=0)
        *val_gdop |= ((exp_gdop)-1023+127)<<23;
      if( ((*(inbuffer+offset++)) & 0x80) > 0) this->gdop = -this->gdop;
      uint32_t * val_pdop = (uint32_t*) &(this->pdop);
      offset += 3;
      *val_pdop = ((uint32_t)(*(inbuffer + offset++))>>5 & 0x07);
      *val_pdop |= ((uint32_t)(*(inbuffer + offset++)) & 0xff)<<3;
      *val_pdop |= ((uint32_t)(*(inbuffer + offset++)) & 0xff)<<11;
      *val_pdop |= ((uint32_t)(*(inbuffer + offset)) & 0x0f)<<19;
      uint32_t exp_pdop = ((uint32_t)(*(inbuffer + offset++))&0xf0)>>4;
      exp_pdop |= ((uint32_t)(*(inbuffer + offset)) & 0x7f)<<4;
      if(exp_pdop !=0)
        *val_pdop |= ((exp_pdop)-1023+127)<<23;
      if( ((*(inbuffer+offset++)) & 0x80) > 0) this->pdop = -this->pdop;
      uint32_t * val_hdop = (uint32_t*) &(this->hdop);
      offset += 3;
      *val_hdop = ((uint32_t)(*(inbuffer + offset++))>>5 & 0x07);
      *val_hdop |= ((uint32_t)(*(inbuffer + offset++)) & 0xff)<<3;
      *val_hdop |= ((uint32_t)(*(inbuffer + offset++)) & 0xff)<<11;
      *val_hdop |= ((uint32_t)(*(inbuffer + offset)) & 0x0f)<<19;
      uint32_t exp_hdop = ((uint32_t)(*(inbuffer + offset++))&0xf0)>>4;
      exp_hdop |= ((uint32_t)(*(inbuffer + offset)) & 0x7f)<<4;
      if(exp_hdop !=0)
        *val_hdop |= ((exp_hdop)-1023+127)<<23;
      if( ((*(inbuffer+offset++)) & 0x80) > 0) this->hdop = -this->hdop;
      uint32_t * val_vdop = (uint32_t*) &(this->vdop);
      offset += 3;
      *val_vdop = ((uint32_t)(*(inbuffer + offset++))>>5 & 0x07);
      *val_vdop |= ((uint32_t)(*(inbuffer + offset++)) & 0xff)<<3;
      *val_vdop |= ((uint32_t)(*(inbuffer + offset++)) & 0xff)<<11;
      *val_vdop |= ((uint32_t)(*(inbuffer + offset)) & 0x0f)<<19;
      uint32_t exp_vdop = ((uint32_t)(*(inbuffer + offset++))&0xf0)>>4;
      exp_vdop |= ((uint32_t)(*(inbuffer + offset)) & 0x7f)<<4;
      if(exp_vdop !=0)
        *val_vdop |= ((exp_vdop)-1023+127)<<23;
      if( ((*(inbuffer+offset++)) & 0x80) > 0) this->vdop = -this->vdop;
      uint32_t * val_tdop = (uint32_t*) &(this->tdop);
      offset += 3;
      *val_tdop = ((uint32_t)(*(inbuffer + offset++))>>5 & 0x07);
      *val_tdop |= ((uint32_t)(*(inbuffer + offset++)) & 0xff)<<3;
      *val_tdop |= ((uint32_t)(*(inbuffer + offset++)) & 0xff)<<11;
      *val_tdop |= ((uint32_t)(*(inbuffer + offset)) & 0x0f)<<19;
      uint32_t exp_tdop = ((uint32_t)(*(inbuffer + offset++))&0xf0)>>4;
      exp_tdop |= ((uint32_t)(*(inbuffer + offset)) & 0x7f)<<4;
      if(exp_tdop !=0)
        *val_tdop |= ((exp_tdop)-1023+127)<<23;
      if( ((*(inbuffer+offset++)) & 0x80) > 0) this->tdop = -this->tdop;
      uint32_t * val_err = (uint32_t*) &(this->err);
      offset += 3;
      *val_err = ((uint32_t)(*(inbuffer + offset++))>>5 & 0x07);
      *val_err |= ((uint32_t)(*(inbuffer + offset++)) & 0xff)<<3;
      *val_err |= ((uint32_t)(*(inbuffer + offset++)) & 0xff)<<11;
      *val_err |= ((uint32_t)(*(inbuffer + offset)) & 0x0f)<<19;
      uint32_t exp_err = ((uint32_t)(*(inbuffer + offset++))&0xf0)>>4;
      exp_err |= ((uint32_t)(*(inbuffer + offset)) & 0x7f)<<4;
      if(exp_err !=0)
        *val_err |= ((exp_err)-1023+127)<<23;
      if( ((*(inbuffer+offset++)) & 0x80) > 0) this->err = -this->err;
      uint32_t * val_err_horz = (uint32_t*) &(this->err_horz);
      offset += 3;
      *val_err_horz = ((uint32_t)(*(inbuffer + offset++))>>5 & 0x07);
      *val_err_horz |= ((uint32_t)(*(inbuffer + offset++)) & 0xff)<<3;
      *val_err_horz |= ((uint32_t)(*(inbuffer + offset++)) & 0xff)<<11;
      *val_err_horz |= ((uint32_t)(*(inbuffer + offset)) & 0x0f)<<19;
      uint32_t exp_err_horz = ((uint32_t)(*(inbuffer + offset++))&0xf0)>>4;
      exp_err_horz |= ((uint32_t)(*(inbuffer + offset)) & 0x7f)<<4;
      if(exp_err_horz !=0)
        *val_err_horz |= ((exp_err_horz)-1023+127)<<23;
      if( ((*(inbuffer+offset++)) & 0x80) > 0) this->err_horz = -this->err_horz;
      uint32_t * val_err_vert = (uint32_t*) &(this->err_vert);
      offset += 3;
      *val_err_vert = ((uint32_t)(*(inbuffer + offset++))>>5 & 0x07);
      *val_err_vert |= ((uint32_t)(*(inbuffer + offset++)) & 0xff)<<3;
      *val_err_vert |= ((uint32_t)(*(inbuffer + offset++)) & 0xff)<<11;
      *val_err_vert |= ((uint32_t)(*(inbuffer + offset)) & 0x0f)<<19;
      uint32_t exp_err_vert = ((uint32_t)(*(inbuffer + offset++))&0xf0)>>4;
      exp_err_vert |= ((uint32_t)(*(inbuffer + offset)) & 0x7f)<<4;
      if(exp_err_vert !=0)
        *val_err_vert |= ((exp_err_vert)-1023+127)<<23;
      if( ((*(inbuffer+offset++)) & 0x80) > 0) this->err_vert = -this->err_vert;
      uint32_t * val_err_track = (uint32_t*) &(this->err_track);
      offset += 3;
      *val_err_track = ((uint32_t)(*(inbuffer + offset++))>>5 & 0x07);
      *val_err_track |= ((uint32_t)(*(inbuffer + offset++)) & 0xff)<<3;
      *val_err_track |= ((uint32_t)(*(inbuffer + offset++)) & 0xff)<<11;
      *val_err_track |= ((uint32_t)(*(inbuffer + offset)) & 0x0f)<<19;
      uint32_t exp_err_track = ((uint32_t)(*(inbuffer + offset++))&0xf0)>>4;
      exp_err_track |= ((uint32_t)(*(inbuffer + offset)) & 0x7f)<<4;
      if(exp_err_track !=0)
        *val_err_track |= ((exp_err_track)-1023+127)<<23;
      if( ((*(inbuffer+offset++)) & 0x80) > 0) this->err_track = -this->err_track;
      uint32_t * val_err_speed = (uint32_t*) &(this->err_speed);
      offset += 3;
      *val_err_speed = ((uint32_t)(*(inbuffer + offset++))>>5 & 0x07);
      *val_err_speed |= ((uint32_t)(*(inbuffer + offset++)) & 0xff)<<3;
      *val_err_speed |= ((uint32_t)(*(inbuffer + offset++)) & 0xff)<<11;
      *val_err_speed |= ((uint32_t)(*(inbuffer + offset)) & 0x0f)<<19;
      uint32_t exp_err_speed = ((uint32_t)(*(inbuffer + offset++))&0xf0)>>4;
      exp_err_speed |= ((uint32_t)(*(inbuffer + offset)) & 0x7f)<<4;
      if(exp_err_speed !=0)
        *val_err_speed |= ((exp_err_speed)-1023+127)<<23;
      if( ((*(inbuffer+offset++)) & 0x80) > 0) this->err_speed = -this->err_speed;
      uint32_t * val_err_climb = (uint32_t*) &(this->err_climb);
      offset += 3;
      *val_err_climb = ((uint32_t)(*(inbuffer + offset++))>>5 & 0x07);
      *val_err_climb |= ((uint32_t)(*(inbuffer + offset++)) & 0xff)<<3;
      *val_err_climb |= ((uint32_t)(*(inbuffer + offset++)) & 0xff)<<11;
      *val_err_climb |= ((uint32_t)(*(inbuffer + offset)) & 0x0f)<<19;
      uint32_t exp_err_climb = ((uint32_t)(*(inbuffer + offset++))&0xf0)>>4;
      exp_err_climb |= ((uint32_t)(*(inbuffer + offset)) & 0x7f)<<4;
      if(exp_err_climb !=0)
        *val_err_climb |= ((exp_err_climb)-1023+127)<<23;
      if( ((*(inbuffer+offset++)) & 0x80) > 0) this->err_climb = -this->err_climb;
      uint32_t * val_err_time = (uint32_t*) &(this->err_time);
      offset += 3;
      *val_err_time = ((uint32_t)(*(inbuffer + offset++))>>5 & 0x07);
      *val_err_time |= ((uint32_t)(*(inbuffer + offset++)) & 0xff)<<3;
      *val_err_time |= ((uint32_t)(*(inbuffer + offset++)) & 0xff)<<11;
      *val_err_time |= ((uint32_t)(*(inbuffer + offset)) & 0x0f)<<19;
      uint32_t exp_err_time = ((uint32_t)(*(inbuffer + offset++))&0xf0)>>4;
      exp_err_time |= ((uint32_t)(*(inbuffer + offset)) & 0x7f)<<4;
      if(exp_err_time !=0)
        *val_err_time |= ((exp_err_time)-1023+127)<<23;
      if( ((*(inbuffer+offset++)) & 0x80) > 0) this->err_time = -this->err_time;
      uint32_t * val_err_pitch = (uint32_t*) &(this->err_pitch);
      offset += 3;
      *val_err_pitch = ((uint32_t)(*(inbuffer + offset++))>>5 & 0x07);
      *val_err_pitch |= ((uint32_t)(*(inbuffer + offset++)) & 0xff)<<3;
      *val_err_pitch |= ((uint32_t)(*(inbuffer + offset++)) & 0xff)<<11;
      *val_err_pitch |= ((uint32_t)(*(inbuffer + offset)) & 0x0f)<<19;
      uint32_t exp_err_pitch = ((uint32_t)(*(inbuffer + offset++))&0xf0)>>4;
      exp_err_pitch |= ((uint32_t)(*(inbuffer + offset)) & 0x7f)<<4;
      if(exp_err_pitch !=0)
        *val_err_pitch |= ((exp_err_pitch)-1023+127)<<23;
      if( ((*(inbuffer+offset++)) & 0x80) > 0) this->err_pitch = -this->err_pitch;
      uint32_t * val_err_roll = (uint32_t*) &(this->err_roll);
      offset += 3;
      *val_err_roll = ((uint32_t)(*(inbuffer + offset++))>>5 & 0x07);
      *val_err_roll |= ((uint32_t)(*(inbuffer + offset++)) & 0xff)<<3;
      *val_err_roll |= ((uint32_t)(*(inbuffer + offset++)) & 0xff)<<11;
      *val_err_roll |= ((uint32_t)(*(inbuffer + offset)) & 0x0f)<<19;
      uint32_t exp_err_roll = ((uint32_t)(*(inbuffer + offset++))&0xf0)>>4;
      exp_err_roll |= ((uint32_t)(*(inbuffer + offset)) & 0x7f)<<4;
      if(exp_err_roll !=0)
        *val_err_roll |= ((exp_err_roll)-1023+127)<<23;
      if( ((*(inbuffer+offset++)) & 0x80) > 0) this->err_roll = -this->err_roll;
      uint32_t * val_err_dip = (uint32_t*) &(this->err_dip);
      offset += 3;
      *val_err_dip = ((uint32_t)(*(inbuffer + offset++))>>5 & 0x07);
      *val_err_dip |= ((uint32_t)(*(inbuffer + offset++)) & 0xff)<<3;
      *val_err_dip |= ((uint32_t)(*(inbuffer + offset++)) & 0xff)<<11;
      *val_err_dip |= ((uint32_t)(*(inbuffer + offset)) & 0x0f)<<19;
      uint32_t exp_err_dip = ((uint32_t)(*(inbuffer + offset++))&0xf0)>>4;
      exp_err_dip |= ((uint32_t)(*(inbuffer + offset)) & 0x7f)<<4;
      if(exp_err_dip !=0)
        *val_err_dip |= ((exp_err_dip)-1023+127)<<23;
      if( ((*(inbuffer+offset++)) & 0x80) > 0) this->err_dip = -this->err_dip;
      uint8_t * position_covariance_val = (uint8_t*) this->position_covariance;
      for( uint8_t i = 0; i < 9; i++){
      uint32_t * val_position_covariancei = (uint32_t*) &(this->position_covariance[i]);
      offset += 3;
      *val_position_covariancei = ((uint32_t)(*(inbuffer + offset++))>>5 & 0x07);
      *val_position_covariancei |= ((uint32_t)(*(inbuffer + offset++)) & 0xff)<<3;
      *val_position_covariancei |= ((uint32_t)(*(inbuffer + offset++)) & 0xff)<<11;
      *val_position_covariancei |= ((uint32_t)(*(inbuffer + offset)) & 0x0f)<<19;
      uint32_t exp_position_covariancei = ((uint32_t)(*(inbuffer + offset++))&0xf0)>>4;
      exp_position_covariancei |= ((uint32_t)(*(inbuffer + offset)) & 0x7f)<<4;
      if(exp_position_covariancei !=0)
        *val_position_covariancei |= ((exp_position_covariancei)-1023+127)<<23;
      if( ((*(inbuffer+offset++)) & 0x80) > 0) this->position_covariance[i] = -this->position_covariance[i];
>>>>>>> 114c33badc7e157f5d09deb42c0831bdbf295d3c
      }
      this->position_covariance_type =  ((uint8_t) (*(inbuffer + offset)));
      offset += sizeof(this->position_covariance_type);
     return offset;
    }

    const char * getType(){ return "gps_common/GPSFix"; };
    const char * getMD5(){ return "3db3d0a7bc53054c67c528af84710b70"; };

  };

}
#endif