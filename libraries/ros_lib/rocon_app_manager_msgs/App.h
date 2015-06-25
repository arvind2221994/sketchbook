#ifndef _ROS_rocon_app_manager_msgs_App_h
#define _ROS_rocon_app_manager_msgs_App_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "rocon_app_manager_msgs/Icon.h"
#include "rocon_app_manager_msgs/PairingClient.h"

namespace rocon_app_manager_msgs
{

  class App : public ros::Msg
  {
    public:
      const char* name;
      const char* display_name;
      const char* description;
      const char* platform;
      const char* status;
      rocon_app_manager_msgs::Icon icon;
      uint8_t pairing_clients_length;
      rocon_app_manager_msgs::PairingClient st_pairing_clients;
      rocon_app_manager_msgs::PairingClient * pairing_clients;

    App():
      name(""),
      display_name(""),
      description(""),
      platform(""),
      status(""),
      icon(),
      pairing_clients_length(0), pairing_clients(NULL)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      uint32_t length_name = strlen(this->name);
      memcpy(outbuffer + offset, &length_name, sizeof(uint32_t));
      offset += 4;
      memcpy(outbuffer + offset, this->name, length_name);
      offset += length_name;
      uint32_t length_display_name = strlen(this->display_name);
      memcpy(outbuffer + offset, &length_display_name, sizeof(uint32_t));
      offset += 4;
      memcpy(outbuffer + offset, this->display_name, length_display_name);
      offset += length_display_name;
      uint32_t length_description = strlen(this->description);
      memcpy(outbuffer + offset, &length_description, sizeof(uint32_t));
      offset += 4;
      memcpy(outbuffer + offset, this->description, length_description);
      offset += length_description;
      uint32_t length_platform = strlen(this->platform);
      memcpy(outbuffer + offset, &length_platform, sizeof(uint32_t));
      offset += 4;
      memcpy(outbuffer + offset, this->platform, length_platform);
      offset += length_platform;
      uint32_t length_status = strlen(this->status);
      memcpy(outbuffer + offset, &length_status, sizeof(uint32_t));
      offset += 4;
      memcpy(outbuffer + offset, this->status, length_status);
      offset += length_status;
      offset += this->icon.serialize(outbuffer + offset);
      *(outbuffer + offset++) = pairing_clients_length;
      *(outbuffer + offset++) = 0;
      *(outbuffer + offset++) = 0;
      *(outbuffer + offset++) = 0;
      for( uint8_t i = 0; i < pairing_clients_length; i++){
      offset += this->pairing_clients[i].serialize(outbuffer + offset);
      }
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      uint32_t length_name;
      memcpy(&length_name, (inbuffer + offset), sizeof(uint32_t));
      offset += 4;
      for(unsigned int k= offset; k< offset+length_name; ++k){
          inbuffer[k-1]=inbuffer[k];
      }
      inbuffer[offset+length_name-1]=0;
      this->name = (char *)(inbuffer + offset-1);
      offset += length_name;
      uint32_t length_display_name;
      memcpy(&length_display_name, (inbuffer + offset), sizeof(uint32_t));
      offset += 4;
      for(unsigned int k= offset; k< offset+length_display_name; ++k){
          inbuffer[k-1]=inbuffer[k];
      }
      inbuffer[offset+length_display_name-1]=0;
      this->display_name = (char *)(inbuffer + offset-1);
      offset += length_display_name;
      uint32_t length_description;
      memcpy(&length_description, (inbuffer + offset), sizeof(uint32_t));
      offset += 4;
      for(unsigned int k= offset; k< offset+length_description; ++k){
          inbuffer[k-1]=inbuffer[k];
      }
      inbuffer[offset+length_description-1]=0;
      this->description = (char *)(inbuffer + offset-1);
      offset += length_description;
      uint32_t length_platform;
      memcpy(&length_platform, (inbuffer + offset), sizeof(uint32_t));
      offset += 4;
      for(unsigned int k= offset; k< offset+length_platform; ++k){
          inbuffer[k-1]=inbuffer[k];
      }
      inbuffer[offset+length_platform-1]=0;
      this->platform = (char *)(inbuffer + offset-1);
      offset += length_platform;
      uint32_t length_status;
      memcpy(&length_status, (inbuffer + offset), sizeof(uint32_t));
      offset += 4;
      for(unsigned int k= offset; k< offset+length_status; ++k){
          inbuffer[k-1]=inbuffer[k];
      }
      inbuffer[offset+length_status-1]=0;
      this->status = (char *)(inbuffer + offset-1);
      offset += length_status;
      offset += this->icon.deserialize(inbuffer + offset);
      uint8_t pairing_clients_lengthT = *(inbuffer + offset++);
      if(pairing_clients_lengthT > pairing_clients_length)
        this->pairing_clients = (rocon_app_manager_msgs::PairingClient*)realloc(this->pairing_clients, pairing_clients_lengthT * sizeof(rocon_app_manager_msgs::PairingClient));
      offset += 3;
      pairing_clients_length = pairing_clients_lengthT;
      for( uint8_t i = 0; i < pairing_clients_length; i++){
      offset += this->st_pairing_clients.deserialize(inbuffer + offset);
        memcpy( &(this->pairing_clients[i]), &(this->st_pairing_clients), sizeof(rocon_app_manager_msgs::PairingClient));
      }
     return offset;
    }

    const char * getType(){ return "rocon_app_manager_msgs/App"; };
    const char * getMD5(){ return "f7c3d8107b83b0a0871d32ad56957836"; };

  };

}
#endif