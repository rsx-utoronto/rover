#ifndef _ROS_inertial_sense_GPS_h
#define _ROS_inertial_sense_GPS_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "std_msgs/Header.h"
#include "geometry_msgs/Vector3.h"

namespace inertial_sense
{

  class GPS : public ros::Msg
  {
    public:
      typedef std_msgs::Header _header_type;
      _header_type header;
      typedef int8_t _num_sat_type;
      _num_sat_type num_sat;
      typedef uint32_t _fix_type_type;
      _fix_type_type fix_type;
      typedef int32_t _cno_type;
      _cno_type cno;
      typedef float _latitude_type;
      _latitude_type latitude;
      typedef float _longitude_type;
      _longitude_type longitude;
      typedef float _altitude_type;
      _altitude_type altitude;
      typedef geometry_msgs::Vector3 _posEcef_type;
      _posEcef_type posEcef;
      typedef geometry_msgs::Vector3 _velEcef_type;
      _velEcef_type velEcef;
      typedef float _hMSL_type;
      _hMSL_type hMSL;
      typedef float _hAcc_type;
      _hAcc_type hAcc;
      typedef float _vAcc_type;
      _vAcc_type vAcc;
      typedef float _sAcc_type;
      _sAcc_type sAcc;
      typedef float _pDop_type;
      _pDop_type pDop;
      enum { GPS_STATUS_FIX_TYPE_NO_FIX =  0 };
      enum { GPS_STATUS_FIX_TYPE_DEAD_RECKONING_ONLY =  256 };
      enum { GPS_STATUS_FIX_TYPE_2D_FIX =  512 };
      enum { GPS_STATUS_FIX_TYPE_3D_FIX =  768 };
      enum { GPS_STATUS_FIX_TYPE_GPS_PLUS_DEAD_RECK =  1024 };
      enum { GPS_STATUS_FIX_TYPE_TIME_ONLY_FIX =  1280 };
      enum { GPS_STATUS_FIX_TYPE_RESERVED1 =  1536 };
      enum { GPS_STATUS_FIX_TYPE_RESERVED2 =  1792 };
      enum { GPS_STATUS_FIX_STATUS_FIX_OK =  65536 };

    GPS():
      header(),
      num_sat(0),
      fix_type(0),
      cno(0),
      latitude(0),
      longitude(0),
      altitude(0),
      posEcef(),
      velEcef(),
      hMSL(0),
      hAcc(0),
      vAcc(0),
      sAcc(0),
      pDop(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      offset += this->header.serialize(outbuffer + offset);
      union {
        int8_t real;
        uint8_t base;
      } u_num_sat;
      u_num_sat.real = this->num_sat;
      *(outbuffer + offset + 0) = (u_num_sat.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->num_sat);
      *(outbuffer + offset + 0) = (this->fix_type >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->fix_type >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->fix_type >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->fix_type >> (8 * 3)) & 0xFF;
      offset += sizeof(this->fix_type);
      union {
        int32_t real;
        uint32_t base;
      } u_cno;
      u_cno.real = this->cno;
      *(outbuffer + offset + 0) = (u_cno.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_cno.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_cno.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_cno.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->cno);
      offset += serializeAvrFloat64(outbuffer + offset, this->latitude);
      offset += serializeAvrFloat64(outbuffer + offset, this->longitude);
      offset += serializeAvrFloat64(outbuffer + offset, this->altitude);
      offset += this->posEcef.serialize(outbuffer + offset);
      offset += this->velEcef.serialize(outbuffer + offset);
      union {
        float real;
        uint32_t base;
      } u_hMSL;
      u_hMSL.real = this->hMSL;
      *(outbuffer + offset + 0) = (u_hMSL.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_hMSL.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_hMSL.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_hMSL.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->hMSL);
      union {
        float real;
        uint32_t base;
      } u_hAcc;
      u_hAcc.real = this->hAcc;
      *(outbuffer + offset + 0) = (u_hAcc.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_hAcc.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_hAcc.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_hAcc.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->hAcc);
      union {
        float real;
        uint32_t base;
      } u_vAcc;
      u_vAcc.real = this->vAcc;
      *(outbuffer + offset + 0) = (u_vAcc.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_vAcc.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_vAcc.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_vAcc.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->vAcc);
      union {
        float real;
        uint32_t base;
      } u_sAcc;
      u_sAcc.real = this->sAcc;
      *(outbuffer + offset + 0) = (u_sAcc.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_sAcc.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_sAcc.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_sAcc.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->sAcc);
      union {
        float real;
        uint32_t base;
      } u_pDop;
      u_pDop.real = this->pDop;
      *(outbuffer + offset + 0) = (u_pDop.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_pDop.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_pDop.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_pDop.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->pDop);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      offset += this->header.deserialize(inbuffer + offset);
      union {
        int8_t real;
        uint8_t base;
      } u_num_sat;
      u_num_sat.base = 0;
      u_num_sat.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->num_sat = u_num_sat.real;
      offset += sizeof(this->num_sat);
      this->fix_type =  ((uint32_t) (*(inbuffer + offset)));
      this->fix_type |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      this->fix_type |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      this->fix_type |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      offset += sizeof(this->fix_type);
      union {
        int32_t real;
        uint32_t base;
      } u_cno;
      u_cno.base = 0;
      u_cno.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_cno.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_cno.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_cno.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->cno = u_cno.real;
      offset += sizeof(this->cno);
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->latitude));
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->longitude));
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->altitude));
      offset += this->posEcef.deserialize(inbuffer + offset);
      offset += this->velEcef.deserialize(inbuffer + offset);
      union {
        float real;
        uint32_t base;
      } u_hMSL;
      u_hMSL.base = 0;
      u_hMSL.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_hMSL.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_hMSL.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_hMSL.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->hMSL = u_hMSL.real;
      offset += sizeof(this->hMSL);
      union {
        float real;
        uint32_t base;
      } u_hAcc;
      u_hAcc.base = 0;
      u_hAcc.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_hAcc.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_hAcc.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_hAcc.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->hAcc = u_hAcc.real;
      offset += sizeof(this->hAcc);
      union {
        float real;
        uint32_t base;
      } u_vAcc;
      u_vAcc.base = 0;
      u_vAcc.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_vAcc.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_vAcc.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_vAcc.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->vAcc = u_vAcc.real;
      offset += sizeof(this->vAcc);
      union {
        float real;
        uint32_t base;
      } u_sAcc;
      u_sAcc.base = 0;
      u_sAcc.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_sAcc.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_sAcc.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_sAcc.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->sAcc = u_sAcc.real;
      offset += sizeof(this->sAcc);
      union {
        float real;
        uint32_t base;
      } u_pDop;
      u_pDop.base = 0;
      u_pDop.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_pDop.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_pDop.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_pDop.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->pDop = u_pDop.real;
      offset += sizeof(this->pDop);
     return offset;
    }

    const char * getType(){ return "inertial_sense/GPS"; };
    const char * getMD5(){ return "6aa847d654b817ff4bb5ba8c773b2a17"; };

  };

}
#endif
