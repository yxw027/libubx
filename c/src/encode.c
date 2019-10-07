/*
 * Copyright (C) 2019 Swift Navigation Inc.
 * Contact: Swift Navigation <dev@swiftnav.com>
 *
 * This source is subject to the license found in the file 'LICENSE' which must
 * be distributed together with this source. All other rights reserved.
 *
 * THIS CODE AND INFORMATION IS PROVIDED "AS IS" WITHOUT WARRANTY OF ANY KIND,
 * EITHER EXPRESSED OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND/OR FITNESS FOR A PARTICULAR PURPOSE.
 */

/* These encoder functions are provided to allow for easier unit testing however
 * they are not robust to be used in production. Before using with real data, we
 * would need to handle ambiguity rollover and code carrier divergance at least
 */

#include <ubx/encode.h>

#include <swiftnav/bits.h>

/** Set bit field in buffer from an unsigned integer.
 * Packs `len` bits into bit position `pos` from the start of the buffer.
 * Maximum bit field length is 64 bits, i.e. `len <= 64`.
 *
 * \param buff
 * \param pos Position in buffer of start of bit field in bits.
 * \param len Length of bit field in bits.
 * \param data Unsigned integer to be packed into bit field.
 */
void ubx_setbitul(uint8_t *buff, uint32_t pos, uint32_t len, uint64_t data) {
  uint64_t mask = ((uint64_t)1) << (len - 1);

  if (len <= 0 || 64 < len) {
    return;
  }

  for (uint32_t i = pos; i < pos + len; i++, mask >>= 1) {
    if (data & mask) {
      buff[i / 8] |= ((uint64_t)1) << (7 - i % 8);
    } else {
      buff[i / 8] &= ~(((uint64_t)1) << (7 - i % 8));
    }
  }
}

/** Set bit field in buffer from a signed integer.
 * Packs `len` bits into bit position `pos` from the start of the buffer.
 * Maximum bit field length is 32 bits, i.e. `len <= 32`.
 *
 * \param buff
 * \param pos Position in buffer of start of bit field in bits.
 * \param len Length of bit field in bits.
 * \param data Signed integer to be packed into bit field.
 */
void ubx_setbitsl(uint8_t *buff, uint32_t pos, uint32_t len, int64_t data) {
  ubx_setbitul(buff, pos, len, (uint64_t)data);
}

/** Serialize the ubx_rawx message
 *
 * \param buff outgoing data buffer
 * \param msg_rawx UBX rawx message to serialize
 * \return number of bits serialized
 */
uint16_t ubx_encode_rawx(const ubx_rawx *msg_rawx, uint8_t buff[]) {
  assert(msg_rawx);

  uint16_t bit = 0;
  setbitu(buff, bit, 8, msg_rawx->class_id);
  bit += 8;
  setbitu(buff, bit, 8, msg_rawx->msg_id);
  bit += 8;

  ubx_setbitul(buff, bit, 64, *((int64_t *)&msg_rawx->rcv_tow));
  bit += 64;
  setbitu(buff, bit, 16, msg_rawx->rcv_wn);
  bit += 16;
  setbits(buff, bit, 16, msg_rawx->leap_second);
  bit += 16;
  setbitu(buff, bit, 8, msg_rawx->num_meas);
  bit += 8;
  setbitu(buff, bit, 8, msg_rawx->rec_status);
  bit += 8;
  setbitu(buff, bit, 8, msg_rawx->version);
  bit += 8;
  setbitu(buff, bit, 4, 0);
  bit += 4;

  for (int i = 0; i < msg_rawx->num_meas; i++) {
    ubx_setbitul(buff, bit, 64, *((int64_t *)&msg_rawx->pseudorange_m[i]));
    bit += 64;
    ubx_setbitul(buff, bit, 64,
                 *((int64_t *)&msg_rawx->carrier_phase_cycles[i]));
    bit += 64;
    setbitu(buff, bit, 32, *((int32_t *)&msg_rawx->doppler_hz[i]));
    bit += 32;
    setbitu(buff, bit, 8, msg_rawx->gnss_id[i]);
    bit += 8;
    setbitu(buff, bit, 8, msg_rawx->sat_id[i]);
    bit += 8;
    setbitu(buff, bit, 8, msg_rawx->sig_id[i]);
    bit += 8;
    setbits(buff, bit, 8, msg_rawx->freq_id[i]);
    bit += 8;
    setbitu(buff, bit, 8, msg_rawx->lock_time[i]);
    bit += 8;
    setbitu(buff, bit, 16, msg_rawx->cno_dbhz[i]);
    bit += 16;
    setbitu(buff, bit, 8, msg_rawx->pr_std_m[i]);
    bit += 8;
    setbitu(buff, bit, 8, msg_rawx->cp_std_cycles[i]);
    bit += 8;
    setbitu(buff, bit, 8, msg_rawx->doppler_std_hz[i]);
    bit += 8;
    setbitu(buff, bit, 8, msg_rawx->track_state[i]);
    bit += 8;
    setbitu(buff, bit, 8, 0);
    bit += 8;
  }
  return bit;
}

/** Serialize the ubx_rawx message
 *
 * \param buff outgoing data buffer
 * \param msg_gps_eph UBX gps eph message to serialize
 * \return number of bits serialized
 */
uint16_t ubx_encode_gps_eph(const ubx_gps_eph *msg_gps_eph, uint8_t buff[]) {
  assert(msg_gps_eph);

  uint16_t bit = 0;
  setbitu(buff, bit, 8, msg_gps_eph->class_id);
  bit += 8;
  setbitu(buff, bit, 8, msg_gps_eph->msg_id);
  bit += 8;
  setbitu(buff, bit, 8, msg_gps_eph->msg_type);
  bit += 8;
  setbitu(buff, bit, 8, msg_gps_eph->version);
  bit += 8;
  setbitu(buff, bit, 8, msg_gps_eph->sat_id);
  bit += 8;
  setbitu(buff, bit, 8, msg_gps_eph->fit_interval);
  bit += 8;
  setbitu(buff, bit, 8, msg_gps_eph->ura_index);
  bit += 8;
  setbitu(buff, bit, 8, msg_gps_eph->sat_health);
  bit += 8;
  setbits(buff, bit, 8, msg_gps_eph->tgd);
  bit += 8;
  setbits(buff, bit, 16, msg_gps_eph->iode);
  bit += 16;
  setbits(buff, bit, 16, msg_gps_eph->toc);
  bit += 16;
  setbitu(buff, bit, 8, 0); /* reserved */
  bit += 8;
  setbits(buff, bit, 8, msg_gps_eph->af2);
  bit += 8;
  setbits(buff, bit, 16, msg_gps_eph->af1);
  bit += 16;
  setbits(buff, bit, 32, msg_gps_eph->af0);
  bit += 32;
  setbits(buff, bit, 16, msg_gps_eph->crs);
  bit += 16;
  setbits(buff, bit, 16, msg_gps_eph->delta_N);
  bit += 16;
  setbits(buff, bit, 32, msg_gps_eph->m0);
  bit += 32;
  setbits(buff, bit, 16, msg_gps_eph->cuc);
  bit += 16;
  setbits(buff, bit, 16, msg_gps_eph->cus);
  bit += 16;
  setbits(buff, bit, 32, msg_gps_eph->e);
  bit += 32;
  setbits(buff, bit, 32, msg_gps_eph->sqrt_A);
  bit += 32;
  setbitu(buff, bit, 16, msg_gps_eph->toe);
  bit += 16;
  setbits(buff, bit, 16, msg_gps_eph->cic);
  bit += 16;
  setbits(buff, bit, 32, msg_gps_eph->omega0);
  bit += 32;
  setbits(buff, bit, 16, msg_gps_eph->cis);
  bit += 16;
  setbits(buff, bit, 16, msg_gps_eph->crc);
  bit += 16;
  setbits(buff, bit, 32, msg_gps_eph->i0);
  bit += 32;
  setbits(buff, bit, 32, msg_gps_eph->omega);
  bit += 32;
  setbits(buff, bit, 32, msg_gps_eph->omega_dot);
  bit += 32;
  setbits(buff, bit, 16, msg_gps_eph->i_dot);
  bit += 16;

  return bit;
}
