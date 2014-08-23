/*
 * Copyright (c) 2010, Joshua Lackey
 * All rights reserved.
 * 
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 * 
 *     *  Redistributions of source code must retain the above copyright
 *        notice, this list of conditions and the following disclaimer.
 *
 *     *  Redistributions in binary form must reproduce the above copyright
 *        notice, this list of conditions and the following disclaimer in the
 *        documentation and/or other materials provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#include "libhackrf/hackrf.h"

#include "usrp_complex.h"
#include "circular_buffer.h"

int hackrf_rx_callback (hackrf_transfer * transfer);

class usrp_source
{
public:
  usrp_source (float sample_rate, long int fpga_master_clock_freq = 52000000);
    usrp_source (unsigned int decimation, long int fpga_master_clock_freq =
		 52000000);
   ~usrp_source ();

  int open (unsigned int subdev);
  int fill (unsigned int num_samples, unsigned int *overrun);
  int tune (double freq);
  int set_freq_correction (int ppm);
  bool set_antenna (int antenna);
  bool set_gain (int amp_gain, int lna_gain, int vga_gain);
  void start ();
  void stop ();
  int flush (unsigned int flush_count = FLUSH_COUNT);
  circular_buffer *get_buffer ();

  float sample_rate ();

  static const unsigned int side_A = 0;
  static const unsigned int side_B = 1;

  double m_center_freq;
  int m_freq_corr;

  int hackrf_rx_count;		// used for and hackrf rx callback
#define USB_PACKET_SIZE     (2 * 16384)
  int8_t ubuf[USB_PACKET_SIZE];	// used for hackrf rx callback


private:
  void calculate_decimation ();

  hackrf_device *dev;

  float m_sample_rate;
  float m_desired_sample_rate;
  unsigned int m_decimation;

  long int m_fpga_master_clock_freq;

  circular_buffer *m_cb;

  /*
   * This mutex protects access to the USRP and daughterboards but not
   * necessarily to any fields in this class.
   */
  pthread_mutex_t m_u_mutex;

  static const unsigned int FLUSH_COUNT = 10;
  static const unsigned int CB_LEN = (16 * 16384);
  static const int NCHAN = 1;
  static const int INITIAL_MUX = -1;
  static const int FUSB_BLOCK_SIZE = 1024;
  static const int FUSB_NBLOCKS = 16 * 8;
  static const char *FPGA_FILENAME ()
  {
    return "std_2rxhb_2tx.rbf";
  }
};
