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


#include <stdio.h>
#include <stdlib.h>
#ifndef _WIN32
#include <unistd.h>
#endif
#include <string.h>
#include <pthread.h>
#define _USE_MATH_DEFINES
#include <math.h>
#include <complex>

#include "usrp_source.h"

extern int g_verbosity;


#ifdef _WIN32
inline double
round (double x)
{
  return floor (x + 0.5);
}
#endif

usrp_source::usrp_source (float sample_rate, long int fpga_master_clock_freq)
{

  m_fpga_master_clock_freq = fpga_master_clock_freq;
  m_desired_sample_rate = sample_rate;
  m_sample_rate = 0.0;
  m_decimation = 0;
  m_cb = new circular_buffer (CB_LEN, sizeof (complex), 0);

  pthread_mutex_init (&m_u_mutex, 0);
}


usrp_source::usrp_source (unsigned int decimation,
			  long int fpga_master_clock_freq)
{

  m_fpga_master_clock_freq = fpga_master_clock_freq;
  m_sample_rate = 0.0;
  m_cb = new circular_buffer (CB_LEN, sizeof (complex), 0);

  pthread_mutex_init (&m_u_mutex, 0);

  m_decimation = decimation & ~1;
  if (m_decimation < 4)
    m_decimation = 4;
  if (m_decimation > 256)
    m_decimation = 256;
}


usrp_source::~usrp_source ()
{

  stop ();
  delete m_cb;
  hackrf_close (dev);
  pthread_mutex_destroy (&m_u_mutex);
}


void
usrp_source::stop ()
{

  pthread_mutex_lock (&m_u_mutex);

  int result;
  result = hackrf_stop_rx (dev);
  if (result != HACKRF_SUCCESS)
    {
      printf ("hackrf_stop_rx() failed: %s (%d)\n",
	      hackrf_error_name ((hackrf_error) result), result);
      pthread_mutex_unlock (&m_u_mutex);
      exit (1);
    }
  pthread_mutex_unlock (&m_u_mutex);
}


void
usrp_source::start ()
{

  pthread_mutex_lock (&m_u_mutex);

  int result;
  result = hackrf_start_rx (dev, hackrf_rx_callback, this);
  if (result != HACKRF_SUCCESS)
    {
      printf ("hackrf_start_rx() failed: %s (%d)\n",
	      hackrf_error_name ((hackrf_error) result), result);
      pthread_mutex_unlock (&m_u_mutex);
      exit (1);
    }
  pthread_mutex_unlock (&m_u_mutex);
}


void
usrp_source::calculate_decimation ()
{

  float decimation_f;

//      decimation_f = (float)m_u_rx->fpga_master_clock_freq() / m_desired_sample_rate;
  m_decimation = (unsigned int) round (decimation_f) & ~1;

  if (m_decimation < 4)
    m_decimation = 4;
  if (m_decimation > 256)
    m_decimation = 256;
}


float
usrp_source::sample_rate ()
{

  return m_sample_rate;

}


int
usrp_source::tune (double freq)
{

  int r = 0;

  pthread_mutex_lock (&m_u_mutex);
  if (freq != m_center_freq)
    {

      r = hackrf_set_freq (dev, (uint64_t) freq);
      if (g_verbosity)
	printf ("hackrf_set_freq: %d\n", int (freq));

      if (r < 0)
	fprintf (stderr, "Tuning failed!\n");
      else
	m_center_freq = freq;
    }

  pthread_mutex_unlock (&m_u_mutex);

  return 1;			//(r < 0) ? 0 : 1;
}

int
usrp_source::set_freq_correction (int ppm)
{
  m_freq_corr = ppm;
  //return rtlsdr_set_freq_correction(dev, ppm);
  return 0;			// TODO: add support for ppm correction
}

bool
usrp_source::set_antenna (int antenna)
{

  return 0;
}

bool
usrp_source::set_gain (int amp_gain, int lna_gain, int vga_gain)
{
  int r = 0;

  lna_gain = (((lna_gain + 7) / 8) * 8);
  vga_gain = (((vga_gain + 1) / 2) * 2);

  if (lna_gain > 40)
    lna_gain = 40;
  if (vga_gain > 62)
    vga_gain = 62;
  if (g_verbosity)
    printf ("hackrf: set gain %d/%d/%d\n", amp_gain, vga_gain, lna_gain);

  if (amp_gain)
    r = hackrf_set_amp_enable (dev, amp_gain);
  if (vga_gain)
    r |= hackrf_set_vga_gain (dev, vga_gain);
  if (lna_gain)
    r |= hackrf_set_lna_gain (dev, lna_gain);
  return (r < 0) ? 0 : 1;
}


/*
 * open() should be called before multiple threads access usrp_source.
 */
int
usrp_source::open (unsigned int subdev)
{
  int i, r, device_count, count;
  uint32_t dev_index = subdev;
  uint32_t samp_rate;

  samp_rate = m_fpga_master_clock_freq; // from constructor
  m_sample_rate = 1000000;


  if (g_verbosity)
    printf ("hackrf_init()\n");
  r = hackrf_init ();
  if (r != HACKRF_SUCCESS)
    {
      printf ("hackrf_init() failed.");
      return EXIT_FAILURE;
    }

  if (g_verbosity)
    printf ("hackrf_open()\n");
  r = hackrf_open (&dev);
  if (r != HACKRF_SUCCESS)
    {
      fprintf (stderr, "Failed to open hackrf device.\n");
      exit (1);
    }

  /* Set the sample rate */
  r = hackrf_set_sample_rate (dev, samp_rate);
  if (g_verbosity)
    printf ("hackrf_set_sample_rate(%u)\n", samp_rate);
  if (r != HACKRF_SUCCESS)
    fprintf (stderr, "WARNING: Failed to set sample rate.\n");

  r = hackrf_set_baseband_filter_bandwidth (dev, 2500000);
  if (r != HACKRF_SUCCESS)
    {
      printf ("hackrf_baseband_filter_bandwidth_set() failed: \n");
      return EXIT_FAILURE;
    }
  return 0;
}

#define USB_PACKET_SIZE		(2 * 16384)
#define FLUSH_SIZE		512

int
hackrf_rx_callback (hackrf_transfer * transfer)
{
  //printf("hackrf_rx_callback()\n");
  usrp_source *u;
  u = (usrp_source *) (transfer->rx_ctx);

  size_t bytes_to_write;
  size_t hackrf_rx_count_new = u->hackrf_rx_count + transfer->valid_length;

  int count_left = USB_PACKET_SIZE - hackrf_rx_count_new;
  if (count_left <= 0)
    {
      bytes_to_write = transfer->valid_length + count_left;
    }
  else
    {
      bytes_to_write = transfer->valid_length;
    }

  //  cout << transfer->valid_length  << " " << hackrf_rx_count << " " << bytes_to_write << "\n";
  if (bytes_to_write != 0)
    {
      memcpy (u->ubuf + u->hackrf_rx_count, transfer->buffer, bytes_to_write);
      //    for (size_t i=0; i<bytes_to_write; i++) {
      //      hackrf_rx_buf[hackrf_rx_count+i] = transfer->buffer[i];
      //    }
      u->hackrf_rx_count = u->hackrf_rx_count + bytes_to_write;
    }
  //  cout << transfer->valid_length  << " " << hackrf_rx_count << " " << bytes_to_write << "\n";

  return (0);
}


int
usrp_source::fill (unsigned int num_samples, unsigned int *overrun_i)
{

  unsigned int i, j, space, overruns = 0;
  complex *c;
  int n_read;
  int result;

  //printf("start fill..\n");

  while ((m_cb->data_available () < num_samples)
	 && (m_cb->space_available () > 0))
    {

      // read one usb packet from hackrf
      pthread_mutex_lock (&m_u_mutex);


      // fill ubuf with sizeof(ubuf) sample points. n_read to indicate samples actually wrote.
      hackrf_rx_count = 0;	// clear counter                                                                                                         
      while (hackrf_is_streaming (dev) != HACKRF_TRUE)
	{
	  printf ("waiting for streaming...(%d)\n",
		  hackrf_is_streaming (dev));
	}

      while (hackrf_is_streaming (dev) == HACKRF_TRUE)
	{
	  //printf("%d\n",hackrf_rx_count );
	  if (hackrf_rx_count == USB_PACKET_SIZE)
	    {
	      break;
	    }
	}

      n_read = hackrf_rx_count;

      pthread_mutex_unlock (&m_u_mutex);

      // write complex<short> input to complex<float> output
      c = (complex *) m_cb->poke (&space);

      // set space to number of complex items to copy
      space = n_read / 2;

      // write data
      for (i = 0, j = 0; i < space; i += 1, j += 2)
	//c[i] = complex((ubuf[j] - 127) * 256, (ubuf[j + 1] - 127) * 256);
	c[i] = complex (ubuf[j] * 256, ubuf[j + 1] * 256);

      // update cb
      m_cb->wrote (i);
    }

  // if the cb is full, we left behind data from the usb packet
  if (m_cb->space_available () == 0)
    {
      fprintf (stderr, "warning: local overrun\n");
      overruns++;
    }

  if (overrun_i)
    *overrun_i = overruns;

  return 0;
}



/*
 * Don't hold a lock on this and use the usrp at the same time.
 */
circular_buffer *
usrp_source::get_buffer ()
{

  return m_cb;
}


int
usrp_source::flush (unsigned int flush_count)
{

  m_cb->flush ();
  //fill(flush_count * FLUSH_SIZE, 0);
  m_cb->flush ();

  return 0;
}
