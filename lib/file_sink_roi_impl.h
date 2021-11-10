/* -*- c++ -*- */
/* 
 * Copyright 2021 <+YOU OR YOUR COMPANY+>.
 * 
 * This is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 3, or (at your option)
 * any later version.
 * 
 * This software is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 * 
 * You should have received a copy of the GNU General Public License
 * along with this software; see the file COPYING.  If not, write to
 * the Free Software Foundation, Inc., 51 Franklin Street,
 * Boston, MA 02110-1301, USA.
 */

#ifndef INCLUDED_ROI_FILE_SINK_ROI_IMPL_H
#define INCLUDED_ROI_FILE_SINK_ROI_IMPL_H

#include <roi/file_sink_roi.h>
#include <gnuradio/fft/fft.h>
#include <boost/thread/mutex.hpp>
#include <cstdio>
#include <pthread.h>


namespace gr {
  namespace roi {

    class file_sink_roi_impl : public file_sink_roi
    {
        typedef gr::fft::fft_complex fft_complex;
        const int NUM=2192;
        const int PSS_LEN=2192;

    private:

         boost::mutex mutex;
         boost::mutex fp_mutex;


        bool status_file; // 标识文件中数据是否有效
        float d_threshold;
        int d_cell_id;
        float d_proportion;

        fft_complex *d_fft;
        unsigned int d_fft_size;
        bool d_forward;
        bool d_shift;
        std::vector<float> d_window;

        float d_energe;

        bool corr_start=false;

        int cnt=0;

        pmt::pmt_t d_port;

        int d_latency;

     public:
        file_sink_roi_impl(const char *filename,bool append, int cell_id,float threshold,float proportion,int fft_size, bool forward, const std::vector<float> &window, bool shift, int nthreads,float energe);
      ~file_sink_roi_impl();

      // Where all the action really happens
//      void forecast (int noutput_items, gr_vector_int &ninput_items_required);

        bool get_status_file(){return status_file;}
        void set_status_file(bool _status_file){
            gr::thread::scoped_lock lock(mutex);
            status_file = _status_file;
        }

        void send_message();
        std::vector<float> xcorr(const gr_complex* in,const gr_complex* data,int num_input,int num_data);
        void find_max(std::vector<float>  output_abs,int &maxindex);

          bool set_window(const std::vector<float> &window);
        std::vector<float> do_fft(const gr_complex *in);
        bool detect_energe(const std::vector<float> &fft_abs,const float * detect_window);

        int general_work(int noutput_items,
           gr_vector_int &ninput_items,
           gr_vector_const_void_star &input_items,
           gr_vector_void_star &output_items);
    };

  } // namespace roi
} // namespace gr

#endif /* INCLUDED_ROI_FILE_SINK_ROI_IMPL_H  */

