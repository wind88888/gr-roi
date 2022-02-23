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
        const int PSSCH_LEN=30720;

    private:

         boost::mutex mutex;
         boost::mutex fp_mutex;


        bool status_file; // 标识文件中数据是否有效
        float d_threshold;//PSSS相关阈值
        float d_threshold_DMRS;
        int d_cell_id;//cell_id

        float d_energe;//能量检测阈值
        float d_energe_DMRS;
        bool corr_start=false;//是否进行相关
//        float d_proportion;//相关次峰与主峰的比例
        unsigned d_detect_wait=0;//检测失败等待次数

        fft_complex *d_fft;
        unsigned int d_fft_size;
        bool d_forward;
        bool d_shift;
        std::vector<float> d_window;


        int d_waitslot=0;//用来计数等待的点数以对应时隙
        int d_timeslot;
        int cnt=0;//存储计数

        int d_save_status=-1;//-1 for no pss has been found;0 for pss has been found ,wait for PSSCH;1 for PSSCH saving.

        pmt::pmt_t d_port;
//        pmt::pmt_t d_port_rx;//需要删除
//        bool rx_file;//需要删除
        int receive_times;

        int d_latency;//收发转换间隔
        bool d_alice;
        bool d_alice_pssfound;
        int d_count;//用来矫正Alice端接收
        bool d_alice_rec=false;
        int d_receive_length=1;


        int times=0;

        public:
            //新增d_energe_DMRS，d_threshold_DMRS
            file_sink_roi_impl(const char *filename,bool append, int cell_id,float threshold,float threshold_DMRS,int rec_len,int fft_size, bool forward,
                               const std::vector<float> &window, bool shift, int nthreads,float energe,float energe_DMRS,int latency,int time_slot,bool alice);
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
//        bool detect_energe(const std::vector<float> &fft_abs,const float * detect_window);
          bool detect_energe(const std::vector<float> &fft_abs);
          bool detect_energe_PSSCH(const std::vector<float> &fft_abs);
          // 接收方消息处理
          void msg_handler(pmt::pmt_t msg);
          void set_rx_file(bool _rx_file);
          bool get_rx_file();

        int general_work(int noutput_items,
           gr_vector_int &ninput_items,
           gr_vector_const_void_star &input_items,
           gr_vector_void_star &output_items);
    };

  } // namespace roi
} // namespace gr

#endif /* INCLUDED_ROI_FILE_SINK_ROI_IMPL_H  */

