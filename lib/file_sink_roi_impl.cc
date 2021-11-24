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

#ifdef HAVE_CONFIG_H
#include "config.h"
#endif

#include <gnuradio/io_signature.h>
#include <gnuradio/thread/thread.h>
#include "file_sink_roi_impl.h"
#include <volk/volk.h>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <ctime>
#include <sys/time.h>


namespace gr {
    namespace roi {

        file_sink_roi::sptr
        file_sink_roi::make(const char* filename, bool append, int cell_id,float threshold,float proportion,
                            int fft_size, bool forward, const std::vector<float> &window, bool shift, int nthreads,//used for fft
                            float energe,int latency,int time_slot
        )
        {
            return gnuradio::get_initial_sptr
                    (new file_sink_roi_impl(filename, append,cell_id, threshold,proportion, fft_size, forward, window, shift, nthreads,energe,latency,time_slot));
        }


        /*
         * The private constructor
         */
        file_sink_roi_impl::file_sink_roi_impl(const char* filename, bool append, int cell_id,float threshold,float proportion,int fft_size, bool forward, const std::vector<float> &window, bool shift, int nthreads,float energe,int latency,int time_slot)
                : gr::block("file_sink_roi",
                            gr::io_signature::make(1, 1, sizeof(gr_complex)),
                            gr::io_signature::make(0, 0, 0)) ,
                  file_sink_base(filename, true, append),
                  d_latency(latency),
                  status_file(false),
                  d_threshold(threshold),
                  d_cell_id(cell_id),
                  d_proportion(proportion),
                  d_fft_size(fft_size),
                  d_forward(forward),
                  d_shift(shift),
                  d_energe(energe),
                  d_timeslot(time_slot)
                  {
                      set_relative_rate(1.0 / 9000);
                      d_port = pmt::mp("msg_status_file");
                      message_port_register_out(d_port);

                      std::cout<<"threshold = "<<d_threshold<<"proportion = "<<d_proportion<<"fft_size="<<d_fft_size<<std::endl;
                      d_fft = new fft_complex(d_fft_size, forward, nthreads);
                      if (!set_window(window)) {
                          throw std::runtime_error("fft_vcc: window not the same length as fft_size\n");
                      }
        }
        std::vector<float> file_sink_roi_impl::xcorr(const gr_complex* in,const gr_complex* data,int num_input,int num_data){
//            gr_complex *input = new gr_complex[num_input+num_data];
//            printf("doing xcorr\n");
//            for(int i=0;i<(num_input+num_data);i++)
//            {
//                if(i<num_input){
//                    input[i]=in[i];
//                }else{
//                    input[i]=0;
//                }
//            }
            std::vector<gr_complex> output(num_input-num_data);
            for(int i=0;i<(num_input-num_data);i++)
//                for(int i=0;i<(num_input);i++)
            {
                for(int j=0;j<num_data;j++)
                    output[i]=output[i]+conj(in[i+j])*data[j];
            }
//            delete [] input;
            std::vector<float>output_abs(num_input-num_data);
            for(int i=0;i<(num_input-num_data);i++)
            {
                output_abs[i]=abs(output[i]);
            }
//            printf("xcorr success\n");
            return output_abs;

        }



        void file_sink_roi_impl::find_max(std::vector<float>  output_abs,int &maxindex){
            int len=output_abs.size();
//            printf("len=%d,doing find_maxtwo\n",len);
            float maxabs=output_abs[0];
            maxindex=0;
            for(int i=1;i<len;i++){
                if(output_abs[i]>maxabs){
                    maxabs=output_abs[i];
                    maxindex=i;
                }
            }
//            printf("find_maxtwo done\n");
        }



        /*
         * Our virtual destructor.
         */
        file_sink_roi_impl::~file_sink_roi_impl() {
        }






//    void
//    file_sink_roi_impl::forecast (int noutput_items, gr_vector_int &ninput_items_required)
//    {
//      /* <+forecast+> e.g. ninput_items_required[0] = noutput_items */
//    }
            void file_sink_roi_impl::send_message() {
                pmt::pmt_t msg_ctl = pmt::make_dict();
                msg_ctl = pmt::dict_add(msg_ctl, pmt::string_to_symbol("status_file"), pmt::from_bool(true));

                pmt::pmt_t msg_data = pmt::make_vector(0, pmt::from_long(0));
//            pmt::pmt_t msg = pmt::cons(msg_ctl, pmt::make_u8vector(0, 0));
                pmt::pmt_t msg = pmt::cons(msg_ctl, msg_data);
                printf("send message start\n");
                message_port_pub(d_port, msg);
                printf("send message end\n");
            }

        bool file_sink_roi_impl::set_window(const std::vector<float> &window) {
            if (window.size() == 0 || window.size() == d_fft_size) {
                d_window = window;
                return true;
            } else {
                return false;
            }
        }

        std::vector<float> file_sink_roi_impl::do_fft(const gr_complex *in) {
            // 处理fft的输入
            if (d_window.size()) {
                gr_complex *dst = d_fft->get_inbuf();
                if (!d_forward && d_shift) {
                    unsigned int offset = (!d_forward && d_shift)?(d_fft_size/2):0;
                    int fft_m_offset = d_fft_size - offset;
                    volk_32fc_32f_multiply_32fc(&dst[fft_m_offset], &in[0], &d_window[0], offset);
                    volk_32fc_32f_multiply_32fc(&dst[0], &in[offset], &d_window[offset], d_fft_size-offset);
                } else {
                    volk_32fc_32f_multiply_32fc(&dst[0], in, &d_window[0], d_fft_size);
                }
            } else {
                if(!d_forward && d_shift) {  // apply an ifft shift on the data
                    gr_complex *dst = d_fft->get_inbuf();
                    unsigned int len = (unsigned int)(floor(d_fft_size/2.0)); // half length of complex array
                    memcpy(&dst[0], &in[len], sizeof(gr_complex)*(d_fft_size - len));
                    memcpy(&dst[d_fft_size - len], &in[0], sizeof(gr_complex)*len);
                }
                else {
                    memcpy(d_fft->get_inbuf(), in, sizeof(gr_complex) * d_fft_size);
                }
            }

            d_fft->execute(); // 计算fft

            // 获取输出
            gr_complex *fft_output = new gr_complex[d_fft_size];
            if(d_forward && d_shift) {  // apply a fft shift on the data
                unsigned int len = (unsigned int)(ceil(d_fft_size/2.0));
                memcpy(&fft_output[0], &d_fft->get_outbuf()[len], sizeof(gr_complex)*(d_fft_size - len));
                memcpy(&fft_output[d_fft_size - len], &d_fft->get_outbuf()[0], sizeof(gr_complex)*len);
            }
            else {
                memcpy (fft_output, d_fft->get_outbuf (), sizeof(gr_complex) * d_fft_size);
            }

            // 将输出的结果取模
            // detect whether res is sine or not
            std::vector<float> fft_abs;
            for (int i=0;i<d_fft_size;i++) {
                fft_abs.push_back(sqrt(pow((fft_output+i)->real(), 2) + pow((fft_output+i)->imag(), 2)));
            }

            delete fft_output; // 销毁并回收内存

            return fft_abs;
        }
        bool file_sink_roi_impl::detect_energe(const std::vector<float> &fft_abs) {
            float sum_signal=0.0;
            float sum_noise=0.0;
            float sum_window=36.0;
            sum_signal=fft_abs[0]*0.7;
            for(int i=1;i<18;i++){
                sum_signal+=fft_abs[i];
            }
            for(int i=d_fft_size;i>(d_fft_size-18);i--){
                sum_signal+=fft_abs[i];
            }
            sum_noise=fft_abs[0]*0.3;
            for(int i=18;i<(d_fft_size-18);i++){
                sum_noise+=fft_abs[i];
            }


            sum_signal=sum_signal/sum_window;
            sum_noise=sum_noise/(d_fft_size-sum_window);

            float snr_ratio=sum_signal/sum_noise;
//            printf("snr_ratio = %f", snr_ratio);

            if(snr_ratio>d_energe){
                return true;}

            return false;
        }

//        bool file_sink_roi_impl::detect_energe(const std::vector<float> &fft_abs,const float *detect_window){
//            float sum_signal=0.0;
//            float sum_noise=0.0;
//            float sum_window=0.0;
//            for(int i=0;i<d_fft_size;i++){
//                sum_signal+=fft_abs[i]*detect_window[i];
//                sum_noise-=fft_abs[i]*(detect_window[i]-1);
//            }
//            for(int i=0;i<d_fft_size;i++)
//                sum_window+=detect_window[i];
//
//            sum_signal=sum_signal/sum_window;
//            sum_noise=sum_noise/(d_fft_size-sum_window);
//
//            float snr_ratio=sum_signal/sum_noise;
//            if(snr_ratio>d_energe)
//                return true;
//            return false;
//        }

        int
            file_sink_roi_impl::general_work(int noutput_items,
                                                 gr_vector_int &ninput_items,
                                                 gr_vector_const_void_star &input_items,
                                                 gr_vector_void_star &output_items) {
            int ret=0;// 记录消耗的item数目
            int First_Detection=0;
            int input_items_num=ninput_items[0];
//            std::cout<<"input_items_num="<<input_items_num<<"ret="<<ret<<std::endl;

//                printf("ret=%d\n",ret);

            const gr_complex *in = (const gr_complex *) input_items[0];


           switch(d_save_status){

               case 1:
                   printf("now start to save PSSCH, %d/%d items have been saved",cnt,PSSCH_LEN);

                   struct timeval timer;
                   gettimeofday(&timer, NULL);
                   std::cout << "receive time: " << timer.tv_sec << "s " << timer.tv_usec << "us"
                   << std::endl;
//                   gr::thread::scoped_lock lock(mutex);
                   do_update();
                   if(!d_fp)
                    return noutput_items;
                   if(cnt==0){
                       ftruncate(fileno(d_fp), 0);
                       rewind(d_fp);
                   }

                if(cnt+input_items_num<=PSSCH_LEN) {
                    while (ret<input_items_num) {
                        int count = fwrite(in, sizeof(gr_complex), input_items_num-ret, d_fp);
                        if (count == 0) {
                            if (ferror(d_fp)) {
                                std::stringstream s;
                                s << "file_sink write failed with error " << fileno(d_fp) << std::endl;
                                throw std::runtime_error(s.str());
                            } else { // is EOF
                                break;
                            }
                        }
                        ret += count;
                        in += count;
                        cnt+=count;
                    }
                    if(cnt==PSSCH_LEN){
                        printf("PSSCH has been saved, items number =%d \n",cnt);
                        d_save_status=-1;
                        cnt=0;
                    }

                }else{
                    int count = fwrite(in, sizeof(gr_complex), PSSCH_LEN-cnt, d_fp);
                    if (count == 0) {
                        if (ferror(d_fp)) {
                            std::stringstream s;
                            s << "file_sink write failed with error " << fileno(d_fp) << std::endl;
                            throw std::runtime_error(s.str());
                        } else { // is EOF
                            break;
                        }
                    }
                    ret += count;
                    in += count;
                    cnt+=count;
                    printf("PSSCH has been saved, saved items number =%d \n",cnt);
                    d_save_status=-1;
                    cnt=0;
                }

                if(d_unbuffered)
                    fflush (d_fp);
                break;


            case 0:
                  printf("waiting for the arrival of %d th time_slot,%d signals have been consumed\n",d_timeslot+1,d_waitslot);
                  if(d_waitslot+input_items_num<=(PSSCH_LEN-2048-160)){
                      ret=input_items_num;
                      in+=ret;
                      d_waitslot+=input_items_num;
                      if(d_waitslot==(PSSCH_LEN-2048-160))
                          printf("PSBCH over\n");
                  }else if(d_waitslot+input_items_num<=(d_timeslot*PSSCH_LEN+PSSCH_LEN-2048-160)){
                      if(d_waitslot<(PSSCH_LEN-2048-160))
                          printf("PSBCH over\n");
                      ret=input_items_num;
                      in+=ret;
                      d_waitslot+=input_items_num;
                      if(d_waitslot==(d_timeslot*PSSCH_LEN+PSSCH_LEN-2048-160)){
                          printf("timeslot arrived,%d signals have been consumed\n",d_waitslot);
                          d_save_status=1;
                          d_waitslot=0;
                      }
                  }else{

                      ret=d_timeslot*PSSCH_LEN+PSSCH_LEN-2048-160-d_waitslot;
                      in+=ret;
                      printf("timeslot arrived%d signals have been consumed\n",d_waitslot+ret);
                      d_save_status=1;
                      d_waitslot=0;
                  }
                  break;
            case -1 :


//                FILE *fwindow;
//                fwindow = fopen("/home/pc/ROI/data/corr_data/PSBCH_FFT_Template", "rb");
//                if (fwindow == NULL) return noutput_items;
//                float detect_window[1024] = {0};
//                fread(detect_window, sizeof(float), d_fft_size, fwindow);
                if(!d_detect_wait) {
                    while (ret + d_fft_size <= input_items_num) {
                        std::vector<float> first_fft_abs = do_fft(in);
//                    printf("detect_energedetect_energedetect_energedetect_energedetect_energe");
                        if (detect_energe(first_fft_abs)) {
                            First_Detection++;
                            printf("signal may start at ret=%d,First_Detection=%d\n", ret, First_Detection);
                            if (First_Detection == 3 || corr_start) {
                                printf("signal start at ret = %d\n", ret);
                                if ((ret + 6592 - d_fft_size * 2) > input_items_num && !corr_start) {
//                                ret = ret - d_fft_size * 2;
                                    printf("left data is not enough\n");
                                    corr_start = true;
                                    break;
                                }
//                            if (!corr_start) {
//                                ret = ret - d_fft_size * 2;
//                                in = in - d_fft_size * 2;
//                            }


                                FILE *fp;
                                fp = fopen("/home/pc/ROI/data/corr_data/Test_Out_Data_1", "rb");
                                if (fp == NULL) return noutput_items;
                                gr_complex data[2192] = {0};
                                fread(data, sizeof(gr_complex), NUM, fp);
                                int corr_len=(input_items_num - ret) > 5000 ? 5000 : (input_items_num - ret);

                                std::vector<float> output_abs = xcorr(in, data,
                                                                      corr_len, NUM);
                                int maxindex;//存入最大的索引值
                                int max_left = 0, max_right = 0;
                                int begin_index = 0;
                                bool pss_found = false;
                                find_max(output_abs, maxindex);
                                std::cout << "the bigest at index " << maxindex << "with output" << output_abs[maxindex]
                                          << std::endl;
//                    printf("abs compute success\n");
                                if (output_abs[maxindex] >= d_threshold) {
                                    /****    相关算法3000点（长度为signal.size()-data.size()）     *****/
//                                    pss_found= true;
//                                    begin_index=maxindex;
                                    /****    相关算法2（长度为signal.size()-data.size()）     *****/
//
                                    max_left = maxindex > PSS_LEN ? (maxindex - PSS_LEN) : -1;
                                    max_right = maxindex  < (corr_len-PSS_LEN*2) ? (maxindex + PSS_LEN) : -1;
                                    if(max_left==-1&&max_right==-1){
                                        pss_found= false;
                                    }
                                    else if (max_left == -1) {
                                        if (output_abs[max_right] > output_abs[maxindex] * d_proportion) {
                                            pss_found = true;
                                            begin_index = maxindex;
                                        }
                                    } else if (max_right == -1) {
                                        if (output_abs[max_left] > output_abs[maxindex] * d_proportion) {
                                            pss_found = true;
                                            begin_index = max_left;
                                        }
                                    }
                                    /****    相关算法1（长度为signal.size()+data.size()）     *****/
//                                    max_left = (maxindex - PSS_LEN) > 0 ? (maxindex - PSS_LEN) : -1;
//                                    max_right = (maxindex + PSS_LEN) < input_items_num ? (maxindex + PSS_LEN) : -1;
//                                    if (max_left != -1 && max_right != -1) {
//                                        if (output_abs[max_left] > output_abs[maxindex] * d_proportion) {
//                                            pss_found = true;
//                                            begin_index = max_left;
//                                        } else if (output_abs[max_right] > output_abs[maxindex] * d_proportion) {
//                                            pss_found = true;
//                                            begin_index = maxindex;
//                                        }
//                                    } else if (max_left == -1) {
//                                        if (output_abs[max_right] > output_abs[maxindex] * d_proportion) {
//                                            pss_found = true;
//                                            begin_index = maxindex;
//                                        }
//                                    } else if (max_right == -1) {
//                                        if (output_abs[max_left] > output_abs[maxindex] * d_proportion) {
//                                            pss_found = true;
//                                            begin_index = max_left;
//                                        }
//                                    }

                                }
                                if (pss_found) {
                                    printf("PSS sequences have been found\n");

                                    std::cout << "the pss begin at index " << begin_index+ret << "with output"
                                              << output_abs[begin_index] << std::endl;

//                                printf(" write items num = %d, input_items_num = %d, ret = %d\n", NUM,
//                                       input_items_num,
//                                       ret);
//                                struct timeval timer;
//                                gettimeofday(&timer, NULL);
//                                std::cout << "receive time: " << timer.tv_sec << "s " << timer.tv_usec << "us"
//                                          << std::endl;
//                                gr::thread::scoped_lock lock(mutex);
//                                do_update();
//                                if (!d_fp)
//                                    return noutput_items;
//                                // 先清空文件
//                                ftruncate(fileno(d_fp), 0);
//                                rewind(d_fp);
//
//                                int t_size = fwrite(in + begin_index, sizeof(gr_complex), NUM, d_fp);
//                                rewind(d_fp);
//
//                                if(d_latency > 0) usleep(d_latency);
//                                status_file = true;
//                                send_message();
//                                if (ferror(d_fp)) {
//                                    std::stringstream s;
//                                    s << "file_sink write failed with error " << fileno(d_fp) << std::endl;
//                                    throw std::runtime_error(s.str());
//                                }
//
//                                if (d_unbuffered) fflush(d_fp);
                                    if(d_timeslot==0){
                                        d_save_status=1;
                                    }else{
                                        d_save_status = 0;
                                    }

                                    corr_start = false;
                                    printf("ready for saving data\n");
                                    ret += begin_index;
                                    in = in + begin_index;
                                    break;

                                } else {
                                    printf("PSS not found,Detect failed\n");
                                    First_Detection = 0;
                                    d_save_status = -1;
                                    corr_start = false;
                                    ret = input_items_num;
                                    d_detect_wait = 3;
                                    break;
                                }

                            }

                        } else {
                            First_Detection = 0;
                            corr_start = false;
                            if (corr_start)
                                printf("corr_start = true ,error occured!\n");
                        }

                        in = in + d_fft_size;
                        ret += d_fft_size;
                    }
                }else{
                    d_detect_wait=d_detect_wait>0?(--d_detect_wait):0;
                    printf("wait for %d times general_work \n",d_detect_wait);
                    ret=input_items_num;
                }
                break;
            }



                // Do <+signal processing+>
                // Tell runtime system how many input items we consumed on
                // each input stream.
                consume_each(ret);

                // Tell runtime system how many output items we produced.
                return noutput_items;
            }

        } /* namespace roi */
    } /* namespace gr */

