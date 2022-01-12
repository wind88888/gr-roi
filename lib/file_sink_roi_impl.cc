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
        file_sink_roi::make(const char* filename, bool append, int cell_id,float threshold,int rec_len,
                            int fft_size, bool forward, const std::vector<float> &window, bool shift, int nthreads,//used for fft
                            float energe,int latency,int time_slot,bool alice
        )
        {
            return gnuradio::get_initial_sptr
                    (new file_sink_roi_impl(filename, append,cell_id, threshold,rec_len, fft_size, forward, window, shift, nthreads,energe,latency,time_slot,alice));
        }


        /*
         * The private constructor
         */
        file_sink_roi_impl::file_sink_roi_impl(const char* filename, bool append, int cell_id,float threshold,int rec_len,int fft_size,
                                               bool forward, const std::vector<float> &window, bool shift, int nthreads,float energe,int latency,int time_slot,bool alice)
                : gr::block("file_sink_roi",
                            gr::io_signature::make(1, 1, sizeof(gr_complex)),
                            gr::io_signature::make(0, 0, 0)) ,
                  file_sink_base(filename, true, append),
                  d_latency(latency),
                  status_file(false),
                  rx_file(false),
                  d_threshold(threshold),
                  d_cell_id(cell_id),
                  d_receive_length(rec_len),
                  d_fft_size(fft_size),
                  d_forward(forward),
                  d_shift(shift),
                  d_energe(energe),
                  d_timeslot(time_slot),
                  d_alice(alice),
                  receive_times(0),
                  d_alice_pssfound(false)
                  {
                      set_relative_rate(1.0 / 9000);

                      d_port = pmt::mp("msg_status_file");
                      d_port_rx=pmt::mp("msg_rx_file");
                      message_port_register_out(d_port);
                      message_port_register_in(d_port_rx);
                      set_msg_handler(d_port_rx, boost::bind(&file_sink_roi_impl::msg_handler, this, _1));

                      std::cout<<"threshold = "<<d_threshold<<"receive_length = "<<d_receive_length<<"fft_size="<<d_fft_size<<std::endl;
                      d_fft = new fft_complex(d_fft_size, forward, nthreads);
                      if (!set_window(window)) {
                          throw std::runtime_error("fft_vcc: window not the same length as fft_size\n");
                      }
                      //alice端自检测测试
//                      if(d_alice){
//                          d_save_status=1;
//                          printf("the character is Alice\n");
//                      }
                      times=0;
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
            float sum_window=9.0/256 *d_fft_size;
            int freq_size=ceil(9.0/512 *d_fft_size);
            sum_signal=fft_abs[0]*0.7;
            for(int i=1;i<freq_size;i++){
                sum_signal+=fft_abs[i];
            }
            for(int i=d_fft_size;i>(d_fft_size-freq_size);i--){
                sum_signal+=fft_abs[i];
            }
            sum_noise=fft_abs[0]*0.3;
            for(int i=freq_size;i<(d_fft_size-freq_size);i++){
                sum_noise+=fft_abs[i];
            }


            sum_signal=sum_signal/sum_window;
            sum_noise=sum_noise/(d_fft_size-sum_window);

            float snr_ratio=sum_signal/sum_noise;
//            printf("sum_window=%f,sum_signal=%f,sum_noise=%f,freq_size=%d,snr_ratio=%f",sum_window,sum_signal,sum_noise,freq_size,snr_ratio);
//            printf("snr_ratio = %f", snr_ratio);

            if(snr_ratio>40){
                return true;}

            return false;
        }

        bool file_sink_roi_impl::detect_energe_PSSCH(const std::vector<float> &fft_abs) {
            float sum_signal=0.0;
            float sum_noise=0.0;
            float sum_window=147.0/256 *d_fft_size;
            int freq_size=ceil(147.0/512 *d_fft_size);
            sum_signal=fft_abs[0]*0.7;
            for(int i=1;i<freq_size;i++){
                sum_signal+=fft_abs[i];
            }
            for(int i=d_fft_size;i>(d_fft_size-freq_size);i--){
                sum_signal+=fft_abs[i];
            }
            sum_noise=fft_abs[0]*0.3;
            for(int i=freq_size;i<(d_fft_size-freq_size);i++){
                sum_noise+=fft_abs[i];
            }


            sum_signal=sum_signal/sum_window;
            sum_noise=sum_noise/(d_fft_size-sum_window);

            float snr_ratio=sum_signal/sum_noise;
//            printf("sum_window=%f,sum_signal=%f,sum_noise=%f,freq_size=%d,snr_ratio=%f",sum_window,sum_signal,sum_noise,freq_size,snr_ratio);
//            printf("snr_ratio = %f", snr_ratio);

            if(snr_ratio>d_energe){
                printf("PSSCH detected with energe %f\n",snr_ratio);
                return true;}

            return false;
        }

//消息接收函数

        void file_sink_roi_impl::msg_handler(pmt::pmt_t msg)
        {

            printf("***** MESSAGE DEBUG PRINT ********\n");
            pmt::pmt_t msg_ctl = pmt::car(msg);
            bool status_rx = pmt::to_bool(pmt::dict_ref(msg_ctl, pmt::mp("status_rx"), pmt::from_bool("false")));
            //bool rx_file = pmt::dict_ref(msg_ctl, pmt::mp("rx_file"), pmt::from_bool("false"));
            printf("rx_file = %d\n", status_rx);
            printf("**********************************\n");
            set_rx_file(status_rx);
        }
        void file_sink_roi_impl::set_rx_file(bool _rx_file) {

            if (rx_file == _rx_file) {
                return;
            }

            rx_file = _rx_file;
        }
        bool file_sink_roi_impl::get_rx_file() {
            return rx_file;
        }



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

               case 2:
                   if(times==0){
                       times=50;
                       ret=input_items_num;
                       times--;
                       break;
                   }
                   if(times!=1){
                       ret=input_items_num;
                       times--;
                       break;
                   }
                   d_save_status=-1;
                   printf("start to send message\n");
                   times=0;
                   status_file = true;
                   send_message();
                   break;



               case 1:
                   /*单次测试*/
//                   if(times){
//                       throw std::runtime_error("receiver one time over\n");
//                   }
                   /*单次测试end*/
                   /*多次测试*/
                   if(receive_times>=10){throw std::runtime_error("receiver  times over\n");}
//alice端自检测删除部分
//                   if(d_alice&&!rx_file){
//                       ret=input_items_num;
//                       printf("%d have been consumed,receive time is %d\n",ret,receive_times);
//                       break;
//                   }
//alice按点数接收PSSCH
//                   if(d_alice&&(d_count>0)&&(d_count<d_timeslot*PSSCH_LEN)){
//                       if(d_count+input_items_num<d_timeslot*PSSCH_LEN){
//                           d_count+=input_items_num;
//                           ret=input_items_num;
//                           printf("%d have been consumed,receive time is %d\n",d_count,receive_times);
//                       }else{
//                           ret=d_timeslot*PSSCH_LEN-d_count;
//                           d_count=0;
//                           printf("%d have been consumed,receive time is %d\n",d_count,receive_times);
//                       }
//                       break;
//                   }

//                  d_count矫正
//alice端自检测删除部分
//                   if(d_alice&&d_count){
//                       d_count--;
//                       ret=input_items_num;
//                       break;
//                   }


/*alice接收端能量检测*/
                   if(d_alice&&!d_alice_rec) {
                       while (ret + d_fft_size <= input_items_num) {
                           std::vector<float> first_fft_abs = do_fft(in);
                           if (detect_energe_PSSCH(first_fft_abs)) {
                               printf("energe detected at ret = %d\n", ret);
                               if(ret>1800){
                                   in = in-1800;
                                   ret = ret-1800;
                               }else{
                                   ret=0;
                               }
                               times=-1;
                               d_alice_rec= true;
                               break;
                           }
                           ret = ret + d_fft_size;
                           in = in + d_fft_size;
                       }
                       if(!d_alice_rec){
                           times++;
                           if(times>11){
                               printf("enege detect failed times of consumed is %d\n",times);
                               times=0;
                               d_alice_rec= true;
                           }
                       }
                       break;
                   }
                   if(d_alice_rec&&(times==-1)) {
                       FILE *fpdmrs;
                       fpdmrs = fopen("/home/pc/ROI/data/corr_data/DMRS", "rb");
                       if (fpdmrs == NULL) return noutput_items;
                       gr_complex data_dmrs[2192] = {0};
                       fread(data_dmrs, sizeof(gr_complex), NUM, fpdmrs);
                       in = in + (2048 * 2 );
                       ret =ret +(2048 * 2 );
                       int corr_len_dmrs = (input_items_num - ret) > 4000 ? 4000 : (input_items_num - ret);
//                       int corr_len_dmrs = input_items_num - ret;
                       printf("start to calculate xcorr ret=%d,corr_len_dmrs =%d \n",ret,corr_len_dmrs);
                       std::vector<float> output_abs = xcorr(in, data_dmrs,
                                                             corr_len_dmrs, NUM);
                       int maxindex_dmrs;//存入最大的索引值
//                                int max_left = 0, max_right = 0;
//                       int begin_index_dmrs = 0;
//                       bool dmrs_found = false;
                       find_max(output_abs, maxindex_dmrs);
                       std::cout << "the bigest at index " << maxindex_dmrs << "with output" << output_abs[maxindex_dmrs]
                                 << std::endl;
//                    printf("abs compute success\n");
                       if (output_abs[maxindex_dmrs] >= 100) {
                           printf("DMRS has been found \n");
                           in=in - (2048 * 2 +160+144)+maxindex_dmrs;
                           ret = ret -(2048 * 2 +160+144)+maxindex_dmrs;
                           times=0;
                       }else{
                           printf("DMRS detect failed\n");
                           in=in - (2048 * 2 );
                           ret = ret -(2048 * 2 );
                           times=0;
                       }
                   }

                   printf("now start to save PSSCH, %d/%d items have been saved",cnt,d_receive_length*PSSCH_LEN);

                   struct timeval timer;
                   gettimeofday(&timer, NULL);
                   std::cout << "receive time: " << timer.tv_sec << "s " << timer.tv_usec << "us"
                   << std::endl;
//                   gr::thread::scoped_lock lock(mutex);
                   do_update();
                   if(!d_fp)
                    return noutput_items;
                   /*多次测试时注销*/
//                   if(cnt==0){
//                       ftruncate(fileno(d_fp), 0);
//                       rewind(d_fp);
//                   }

                if(cnt+input_items_num<=d_receive_length*PSSCH_LEN) {
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
                    if(cnt==d_receive_length*PSSCH_LEN){
                        printf("PSSCH has been saved, items number =%d \n",cnt);
                        if(d_alice){
                            d_save_status=2;
                            /*单次测试*/
//                            times=times+1;
                            /*单次测试end*/
//                            printf("times=%d\n",times);
                        } else{
                            d_save_status=-1;
                            d_detect_wait=3;
                        }
                        cnt=0;

//                        if(d_latency > 0) usleep(d_latency);
//                        status_file = true;
//                        send_message();
                        if(d_alice){
                            receive_times++;
                            rx_file= false;
//                            d_count=d_latency;//矫正Alice端接收
                            d_alice_rec= false;
                            printf("%d times received\n",receive_times);

                        }

                    }

                }else{
                    int count = fwrite(in, sizeof(gr_complex), d_receive_length*PSSCH_LEN-cnt, d_fp);
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
                    if(d_alice){
                        d_save_status=2;
                        /*单次测试*/
//                        times=times+1;
                        /*单次测试end*/
//                        printf("times=%d\n",times);
                    } else{
                        d_save_status=-1;
                        d_detect_wait=3;
                    }
                    /*单次测试*/
//                    if(times>=2){
//                        throw std::runtime_error("receiver one time over\n");
//                    }
                    /*单次测试end*/
                    cnt=0;
//                    if(d_latency > 0) usleep(d_latency);
//                    status_file = true;
//                    send_message();
                    if(d_alice){
                        receive_times++;
//                        rx_file= false;
                        printf("%d times received\n",receive_times);
//                        d_count=d_latency;//矫正Alice端接收
                        d_alice_rec= false;
                    }
                }

                if(d_unbuffered)
                    fflush (d_fp);
                break;


            case 0:
                if(!d_alice){
                    printf("waiting for the arrival of %d th time_slot,%d signals have been consumed\n",d_timeslot+1,d_waitslot);
                }else if(d_waitslot==0){
                    printf("Alice is waiting for the arrival of %d th time_slot\n",d_timeslot+1);
                }
                if(d_alice&&(!d_alice_pssfound)){
                    if(d_waitslot+input_items_num<=(d_timeslot+1)*PSSCH_LEN+8000){
                        ret = input_items_num;
                        in += ret;
                        d_waitslot += input_items_num;
                        if (d_waitslot == ((d_timeslot+1) * PSSCH_LEN+8000)) {
                            printf("timeslot arrived,%d signals have been consumed\n", d_waitslot);
                            d_save_status = 1;
                            d_waitslot = 0;
                        }

                    }else{
                        ret = (d_timeslot+1) * PSSCH_LEN + 8000- d_waitslot;
                        in += ret;
                        printf("timeslot arrived%d signals have been consumed\n", d_waitslot + ret);
                        d_save_status = 1;
                        d_waitslot = 0;
                    }
                }else {

                    if (d_waitslot + input_items_num <= (PSSCH_LEN - 2048 - 160)) {
                        ret = input_items_num;
                        in += ret;
                        d_waitslot += input_items_num;
                        if (d_waitslot == (PSSCH_LEN - 2048 - 160))
                            printf("PSBCH over\n");
                    } else if (d_waitslot + input_items_num <= (d_timeslot * PSSCH_LEN + PSSCH_LEN - 2048 - 160)) {
                        if (d_waitslot < (PSSCH_LEN - 2048 - 160))
                            printf("PSBCH over\n");
                        if (d_alice && (d_waitslot <= 20512)) {
                            d_waitslot = d_waitslot - 2048 - 160;
                            printf("Alice's waiting\n");
                        }
                        ret = input_items_num;
                        in += ret;
                        d_waitslot += input_items_num;
                        if (d_waitslot == (d_timeslot * PSSCH_LEN + PSSCH_LEN - 2048 - 160)) {
                            printf("timeslot arrived,%d signals have been consumed\n", d_waitslot);
                            d_save_status = 1;
                            d_alice_pssfound=false;
                            d_waitslot = 0;
                        }
                    } else {

                        ret = d_timeslot * PSSCH_LEN + PSSCH_LEN - 2048 - 160 - d_waitslot;
                        in += ret;
                        printf("timeslot arrived%d signals have been consumed\n", d_waitslot + ret);
                        d_save_status = 1;
                        d_alice_pssfound=false;
                        d_waitslot = 0;
                    }

                }

                  break;
            case -1 :

//加入能量检测窗
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
                                int shift_index=0;
                                int si=d_fft_size/128;
                                switch (si) {
                                    case 0:
                                        shift_index=1400;
                                        break;
                                    case 1:
                                        shift_index=1400;
                                        break;
                                    case 2:
                                        shift_index=1200;
                                        break;
                                    case 4:
                                        shift_index=800;
                                        break;
                                    case 8:
                                        shift_index=0;
                                        break;
                                    default:
                                        shift_index=0;
                                        break;
                                }
                                printf("signal start at ret = %d,shift_index=%d\n", ret,shift_index);
                                if ((ret + 3000 + shift_index ) > input_items_num && !corr_start) {
//                                  ret = ret - d_fft_size * 2;
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
                                ret=ret+shift_index;
                                in=in+shift_index;
                                int corr_len=(input_items_num - ret) > 3000 ? 3000 : (input_items_num - ret);
                                std::vector<float> output_abs = xcorr(in, data,
                                                                      corr_len, NUM);
                                int maxindex;//存入最大的索引值
//                                int max_left = 0, max_right = 0;
                                int begin_index = 0;
                                bool pss_found = false;
                                find_max(output_abs, maxindex);
                                std::cout << "the bigest at index " << maxindex << "with output" << output_abs[maxindex]
                                          << std::endl;
//                    printf("abs compute success\n");
                                if (output_abs[maxindex] >= d_threshold) {
                                    /****    相关算法3000点（长度为signal.size()-data.size()）     *****/
                                    pss_found= true;
                                    begin_index=maxindex;
                                    /****    相关算法2（长度为signal.size()-data.size()）     *****/
////
//                                    max_left = maxindex > PSS_LEN ? (maxindex - PSS_LEN) : -1;
//                                    max_right = maxindex  < (corr_len-PSS_LEN*2) ? (maxindex + PSS_LEN) : -1;
//                                    if(max_left==-1&&max_right==-1){
//                                        pss_found= false;
//                                    }
//                                    else if (max_left == -1) {
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
                                    d_alice_pssfound=true;
                                    printf("d_alice_pssfound=%d\n",d_alice_pssfound);
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
                                    ret = ret + begin_index ;
                                    in = in + begin_index;
                                    break;

                                } else {
                                    printf("PSS not found,Detect failed\n");
                                    First_Detection = 0;
                                    if(d_alice){
                                        d_save_status = 0;
                                        ret = input_items_num;
                                        d_alice_pssfound=false;
                                    } else{
                                        d_save_status = -1;
                                        d_detect_wait = 3;
                                        ret = input_items_num;
                                    }
                                    corr_start = false;


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

