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
        file_sink_roi::make(const char* filename, bool append, int cell_id,float threshold,float proportion) {
            return gnuradio::get_initial_sptr
                    (new file_sink_roi_impl(filename, append,cell_id, threshold,proportion));
        }


        /*
         * The private constructor
         */
        file_sink_roi_impl::file_sink_roi_impl(const char* filename, bool append, int cell_id,float threshold,float proportion)
                : gr::block("file_sink_roi",
                            gr::io_signature::make(1, 1, sizeof(gr_complex)),
                            gr::io_signature::make(0, 0, 0)) ,
                  file_sink_base(filename, true, append),
                  status_file(false),
                  d_threshold(threshold),
                  d_cell_id(cell_id),
                  d_proportion(proportion)
                  {
                      set_relative_rate(1.0 / 9000);
                      d_port = pmt::mp("msg_status_file");
                      message_port_register_out(d_port);
        }
        std::vector<float> file_sink_roi_impl::xcorr(const gr_complex* in,const gr_complex* data,int num_input,int num_data){
            gr_complex *input = new gr_complex[num_input+num_data];
//            printf("doing xcorr\n");
            for(int i=0;i<(num_input+num_data);i++)
            {
                if(i<num_input){
                    input[i]=in[i];
                }else{
                    input[i]=0;
                }
            }
            std::vector<gr_complex> output(num_input);
            for(int i=0;i<num_input;i++)
            {
                for(int j=0;j<num_data;j++)
                    output[i]=output[i]+input[i+j]*data[j];
            }
            delete [] input;
            std::vector<float>output_abs(num_input);
            for(int i=0;i<num_input;i++)
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


            int
            file_sink_roi_impl::general_work(int noutput_items,
                                                 gr_vector_int &ninput_items,
                                                 gr_vector_const_void_star &input_items,
                                                 gr_vector_void_star &output_items) {
            int ret=0;// 记录消耗的item数目
            int input_items_num=ninput_items[0];
            std::cout<<"input_items_num="<<input_items_num<<"ret="<<ret<<std::endl;
            std::cout<<"threshold = "<<d_threshold<<"proportion = "<<d_proportion<<std::endl;
//                printf("ret=%d\n",ret);

            const gr_complex *in = (const gr_complex *) input_items[0];


                FILE* fp;
//                printf("file read begin\n");
                fp=fopen("/home/alex/ROI/data/corr_data/Test_Out_Data_1","rb");
                if(fp==NULL) return noutput_items;
//                printf("file read success\n");


                gr_complex data[NUM]= {0};

                fread(data,sizeof(gr_complex),NUM,fp);
//                printf("fread success\n");
                while(ret<input_items_num) {
                    std::vector<float> output_abs = xcorr(in, data, input_items_num, NUM);


                    int maxindex;//存入最大的索引值
                    int max_left=0,max_right=0;
                    int begin_index=0;
                    bool pss_found=false;
                    find_max(output_abs, maxindex);
                    std::cout << "the bigest at index " <<maxindex << "with output" <<output_abs[maxindex] << std::endl;
//                    printf("abs compute success\n");
                    if (output_abs[maxindex]>=d_threshold) {
                        max_left=(maxindex-PSS_LEN)>0?(maxindex-PSS_LEN):-1;
                        max_right=(maxindex+PSS_LEN)<input_items_num?(maxindex+PSS_LEN):-1;
                        if(max_left!=-1&&max_right!=-1){
                            if(output_abs[max_left]>output_abs[maxindex]*d_proportion){
                                pss_found= true;
                                begin_index=max_left;
                            }else if(output_abs[max_right]>output_abs[maxindex]*d_proportion){
                                pss_found= true;
                                begin_index=maxindex;
                            }
                        }else if(max_left==-1){
                            if(output_abs[max_right]>output_abs[maxindex]*d_proportion){
                                pss_found=true;
                                begin_index=maxindex;
                            }
                        }else if(max_right==-1){
                            if(output_abs[max_left]>output_abs[maxindex]*d_proportion){
                                pss_found= true;
                                begin_index=max_left;
                            }
                        }

//                        std::cout<<"maxindex="<<maxindex<<"with output:"<<output_abs[maxindex]<<std::endl;
//                        if ((maxindex < NUM) && ((maxindex + NUM) < input_items_num)) {
//                            if(output_abs[maxindex+NUM]>=(output_abs[maxindex]*d_proportion){
//                                pss_found= true;
//                                max_right=maxindex+NUM;
//                            }
//
//                        } else if((maxindex+NUM)>input_items_num){
//                            if(output_abs[maxindex-NUM]>=(output_abs[maxindex]*d_proportion)) {
//                                pss_found = true;
//                                max_left=maxindex-NUM;
//                            }
//                        }else{
//                            if((output_abs[maxindex+NUM]>=output_abs[maxindex-NUM])&&(output_abs[maxindex+NUM]>=(output_abs[maxindex]*d_proportion))){
//                                pss_found = true;
//                                max_right=maxindex+NUM;
//                            }else if(output_abs[maxindex-NUM]>=(output_abs[maxindex]*d_proportions)){
//                                pss_found = true;
//                                max_left=maxindex-NUM;
//                            }
//                        }
                    }
                    if(pss_found){
                        printf("PSS sequences have been found\n");

                        std::cout << "the pss begin at index " <<begin_index << "with output" <<output_abs[begin_index] << std::endl;
//                       std::cout << "the pss begin at index " <<maxindex << "with output" <<output_abs[maxindex] << std::endl;
                        printf(" write items num = %d, input_items_num = %d, ret = %d\n", NUM, input_items_num, ret);
                        struct timeval timer;
                        gettimeofday(&timer, NULL);
                        std::cout << "receive time: " << timer.tv_sec << "s " << timer.tv_usec << "us" << std::endl;
                        gr::thread::scoped_lock lock(mutex);
                        do_update();
                        if (!d_fp)
                            return noutput_items;

                        // 先清空文件
                        ftruncate(fileno(d_fp), 0);
                        rewind(d_fp);

//                        int t_size = fwrite(in, sizeof(gr_complex), 8512 - 1504 + d_fft_size, d_fp);
                        int t_size = fwrite(in + begin_index, sizeof(gr_complex), NUM, d_fp);
//                        int t_size = fwrite(in + maxindex, sizeof(gr_complex), NUM, d_fp);
                        rewind(d_fp);

//                            printf("written data size = %d, file size = %d\n", t_size, file_size);

                        status_file = true;
                        send_message();
                        if (ferror(d_fp)) {
                            std::stringstream s;
                            s << "file_sink write failed with error " << fileno(d_fp) << std::endl;
                            throw std::runtime_error(s.str());
                        }

                        if (d_unbuffered) fflush(d_fp);

                        ret += input_items_num;

                    }
                    printf("PSS not found\n");
                    if (input_items_num >= (PSS_LEN * 2)) {
                        ret += (input_items_num - PSS_LEN * 2);
                    }
                    ret += input_items_num;
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

