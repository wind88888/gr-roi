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


#ifndef INCLUDED_ROI_FILE_SINK_ROI_H
#define INCLUDED_ROI_FILE_SINK_ROI_H

#include <roi/api.h>
#include <gnuradio/sync_block.h>
#include <gnuradio/block.h>
#include <gnuradio/blocks/file_sink_base.h>

namespace gr {
  namespace roi {

    /*!
     * \brief <+description of block+>
     * \ingroup roi
     *
     */
    class ROI_API file_sink_roi : virtual public gr::block,
                                      virtual public gr::blocks::file_sink_base
    {
     public:
      typedef boost::shared_ptr<file_sink_roi> sptr;


        virtual bool get_status_file() = 0;
        virtual void set_status_file(bool _status_file) = 0;



      /*!
       * \brief Return a shared_ptr to a new instance of roi::file_sink_roi.
       *
       * To avoid accidental use of raw pointers, roi::file_sink_roi's
       * constructor is in a private implementation
       * class. roi::file_sink_roi::make is the public interface for
       * creating new instances.
       */
      static sptr make(const char *filename, bool append,  int cell_id, float threshold, float proportion, int fft_size, bool forward, const std::vector<float> &window, bool shift, int nthreads,float energe);
    };

  } // namespace roi
} // namespace gr

#endif /* INCLUDED_ROI_FILE_SINK_ROI_H */

