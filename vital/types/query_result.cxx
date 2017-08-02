/*ckwg +29
 * Copyright 2017 by Kitware, Inc.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *  * Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 *
 *  * Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 *
 *  * Neither name of Kitware, Inc. nor the names of any contributors may be used
 *    to endorse or promote products derived from this software without specific
 *    prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS ``AS IS''
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE AUTHORS OR CONTRIBUTORS BE LIABLE FOR
 * ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

/**
 * \file
 * \brief This file contains the implementation of a query result.
 */

#include "query_result.h"

namespace kwiver {
namespace vital {

// ----------------------------------------------------------------------------
query_result
::query_result()
{
}

// ----------------------------------------------------------------------------
uid
query_result
::query_id() const
{
  return m_query_id;
}

// ----------------------------------------------------------------------------
void
query_result
::set_query_id( uid const& id )
{
  m_query_id = id;
}

// ----------------------------------------------------------------------------
timestamp
query_result
::start_time() const
{
  return m_start_time;
}

// ----------------------------------------------------------------------------
timestamp
query_result
::end_time() const
{
  return m_end_time;
}

// ----------------------------------------------------------------------------
void
query_result
::set_temporal_bounds( timestamp const& lower, timestamp const& upper )
{
  m_start_time = lower;
  m_end_time = upper;
}

// ----------------------------------------------------------------------------
std::vector< bounding_box_i >
query_result
::spatial_regions() const
{
  return m_spatial_regions;
}

// ----------------------------------------------------------------------------
void
query_result
::set_spatial_regions( std::vector< bounding_box_i > const& r )
{
  m_spatial_regions = r;
}

// ----------------------------------------------------------------------------
std::string
query_result
::stream_query_id() const
{
  return m_stream_query_id;
}

// ----------------------------------------------------------------------------
void
query_result
::set_stream_query_id( std::string const& l )
{
  m_stream_query_id = l;
}

// ----------------------------------------------------------------------------
std::vector< image >
query_result
::image_data() const
{
  return m_image_data;
}

// ----------------------------------------------------------------------------
void
query_result
::set_image_data( std::vector< image > const& i )
{
  m_image_data = i;
}

} } // end namespace
