/*ckwg +29
 * Copyright 2016 by Kitware, Inc.
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
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
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
 * \brief Interface for MatLab engine interface class.
 */

#ifndef VITAL_MATLAB_ENGINE_H
#define VITAL_MATLAB_ENGINE_H

#include <vital/vital_config.h>
#include <vital/noncopyable.h>
#include <vital/logger/logger.h>
#include <vital/bindings/matlab/matlab_array.h>
#include <vital/bindings/matlab/vital_matlab_export.h>

// Matlab includes
#include <engine.h>

#include <cstdlib>

namespace kwiver {
namespace vital {
namespace matlab {

// -----------------------------------------------------------------
/**
 * @brief MatLab engine interface.
 *
 * This class represents a single user Matlab engine instance.
 */
class VITAL_MATLAB_EXPORT matlab_engine
  : private noncopyable
{
public:
  /**
   * @brief Create new matlab engine instance.
   *
   *
   * @throws matlab_exception when error is encountered.
   */
  matlab_engine();
  ~matlab_engine();

  /**
   * @brief Evaluate command string.
   *
   * This method evaluates the specified string as a MatLab
   * program. The results are returned in the output string. Call
   * engine_output() to retrieve the output string.
   *
   * @param cmd String to evaluate
   */
  void eval( const std::string& cmd );

  /**
   * @brief Get variable value form MatLab engine.
   *
   * This method reads the contents of the specified MatLab
   * variable. The limit for the size of data transferred is 2 GB.
   *
   * @param name Name of variable
   *
   * @return Pointer to a newly allocated mxArray structure, or NULL
   * if the attempt fails. engGetVariable fails if the named variable
   * does not exist.
   */
  std::shared_ptr<mxArray> get_variable( const std::string& name );

  /**
   * @brief Set named variable in MatLab engine.
   *
   * This method sets the value of the specified variable to the
   * supplied value.  If the mxArray does not exist in the workspace,
   * the function creates it. If an mxArray with the same name exists
   * in the workspace, the function replaces the existing mxArray with
   * the new mxArray.
   *
   * The limit for the size of data transferred is 2 GB.
   *
   * Do not use MATLAB® function names for variable names. Common
   * variable names that conflict with function names include i, j,
   * mode, char, size, or path. To determine whether a particular name
   * is associated with a MATLAB function, use the which function.
   *
   * The engine application owns the original mxArray and is
   * responsible for freeing its memory. Although the engPutVariable
   * function sends a copy of the mxArray to the MATLAB workspace, the
   * engine application does not need to account for or free memory
   * for the copy.
   *
   * @return
   */
  void put_variable( const std::string& name, mxArraySptr val);


  /**
   * @brief Return visibility of matlab engine.
   *
   * This method returns the current visibility setting for MATLAB®
   * engine session. A visible engine session runs in a window on
   * the Windows® desktop, thus making the engine available for user
   * interaction.
   *
   * @return \b true if window is visible
   */
  bool get_visible();

  /**
   * @brief Set visibility attribute for matlab engine.
   *
   * This method makes the window for the MATLAB® engine session
   * either visible or invisible on the Windows® desktop. You can use
   * this function to enable or disable user interaction with the
   * Matlab engine session.
   *
   * @param vis Visibility state.
   */
  void set_visible( bool vis );

  /**
   * @brief Get output from eval_string().
   *
   * This method returns the output from the last call to
   * eval_string().
   *
   * @return The output text from the last call to eval_string().
   */
  std::string engine_output() const;

private:
  kwiver::vital::logger_handle_t m_logger;
  Engine* m_engine_handle;
  char* m_output_buffer;

}; // end class matlab_engine

} } } // end namespace vital

#endif /* VITAL_MATLAB_ENGINE_H */