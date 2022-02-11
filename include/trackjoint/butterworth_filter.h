// Copyright 2022 PickNik Inc.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
//    * Redistributions of source code must retain the above copyright
//      notice, this list of conditions and the following disclaimer.
//
//    * Redistributions in binary form must reproduce the above copyright
//      notice, this list of conditions and the following disclaimer in the
//      documentation and/or other materials provided with the distribution.
//
//    * Neither the name of the PickNik Inc. nor the names of its
//      contributors may be used to endorse or promote products derived from
//      this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.

/* Author: Andy Zelenak
   Desc: A Butterworth low-pass filter.
*/

#pragma once

#include <array>
#include <cmath>
#include <cstddef>

namespace trackjoint
{
/**
 * Class ButterworthFilter - Implementation of a signal filter to reduce noise.
 * This is a first-order Butterworth low-pass filter. First-order was chosen for 2 reasons:
 * - It doesn't overshoot
 * - Computational efficiency
 */
class ButterworthFilter
{
public:
  /**
   * Constructor.
   * @param low_pass_filter_coeff Larger filter_coeff-> more smoothing of servo commands, but more lag.
   * Rough plot, with cutoff frequency on the y-axis:
   * https://www.wolframalpha.com/input/?i=plot+arccot(c)
   */
  ButterworthFilter(long double low_pass_filter_coeff);
  ButterworthFilter() = delete;

  long double filter(long double new_measurement);

  void reset(const long double data);

private:
  static constexpr size_t FILTER_LENGTH = 2;
  std::array<long double, FILTER_LENGTH> previous_measurements_;
  long double previous_filtered_measurement_;
  // Scale and feedback term are calculated from supplied filter coefficient
  long double scale_term_;
  long double feedback_term_;
};
}  // namespace trackjoint
