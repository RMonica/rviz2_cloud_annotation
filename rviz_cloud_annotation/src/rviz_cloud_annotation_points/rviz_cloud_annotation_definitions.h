/*
 * Copyright (c) 2016-2023, Riccardo Monica
 *   RIMLab, Department of Engineering and Architecture
 *   University of Parma, Italy
 *   http://www.rimlab.ce.unipr.it/
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 *
 * 3. Neither the name of the copyright holder nor the names of its
 *    contributors may be used to endorse or promote products derived from this
 *    software without specific prior written permission.
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
 *
 */

#ifndef RVIZ_CLOUD_ANNOTATION_DEFINITIONS_H
#define RVIZ_CLOUD_ANNOTATION_DEFINITIONS_H

// parameters for smart labeling
  // neighborhood graph distance

#define PARAM_NAME_NEIGH_SEARCH_TYPE          "neigh_search_type"
#define PARAM_DEFAULT_NEIGH_SEARCH_TYPE       (0)
#define PARAM_VALUE_NEIGH_SEARCH_FIXED_DISTANCE (0) // neigh_search_params is the distance (float)
#define PARAM_VALUE_NEIGH_SEARCH_KNN_ATMOST     (1) // neigh_search_params is the maximum number of neighbors (integer)
#define PARAM_VALUE_NEIGH_SEARCH_KNN_ATLEAST    (2) // neigh_search_params is the minimum number of neighbors (integer)

// this is always a string, content depends on neigh_search_type
#define PARAM_NAME_NEIGH_SEARCH_PARAMS        "neigh_search_params"
#define PARAM_DEFAULT_NEIGH_SEARCH_PARAMS     ""

  // max label size per control point
#define PARAM_NAME_MAX_DISTANCE               "max_label_distance"
#define PARAM_DEFAULT_MAX_DISTANCE            (0.1)

#define PARAM_NAME_COLOR_IMPORTANCE           "color_importance"
#define PARAM_DEFAULT_COLOR_IMPORTANCE        (0.0)

#define PARAM_NAME_POSITION_IMPORTANCE        "position_importance"
#define PARAM_DEFAULT_POSITION_IMPORTANCE     (1.0)

#define PARAM_NAME_NORMAL_IMPORTANCE          "normal_importance"
#define PARAM_DEFAULT_NORMAL_IMPORTANCE       (0.0)
// end parameters for smart labeling

// this number of weight steps, plus the step 0
#define PARAM_NAME_WEIGHT_STEPS               "weight_steps"
#define PARAM_DEFAULT_WEIGHT_STEPS            (10)

#endif // RVIZ_CLOUD_ANNOTATION_DEFINITIONS_H
