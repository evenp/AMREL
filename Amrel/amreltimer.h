/*  Copyright 2021 Philippe Even and Phuc Ngo,
      authors of paper:
      Even, P., and Ngo, P., 2021,
      Automatic forest road extraction fromLiDAR data of mountainous areas.
      In the First International Joint Conference of Discrete Geometry
      and Mathematical Morphology (Springer LNCS 12708), pp. 93-106.

    This program is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program.  If not, see <https://www.gnu.org/licenses/>.
*/

#ifndef AMREL_TIMER_H
#define AMREL_TIMER_H

#include "amreltool.h"


/** 
 * @class AmrelTimer amreltimer.h
 * \brief Time performance tester for AMREL.
 */
class AmrelTimer
{
public:

  /** Tested AMREL step : no test. */
  static const int NO_TEST;
  /** Tested AMREL step : all steps. */
  static const int FULL;
  /** Tested AMREL step : all steps, tile loading excluded. */
  static const int FULL_WITHOUT_LOAD;
  /** Tested AMREL step : tile loading. */
  static const int ONLY_LOAD;
  /** Tested AMREL step : all AMREL steps. */
  static const int BY_STEP;


  /**
   * \brief Creates a time performance tester.
   * @param amreltool Road extractor to be tested.
   */
  AmrelTimer (AmrelTool *amreltool);

  /**
   * \brief Deletes the configuration.
   */
  ~AmrelTimer ();

  /**
   * \brief Requests a specific time performance on AMREL.
   * @param type Require test type.
   */
  inline void request (int type) { test_type = type; }

  /**
   * \brief Requests a specific time performance on AMREL.
   * @param type Require test type.
   */
  inline bool isRequested () const { return (test_type != NO_TEST); }

  /**
   * \brief Sets the number of test repetitions
   * @param type Requested number of test repetitions.
   */
  inline void repeat (int count) { test_count = count; }

  /**
   * \brief Runs AMREL time performance tests.
   */
  void run ();

  /**
   * \brief Tests tile loading performance.
   */
  void tileLoadPerf ();

  /**
   * Runs detection performance.
   * @param with_load Local memory allocation if true.
   */
  void performanceTest (bool with_load);

  /**
   * Runs detection performance on all the steps.
   */
  void allStepsTest ();

  /**
   * Runs detection performance on all the seed selection steps.
   */
  void sawingTest ();

  /**
   * Runs detection performance on slope shading step.
   */
  void shadingTest ();

  /**
   * Runs detection performance on RORPO step.
   */
  void rorpoTest ();

  /**
   * Runs detection performance on Sobel step.
   */
  void sobelTest ();

  /**
   * Runs detection performance on FBSD step.
   */
  void fbsdTest ();

  /**
   * Runs detection performance on ASD road extraction step.
   */
  void asdTest ();

  /**
   * Runs detection performance on seed generation step.
   */
  void seedsTest ();


private:

  /** Pointer to road extractor. */
  AmrelTool *amrel;
  /** Time performance test type. */
  int test_type;
  /** Test repetition number. */
  int test_count;

};
#endif
