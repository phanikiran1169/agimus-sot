// Copyright 2021 CNRS - Airbus SAS
// Author: Florent Lamiraux
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are
// met:

// 1. Redistributions of source code must retain the above copyright
// notice, this list of conditions and the following disclaimer.

// 2. Redistributions in binary form must reproduce the above copyright
// notice, this list of conditions and the following disclaimer in the
// documentation and/or other materials provided with the distribution.

// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
// "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
// LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
// A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
// HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
// SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
// LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
// DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
// THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
// (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
// OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

#ifndef AGIMUS_SOT_PART_LOCALIZATION_HH
#define AGIMUS_SOT_PART_LOCALIZATION_HH

#include <dynamic-graph/all-commands.h>
#include <dynamic-graph/entity.h>
#include <dynamic-graph/signal.h>
#include <dynamic-graph/signal-ptr.h>
#include <dynamic-graph/signal-time-dependent.h>

#include <sot/core/matrix-geometry.hh>

#include <agimus/sot/config.hh>

namespace dynamicgraph {
namespace agimus {

/// Localization of an object with a sensor mounted on a robot
///
/// This entity performs localization of an object with a sensor mounted on
/// a robot end-effector. The sensor is assumed to measure the relative pose
/// of the object with respect to the sensor frame.
///
/// The object is assumed to be static. measurement by the the sensor is
/// performed on demand. The position of the object in the world frame is
/// computed from the position of the end effector in the world frame.
///
/// The commands are
/// \li trigger that triggers measurement of the relative pose of the object
///     with respect to the sensor. Upon calling, the entity waits for the
///     measurement to be available and records the value once available.
///
/// The input signals are
/// \li cMo: measured: relative pose of the object with respect to the sensor,
/// \li cMoAvailable: whether the measurement cMo is available,
/// \li wMc: pose of the sensor in the world frame (usually computed by forward
///     kinematics.
///
/// The output signals are
/// \li wMo latest measure of the pose of the object in the world frame,
/// \li cMo estimated pose of the object in the camera frame built from the
///     latest measurement wMo and the motion of the camera since then.
/// \li done a bolean value set to false when trigger is set to true and
///     set to true once the measurement of cMo is performed.
class AGIMUS_SOT_DLLAPI ObjectLocalization : public Entity
{
 public:
  typedef dynamicgraph::sot::MatrixHomogeneous MatrixHomogeneous;
  static const std::string CLASS_NAME;
  virtual void display(std::ostream &os) const;
  virtual const std::string &getClassName(void) const { return CLASS_NAME; }

  /// Constructor
  ObjectLocalization(const std::string& name);

  // Signals
  SignalPtr<MatrixHomogeneous, int> cMoMeasuredSIN;
  SignalPtr<bool, int> cMoAvailableSIN;
  SignalPtr<MatrixHomogeneous, int> wMcSIN;

  Signal<MatrixHomogeneous, int> wMoSOUT;
  SignalTimeDependent<MatrixHomogeneous, int> cMoSOUT;
  Signal<bool, int> doneSOUT;
 private:
  void addCommands();
  MatrixHomogeneous& compute_cMo(MatrixHomogeneous& res, int t);
  void trigger(const int&);
  void setVisualServoingMode(const bool& visualServoing){
    visualServoingMode_ = visualServoing;
  }
  bool wMoInitialized_;
  bool visualServoingMode_;
}; // class PartLocalization
} // namespace agimus
} // namespace dynamicgraph
#endif // AGIMUS_SOT_PART_LOCALIZATION_HH
