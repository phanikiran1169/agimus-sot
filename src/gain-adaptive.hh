#ifndef AGIMUS_SOT_GAIN_ADAPTATIVE_HH
#define AGIMUS_SOT_GAIN_ADAPTATIVE_HH

#include <dynamic-graph/linear-algebra.h>
#include <dynamic-graph/all-signals.h>
#include <dynamic-graph/entity.h>

#include <agimus/sot/config.hh>

namespace dynamicgraph {
namespace agimus {

/** Exponentially decreasing gain that is safer than
 * dynamicgraph::sot::GainAdaptive
 *
 * It follows the law \f[ g(e) = a \exp (-b ||e||) + c \frac{\tanh(d ||e||)}{||e||} \f].
 *
 * Use the following code to see the curves:
 * \code
 * from agimus_sot.sot import SafeGainAdaptive
 * import numpy, matplotlib.pyplot as plt
 * gain = SafeGainAdaptive('g')
 * #g.computeParameters(0.1, 1., 0.1, 2.)
 *
 * errors = numpy.linspace(0, 5., 1000)
 * def compute(e):
 *     t = gain.error.time + 1
 *     gain.error.value = (e,)
 *     gain.error.time = t
 *     gain.gain.recompute(t)
 *     return gain.gain.value
 *
 * gains = [ compute(e) for e in errors ]
 *
 * lg = plt.plot(errors, gains, 'r', label="Gain")
 * ld = plt.twinx().plot(errors, [ g*e for e,g in zip(errors,gains) ], 'b', label="Derivative")
 * lines = lg + ld
 * plt.legend(lines, [l.get_label() for l in lines])
 * plt.show()
 * \endcode
 */
class AGIMUS_SOT_DLLAPI SafeGainAdaptive : public Entity {
public: /* --- ENTITY INHERITANCE --- */
  static const std::string CLASS_NAME;
  virtual void display(std::ostream &os) const;
  virtual const std::string &getClassName(void) const { return CLASS_NAME; }

protected:
  double a, b, c, d;
  Vector cs;

public: /* --- CONSTRUCTORS ---- */
  SafeGainAdaptive(const std::string &name);

  /** \brief Set the gain parameters.
   * \param a, b, c, d the gain parameters.
   */
  void setParameters(const double &a, const double &b, const double &c, const double &d);


  /** \brief Compute the parameters from specific values.
   * \param valueAt0 value of the gain at zero,
   * \param valueAtInfty value of \f$ gain(||e||) ||e|| \f$ when
   *                     \f$ ||e||\to\infty \f$
   * \param errNormOfMaxExp norm of the error for which the exponential part is
   *                        maximum
   * \param errNormOfSwitch norm of the error for which the hyperbolic part is
   *                        half of its final value
   */
  void computeParameters(const double &valueAt0, const double &valueAtInfty,
                         const double &errNormOfMaxExp,
                         const double &errNormOfHalfHyp);

public: /* --- SIGNALS --- */
  SignalPtr<Vector, int> errorSIN;
  SignalTimeDependent<double, int> gainSOUT;

protected:
  double &computeGain(double &res, int t);

private:
  void addCommands();
};

} /* namespace agimus */
} /* namespace dynamicgraph */

#endif // AGIMUS_SOT_GAIN_ADAPTATIVE_HH
