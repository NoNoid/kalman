//
//  Car1MeasurementModel.hpp
//  Kalman
//
//  Created by Sumeru Chatterjee on 1/13/16.
//  Copyright © 2016 Sumeru Chatterjee. All rights reserved.
//

#ifndef Car1MeasurementModel_h
#define Car1MeasurementModel_h

namespace KalmanExamples
{

namespace Car1
{

/**
 * @brief Measurement vector measuring the car measurements
 *
 * @param T Numeric scalar type
 */
template <typename T>
class CarMeasurement : public Kalman::Vector<T, 4>
{
public:
	KALMAN_VECTOR(CarMeasurement, T, 4)

	static constexpr size_t X = 0;

	static constexpr size_t Y = 1;

	static constexpr size_t VELOCITY = 2;

	static constexpr size_t YAWRATE = 3;

	T x() const { return (*this)[X]; }
	T y() const { return (*this)[Y]; }
	T velocity() const { return (*this)[VELOCITY]; }
	T yawrate() const { return (*this)[YAWRATE]; }

	T& x() { return (*this)[X]; }
	T& y() { return (*this)[Y]; }
	T& velocity() { return (*this)[VELOCITY]; }
	T& yawrate() { return (*this)[YAWRATE]; }
};

/**
* @brief Measurement model for measuring the state from sensors from a car
*
* @param T Numeric scalar type
* @param CovarianceBase Class template to determine the covariance representation
*                       (as covariance matrix (StandardBase) or as lower-triangular
*                       coveriace square root (SquareRootBase))
*/
template <typename T, template <class> class CovarianceBase = Kalman::StandardBase>
class CarMeasurementModel : public Kalman::LinearizedMeasurementModel<State<T>, CarMeasurement<T>, CovarianceBase>
{
public:
	//! State type shortcut definition
	typedef KalmanExamples::Car1::State<T> S;

	//! Measurement type shortcut definition
	typedef KalmanExamples::Car1::CarMeasurement<T> M;

	/**
     * @brief Constructor
     *
     * @param x x
     * @param y y
     * @param velocity velocity
     * @param yawrate yawrate
     */
	CarMeasurementModel(T x, T y, T velocity, T yawrate)
	{
        x_ = x;
        y_ = y;
        velocity_ = velocity;
        yawrate_ = yawrate;

		// Setup noise jacobian. As this one is static, we can define it once
		// and do not need to update it dynamically
		this->V.setIdentity();
	}

	/**
     * @brief Definition of (possibly non-linear) measurement function
     *
     * This function maps the system state to the measurement that is expected
     * to be received from the sensor assuming the system is currently in the
     * estimated state.
     *
     * @param [in] x The system state in current time-step
     * @returns The (predicted) sensor measurement for the system state
     */
	M h(const S& x) const
	{
		M measurement;

        measurement.x() = x_;
        measurement.y() = y_;
        measurement.velocity() = velocity_;
        measurement.yawrate() = yawrate_;

		return measurement;
	}

protected:
    
    T x_;
    T y_;
    T velocity_;
    T yawrate_;

protected:
	/**
     * @brief Update jacobian matrices for the system state transition function using current state
     *
     * This will re-compute the (state-dependent) elements of the jacobian matrices
         * to linearize the non-linear measurement function \f$h(x)\f$ around the
         * current state \f$x\f$.
         *
         * @note This is only needed when implementing a LinearizedSystemModel,
         *       for usage with an ExtendedKalmanFilter or SquareRootExtendedKalmanFilter.
         *       When using a fully non-linear filter such as the UnscentedKalmanFilter
         *       or its square-root form then this is not needed.
         *
         * @param x The current system state around which to linearize
         * @param u The current system control input
         */
	void updateJacobians(const S& x)
	{
		// H = dh/dx (Jacobian of measurement function w.r.t. the state)
		this->H.setZero();
        
        this->H( M::VELOCITY, S::VELOCITY ) = 1;
        this->H( M::YAWRATE,  S::YAWRATE ) = 1;

        //If GPS is available TODO: Find the correct way to figure out if GPS is available
        if (x.x() > 0 && x.y() > 0)
        {
            this->H( M::X, S::X ) = 1;
            this->H( M::Y, S::Y ) = 1;
        }
	}
};

} // namespace Car1

} // namespace KalmanExamples

#endif /* Car1MeasurementModel_h */
