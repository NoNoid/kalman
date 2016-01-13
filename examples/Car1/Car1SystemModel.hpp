//
//  Car1SystemModel.hpp
//  Kalman
//
//  Created by Sumeru Chatterjee on 1/13/16.
//  Copyright © 2016 Sumeru Chatterjee. All rights reserved.
//

#ifndef Car1SystemModel_h
#define Car1SystemModel_h

#include <kalman/LinearizedSystemModel.hpp>

#include <math.h>

using namespace std;

namespace KalmanExamples
{

namespace Car1
{

/**
* @brief System state vector-type for a car
*
* This is a system state for a very simple car model that
* is characterized by its (x,y)-Position, heading, velocity and yaw rate.
*
* @param T Numeric scalar type
*/
template <typename T>
class State : public Kalman::Vector<T, 5>
{
public:
	KALMAN_VECTOR(State, T, 5)

	//! X-position
	static constexpr size_t X = 0;
	//! Y-Position
	static constexpr size_t Y = 1;
	//! Orientation
	static constexpr size_t HEADING = 2;
	//! Velocity
	static constexpr size_t VELOCITY = 3;
	//! Yaw Rate
	static constexpr size_t YAWRATE = 4;

	T x() const { return (*this)[X]; }
	T y() const { return (*this)[Y]; }
	T heading() const { return (*this)[HEADING]; }
	T velocity() const { return (*this)[VELOCITY]; }
	T yawrate() const { return (*this)[YAWRATE]; }

	T& x() { return (*this)[X]; }
	T& y() { return (*this)[Y]; }
	T& heading() { return (*this)[HEADING]; }
	T& velocity() { return (*this)[VELOCITY]; }
	T& yawrate() { return (*this)[YAWRATE]; }
};

/**
* @brief System control-input vector-type for a car
*
* This is the system control-input of a very simple car that
* can control the velocity in its current direction as well as the
* change in direction.
*
* @param T Numeric scalar type
*/
template <typename T>
class Control : public Kalman::Vector<T, 2>
{
public:
	KALMAN_VECTOR(Control, T, 2)

	//! Velocity
	static constexpr size_t VELOCITY = 0;
	//! Angular Rate (Orientation-change)
	static constexpr size_t YAWRATE = 1;

	T velocity() const { return (*this)[VELOCITY]; }
	T yawrate() const { return (*this)[YAWRATE]; }

	T& velocity() { return (*this)[VELOCITY]; }
	T& yawrate() { return (*this)[YAWRATE]; }
};

/**
* @brief System model for a simple car
*
* This is the system model defining how our car moves from one
* time-step to the next, i.e. how the system state evolves over time.
*
* @param T Numeric scalar type
* @param CovarianceBase Class template to determine the covariance representation
*                       (as covariance matrix (StandardBase) or as lower-triangular
*                       coveriace square root (SquareRootBase))
*/
template <typename T, template <class> class CovarianceBase = Kalman::StandardBase>
class SystemModel : public Kalman::LinearizedSystemModel<State<T>, Control<T>, CovarianceBase>
{
public:
	//! State type shortcut definition
	typedef State<T> S;

	//! Control type shortcut definition
	typedef Control<T> C;

	/**
    * @brief Definition of (non-linear) state transition function
    *
    * This function defines how the system state is propagated through time,
    * i.e. it defines in which state \f$\hat{x}_{k+1}\f$ is system is expected to
    * be in time-step \f$k+1\f$ given the current state \f$x_k\f$ in step \f$k\f$ and
    * the system control input \f$u\f$.
    *
    * @param [in] x The system state in current time-step
    * @param [in] u The control vector input
    * @returns The (predicted) system state in the next time-step
    */
	S f(const S& x, const C& u) const
	{
        //! Important: We are going to ignore the control input
        
		//! Predicted state vector after transition
		S x_;
        
        float dt = 1;
        
        if (fabs(x.yawrate() < 0.0001)) // driving straight
        {
            x_.x()          = x.x() + x.velocity() * cos(x.heading()) * dt;
            x_.y()          = x.y() + x.velocity() * sin(x.heading()) * dt;
            x_.heading()    = x.heading();
            x_.velocity()   = x.velocity();
            x_.yawrate()    = 0.0000001; // avoid numerical issues in Jacobians
        }
        else //turning
        {
            x_.x()          = x.x() + (x.velocity() / x.yawrate()) * ( sin( dt * x.yawrate() + x.heading()) - sin(x.heading()));
            x_.x()          = x.y() + (x.velocity() / x.yawrate()) * (-cos( dt * x.yawrate() + x.heading()) + cos(x.heading()));
            x_.heading()    = fmod((x.heading() + x.yawrate() * dt + M_PI),(2.0 * M_PI)) - M_PI;
            x_.velocity()   = x.velocity();
            x_.yawrate()    = x.yawrate();
        }

		// Return transitioned state vector
		return x_;
	}

protected:
	/**
     * @brief Update jacobian matrices for the system state transition function using current state
     *
     * This will re-compute the (state-dependent) elements of the jacobian matrices
     * to linearize the non-linear state transition function \f$f(x,u)\f$ around the
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
	void updateJacobians(const S& x, const C& u)
	{
        float dt = 1;
        
		// F = df/dx (Jacobian of state transition w.r.t. the state)
		this->F.setZero();

		this->F(S::X, S::X) = 1;
        this->F(S::Y, S::Y) = 1;
        this->F(S::HEADING, S::HEADING) = 1;
        this->F(S::VELOCITY, S::VELOCITY) = 1;
        this->F(S::YAWRATE, S::YAWRATE) = 1;
        
		this->F(S::X, S::HEADING)  = (x.velocity() / x.yawrate()) * ( - cos(x.heading()) + cos( dt * x.yawrate() + x.heading()) );
        this->F(S::Y, S::HEADING)  = (x.velocity() / x.yawrate()) * ( - sin(x.heading()) + sin( dt * x.yawrate() + x.heading()) );
        
        this->F(S::X, S::VELOCITY) = (1            / x.yawrate()) * ( - sin(x.heading()) + sin( dt * x.yawrate() + x.heading()) );
        this->F(S::Y, S::VELOCITY) = (1            / x.yawrate()) * (   cos(x.heading()) - cos( dt * x.yawrate() + x.heading()) );

        this->F(S::X, S::YAWRATE)  =  - (x.velocity() / pow(x.yawrate(),2)) * ( - sin(x.heading()) + sin( dt * x.yawrate() + x.heading()) )
                                      + (dt * x.velocity() / x.yawrate()) * (cos( dt * x.yawrate() + x.heading()));
        
        this->F(S::Y, S::YAWRATE)  =  - (x.velocity() / pow(x.yawrate(),2)) * (  cos(x.heading()) - cos( dt * x.yawrate() + x.heading()) )
                                      + (dt * x.velocity() / x.yawrate()) * (sin( dt * x.yawrate() + x.heading()));

        this->F(S::HEADING, S::YAWRATE) = dt;

		// W = df/dw (Jacobian of state transition w.r.t. the noise)
		this->W.setIdentity();
		// TODO: more sophisticated noise modelling
		//       i.e. The noise affects the the direction in which we move as
		//       well as the velocity (i.e. the distance we move)
	}
};

} // namespace Car1

} // namespace KalmanExamples

#endif /* Car1SystemModel_h */
