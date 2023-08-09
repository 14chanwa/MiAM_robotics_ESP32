/// \file PID.h
/// \brief Implementation of a PID controller
/// \author MiAM Robotique, Matthieu Vigne
/// \copyright GNU GPLv3
#ifndef MIAM_PID
#define MIAM_PID

    namespace miam{
        class PID
        {
            public:
                /// \brief Default constructor.
                PID();

                /// \brief Constructor.
                /// \param[in] Kp Proportional gain.
                /// \param[in] Kd Derivative gain.
                /// \param[in] Ki Integral gain.
                /// \param[in] maxIntegral Maximum value of Ki * integral_
                PID(float const& Kp, float const& Kd, float const& Ki, float const& maxIntegral);

                /// \brief Compute PID output.
                /// \param[in] error Value of current error (current - target).
                /// \param[in] dt Time since last call, used to compute integral and derivative.
                float computeValue(float const& error, float const& dt);

                /// \brief Compute PID output.
                /// \param[in] error Value of current error (current - target).
                /// \param[in] errorDerivative Derivative of the error term.
                /// \param[in] dt Time since last call, used to compute integral and derivative.
                float computeValue(float const& error, float const& errorDerivative, float const& dt);

                /// \brief Reset the integral to a specific value (default 0)
                void resetIntegral(float const& value = 0.0);

                float getCorrection();
                float getIntegral();

            private:
                float Kp_; ///< Proportional gain.
                float Kd_; ///< Derivative gain.
                float Ki_; ///< Integral gain.
                float maxIntegral_; ///< Maximum integral value.
                float integral_; ///< Current integral value.
                float previousError_; ///< Previous error, used to compute derivative.
                float lastCorrection_; ///< Last correction value computed.
        };
    }
#endif
