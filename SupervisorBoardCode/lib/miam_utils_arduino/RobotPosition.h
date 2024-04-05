/// \file trajectory/RobotPosition.h
/// \brief Class representing the robot position (x,y, angle).
/// \author MiAM Robotique, Matthieu Vigne
/// \copyright GNU GPLv3
#ifndef MIAM_ROBOT_POSITION
#define MIAM_ROBOT_POSITION

    class DrivetrainKinematics;
    class WheelSpeed;

    namespace miam{
        /// \brief Robot coordinates on the table.
        class RobotPosition
        {
            public:
                /// \brief Default constructor.
                RobotPosition();

                /// \brief Constructor.
                ///
                /// \param[in] xIn x coordinate.
                /// \param[in] yIn y coordinate.
                /// \param[in] thetaIn theta coordinate.
                RobotPosition(float const& xIn, float const& yIn, float const& thetaIn);

                /// \brief Sum two position.
                ///
                /// \details This function returns the term by term sum of the two points (i.e. coordinates are
                ///          the sum of the input coordinates, angle is the sum of both angles).
                friend RobotPosition operator+(RobotPosition const& p1, RobotPosition const& p2);

                /// \brief Difference between two position.
                ///
                /// \details This function returns the term by term difference of the two points (i.e. coordinates are
                ///          the difference of the input coordinates, angle is the difference of both angles).
                friend RobotPosition operator-(RobotPosition const& p1, RobotPosition const& p2);

                /// \brief Multiply a position by a scalar.
                ///
                /// \details This function returns the term by term multiplication of the point coordinates by a scalar.
                friend RobotPosition operator*(float const& scalar, RobotPosition const& p1);
                friend RobotPosition operator*(RobotPosition const& p1, float const& scalar);

                /// \brief Division by a scalar.
                ///
                /// \details This function returns the term by term division of the point coordinates by a scalar.
                ///          If scalar is too close to zero, this function return the original point.
                friend RobotPosition operator/(RobotPosition const& p1, float const& scalar);

                /// \brief Return the norm of the (x,y) vector.
                float norm() const;

                /// \brief Normalize the (x, y) vector.
                ///
                /// \details If the norm is too close to zero, this function does nothing. The angle is untouched by
                ///          this function.
                void normalize();


                /// \brief Project current vector onto another vector.
                ///
                /// \details Perform the projection of the (x, y) vector onto the projection target, i.e
                ///          colinear + normal = vector. The angle of both colinear and normal is set to 0.
                ///
                /// \param[in] projectionTarget Vector to which to project.
                /// \param[out] colinear Vector colinear to projectionTarget.
                /// \param[out] normal Vector normal to projectionTarget.
                void projectOnto(RobotPosition const& projectionTarget, RobotPosition & colinear, RobotPosition & normal);

                /// \brief Dot product,
                ///
                /// \details Return the dot product between both vectors. Angle is not taken into account.
                /// \param[in] secondVector Vector with witch to compute the dot product.
                /// \return The dot product, i.e x * secondVector.x + y * secondVector.y
                float dot(RobotPosition const& secondVector) const;

                /// \brief Cross product,
                ///
                /// \details Return the cross product between both vectors. Angle is not taken into account.
                /// \param[in] secondVector Vector with witch to compute the dot product.
                /// \return The cross product, i.e. x * secondVector.y - y * secondVector.x
                float cross(RobotPosition const& secondVector) const;


                /// \brief Rotate the position by a given angle.
                ///
                /// \param[in] thetaIn Rotation angle.
                /// \return Position rotated by the given angle.
                RobotPosition rotate(float const& thetaIn);

                float x;    ///< X coordinate of the robot, in mm.
                float y;    ///< Y coordinate of the robot, in mm. Notice that y axis is taken positive when pointing downward.
                float theta;    ///< Angle of the robot, in rad.
        };
    }
#endif
