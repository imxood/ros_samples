#ifndef PARAMETERS_H
#define PARAMETERS_H

/*****************************************************************************
 ** Includes
 *****************************************************************************/

#include <string>

/*****************************************************************************
 ** Namespaces
 *****************************************************************************/

namespace robot
{

/*****************************************************************************
 ** Interface
 *****************************************************************************/
/**
 * @brief 机器人所需要的参数列表.
 */
class Parameters
{
public:
    Parameters() :
        device_port("/dev/ttyUSB0"),
        sigslots_namespace("/robot"),
        left_wheel_joint("left_wheel"),
        right_wheel_joint("right_wheel"),
        ommi_wheel_joint("ommi_wheel")
    {
    }

    std::string device_port;
    std::string sigslots_namespace;
    std::string left_wheel_joint;
    std::string right_wheel_joint;
    std::string ommi_wheel_joint;


    /**
   * @brief A validator to ensure the user has supplied correct/sensible parameter values.
   *
   * This validates the current parameters and if invalid, puts an error string in error_msg.
   *
   * @return bool : true if valid, false otherwise.
   */
    bool validate()
    {
        // not doing anything right now -  delete it, if we can find a use case ...
        return true;
    }

    std::string error_msg; /**< @brief Provides error messages when parameter validation fails (internal purposes only) **/
};

} // namespace kobuki



#endif // PARAMETERS_H
