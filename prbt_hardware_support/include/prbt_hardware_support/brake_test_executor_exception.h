/*
 * Copyright (c) 2019 Pilz GmbH & Co. KG
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU Lesser General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.

 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU Lesser General Public License for more details.

 * You should have received a copy of the GNU Lesser General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#ifndef PRBT_HARDWARE_SUPPORT_BRAKE_TEST_EXECUTOR_EXCEPTION_H
#define PRBT_HARDWARE_SUPPORT_BRAKE_TEST_EXECUTOR_EXCEPTION_H

#include <stdexcept>

#include <prbt_hardware_support/BrakeTestErrorCodes.h>

namespace prbt_hardware_support
{

/**
 * @brief Exception thrown by the BrakeTestExecutor.
 */
class BrakeTestExecutorException : public std::runtime_error
{
public:
  BrakeTestExecutorException(const std::string &what_arg,
                             const int8_t error_value = BrakeTestErrorCodes::FAILURE);

public:
  int8_t getErrorValue() const;

private:
  int8_t error_value_;
};

}

#endif // PRBT_HARDWARE_SUPPORT_BRAKE_TEST_EXECUTOR_EXCEPTION_H
