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

#ifndef OPERATION_MODE_H
#define OPERATION_MODE_H

#include <iostream>

namespace prbt_hardware_support
{

enum class OperationMode
{
    UNKNOWN = 0,
    T1 = 1,
    T2 = 2,
    AUTO = 3
};


inline std::ostream& operator<<(std::ostream& os, const OperationMode& mode)
{
    switch(mode)
    {
      case OperationMode::T1:
        os << "T1";
        break;
      case OperationMode::T2:
        os << "T2";
        break;
      case OperationMode::AUTO:
        os << "AUTOMATIC";
        break;
      default:
        os << "UNKNOWN";
        break;
    }
    return os;
}

}

#endif //OPERATION_MODE_H