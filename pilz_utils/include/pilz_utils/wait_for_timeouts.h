/*
 * Copyright (c) 2019 Pilz GmbH & Co. KG
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#ifndef PILZ_UTILS_WAIT_FOR_TIMEOUTS_H
#define PILZ_UTILS_WAIT_FOR_TIMEOUTS_H

namespace pilz_utils
{
static constexpr double DEFAULT_RETRY_TIMEOUT{ 0.2 };
static constexpr double DEFAULT_MSG_OUTPUT_PERIOD{ 5.0 };

}  // namespace pilz_utils

#endif  // PILZ_UTILS_WAIT_FOR_TIMEOUTS_H
