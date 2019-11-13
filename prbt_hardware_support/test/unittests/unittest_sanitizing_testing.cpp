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

#include <iostream>
#include <stdio.h>
#include <limits>

#include <gtest/gtest.h>
#include <gmock/gmock.h>


namespace sanitizing_test
{

TEST(BrakeTestExecutorTest, testAddressSanitizing)
{
  char *x = (char*)malloc(10 * sizeof(char*));
  free(x);
  std::cout << x[5] << std::endl;
}

TEST(BrakeTestExecutorTest, testMemorySanitizing)
{
  int* a = new int[10];
  a[5] = 0;
  if (a[11])
  {
    printf("xx\n");
  }
}

TEST(BrakeTestExecutorTest, testUndefinedBehaviorSanitizing)
{
  int k = std::numeric_limits<int>::max();
  k++;
}


} // namespace sanitizing_test

int main(int argc, char *argv[])
{
  testing::InitGoogleMock(&argc, argv);
  return RUN_ALL_TESTS();
}
