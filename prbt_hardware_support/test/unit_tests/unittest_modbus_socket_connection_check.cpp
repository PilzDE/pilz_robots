/*
 * Copyright (c) 2020 Pilz GmbH & Co. KG
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

#include <gtest/gtest.h>

#include <numeric>
#include <modbus/modbus.h>

#include <prbt_hardware_support/libmodbus_client.h>
#include <prbt_hardware_support/modbus_socket_connection_check.h>
#include <prbt_hardware_support/pilz_modbus_server_mock.h>
#include <prbt_hardware_support/pilz_modbus_exceptions.h>
#include <prbt_hardware_support/client_tests_common.h>

namespace modbus_socket_connection_check_test
{
using namespace prbt_hardware_support;

// Each testcase should have its own port in order to avoid conflicts between them
constexpr unsigned int START_PORT{ 20600 };
constexpr unsigned int END_PORT{ 20700 };
static unsigned int ACTIVE_PORT_IDX{ 0 };
static std::vector<unsigned int> PORTS_FOR_TEST(END_PORT - START_PORT);

constexpr unsigned int DEFAULT_REGISTER_SIZE{ 514 };
constexpr unsigned int DEFAULT_WRITE_IDX{ 512 };

class ModbusSocketConnectionTest : public testing::Test
{
public:
  static void SetUpTestCase();  // NOLINT
  void TearDown() override;
  unsigned int testPort();

protected:
  void shutdownModbusServer(PilzModbusServerMock* server, LibModbusClient& client);
};

void ModbusSocketConnectionTest::TearDown()
{
  // Use next port on next test
  ACTIVE_PORT_IDX++;
}

unsigned int ModbusSocketConnectionTest::testPort()
{
  return PORTS_FOR_TEST.at(ACTIVE_PORT_IDX % PORTS_FOR_TEST.size());
}

void ModbusSocketConnectionTest::SetUpTestCase()  // NOLINT
{
  std::iota(PORTS_FOR_TEST.begin(), PORTS_FOR_TEST.end(), START_PORT);
}

void ModbusSocketConnectionTest::shutdownModbusServer(PilzModbusServerMock* server, LibModbusClient& client)
{
  server->setTerminateFlag();
  RegCont reg_to_write_by_client{ 1 };
  try
  {
    client.writeReadHoldingRegister(DEFAULT_REGISTER_SIZE, reg_to_write_by_client, DEFAULT_WRITE_IDX,
                                    DEFAULT_REGISTER_SIZE - DEFAULT_WRITE_IDX);
  }
  catch (const ModbusExceptionDisconnect& /* ex */)
  {
    // Tolerated exception
  }

  server->terminate();
}

/**
 * @brief Tests that the checkIPConnection function will respond properly on presense of a modbus server and also if
 * it misses.
 *
 * @note To keep things simple timeout and repeats are not altered.
 */
TEST_F(ModbusSocketConnectionTest, checkIPConnectionServerPresentIpPortCorrect)
{
  EXPECT_FALSE(checkIPConnection(LOCALHOST, testPort())) << "No Server";
  std::shared_ptr<PilzModbusServerMock> server(new PilzModbusServerMock(DEFAULT_REGISTER_SIZE));
  server->startAsync(LOCALHOST, testPort());
  LibModbusClient client;
  // Needed to make sure server is actually present. (Not optimal,
  // needs adaption inside the server mock)
  EXPECT_TRUE(client.init(LOCALHOST, testPort())) << "Server not present";

  EXPECT_TRUE(checkIPConnection(LOCALHOST, testPort())) << "Server present ip+port correct, TRUE expected";
  shutdownModbusServer(server.get(), client);
  client.close();
}

TEST_F(ModbusSocketConnectionTest, checkIPConnectionPortWrong)
{
  EXPECT_FALSE(checkIPConnection(LOCALHOST, testPort())) << "No Server";
  std::shared_ptr<PilzModbusServerMock> server(new PilzModbusServerMock(DEFAULT_REGISTER_SIZE));
  server->startAsync(LOCALHOST, testPort());
  LibModbusClient client;

  // Needed to make sure server is actually present. (Not optimal,
  // needs adaption inside the server mock)
  EXPECT_TRUE(client.init(LOCALHOST, testPort())) << "Server not present";
  unsigned int wrong_port = 4711;
  EXPECT_FALSE(checkIPConnection(LOCALHOST, wrong_port)) << "Server present ip correct port wrong, FALSE expected";
  shutdownModbusServer(server.get(), client);
  client.close();
}

TEST_F(ModbusSocketConnectionTest, checkIPConnectionIpWrong)
{
  EXPECT_FALSE(checkIPConnection(LOCALHOST, testPort())) << "No Server";

  std::shared_ptr<PilzModbusServerMock> server(new PilzModbusServerMock(DEFAULT_REGISTER_SIZE));
  server->startAsync(LOCALHOST, testPort());
  LibModbusClient client;

  // Needed to make sure server is actually present. (Not optimal,
  // needs adaption inside the server mock)
  EXPECT_TRUE(client.init(LOCALHOST, testPort())) << "Server not present";
  EXPECT_FALSE(checkIPConnection("192.192.192.192", testPort())) << "Server present ip wrong port correct";
  shutdownModbusServer(server.get(), client);
  client.close();
}

TEST_F(ModbusSocketConnectionTest, checkIPConnectionIpWrongPortWrong)
{
  EXPECT_FALSE(checkIPConnection(LOCALHOST, testPort())) << "No Server";

  std::shared_ptr<PilzModbusServerMock> server(new PilzModbusServerMock(DEFAULT_REGISTER_SIZE));
  server->startAsync(LOCALHOST, testPort());
  LibModbusClient client;

  // Needed to make sure server is actually present. (Not optimal,
  // needs adaption inside the server mock)
  EXPECT_TRUE(client.init(LOCALHOST, testPort())) << "Server not present";
  unsigned int wrong_port = 4711;
  ASSERT_NE(wrong_port, testPort());
  EXPECT_FALSE(checkIPConnection("192.192.192.192", wrong_port))
      << "Server present ip wrong port wrong, FALSE expected";
  shutdownModbusServer(server.get(), client);
  client.close();
}

}  // namespace modbus_socket_connection_check_test

int main(int argc, char* argv[])
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
