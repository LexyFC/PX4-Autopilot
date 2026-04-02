/****************************************************************************
 *
 *   Copyright (c) 2026 PX4 Development Team. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

/**
 * @file test_mavlink_forwarding_bench.cpp
 *
 * MAVLink forwarding mutex benchmark.
 * Sends high-rate broadcast messages to multiple PX4 MAVLink instances
 * to measure the cost of mavlink_module_mutex in the forwarding path.
 *
 * Usage:
 *   1. Start PX4 SITL: make px4_sitl_sih
 *   2. Build and run this benchmark (see CMakeLists.txt)
 *   3. After the run completes, dump perf counters in the PX4 console:
 *      > perf
 *
 * The benchmark connects to two UDP ports (GCS + offboard) and floods
 * broadcast DEBUG_FLOAT_ARRAY messages. Since target_system=0 (broadcast),
 * every message hits forward_message() and gets forwarded to all other
 * instances, creating real mutex contention between receiver threads.
 */

#include <mavsdk/mavsdk.h>
#include <mavsdk/plugins/mavlink_passthrough/mavlink_passthrough.h>
#include <mavsdk/plugins/shell/shell.h>
#include <mavsdk/plugins/telemetry/telemetry.h>

#include <atomic>
#include <chrono>
#include <cinttypes>
#include <cmath>
#include <csignal>
#include <cstring>
#include <iostream>
#include <thread>
#include <vector>

using namespace mavsdk;
using namespace std::chrono;
using namespace std::chrono_literals;

static std::atomic<bool> g_stop{false};

static void signal_handler(int /*signum*/)
{
	g_stop.store(true);
}

struct SenderStats {
	std::atomic<uint64_t> messages_sent{0};
	std::atomic<uint64_t> send_errors{0};
};

// Send broadcast DEBUG_FLOAT_ARRAY at the given rate on a single connection.
// Broadcast (sysid=0) ensures the message always enters forward_message().
static void sender_thread(MavlinkPassthrough &passthrough,
			  int rate_hz,
			  SenderStats &stats)
{
	const auto interval = microseconds(1'000'000 / rate_hz);
	auto next_send = steady_clock::now();

	while (!g_stop.load()) {
		// DEBUG_FLOAT_ARRAY is a good choice: variable-length, commonly supported,
		// and has no side effects on PX4 when received.
		// No target fields, so it defaults to broadcast forwarding in PX4.
		auto result = passthrough.queue_message(
		[](MavlinkAddress mavlink_address, uint8_t channel) {
			mavlink_message_t msg{};
			mavlink_msg_debug_float_array_pack_chan(
				mavlink_address.system_id,
				mavlink_address.component_id,
				channel,
				&msg,
				0,           // time_usec
				"bench",     // name
				0,           // array_id
				nullptr      // data (empty)
			);
			return msg;
		});

		if (result == MavlinkPassthrough::Result::Success) {
			stats.messages_sent.fetch_add(1, std::memory_order_relaxed);

		} else {
			stats.send_errors.fetch_add(1, std::memory_order_relaxed);
		}

		next_send += interval;
		std::this_thread::sleep_until(next_send);
	}
}

static void print_usage(const char *bin)
{
	std::cout << "Usage: " << bin << " [options]\n"
		  << "  --duration <sec>     Test duration in seconds (default: 60)\n"
		  << "  --rate <hz>          Messages per second per connection (default: 200)\n"
		  << "  --connections <n>    Number of UDP connections (default: 2)\n"
		  << "  --url <url>          MAVSDK connection URL (e.g. serial:///dev/ttyACM0:57600)\n"
		  << "                       When set, uses a single connection (ignores --connections)\n"
		  << "  --gcs-port <port>    GCS UDP port (default: 14550)\n"
		  << "  --api-port <port>    Offboard/API UDP port (default: 14540)\n"
		  << "  --report <file>      Write CSV report to file (default: stdout)\n"
		  << std::endl;
}

int main(int argc, char *argv[])
{
	int duration_sec = 60;
	int rate_hz = 200;
	int num_connections = 2;
	int gcs_port = 14550;
	int api_port = 14540;
	std::string report_file;
	std::string connection_url;

	for (int i = 1; i < argc; ++i) {
		std::string arg(argv[i]);

		if (arg == "--duration" && i + 1 < argc) {
			duration_sec = std::atoi(argv[++i]);

		} else if (arg == "--rate" && i + 1 < argc) {
			rate_hz = std::atoi(argv[++i]);

		} else if (arg == "--connections" && i + 1 < argc) {
			num_connections = std::atoi(argv[++i]);

		} else if (arg == "--url" && i + 1 < argc) {
			connection_url = argv[++i];
			num_connections = 1;

		} else if (arg == "--gcs-port" && i + 1 < argc) {
			gcs_port = std::atoi(argv[++i]);

		} else if (arg == "--api-port" && i + 1 < argc) {
			api_port = std::atoi(argv[++i]);

		} else if (arg == "--report" && i + 1 < argc) {
			report_file = argv[++i];

		} else if (arg == "--help" || arg == "-h") {
			print_usage(argv[0]);
			return 0;

		} else {
			std::cerr << "Unknown argument: " << arg << std::endl;
			print_usage(argv[0]);
			return 1;
		}
	}

	if (duration_sec <= 0 || rate_hz <= 0 || num_connections <= 0) {
		std::cerr << "Error: --duration, --rate, and --connections must be positive" << std::endl;
		return 1;
	}

	if (num_connections > 10) {
		std::cerr << "Error: max 10 connections (sysid 245-254)" << std::endl;
		return 1;
	}

	signal(SIGINT, signal_handler);
	signal(SIGTERM, signal_handler);

	std::cout << "=== MAVLink Forwarding Mutex Benchmark ===" << std::endl;
	std::cout << "Duration:    " << duration_sec << "s" << std::endl;
	std::cout << "Rate:        " << rate_hz << " Hz per connection" << std::endl;
	std::cout << "Connections: " << num_connections << std::endl;
	std::cout << std::endl;

	// Build list of connection URLs
	std::vector<std::string> urls;

	if (!connection_url.empty()) {
		urls.push_back(connection_url);

	} else {
		std::vector<int> ports;

		if (num_connections >= 1) {
			ports.push_back(api_port);
		}

		if (num_connections >= 2) {
			ports.push_back(gcs_port);
		}

		for (int i = 2; i < num_connections; ++i) {
			ports.push_back(api_port + i);
		}

		for (int p : ports) {
			urls.push_back("udp://:" + std::to_string(p));
		}
	}

	// Create MAVSDK instances and connect
	std::vector<std::unique_ptr<Mavsdk>> mavsdks;
	std::vector<std::unique_ptr<MavlinkPassthrough>> passthroughs;

	for (size_t i = 0; i < urls.size(); ++i) {
		auto config = Mavsdk::Configuration(
				      static_cast<uint8_t>(245 + i),  // unique sysid per connection
				      MAV_COMP_ID_MISSIONPLANNER,
				      false                           // not autopilot
			      );
		auto mavsdk = std::make_unique<Mavsdk>(config);

		std::cout << "Connecting to " << urls[i] << " (sysid=" << (245 + i) << ")..." << std::endl;

		auto result = mavsdk->add_any_connection(urls[i]);

		if (result != ConnectionResult::Success) {
			std::cerr << "Connection failed: " << urls[i] << std::endl;
			return 1;
		}

		// Wait for system discovery
		std::cout << "  Waiting for system..." << std::flush;
		bool found = false;

		for (int t = 0; t < 100; ++t) {  // 10 second timeout for serial
			if (!mavsdk->systems().empty()) {
				found = true;
				break;
			}

			std::this_thread::sleep_for(100ms);
		}

		if (!found) {
			std::cerr << "\n  Timeout waiting for system on " << urls[i] << std::endl;
			return 1;
		}

		std::cout << " OK" << std::endl;

		auto system = mavsdk->systems().front();
		passthroughs.push_back(std::make_unique<MavlinkPassthrough>(system));
		mavsdks.push_back(std::move(mavsdk));
	}

	std::cout << std::endl;
	std::cout << "Starting " << passthroughs.size() << " sender threads at " << rate_hz << " Hz each..." << std::endl;
	std::cout << "Total expected rate: " << (rate_hz * static_cast<int>(passthroughs.size())) << " msg/s" << std::endl;
	std::cout << std::endl;

	// Launch sender threads
	std::vector<SenderStats> stats(passthroughs.size());
	std::vector<std::thread> threads;

	auto start_time = steady_clock::now();

	for (size_t i = 0; i < passthroughs.size(); ++i) {
		threads.emplace_back(sender_thread, std::ref(*passthroughs[i]), rate_hz, std::ref(stats[i]));
	}

	// Progress reporting
	auto report_interval = 5s;
	auto next_report = start_time + report_interval;

	while (!g_stop.load()) {
		auto now = steady_clock::now();

		if (now >= start_time + seconds(duration_sec)) {
			break;
		}

		if (now >= next_report) {
			auto elapsed = duration_cast<seconds>(now - start_time).count();
			uint64_t total_sent = 0;
			uint64_t total_errors = 0;

			for (auto &s : stats) {
				total_sent += s.messages_sent.load(std::memory_order_relaxed);
				total_errors += s.send_errors.load(std::memory_order_relaxed);
			}

			double actual_rate = static_cast<double>(total_sent) / static_cast<double>(elapsed);
			std::cout << "[" << elapsed << "s] sent=" << total_sent
				  << " errors=" << total_errors
				  << " rate=" << static_cast<int>(actual_rate) << " msg/s"
				  << std::endl;

			next_report += report_interval;
		}

		std::this_thread::sleep_for(100ms);
	}

	// Stop senders
	g_stop.store(true);

	for (auto &t : threads) {
		t.join();
	}

	auto end_time = steady_clock::now();
	double total_elapsed = duration_cast<milliseconds>(end_time - start_time).count() / 1000.0;

	// Final report
	std::cout << std::endl;
	std::cout << "=== Results ===" << std::endl;
	std::cout << "Elapsed:     " << total_elapsed << "s" << std::endl;

	uint64_t grand_total_sent = 0;
	uint64_t grand_total_errors = 0;

	for (size_t i = 0; i < stats.size(); ++i) {
		uint64_t sent = stats[i].messages_sent.load();
		uint64_t errors = stats[i].send_errors.load();
		double rate = static_cast<double>(sent) / total_elapsed;

		std::cout << "  Connection " << i << " (" << urls[i] << "): "
			  << "sent=" << sent
			  << " errors=" << errors
			  << " rate=" << static_cast<int>(rate) << " msg/s"
			  << std::endl;

		grand_total_sent += sent;
		grand_total_errors += errors;
	}

	double grand_rate = static_cast<double>(grand_total_sent) / total_elapsed;
	std::cout << "  Total: sent=" << grand_total_sent
		  << " errors=" << grand_total_errors
		  << " rate=" << static_cast<int>(grand_rate) << " msg/s"
		  << std::endl;

	std::cout << std::endl;
	std::cout << "Now run 'perf' in the PX4 console to dump forwarding perf counters." << std::endl;
	std::cout << "Look for:" << std::endl;
	std::cout << "  mavlink: fwd_msg total   -- total forward_message() time" << std::endl;
	std::cout << "  mavlink: fwd_msg lock    -- time under mavlink_module_mutex" << std::endl;
	std::cout << "  mavlink: comp_seen total -- static component_was_seen() time" << std::endl;
	std::cout << "  mavlink: unsigned_cb     -- accept_unsigned_callback time" << std::endl;

	// CSV report output
	FILE *out = stdout;

	if (!report_file.empty()) {
		out = fopen(report_file.c_str(), "w");

		if (!out) {
			std::cerr << "Failed to open report file: " << report_file << std::endl;
			out = stdout;
		}
	}

	fprintf(out, "connection,url,sysid,messages_sent,send_errors,duration_s,rate_hz_requested,rate_hz_actual\n");

	for (size_t i = 0; i < stats.size(); ++i) {
		uint64_t sent = stats[i].messages_sent.load();
		uint64_t errors = stats[i].send_errors.load();
		double rate = static_cast<double>(sent) / total_elapsed;

		fprintf(out, "%zu,%s,%d,%" PRIu64 ",%" PRIu64 ",%.1f,%d,%.1f\n",
			i, urls[i].c_str(), static_cast<int>(245 + i),
			sent, errors, total_elapsed, rate_hz, rate);
	}

	if (out != stdout) {
		fclose(out);
		std::cout << "CSV report written to: " << report_file << std::endl;
	}

	// Read bench counters from the board via MAVLink shell
	std::cout << std::endl;
	std::cout << "=== Reading board perf counters ===" << std::endl;

	auto system = mavsdks.front()->systems().front();
	Shell shell(system);

	std::string shell_output;
	std::atomic<bool> got_bench{false};

	shell.subscribe_receive([&](std::string output) {
		shell_output += output;
		std::cout << output;

		if (output.find("fwd_msg lock") != std::string::npos) {
			got_bench.store(true);
		}
	});

	shell.send("mavlink status");

	for (int i = 0; i < 100 && !got_bench.load(); ++i) {
		std::this_thread::sleep_for(100ms);
	}

	std::this_thread::sleep_for(500ms);

	return 0;
}
