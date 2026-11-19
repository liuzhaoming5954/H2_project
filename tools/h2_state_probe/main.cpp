// Passive DDS telemetry only. No control client or command publisher is instantiated.
#include <unitree/idl/hg/IMUState_.hpp>
#include <unitree/idl/hg/LowState_.hpp>
#include <unitree/robot/channel/channel_subscriber.hpp>

#include <array>
#include <chrono>
#include <cerrno>
#include <cmath>
#include <csignal>
#include <cstdlib>
#include <cstring>
#include <filesystem>
#include <iomanip>
#include <iostream>
#include <limits>
#include <mutex>
#include <sstream>
#include <stdexcept>
#include <string>
#include <thread>
#include <type_traits>
#include <fcntl.h>
#include <net/if.h>
#include <unistd.h>

namespace {
using LowState = unitree_hg::msg::dds_::LowState_;
using ImuState = unitree_hg::msg::dds_::IMUState_;
using Clock = std::chrono::steady_clock;
constexpr char kRevision[] = "8e1b927586712edd6a4c29d50dd52dff1e163439";
volatile std::sig_atomic_t stop_requested = 0;
void on_signal(int signal) { stop_requested = signal; }

struct Options {
  std::string interface, output;
  double duration = 30.0, timeout = 5.0, sample_hz = 50.0;
  std::size_t max_bytes = 256U * 1024U * 1024U;
  bool secondary = true;
};

void usage() {
  std::cout << "Usage: h2_state_probe --interface IFACE --output /absolute/new.jsonl\n"
               "  [--duration 30] [--timeout 5] [--sample-hz 50] [--max-mib 256]\n"
               "  [--lowstate-only]\n"
               "Passively samples rt/lowstate and rt/secondary_imu in Unitree DDS domain 0.\n"
               "Default requires fresh samples from both streams. No ROS, RPC or motion commands.\n"
               "Use a clean non-ROS shell. Output must be a NEW file; fields retain raw SDK semantics.\n";
}

double number(const std::string& value, double minimum, double maximum) {
  std::size_t used = 0;
  const double parsed = std::stod(value, &used);
  if (used != value.size() || !std::isfinite(parsed) || parsed < minimum || parsed > maximum)
    throw std::invalid_argument("numeric argument is outside its allowed range");
  return parsed;
}

Options arguments(int argc, char** argv) {
  Options o;
  for (int i = 1; i < argc; ++i) {
    const std::string key = argv[i];
    if (key == "--lowstate-only") { o.secondary = false; continue; }
    if (i + 1 == argc) throw std::invalid_argument("missing value for " + key);
    const std::string value = argv[++i];
    if (key == "--interface") o.interface = value;
    else if (key == "--output") o.output = value;
    else if (key == "--duration") o.duration = number(value, 1.0, 300.0);
    else if (key == "--timeout") o.timeout = number(value, 0.5, 30.0);
    else if (key == "--sample-hz") o.sample_hz = number(value, 1.0, 500.0);
    else if (key == "--max-mib") o.max_bytes = static_cast<std::size_t>(number(value, 1.0, 1024.0) * 1024.0 * 1024.0);
    else throw std::invalid_argument("unknown argument: " + key);
  }
  if (o.interface.empty() || if_nametoindex(o.interface.c_str()) == 0)
    throw std::invalid_argument("--interface must name an existing explicitly selected interface");
  if (o.output.empty() || !std::filesystem::path(o.output).is_absolute())
    throw std::invalid_argument("--output must be an absolute NEW file path");
  if (std::getenv("ROS_DISTRO") || std::getenv("AMENT_PREFIX_PATH") || std::getenv("COLCON_PREFIX_PATH"))
    throw std::invalid_argument("use a clean shell without a sourced ROS/colcon environment");
  // An inherited library override could load ROS's Cyclone DDS into this process.
  if (std::getenv("LD_LIBRARY_PATH") && std::strlen(std::getenv("LD_LIBRARY_PATH")) != 0)
    throw std::invalid_argument("unset LD_LIBRARY_PATH so the private SDK DDS libraries are used");
  return o;
}

std::string json_string(const std::string& text) {
  std::ostringstream out;
  out << '"';
  for (unsigned char c : text) {
    if (c == '"' || c == '\\') out << '\\' << c;
    else if (c < 0x20) out << "\\u" << std::hex << std::setw(4) << std::setfill('0') << static_cast<unsigned>(c) << std::dec;
    else out << c;
  }
  return out.str() + '"';
}

template<class Number> void json_number(std::ostream& out, Number value, std::uint64_t& nonfinite) {
  if constexpr (std::is_floating_point_v<Number>) {
    if (!std::isfinite(value)) { out << "null"; ++nonfinite; }
    else out << std::setprecision(std::numeric_limits<Number>::max_digits10) << value;
  } else out << +value;
}

template<class Array> void json_array(std::ostream& out, const Array& values, std::uint64_t& nonfinite) {
  out << '[';
  bool first = true;
  for (auto value : values) {
    if (!first) out << ',';
    first = false;
    json_number(out, value, nonfinite);
  }
  out << ']';
}

void json_imu(std::ostream& out, const ImuState& imu, std::uint64_t& nonfinite) {
  out << "{\"quaternion_raw\":"; json_array(out, imu.quaternion(), nonfinite);
  out << ",\"gyroscope_raw\":"; json_array(out, imu.gyroscope(), nonfinite);
  out << ",\"accelerometer_raw\":"; json_array(out, imu.accelerometer(), nonfinite);
  out << ",\"rpy_raw\":"; json_array(out, imu.rpy(), nonfinite);
  out << ",\"temperature_raw\":" << imu.temperature() << '}';
}

// Same polynomial/object-word algorithm as the official pinned H2 example.
std::uint32_t crc(const LowState& state) {
  static_assert(sizeof(LowState) % sizeof(std::uint32_t) == 0);
  const auto* bytes = reinterpret_cast<const unsigned char*>(&state);
  std::uint32_t result = 0xFFFFFFFF;
  for (std::size_t i = 0; i < sizeof(LowState) / 4 - 1; ++i) {
    std::uint32_t word;
    std::memcpy(&word, bytes + i * 4, sizeof(word));
    for (std::uint32_t bit = 0x80000000; bit != 0; bit >>= 1) {
      result = (result & 0x80000000) ? (result << 1) ^ 0x04c11db7 : result << 1;
      if (word & bit) result ^= 0x04c11db7;
    }
  }
  return result;
}

std::int64_t system_ns() {
  return std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::system_clock::now().time_since_epoch()).count();
}
std::int64_t steady_ns() {
  return std::chrono::duration_cast<std::chrono::nanoseconds>(Clock::now().time_since_epoch()).count();
}

template<class State> struct Snapshot {
  State state;
  std::uint64_t received = 0;
  std::int64_t host_system_ns = 0, host_steady_ns = 0;
  void update(const void* message) {
    state = *static_cast<const State*>(message);
    ++received;
    host_system_ns = system_ns();
    host_steady_ns = steady_ns();
  }
};

class Output {
 public:
  explicit Output(const std::string& path) {
    fd_ = ::open(path.c_str(), O_WRONLY | O_CREAT | O_EXCL | O_CLOEXEC, 0644);
    if (fd_ < 0) throw std::runtime_error("cannot create NEW output: " + std::string(std::strerror(errno)));
  }
  ~Output() { if (fd_ >= 0) ::close(fd_); }
  void line(const std::string& value) {
    const std::string data = value + '\n';
    std::size_t done = 0;
    while (done < data.size()) {
      const auto count = ::write(fd_, data.data() + done, data.size() - done);
      if (count < 0 && errno == EINTR) continue;
      if (count <= 0) throw std::runtime_error("output write failed: " + std::string(std::strerror(errno)));
      done += static_cast<std::size_t>(count);
    }
    bytes += data.size();
  }
  std::size_t bytes = 0;
 private:
  int fd_ = -1;
};

std::string prefix(const char* stream, std::uint64_t sequence, std::int64_t wall, std::int64_t steady) {
  return "{\"kind\":\"sample\",\"stream\":" + json_string(stream) + ",\"receipt_sequence\":" + std::to_string(sequence) +
         ",\"host_receipt_system_ns\":" + std::to_string(wall) + ",\"host_receipt_steady_ns\":" + std::to_string(steady);
}

int run(const Options& o) {
  Output output(o.output);  // O_EXCL is checked before opening any DDS participant.
  output.line("{\"kind\":\"metadata\",\"sdk_revision\":" + json_string(kRevision) +
    ",\"interface\":" + json_string(o.interface) + ",\"dds_domain\":0,\"duration_s\":" + std::to_string(o.duration) +
    ",\"sample_hz\":" + std::to_string(o.sample_hz) + ",\"require_secondary_imu\":" + (o.secondary ? "true" : "false") +
    ",\"semantics\":\"raw SDK fields; units, quaternion ordering, tick timebase and frame require target verification\","
    "\"sampling\":\"latest sample per interval; receipt_sequence gaps are intentional host downsampling, not a measured network loss rate\","
    "\"validation_scope\":\"CRC and finite-value checks cover logged sampled snapshots only; unlogged received states are not validated\","
    "\"h2_motors\":31,\"idl_motor_slots\":35}");

  std::mutex mutex;
  Snapshot<LowState> low;
  Snapshot<ImuState> secondary;
  unitree::robot::ChannelFactory::Instance()->Init(0, o.interface);
  unitree::robot::ChannelSubscriber<LowState> low_sub("rt/lowstate");
  unitree::robot::ChannelSubscriber<ImuState> secondary_sub("rt/secondary_imu");
  low_sub.InitChannel([&](const void* message) { std::lock_guard<std::mutex> lock(mutex); low.update(message); }, 1);
  if (o.secondary)
    secondary_sub.InitChannel([&](const void* message) { std::lock_guard<std::mutex> lock(mutex); secondary.update(message); }, 1);

  const auto start = Clock::now();
  std::uint64_t low_written = 0, secondary_written = 0, last_low = 0, last_secondary = 0, bad_crc = 0, nonfinite = 0;
  std::string reason = "complete";
  auto next = start;
  while (!stop_requested && std::chrono::duration<double>(Clock::now() - start).count() < o.duration) {
    Snapshot<LowState> current_low;
    Snapshot<ImuState> current_secondary;
    { std::lock_guard<std::mutex> lock(mutex); current_low = low; current_secondary = secondary; }
    const auto now_ns = steady_ns();
    const double elapsed = std::chrono::duration<double>(Clock::now() - start).count();
    auto stale = [&](auto& snapshot) { return snapshot.received == 0 ? elapsed > o.timeout : (now_ns - snapshot.host_steady_ns) * 1e-9 > o.timeout; };
    if (stale(current_low)) { reason = current_low.received ? "stale_lowstate" : "missing_lowstate"; break; }
    if (o.secondary && stale(current_secondary)) { reason = current_secondary.received ? "stale_secondary_imu" : "missing_secondary_imu"; break; }
    if (output.bytes + 16384 > o.max_bytes) { reason = "size_limit"; break; }
    if (current_low.received != last_low) {
      const auto& state = current_low.state;
      const bool valid_crc = crc(state) == state.crc();
      if (!valid_crc) ++bad_crc;
      std::ostringstream line;
      line << prefix("rt/lowstate", current_low.received, current_low.host_system_ns, current_low.host_steady_ns)
           << ",\"tick_raw\":" << state.tick() << ",\"mode_pr_raw\":" << +state.mode_pr()
           << ",\"mode_machine_raw\":" << +state.mode_machine() << ",\"version\":";
      json_array(line, state.version(), nonfinite);
      line << ",\"crc_raw\":" << state.crc() << ",\"crc_valid\":" << (valid_crc ? "true" : "false") << ",\"imu_raw\":";
      json_imu(line, state.imu_state(), nonfinite);
      line << ",\"motors_raw\":[";
      for (std::size_t i = 0; i < state.motor_state().size(); ++i) {
        const auto& motor = state.motor_state()[i];
        if (i) line << ',';
        line << "{\"index\":" << i << ",\"mode\":" << +motor.mode() << ",\"q\":"; json_number(line, motor.q(), nonfinite);
        line << ",\"dq\":"; json_number(line, motor.dq(), nonfinite);
        line << ",\"ddq\":"; json_number(line, motor.ddq(), nonfinite);
        line << ",\"tau_est\":"; json_number(line, motor.tau_est(), nonfinite);
        line << ",\"temperature\":"; json_array(line, motor.temperature(), nonfinite);
        line << ",\"vol\":"; json_number(line, motor.vol(), nonfinite);
        line << ",\"sensor\":"; json_array(line, motor.sensor(), nonfinite);
        line << ",\"motorstate\":" << motor.motorstate() << '}';
      }
      line << "],\"wireless_remote_raw\":"; json_array(line, state.wireless_remote(), nonfinite);
      line << '}'; output.line(line.str());
      last_low = current_low.received; ++low_written;
    }
    if (o.secondary && current_secondary.received != last_secondary) {
      std::ostringstream line;
      line << prefix("rt/secondary_imu", current_secondary.received, current_secondary.host_system_ns, current_secondary.host_steady_ns) << ",\"imu_raw\":";
      json_imu(line, current_secondary.state, nonfinite);
      line << '}'; output.line(line.str());
      last_secondary = current_secondary.received; ++secondary_written;
    }
    next += std::chrono::duration_cast<Clock::duration>(std::chrono::duration<double>(1.0 / o.sample_hz));
    std::this_thread::sleep_until(next);
  }
  low_sub.CloseChannel();
  if (o.secondary) secondary_sub.CloseChannel();
  if (stop_requested) reason = "signal";
  else if (!low_written) reason = "missing_lowstate";
  else if (o.secondary && !secondary_written) reason = "missing_secondary_imu";
  else if (reason == "complete" && bad_crc) reason = "invalid_lowstate_crc";
  else if (reason == "complete" && nonfinite) reason = "non_finite_state";
  output.line("{\"kind\":\"summary\",\"reason\":" + json_string(reason) + ",\"passed\":" + (reason == "complete" ? "true" : "false") +
    ",\"lowstate_received\":" + std::to_string(low.received) + ",\"lowstate_written\":" + std::to_string(low_written) +
    ",\"secondary_imu_received\":" + std::to_string(secondary.received) + ",\"secondary_imu_written\":" + std::to_string(secondary_written) +
    ",\"bad_crc_samples\":" + std::to_string(bad_crc) + ",\"nonfinite_fields\":" + std::to_string(nonfinite) + "}");
  std::cout << "Passive recording: " << reason << "; lowstate=" << low_written << "; secondary_imu=" << secondary_written << '\n';
  return reason == "complete" ? 0 : 3;
}
}  // namespace

int main(int argc, char** argv) {
  if (argc == 2 && std::string(argv[1]) == "--help") { usage(); return 0; }
  if (argc == 2 && std::string(argv[1]) == "--version") { std::cout << "h2_state_probe sdk=" << kRevision << '\n'; return 0; }
  std::signal(SIGINT, on_signal);
  std::signal(SIGTERM, on_signal);
  try { return run(arguments(argc, argv)); }
  catch (const std::exception& error) { std::cerr << "Passive state probe: " << error.what() << '\n'; return 2; }
}
