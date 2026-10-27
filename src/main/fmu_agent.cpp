//   _____ __  __ ___      _                    _
//  |  ___|  \/  |_ _|    / \   __ _  ___ _ __ | |_
//  | |_  | |\/| || |    / _ \ / _` |/ _ \ '_ \| __|
//  |  _| | |  | || |   / ___ \ (_| |  __/ | | | |_
//  |_|   |_|  |_|___| /_/   \_\__, |\___|_| |_|\__|
//                             |___/

#include "../FmuInstance.hpp"
#include "../goback.hpp"
#include <agent.hpp>
#include <chrono>
#include <cxxopts.hpp>
#include <filesystem>
#include <mads.hpp>
#include <rang.hpp>
#include <algorithm>
#include <array>
#ifndef __linux__
#include <format>
#endif 

using namespace std;
using namespace chrono_literals;
using namespace cxxopts;
using json = nlohmann::json;
using namespace Mads;

bool inspect_fmu(filesystem::path const &path) {
  if (!filesystem::exists(path)) {
    cout << fg::red << "No file at " << path << fg::reset << endl;
    return false;
  }
  auto fmu = FmuWrapper(path.string(), "plant");
  auto model_name = fmu.model_name();
  
  cout << style::bold 
       << "Inspecting model at:    " << fg::green << path.string() << fg::reset
       << "\nModel name (from .fmu): " << fg::green << model_name << fg::reset
       << "\nAgent name (default):   " << fg::green << "fmu_" << model_name
       << fg::reset << style::reset << endl;
  fmu.list_variables(cout);
  cout << endl
       << style::bold << "INI settings section (defaults):" << endl
       << style::reset << fg::green
       << "[fmu_" << model_name << "]"
       << "\nperiod = 100"
       << "\npub_topic = \"fmu_" <<  model_name << "\""
       << "\nsub_topic = [\"" <<  model_name << "_control\"]"
       << "\nrelative_tol = 1e-4"
       << "\nabsolute_tol = 1e-5"
       << "\nhmin_tol = 1e-10" 
       << "\njacobian = false       # Model Exchange only"
       << "\njacobian_every = 1     # publish every N steps"
       << "\nclosed_loop = false    # Model Exchange only, implies trigger_mode"
       << "\n[fmu_" << model_name << ".parameters]"
       << "\n# default values from FMU";
  for (const auto &param : fmu.get_param_names()) {
    cout << "\n" << param << " = " << fmu.get_real(param);
  }
  cout << fg::reset << endl;

  cout << style::bold << "\nDependencies:" << style::reset << endl;
  auto deps = fmu.get_binary_dependencies();
  for (auto &s : deps) {
    cout << "  " << s << endl;
  }
  return true;
}

int main(int argc, char *const *argv) {
  // Mads-related
  string settings_uri = SETTINGS_URI;
  bool crypto = false;
  bool trigger_mode = false;
  filesystem::path key_dir(Mads::exec_dir() + "/../etc");
  string client_key_name = "client";
  string server_key_name = "broker";
  Mads::auth_verbose auth_verbose = auth_verbose::off;
  string agent_name = argv[0], agent_id;
  chrono::milliseconds period{100};
  // FMI-related
  filesystem::path fmu_path;
  FmuWrapper fmu;
  string plant_name = "plant";
  double relative_tol = 1e-5;
  double absolute_tol = 1e-7;
  double hmin = 1e-12;
  bool jacobian = false;
  int jacobian_every = 1; // just if it is needed to jump steps
  unsigned long step_count = 0;
  bool closed_loop = false;

  // Parse command line options ================================================
  Options options(argv[0]);
  // if needed, add here further CLI options
  // clang-format off
  options.add_options()
    ("f, fmu", "FMU file to load", value<filesystem::path>())
    ("p,period", "Sampling period (default 100 ms)", value<size_t>())
    ("n,name", "Agent name (default to fmu_<model name>)", value<string>())
    ("t,trigger", "Enable trigger mode")
    ("j,jacobian", "Publish the Jacobian A=df/dx, B=df/du (Model Exchange only)")
    ("c,closed-loop", "Accept state corrections from MADS (Model Exchange only, implies trigger mode)")
    ("i,agent-id", "Agent ID to be added to JSON frames", value<string>())
    ("inspect", "Inspect the FMU and exit");
  // clang-format on
  options.parse_positional({"fmu"});
  options.positional_help("<module.fmu>");
  SETUP_OPTIONS(options, Agent);

  if (options_parsed.count("fmu") != 0) {
    fmu_path = options_parsed["fmu"].as<filesystem::path>();
    if (fmu_path.extension() != ".fmu") {
      cerr << fg::red << "FMU file must have .fmu extension" << fg::reset
           << endl;
      exit(EXIT_FAILURE);
    }
    if (!filesystem::exists(fmu_path)) {
      cerr << fg::red << "No FMU file at path " << fmu_path.string()
           << fg::reset << endl;
      exit(EXIT_FAILURE);
    }
  } else {
    cerr << fg::red << "Missing mandatory FMU path" << fg::reset << endl;
    exit(EXIT_FAILURE);
  }
  if (options_parsed.count("inspect") != 0) {
    if (inspect_fmu(fmu_path))
      exit(EXIT_SUCCESS);
    else
      exit(EXIT_FAILURE);
  }
  
  // Load FMU
  FmuWrapper plant(fmu_path.string(), "plant");
  agent_name = string("fmu_") + plant.model_name();
  if (options_parsed.count("name") != 0) {
    agent_name = options_parsed["name"].as<string>();
  }

  // Create Agent
  Agent agent(agent_name, settings_uri);
  if (options_parsed.count("agent-id")) {
    agent.set_agent_id(options_parsed["agent-id"].as<string>());
  }
  if (crypto) {
    agent.set_key_dir(key_dir);
    agent.client_key_name = client_key_name;
    agent.server_key_name = server_key_name;
    agent.auth_verbose = auth_verbose;
  }
  try {
    agent.init(crypto);
  } catch (const std::exception &e) {
    std::cout << fg::red << "Error initializing agent: " << e.what()
              << fg::reset << endl;
    exit(EXIT_FAILURE);
  }

  // Settings
  json settings = agent.get_settings();
  period = chrono::milliseconds(settings.value("period", 100));
  if (options_parsed.count("period") != 0) {
    period = chrono::milliseconds(options_parsed["period"].as<size_t>());
  }
  plant_name = settings.value("plant_name", plant_name);
  relative_tol = settings.value("relative_tol", relative_tol);
  absolute_tol = settings.value("absolute_tol", absolute_tol);
  hmin = settings.value("hmin", hmin);

  trigger_mode = settings.value("trigger_mode", trigger_mode);
  if (options_parsed.count("trigger") != 0) {
    trigger_mode = true;
  }

  jacobian = settings.value("jacobian", jacobian);
  if (options_parsed.count("jacobian") != 0) {
    jacobian = true;
  }
  jacobian_every = max(1, settings.value("jacobian_every", jacobian_every));
  if (jacobian && plant.get_type() != static_cast<int>(FmuType::ModelExchange)) {
    cout << fg::yellow << "Jacobian is available for Model Exchange FMUs only: disabled"
         << fg::reset << endl;
    jacobian = false;
  }
  closed_loop = settings.value("closed_loop", closed_loop);
  if (options_parsed.count("closed-loop") != 0) {
    closed_loop = true;
  }
  if (closed_loop && plant.get_type() != static_cast<int>(FmuType::ModelExchange)) {
    cout << fg::yellow << "Closed loop is available for Model Exchange FMUs only: disabled"
         << fg::reset << endl;
    closed_loop = false;
  }
  if (closed_loop && !trigger_mode) {
    cout << fg::yellow << "Closed loop needs trigger mode: trigger mode enabled"
         << fg::reset << endl;
    trigger_mode = true;
  }

  // if jacobian is enabled
  auto add_jacobian = [&](json &st) {
    if (jacobian && (step_count++ % jacobian_every == 0)) {
      json jac;
      plant.get_jacobian(jac);
      st["jacobian"] = jac;
    }
  };

  // Closed loop (Model Exchange only) ----------------------------------------
  // State names in fmi3GetContinuousStates order (= rows of the Jacobian)
  const vector<string> state_names =
      closed_loop ? plant.get_state_names() : vector<string>{};

  // Apply a state update received from MADS:
  //   "fmu_state"            : absolute values
  //   "fmu_state_correction" : added to the current values
  // each either an array with all the states (Jacobian order) or an object
  // {state name: value} with a subset of them. All or nothing: if anything is
  // wrong, no state is changed. Returns "none", "absolute", "additive",
  // "absolute+additive" or "rejected: <reason>".
  auto apply_state = [&](json const &in) -> string {
    const bool has_abs = in.contains("fmu_state");
    const bool has_corr = in.contains("fmu_state_correction");
    if (!has_abs && !has_corr) return "none";
    vector<double> x;
    plant.get_continuous_states(x);
    auto update = [&](json const &j, bool additive) -> string {
      if (j.is_array()) {
        if (j.size() != x.size()) {
          return "expected " + to_string(x.size()) + " values, got " +
                 to_string(j.size());
        }
        for (size_t i = 0; i < x.size(); ++i) {
          if (!j[i].is_number()) return "element " + to_string(i) + " is not a number";
          x[i] = additive ? x[i] + j[i].get<double>() : j[i].get<double>();
        }
        return "";
      }
      if (j.is_object()) {
        for (auto const &[k, v] : j.items()) {
          auto it = find(state_names.begin(), state_names.end(), k);
          if (it == state_names.end()) return "unknown state '" + k + "'";
          if (!v.is_number()) return "state '" + k + "' is not a number";
          const size_t i = it - state_names.begin();
          x[i] = additive ? x[i] + v.get<double>() : v.get<double>();
        }
        return "";
      }
      return "must be an array or an object";
    };
    string err;
    if (has_abs && !(err = update(in["fmu_state"], false)).empty())
      return "rejected: fmu_state " + err;
    if (has_corr && !(err = update(in["fmu_state_correction"], true)).empty())
      return "rejected: fmu_state_correction " + err;
    plant.set_continuous_states(x);
    return has_abs && has_corr ? "absolute+additive" : (has_abs ? "absolute" : "additive");
  };

  // Add the (predicted) state to the status, for the filter to correct
  auto add_state = [&](json &st, string const &correction) {
    vector<double> x, xdot;
    plant.get_continuous_states(x);
    plant.get_continuous_state_derivatives(xdot);
    st["state"] = {{"names", state_names}, {"x", x}, {"xdot", xdot},
                   {"correction", correction}};
  };

  // Connection
  agent.enable_remote_control();
  agent.connect();
  agent.register_event(event_type::startup);
  agent.info();
  if (trigger_mode) {
    period = 0ms; // not used in trigger mode, but set to 0 for clarity in output
    cout << "  period:           " << style::bold << fg::yellow 
         << "Trigger mode enabled" << fg::reset
         << style::reset << endl;
  } else {
    cout << "  period:           " << style::bold << period.count() << " ms" 
         << style::reset << endl;
  }
  cout << "  FMU file path:    " << style::bold << fmu_path << style::reset
       << endl
       << "  FMU type:          " << style::bold << ((plant.get_type() == 1) ? "Model Exchange" : "Co-Simulation") << style::reset << endl
       << "  relative tol:     " << style::bold << relative_tol << style::reset
       << endl
       << "  absolutre tol:    " << style::bold << absolute_tol << style::reset
       << endl
       << "  hmin:             " << style::bold << hmin << style::reset
       << endl
       << "  jacobian:        " << style::bold
       << (jacobian ? (plant.provides_directional_derivatives()
                           ? "directional derivatives"
                           : "central differences")
                    : "off")
       << (jacobian ? ", every " + to_string(jacobian_every) + " step(s)" : "")
       << style::reset << endl
       << "  closed loop:     " << style::bold
       << (closed_loop ? "on, " + to_string(state_names.size()) + " states" : "off")
       << style::reset << endl;
  plant._solver_params._rel_tol = 1e-5;
  plant._solver_params._abs_tol = 1e-7;
  plant._solver_params._hmin = 1e-12;

  if (settings.contains("parameters")) {
    cout << style::bold << "Overriding default FMU parameters (" 
         << settings["parameters"].size() << ") from settings:" 
         << style::reset << endl;
    int i = 0;
    for (auto const &[param, value] : settings["parameters"].items()) {
      cout << "  " << param << ": " << style::bold << value 
           << style::reset;
      if (!value.is_number()) {
        cout << fg::red << "  NOT A NUMBER, SKIPPING" << fg::reset;
      } else {
        plant.set_real(param, value.get<double>());
      }
      cout << endl;
      i++;
    }
    cout << "  " << i << " parameters overridden" << endl;
  }
  cout << endl << endl << endl; // to compensate for next goback(3)



  auto last_timestep = chrono::steady_clock::now();
  chrono::steady_clock::time_point now;
  double dt = 0, t = 0, t_in = 0, t_msg = 0;
  int dtus = 0, step_s = 0;
  static double t_buffer = 0.0;
  json status;
  array<string, 3> console_out;

  // Process a received JSON message: update plant inputs and console_out.
  // early_exit=true (free-running): returns on reset or missing fmu_input,
  //   skipping set_real so integration proceeds with unchanged inputs.
  // early_exit=false (trigger): continues past reset and applies fmu_input.
  auto process_input = [&](json const &in, bool early_exit = false) {
    if (in.value("fmu_reset", false)) {
      plant.reset();
      t_msg = t;
      t = 0;
      console_out[0] = to_string(t_msg) + " s, Reset received";
      if (early_exit) return;
    }
    if (!in["fmu_input"].is_object()) {
      t_msg = t;
      console_out[0] = to_string(t_msg) + " s, Missing fmu_input";
      if (early_exit) return;
    }
    t_in = t;
    console_out[1] = to_string(t_in) + " s, " + in["fmu_input"].dump();
    for (auto const &[k, v] : in["fmu_input"].items()) {
      if (v.is_array()) {
        plant.set_real(k, v.get<std::vector<double>>());
      } else if (v.is_number()) {
        plant.set_real(k, v.get<double>());
      } else {
        console_out[1] = " (skipped, not a number or array)";
      }
    }
  };

  if (trigger_mode && closed_loop) {
    // Closed loop: on each message FIRST apply reset / inputs / state update
    // (the correction refers to the state published with the previous
    // message), THEN integrate by "fmu_dt" seconds (wall-clock time since the
    // previous message if absent; 0 = apply and publish without stepping) and
    // publish the predicted state for the next correction.
    agent.loop([&]() -> chrono::milliseconds {
      if (agent.receive(false) == message_type::json && agent.last_topic() != "control") {
        now = chrono::steady_clock::now();
        auto msg = agent.last_message();
        auto in = json::parse(get<1>(msg));
        if (in.contains("fmu_reset") || in.contains("fmu_input")) {
          process_input(in, true); // a reset skips the inputs of its message
        }
        string correction;
        try {
          correction = apply_state(in);
        } catch (const std::exception &e) {
          correction = string("rejected: ") + e.what();
        }
        if (correction.rfind("rejected", 0) == 0) {
          console_out[0] = to_string(t) + " s, State " + correction;
        }
        if (in.contains("fmu_dt") && in["fmu_dt"].is_number()) {
          dt = in["fmu_dt"].get<double>();
        } else {
          dt = chrono::duration_cast<chrono::microseconds>(now - last_timestep)
                   .count() / 1e6;
        }
        last_timestep = now;
        if (dt > 0) {
          t += dt;
          plant.do_step(dt);
        }
        console_out[2] = to_string(t) + " s";
        plant.get_status(status);
        add_state(status, correction);
        add_jacobian(status);
        auto tet = chrono::steady_clock::now() - now;
        status["TET"] =
          chrono::duration_cast<chrono::microseconds>(tet).count();
        agent.publish(status);
      }

      cout << goback(4) << fg::yellow << "Last message: " << console_out[0]
        << fg::reset << endl
        << "Received: " << console_out[1] << endl
        << "Status update after: " << console_out[2] << endl
        << "Simulation dt: " << dt << "s" << endl;

      return 0ms;
    });
  } else if (trigger_mode) {
    agent.loop([&]() -> chrono::milliseconds {
      if (agent.receive(false) == message_type::json && agent.last_topic() != "control") {
        now = chrono::steady_clock::now();

        dt = chrono::duration_cast<chrono::microseconds>(now - last_timestep)
                .count();
        last_timestep = now;

        if(plant.get_fixed_step()){

          t_buffer += dt;
          step_s = plant.get_step_size() * 1e6; // us
          
          int num_steps = t_buffer / step_s;
          if(num_steps > 0){
            dt = num_steps * step_s; // force dt
            t_buffer -= dt; // t_buffer will contain the residual
          } else{
            console_out[2] = "Skipped step: " + to_string(t_buffer) + "s";
            cout << goback(3) << fg::yellow << "Last message: " << console_out[0]
                 << fg::reset << endl
                 << "Received: " << console_out[1] << endl
                 << "Status update after: " << console_out[2] << endl
                 << "buffer: " << t_buffer << " dt: " << dt << endl;
            return 0ms;
          }
        }

        dt = dt / 1e6;
        t += dt;
        console_out[2] = to_string(t) + " s";
        plant.do_step(dt);
        plant.get_status(status);
        add_jacobian(status);
        auto tet = chrono::steady_clock::now() - now;
        status["TET"] =
          chrono::duration_cast<chrono::microseconds>(tet).count();
        agent.publish(status);
        auto msg = agent.last_message();
        auto in = json::parse(get<1>(msg));
        process_input(in);
      }

      cout << goback(4) << fg::yellow << "Last message: " << console_out[0]
        << fg::reset << endl
        << "Received: " << console_out[1] << endl
        << "Status update after: " << console_out[2] << endl
        << "Simulation dt: " << dt << "s" << endl;
      
      return 0ms;
    });
  } else {
    agent.loop([&]() -> chrono::milliseconds {
      // timing
      now = chrono::steady_clock::now();
      if(plant.get_fixed_step()){
        dt = static_cast<double>(period.count()) / 1e3;
        dtus = dt * 1e6;
        step_s = plant.get_step_size() * 1e6; // us
        if(dtus % step_s != 0){ // comparison in microseconds

          throw std::runtime_error("The agent period must be a multiple of the model's fixed step");
        }

      } else{
        dt = chrono::duration_cast<chrono::microseconds>(now - last_timestep)
                      .count() / 1e6;

          last_timestep = now;
      }
      t += dt;
      // input
      if (agent.receive(true) == message_type::json &&
          agent.last_topic() != "control") {
        auto msg = agent.last_message();
        auto in = json::parse(get<1>(msg));
        process_input(in, true);
      }
      // Integrate
      plant.do_step(dt);
      plant.get_status(status);
      add_jacobian(status);
      console_out[2] = to_string(t) + " s";
      auto tet = chrono::steady_clock::now() - now;
      status["TET"] =
          chrono::duration_cast<chrono::microseconds>(tet).count();
      // output
      agent.publish(status);
      cout << goback(3) << fg::yellow << "Last message: " << console_out[0]
            << fg::reset << endl
            << "Received: " << console_out[1] << endl
            << "Status update after: " << console_out[2] << endl;
      return 0ms;
    }, period);
  }
  cout << endl << fg::green << "FMU agent stopped" << fg::reset << endl;
  agent.register_event(event_type::shutdown);
  agent.disconnect();
  if (agent.restart()) {
    auto cmd = string(MADS_PREFIX) + argv[0];
    cout << "Restarting " << cmd << "..." << endl;
    execvp(cmd.c_str(), argv);
  }
  return 0;
}
