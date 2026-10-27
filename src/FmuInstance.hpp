#pragma once

#include <string>
#include <iostream>
#include <unordered_map>
#include <stdexcept>
#include <vector>
#include <map>
#include <nlohmann/json.hpp>

extern "C" {
#ifdef _WIN32
#define NOMINMAX
#ifndef WIN32_LEAN_AND_MEAN
#define WIN32_LEAN_AND_MEAN
#endif
#include <winsock2.h>
#endif
#include <fmi4c.h>
}

using json = nlohmann::json;

enum class FmuType{
    Unknown,
    ModelExchange,
    CoSimulation
};

class FmuWrapper {
public:
    FmuWrapper() {}

    /// Constructor loads the FMU and sets up the instance
    /// @param fmu_path Path to the .fmu file
    /// @param instance_name Name for the FMU instance
    /// @param start_time Initial simulation time (default: 0.0)
    /// @throws std::runtime_error if FMU loading or instantiation fails
    FmuWrapper(const std::string& fmu_path,
               const std::string& instance_name,
               double start_time = 0.0);

    /// Destructor cleans up FMU resources
    ~FmuWrapper();

    // Prevent copying
    FmuWrapper(const FmuWrapper&) = delete;
    FmuWrapper& operator=(const FmuWrapper&) = delete;

    // Allow moving
    FmuWrapper(FmuWrapper&& other) noexcept;
    FmuWrapper& operator=(FmuWrapper&& other) noexcept;

    std::string model_name();

    void reset();

    /// Execute one simulation step
    /// @param dt Step size (delta time)
    /// @throws std::runtime_error if step execution fails
    void do_step(double dt);

    /// Set a real-valued variable by name
    /// @param name Variable name
    /// @param value Value to set
    /// @throws std::runtime_error if variable not found or set fails
    void set_real(const std::string& name, double value);
    void set_real(const std::string& name, const std::vector<double>& values);
    void set_bool(const std::string& name, bool value);

    /// Get a real-valued variable by name
    /// @param name Variable name
    /// @return Variable value
    /// @throws std::runtime_error if variable not found or get fails
    double get_real(const std::string& name) const;
    void get_real(const std::string& name, std::vector<double>& values) const;
    bool get_bool(const std::string& name) const;

    /// Get current simulation time
    /// @return Current time
    double get_time() const { return _current_time; }

    size_t get_outputs(std::map<std::string, double> &outs) const;
    size_t get_inputs(std::map<std::string, double> &ins) const;
    size_t get_params(std::map<std::string, double> &params) const;
    size_t get_indep(std::map<std::string, double> &indep) const;
    
    json value(std::string const &n);
    void get_status(json &status);

    void list_variables(std::ostream &s = std::cout);

    std::vector<std::string> get_param_names() const;
    std::vector<std::string> get_input_names() const;
    std::vector<std::string> get_output_names() const;
    std::vector<std::string> get_indep_names() const;
    std::vector<std::string> get_binary_dependencies() const;

    int get_type() const { return static_cast<int>(_type); }
    bool get_fixed_step() const { return _fixed_step; }
    double get_step_size() const { return _step_size; }

    /// Linearize the Model Exchange dynamics dx/dt = f(x, u) at the current
    /// state and inputs: A = df/dx (nx x nx), B = df/du (nx x nu).
    /// Uses fmi3GetDirectionalDerivative when the FMU declares
    /// providesDirectionalDerivatives="true", otherwise central finite
    /// differences on fmi3GetContinuousStateDerivatives. The FMU states and
    /// inputs are restored before returning.
    /// Fills jac with: method, states, inputs, x, xdot, A, B (row-major,
    /// row i = derivative of state i).
    /// @throws std::runtime_error if the FMU is not Model Exchange
    void get_jacobian(json &jac);
    bool provides_directional_derivatives() const { return _provides_dir_der; }

    /// Continuous states (Model Exchange only), in fmi3GetContinuousStates
    /// order, i.e. the same order as the rows of the Jacobian.
    /// @throws std::runtime_error if the FMU is not Model Exchange
    size_t get_num_states();
    std::vector<std::string> get_state_names();
    void get_continuous_states(std::vector<double> &x);
    void set_continuous_states(const std::vector<double> &x);
    /// dx/dt = f(x, u) at the current states and inputs
    void get_continuous_state_derivatives(std::vector<double> &xdot);

    struct SolverParams {
        double _rel_tol = 1e-6;
        double _abs_tol = 1e-8;
        double _hmin = 1e-12;
    };

    SolverParams _solver_params;

private:
    fmuHandle* _fmu = nullptr;
    fmi3InstanceHandle* _instance = nullptr;
    std::string _fmu_path = "", _instance_name = "";
    bool _initialized = false;
    std::vector<std::string> _out_vars;
    std::vector<std::string> _in_vars;
    std::vector<std::string> _param_vars;
    std::vector<std::string> _indep_vars;

    double _start_time, _current_time;
    std::unordered_map<std::string, fmi3ValueReference> _var_cache;
    mutable std::unordered_map<std::string, size_t> _real_array_len_cache;
    mutable std::unordered_map<std::string, std::string> _var_dimensions_cache;
    mutable std::string _model_description_xml;

    // Static enum-to-string translation maps
    static const std::unordered_map<int, std::string> _initial_kind_map;
    static const std::unordered_map<fmi3Causality, std::string> _causality_map;
    static const std::unordered_map<fmi3DataType, std::string> _data_type_map;

    // FMU type variable
    FmuType _type = FmuType::Unknown;
    bool _fixed_step = false;
    double _step_size = 0.0;

    // Jacobian (Model Exchange only)
    struct JacInput {
        std::string name;
        fmi3ValueReference vr;
        size_t len;
    };
    bool _provides_dir_der = false;
    std::vector<std::string> _state_names;        // from modelDescription.xml
    std::vector<fmi3ValueReference> _state_vrs;   // in fmi3GetContinuousStates order
    std::vector<fmi3ValueReference> _der_vrs;     // matching state derivatives
    std::vector<JacInput> _jac_inputs;            // Float64 inputs, flattened
    bool _jac_inputs_ready = false;

    /// Resolve variable name to value reference
    fmi3ValueReference resolve_var_ref(const std::string& name) const;
    size_t resolve_real_array_length(const std::string& name) const;
    std::string resolve_var_dimensions(const std::string& name) const;
    
    /// Check FMI status and throw if error
    void check_status(fmi3Status status, const std::string& context) const;
    
    /// Helper methods to translate enums to strings
    static std::string get_initial_kind_string(fmi3Initial kind);
    static std::string get_causality_string(fmi3Causality causality);
    static std::string get_data_type_string(fmi3DataType dataType);
    
    /// Handle discrete state updates and events
    void handle_events();

    /// fmi3GetContinuousStateDerivatives, made safe for FMUs (e.g. Simulink)
    /// that refresh their outputs lazily on get calls only
    void get_state_derivatives(fmi3Float64 *derivatives, size_t nx,
                               const std::string &context);
    std::vector<fmi3Float64> _deriv_scratch;
};
