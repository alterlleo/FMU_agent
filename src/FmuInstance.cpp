#include "FmuInstance.hpp"
#include <rang.hpp>
#include <algorithm>
#include <array>
#include <chrono>
#include <cctype>
#include <cmath>
#include <cstdio>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <limits>
#include <sstream>
#include <stdexcept>
#include <vector>

using namespace std;
using namespace rang;
namespace fs = std::filesystem;

namespace {

std::string trim_copy(const std::string &s) {
  const auto begin = s.find_first_not_of(" \t\r\n");
  if (begin == std::string::npos) {
    return "";
  }
  const auto end = s.find_last_not_of(" \t\r\n");
  return s.substr(begin, end - begin + 1);
}

std::string shell_quote_posix(const std::string &s) {
  std::string out = "'";
  for (char c : s) {
    if (c == '\'') {
      out += "'\"'\"'";
    } else {
      out += c;
    }
  }
  out += "'";
  return out;
}

std::string read_text_file(const fs::path &path) {
  std::ifstream in(path);
  if (!in) {
    throw std::runtime_error("Unable to open file: " + path.string());
  }
  std::ostringstream ss;
  ss << in.rdbuf();
  return ss.str();
}

std::string run_command_capture(const std::string &command);

bool extract_xml_attribute(const std::string &tag, const std::string &attr,
                           std::string &value) {
  const std::string needle = attr + "=";
  const std::size_t key_pos = tag.find(needle);
  if (key_pos == std::string::npos) {
    return false;
  }
  const std::size_t quote_pos = key_pos + needle.size();
  if (quote_pos >= tag.size() ||
      (tag[quote_pos] != '"' && tag[quote_pos] != '\'')) {
    return false;
  }
  const char quote = tag[quote_pos];
  const std::size_t value_begin = quote_pos + 1;
  const std::size_t value_end = tag.find(quote, value_begin);
  if (value_end == std::string::npos) {
    return false;
  }
  value = tag.substr(value_begin, value_end - value_begin);
  return true;
}

struct RealArrayMetadata {
  bool _is_float64 = false;
  bool _is_array = false;
  std::size_t _rank = 0;
  std::size_t _length = 0;
};

RealArrayMetadata parse_real_array_metadata(const std::string &xml,
                                            const std::string &name) {
  constexpr const char *tags[] = {"Float64",      "Float32", "Int8",   "UInt8",
                                  "Int16",        "UInt16",  "Int32",  "UInt32",
                                  "Int64",        "UInt64",  "Boolean", "String",
                                  "Binary",       "Enumeration", "Clock"};

  for (const auto *tag : tags) {
    const std::string marker = std::string("<") + tag;
    std::size_t pos = 0;
    while ((pos = xml.find(marker, pos)) != std::string::npos) {
      const std::size_t tag_end = xml.find('>', pos);
      if (tag_end == std::string::npos) {
        break;
      }
      const std::string header = xml.substr(pos, tag_end - pos + 1);
      std::string var_name;
      if (!extract_xml_attribute(header, "name", var_name) || var_name != name) {
        pos = tag_end + 1;
        continue;
      }

      RealArrayMetadata meta;
      meta._is_float64 = std::string(tag) == "Float64";
      if (header.size() >= 2 && header[header.size() - 2] == '/') {
        return meta;
      }

      const std::string end_tag = std::string("</") + tag + ">";
      const std::size_t close_pos = xml.find(end_tag, tag_end + 1);
      if (close_pos == std::string::npos) {
        throw std::runtime_error("Malformed modelDescription.xml for variable " +
                                 name);
      }

      const std::string body = xml.substr(tag_end + 1, close_pos - tag_end - 1);
      std::size_t dim_pos = 0;
      while ((dim_pos = body.find("<Dimension", dim_pos)) != std::string::npos) {
        const std::size_t dim_end = body.find('>', dim_pos);
        if (dim_end == std::string::npos) {
          throw std::runtime_error(
              "Malformed Dimension tag in modelDescription.xml for variable " +
              name);
        }
        const std::string dim_tag =
            body.substr(dim_pos, dim_end - dim_pos + 1);
        meta._is_array = true;
        meta._rank++;

        std::string start_value;
        if (!extract_xml_attribute(dim_tag, "start", start_value)) {
          throw std::runtime_error(
              "Variable " + name +
              " has non-literal or missing array dimension; unsupported");
        }

        std::size_t consumed = 0;
        const unsigned long long len =
            std::stoull(start_value, &consumed, 10);
        if (consumed != start_value.size() || len == 0ULL) {
          throw std::runtime_error(
              "Variable " + name +
              " has invalid array dimension in modelDescription.xml");
        }
        if (len > std::numeric_limits<std::size_t>::max()) {
          throw std::runtime_error(
              "Variable " + name +
              " array dimension does not fit in host size_t");
        }
        if (meta._rank == 1) {
          meta._length = static_cast<std::size_t>(len);
        }
        dim_pos = dim_end + 1;
      }
      return meta;
    }
  }

  throw std::runtime_error("Variable not found in modelDescription.xml: " + name);
}

std::string read_model_description_xml(const std::string &fmu_path) {
  const fs::path path(fmu_path);
  if (fs::is_directory(path)) {
    return read_text_file(path / "modelDescription.xml");
  }

#if defined(_WIN32)
  const auto now =
      std::chrono::high_resolution_clock::now().time_since_epoch().count();
  const fs::path temp_dir =
      fs::temp_directory_path() / ("fmu_xml_" + std::to_string(now));
  try {
    fs::create_directories(temp_dir);
    const std::string extract_cmd = "tar -xf \"" + path.string() + "\" -C \"" +
                                    temp_dir.string() + "\" modelDescription.xml";
    run_command_capture(extract_cmd);
    const std::string xml = read_text_file(temp_dir / "modelDescription.xml");
    std::error_code ec;
    fs::remove_all(temp_dir, ec);
    return xml;
  } catch (...) {
    std::error_code ec;
    fs::remove_all(temp_dir, ec);
    throw;
  }
#else
  const std::string quoted_fmu = shell_quote_posix(path.string());
  return run_command_capture("unzip -p " + quoted_fmu + " modelDescription.xml");
#endif
}

std::string parse_variable_dimensions_string(const std::string &xml,
                                            const std::string &name) {
  constexpr const char *tags[] = {"Float64",      "Float32", "Int8",   "UInt8",
                                  "Int16",        "UInt16",  "Int32",  "UInt32",
                                  "Int64",        "UInt64",  "Boolean", "String",
                                  "Binary",       "Enumeration", "Clock"};

  for (const auto *tag : tags) {
    const std::string marker = std::string("<") + tag;
    std::size_t pos = 0;
    while ((pos = xml.find(marker, pos)) != std::string::npos) {
      const std::size_t tag_end = xml.find('>', pos);
      if (tag_end == std::string::npos) {
        break;
      }
      const std::string header = xml.substr(pos, tag_end - pos + 1);
      std::string var_name;
      if (!extract_xml_attribute(header, "name", var_name) || var_name != name) {
        pos = tag_end + 1;
        continue;
      }
      if (header.size() >= 2 && header[header.size() - 2] == '/') {
        return "";
      }

      const std::string end_tag = std::string("</") + tag + ">";
      const std::size_t close_pos = xml.find(end_tag, tag_end + 1);
      if (close_pos == std::string::npos) {
        throw std::runtime_error("Malformed modelDescription.xml for variable " +
                                 name);
      }

      const std::string body = xml.substr(tag_end + 1, close_pos - tag_end - 1);
      std::vector<std::string> dims;
      std::size_t dim_pos = 0;
      while ((dim_pos = body.find("<Dimension", dim_pos)) != std::string::npos) {
        const std::size_t dim_end = body.find('>', dim_pos);
        if (dim_end == std::string::npos) {
          throw std::runtime_error(
              "Malformed Dimension tag in modelDescription.xml for variable " +
              name);
        }
        const std::string dim_tag = body.substr(dim_pos, dim_end - dim_pos + 1);
        std::string dim_value;
        if (!extract_xml_attribute(dim_tag, "start", dim_value)) {
          dim_value = "?";
        }
        dims.push_back(dim_value);
        dim_pos = dim_end + 1;
      }
      if (dims.empty()) {
        return "";
      }
      std::ostringstream ss;
      ss << "[";
      for (std::size_t i = 0; i < dims.size(); ++i) {
        if (i > 0) {
          ss << "x";
        }
        ss << dims[i];
      }
      ss << "]";
      return ss.str();
    }
  }
  return "";
}

std::string run_command_capture(const std::string &command) {
#ifdef _WIN32
  FILE *pipe = _popen(command.c_str(), "r");
#else
  FILE *pipe = popen(command.c_str(), "r");
#endif
  if (!pipe) {
    throw std::runtime_error("Failed to execute command: " + command);
  }

  std::array<char, 4096> buffer{};
  std::string output;
  while (fgets(buffer.data(), static_cast<int>(buffer.size()), pipe) != nullptr) {
    output += buffer.data();
  }

#ifdef _WIN32
  const int rc = _pclose(pipe);
#else
  const int rc = pclose(pipe);
#endif
  if (rc != 0) {
    throw std::runtime_error("Command failed (" + std::to_string(rc) + "): " +
                             command + "\n" + output);
  }
  return output;
}

std::string get_platform_dir() {
#if defined(_WIN32)
#if defined(_M_X64) || defined(__x86_64__)
  return "x86_64-windows";
#elif defined(_M_IX86) || defined(__i386__)
  return "x86-windows";
#elif defined(_M_ARM64) || defined(__aarch64__)
  return "aarch64-windows";
#else
  return "x86_64-windows";
#endif
#elif defined(__APPLE__)
#if defined(__aarch64__) || defined(__arm64__)
  return "aarch64-darwin";
#elif defined(__x86_64__)
  return "x86_64-darwin";
#else
  throw std::runtime_error("Unsupported macOS architecture for FMU binaries");
#endif
#elif defined(__linux__)
#if defined(__aarch64__)
  return "aarch64-linux";
#elif defined(__x86_64__)
  return "x86_64-linux";
#else
  throw std::runtime_error("Unsupported Linux architecture for FMU binaries");
#endif
#else
  throw std::runtime_error("Unsupported platform");
#endif
}

std::string get_binary_extension() {
#if defined(_WIN32)
  return ".dll";
#elif defined(__APPLE__)
  return ".dylib";
#else
  return ".so";
#endif
}

std::string find_model_identifier(const std::string &xml) {
  constexpr const char *tags[] = {"ModelExchange", "CoSimulation"};
  for (const auto *tag : tags) {
    const std::string marker = std::string("<") + tag;
    std::size_t pos = 0;
    while ((pos = xml.find(marker, pos)) != std::string::npos) {
      const std::size_t close = xml.find('>', pos);
      if (close == std::string::npos) {
        break;
      }
      const std::string header = xml.substr(pos, close - pos + 1);
      const std::size_t key = header.find("modelIdentifier=");
      if (key != std::string::npos) {
        const std::size_t quote_pos = key + std::string("modelIdentifier=").size();
        if (quote_pos < header.size() &&
            (header[quote_pos] == '"' || header[quote_pos] == '\'')) {
          const char q = header[quote_pos];
          const std::size_t value_begin = quote_pos + 1;
          const std::size_t value_end = header.find(q, value_begin);
          if (value_end != std::string::npos && value_end > value_begin) {
            return header.substr(value_begin, value_end - value_begin);
          }
        }
      }
      pos = close + 1;
    }
  }
  return "";
}

std::vector<std::string> list_binaries_in_directory(const fs::path &dir,
                                                    const std::string &ext) {
  std::vector<std::string> matches;
  if (!fs::exists(dir) || !fs::is_directory(dir)) {
    return matches;
  }
  for (const auto &entry : fs::directory_iterator(dir)) {
    if (!entry.is_regular_file()) {
      continue;
    }
    if (entry.path().extension() == ext) {
      matches.push_back(entry.path().string());
    }
  }
  return matches;
}

std::vector<std::string> parse_zip_listing_for_binaries(
    const std::string &listing, const std::string &platform_dir,
    const std::string &ext) {
  std::vector<std::string> entries;
  const std::string prefix = "binaries/" + platform_dir + "/";
  std::istringstream iss(listing);
  std::string line;
  while (std::getline(iss, line)) {
    line = trim_copy(line);
    if (line.rfind(prefix, 0) == 0 && line.size() >= ext.size() &&
        line.substr(line.size() - ext.size()) == ext) {
      entries.push_back(line);
    }
  }
  return entries;
}

std::vector<std::string> parse_dependency_output(const std::string &output) {
  std::vector<std::string> deps;
  std::istringstream iss(output);
  std::string line;

#if defined(_WIN32)
  while (std::getline(iss, line)) {
    line = trim_copy(line);
    if (line.empty()) {
      continue;
    }
    std::string lower = line;
    std::transform(lower.begin(), lower.end(), lower.begin(),
                   [](unsigned char c) { return static_cast<char>(std::tolower(c)); });
    const auto pos = lower.rfind(".dll");
    if (pos == std::string::npos || pos + 4 != lower.size()) {
      continue;
    }
    const auto sep = line.find_first_of(" \t");
    const std::string dep = (sep == std::string::npos) ? line : line.substr(0, sep);
    if (!dep.empty()) {
      deps.push_back(dep);
    }
  }
#elif defined(__APPLE__)
  bool first_line = true;
  while (std::getline(iss, line)) {
    line = trim_copy(line);
    if (line.empty()) {
      continue;
    }
    if (first_line) {
      first_line = false;
      continue;
    }
    const auto pos = line.find_first_of(" \t");
    const std::string dep = (pos == std::string::npos) ? line : line.substr(0, pos);
    if (!dep.empty()) {
      deps.push_back(dep);
    }
  }
#else
  while (std::getline(iss, line)) {
    line = trim_copy(line);
    if (line.empty() || line.find("statically linked") != std::string::npos) {
      continue;
    }
    std::string dep;
    const auto arrow = line.find("=>");
    if (arrow != std::string::npos) {
      dep = trim_copy(line.substr(0, arrow));
    } else {
      const auto pos = line.find_first_of(" \t");
      dep = (pos == std::string::npos) ? line : line.substr(0, pos);
    }
    if (!dep.empty()) {
      deps.push_back(dep);
    }
  }
#endif
  return deps;
}

// Read the continuous states of a Model Exchange FMU from modelDescription.xml.
// The order of <ContinuousStateDerivative> in <ModelStructure> is the order of
// fmi3GetContinuousStates; each derivative variable points to its state via
// the "derivative" attribute (a value reference in FMI 3.0).
void parse_continuous_states(const std::string &xml,
                             std::vector<std::string> &state_names,
                             std::vector<fmi3ValueReference> &state_vrs,
                             std::vector<fmi3ValueReference> &der_vrs) {
  state_names.clear();
  state_vrs.clear();
  der_vrs.clear();

  struct XmlVar {
    std::string name;
    bool has_derivative = false;
    fmi3ValueReference derivative = 0;
  };
  std::unordered_map<fmi3ValueReference, XmlVar> vars;

  const std::size_t mv_begin = xml.find("<ModelVariables");
  const std::size_t mv_end = xml.find("</ModelVariables>");
  if (mv_begin == std::string::npos || mv_end == std::string::npos) {
    return;
  }
  constexpr const char *tags[] = {"<Float64 ", "<Float32 "};
  for (const auto *marker : tags) {
    std::size_t pos = mv_begin;
    while ((pos = xml.find(marker, pos)) != std::string::npos && pos < mv_end) {
      const std::size_t tag_end = xml.find('>', pos);
      if (tag_end == std::string::npos) {
        break;
      }
      const std::string header = xml.substr(pos, tag_end - pos + 1);
      std::string name, vr, der;
      if (extract_xml_attribute(header, "name", name) &&
          extract_xml_attribute(header, "valueReference", vr)) {
        XmlVar v;
        v.name = name;
        if (extract_xml_attribute(header, "derivative", der)) {
          v.has_derivative = true;
          v.derivative = static_cast<fmi3ValueReference>(std::stoul(der));
        }
        vars[static_cast<fmi3ValueReference>(std::stoul(vr))] = v;
      }
      pos = tag_end + 1;
    }
  }

  const std::size_t ms_begin = xml.find("<ModelStructure");
  if (ms_begin == std::string::npos) {
    return;
  }
  const std::string marker = "<ContinuousStateDerivative";
  std::size_t pos = ms_begin;
  while ((pos = xml.find(marker, pos)) != std::string::npos) {
    const std::size_t tag_end = xml.find('>', pos);
    if (tag_end == std::string::npos) {
      break;
    }
    const std::string header = xml.substr(pos, tag_end - pos + 1);
    std::string vr;
    if (extract_xml_attribute(header, "valueReference", vr)) {
      const auto der_vr = static_cast<fmi3ValueReference>(std::stoul(vr));
      const auto it = vars.find(der_vr);
      if (it != vars.end() && it->second.has_derivative) {
        const auto st = vars.find(it->second.derivative);
        der_vrs.push_back(der_vr);
        state_vrs.push_back(it->second.derivative);
        state_names.push_back(st != vars.end() ? st->second.name
                                               : "x" + std::to_string(state_names.size()));
      }
    }
    pos = tag_end + 1;
  }
}

} // namespace

// Static map initializations for enum-to-string translation
const std::unordered_map<int, std::string> FmuWrapper::_initial_kind_map = {
    {fmi3Initial::fmi3InitialExact, "exact"},
    {fmi3Initial::fmi3InitialApprox, "approx"},
    {fmi3Initial::fmi3InitialCalculated, "calculated"},
    {fmi3Initial::fmi3InitialUndefined, "undefined"}};

const std::unordered_map<fmi3Causality, std::string>
    FmuWrapper::_causality_map = {
        {fmi3Causality::fmi3CausalityStructuralParameter,
         "structuralParameter"},
        {fmi3Causality::fmi3CausalityOutput, "output"},
        {fmi3Causality::fmi3CausalityInput, "input"},
        {fmi3Causality::fmi3CausalityIndependent, "independent"},
        {fmi3Causality::fmi3CausalityCalculatedParameter,
         "calculatedParameter"},
        {fmi3Causality::fmi3CausalityParameter, "parameter"},
        {fmi3Causality::fmi3CausalityLocal, "local"}};

const std::unordered_map<fmi3DataType, std::string> FmuWrapper::_data_type_map =
    {{fmi3DataType::fmi3DataTypeFloat64, "Float64"},
     {fmi3DataType::fmi3DataTypeFloat32, "Float32"},
     {fmi3DataType::fmi3DataTypeInt8, "Int8"},
     {fmi3DataType::fmi3DataTypeUInt8, "UInt8"},
     {fmi3DataType::fmi3DataTypeInt16, "Int16"},
     {fmi3DataType::fmi3DataTypeUInt16, "UInt16"},
     {fmi3DataType::fmi3DataTypeInt32, "Int32"},
     {fmi3DataType::fmi3DataTypeUInt32, "UInt32"},
     {fmi3DataType::fmi3DataTypeInt64, "Int64"},
     {fmi3DataType::fmi3DataTypeUInt64, "UInt64"},
     {fmi3DataType::fmi3DataTypeBoolean, "Boolean"},
     {fmi3DataType::fmi3DataTypeString, "String"},
     {fmi3DataType::fmi3DataTypeBinary, "Binary"},
     {fmi3DataType::fmi3DataTypeEnumeration, "Enumeration"},
     {fmi3DataType::fmi3DataTypeClock, "Clock"}};

typedef union {
  double dbl;
  float flt;
  int8_t i8;
  uint8_t u8;
  int16_t i16;
  uint16_t u16;
  int32_t i32;
  uint32_t u32;
  int64_t i64;
  uint64_t u64;
  bool b;
} data_type_t;

// Helper methods to translate enums to strings
std::string FmuWrapper::get_initial_kind_string(fmi3Initial kind) {
  auto it = _initial_kind_map.find(kind);
  return (it != _initial_kind_map.end())
             ? it->second
             : "unknown(" + std::to_string(kind) + ")";
}

std::string FmuWrapper::get_causality_string(fmi3Causality causality) {
  auto it = _causality_map.find(causality);
  return (it != _causality_map.end())
             ? it->second
             : "unknown(" + std::to_string(causality) + ")";
}

std::string FmuWrapper::get_data_type_string(fmi3DataType dataType) {
  auto it = _data_type_map.find(dataType);
  return (it != _data_type_map.end())
             ? it->second
             : "unknown(" + std::to_string(dataType) + ")";
}

FmuWrapper::FmuWrapper(const std::string &fmu_path,
                       const std::string &instance_name, double start_time)
    : _fmu_path(fmu_path), _instance_name(instance_name),
      _start_time(start_time), _current_time(start_time) {
  // Load the FMU file
  _fmu = fmi4c_loadFmu(fmu_path.c_str(), instance_name.c_str());

  if (!_fmu) {
    throw std::runtime_error("Failed to load FMU: " + fmu_path);
  }

  auto v = fmi4c_getFmiVersion(_fmu);
  if (v != 3) {
    throw std::runtime_error(std::string("Unsupported FMU version: ") +
                             std::to_string(v));
  }

  _model_description_xml = read_model_description_xml(_fmu_path);
  bool cs = _model_description_xml.find("<CoSimulation") != std::string::npos;
  bool me = _model_description_xml.find("<ModelExchange") != std::string::npos;

  if(cs){

    _type = FmuType::CoSimulation;

    size_t cs_pos = _model_description_xml.find("<CoSimulation");
    if(cs_pos != string::npos){

      size_t cs_end = _model_description_xml.find(">", cs_pos);
      string cs_tag = _model_description_xml.substr(cs_pos, cs_end - cs_pos + 1);

      string var_step;
      if(extract_xml_attribute(cs_tag, "canHandleVariableCommunicationStepSize", var_step)){
        _fixed_step = !(var_step == "true" || var_step == "1");

        std::string step_string;
        if (extract_xml_attribute(cs_tag, "fixedInternalStepSize", step_string)) {
          _step_size = std::stod(step_string);
        }
        
      } else{
        _fixed_step = false;
      }
    }

    _instance = fmi3_instantiateCoSimulation(
      _fmu,
      fmi3False, // visible
      fmi3False, // loggingOn
      fmi3False, // eventModeUsed
      fmi3False, // earlyReturnAllowed
      nullptr,   // requiredIntermediateVariables
      0,         // nRequiredIntermediateVariables
      nullptr,   // instanceEnvironment
      nullptr,   // logMessage
      nullptr    // intermediateUpdate
    );

  } else if(me){

    _type = FmuType::ModelExchange;
    _instance = fmi3_instantiateModelExchange(_fmu, false, false, nullptr, nullptr);

    size_t me_pos = _model_description_xml.find("<ModelExchange");
    size_t me_end = _model_description_xml.find(">", me_pos);
    string me_tag = _model_description_xml.substr(me_pos, me_end - me_pos + 1);
    string dir_der;
    if (extract_xml_attribute(me_tag, "providesDirectionalDerivatives", dir_der) ||
        extract_xml_attribute(me_tag, "providesDirectionalDerivative", dir_der)) {
      _provides_dir_der = (dir_der == "true" || dir_der == "1");
    }
    parse_continuous_states(_model_description_xml, _state_names, _state_vrs, _der_vrs);

  } else{

    throw std::runtime_error("FMU Type: Unknown");
  }

  if (!_instance) {
    fmi4c_freeFmu(_fmu);
    _fmu = nullptr;
    throw std::runtime_error("Failed to instantiate FMU: " + instance_name);
  }

  // Enter initialization mode
  fmi3Status status = fmi3_enterInitializationMode(_instance,
                                                   false, // toleranceDefined
                                                   0.0,   // tolerance
                                                   start_time, // startTime
                                                   false, // stopTimeDefined
                                                   0.0    // stopTime
  );
  check_status(status, "enterInitializationMode");

  // Exit initialization mode to prepare for stepping
  status = fmi3_exitInitializationMode(_instance);
  check_status(status, "exitInitializationMode");

  int vnum = fmi3_getNumberOfVariables(_fmu);
  for (int i = 0; i < vnum; ++i) {
    auto v = fmi3_getVariableByIndex(_fmu, i + 1);
    if (fmi3_getVariableCausality(v) == fmi3Causality::fmi3CausalityOutput) {
      _out_vars.push_back(fmi3_getVariableName(v));
    } else if (fmi3_getVariableCausality(v) ==
               fmi3Causality::fmi3CausalityInput) {
      _in_vars.push_back(fmi3_getVariableName(v));
    } else if (fmi3_getVariableCausality(v) ==
               fmi3Causality::fmi3CausalityParameter) {
      _param_vars.push_back(fmi3_getVariableName(v));
    } else if (fmi3_getVariableCausality(v) ==
               fmi3Causality::fmi3CausalityIndependent) {
      _indep_vars.push_back(fmi3_getVariableName(v));
    }
  }
  _initialized = true;
}

FmuWrapper::~FmuWrapper() {
  if (_instance) {
    // Terminate simulation
    fmi3_terminate(_instance);
    fmi3_freeInstance(_instance);
    _instance = nullptr;
  }
  if (_fmu) {
    fmi4c_freeFmu(_fmu);
    _fmu = nullptr;
  }
}

FmuWrapper::FmuWrapper(FmuWrapper &&other) noexcept
    : _fmu(other._fmu), _instance(other._instance),
      _fmu_path(std::move(other._fmu_path)),
      _instance_name(std::move(other._instance_name)),
      _start_time(other._start_time), _current_time(other._current_time),
      _var_cache(std::move(other._var_cache)) {
  other._fmu = nullptr;
  other._instance = nullptr;
}

FmuWrapper &FmuWrapper::operator=(FmuWrapper &&other) noexcept {
  if (this != &other) {
    // Clean up current resources
    if (_instance) {
      fmi3_terminate(_instance);
      fmi3_freeInstance(_instance);
    }
    if (_fmu) {
      fmi4c_freeFmu(_fmu);
    }

    // Move from other
    _fmu = other._fmu;
    _instance = other._instance;
    _fmu_path = std::move(other._fmu_path);
    _instance_name = std::move(other._instance_name);
    _start_time = other._start_time;
    _current_time = other._current_time;
    _var_cache = std::move(other._var_cache);

    other._fmu = nullptr;
    other._instance = nullptr;
  }
  return *this;
}

std::string FmuWrapper::model_name() {
  if (!_instance || !_initialized) {
    throw std::runtime_error("FMU instance not initialized");
  }
  return fmi3_modelName(_instance->fmu);
}

void FmuWrapper::reset() {
  if (!_instance || !_initialized) {
    throw std::runtime_error("FMU instance not initialized");
  }
  check_status(fmi3_reset(_instance), "Resetting the model");
}

void FmuWrapper::do_step(double dt) {

  if (!_instance || !_initialized) {
    throw std::runtime_error("FMU instance not initialized");
  }

  if (_type == FmuType::CoSimulation) { // CO-SIMULATION

    fmi3Boolean eventHandlingNeeded = fmi3False;
    fmi3Boolean terminateSimulation = fmi3False;
    fmi3Boolean earlyReturn = fmi3False;
    fmi3Float64 lastSuccessfulTime = _current_time;

    // exported solver within FMU
    fmi3Status status = fmi3_doStep(_instance,
                                    _current_time,
                                    dt,
                                    fmi3True, // noSetFMUStatePriorToCurrentPoint
                                    &eventHandlingNeeded,
                                    &terminateSimulation,
                                    &earlyReturn,
                                    &lastSuccessfulTime);
                                    
    check_status(status, "doStep");

    if (terminateSimulation) {
      throw std::runtime_error("FMU requested termination during doStep");
    }

    _current_time += dt;

    if (eventHandlingNeeded) {
        fmi3_enterEventMode(_instance);
        handle_events();
        fmi3_enterStepMode(_instance);
    }

  } else if(_type == FmuType::ModelExchange){ // MODEL EXCHANGE

    // Get the number of continuous states
    size_t numStates = 0;
    fmi3Status status = fmi3_getNumberOfContinuousStates(_instance, &numStates);
    check_status(status, "getNumberOfContinuousStates");

    if (numStates == 0) {
      // No states to integrate, just advance time and handle events
      _current_time += dt;
      handle_events();
      return;
    }

    // Adaptive RK45 integration parameters
    const double &relTol = _solver_params._rel_tol;
    const double &absTol = _solver_params._abs_tol;
    const double &hmin = _solver_params._hmin;

    double t = _current_time;
    double tend = _current_time + dt;
    double h = dt; // Initial step size

    std::vector<fmi3Float64> y(numStates);
    std::vector<fmi3Float64> ytemp(numStates);
    std::vector<fmi3Float64> k1(numStates), k2(numStates), k3(numStates);
    std::vector<fmi3Float64> k4(numStates), k5(numStates), k6(numStates);
    std::vector<fmi3Float64> yerr(numStates);

    // Get initial state
    status = fmi3_getContinuousStates(_instance, y.data(), numStates);
    check_status(status, "getContinuousStates");

    // Adaptive stepping loop
    while (t < tend && h > hmin) {
      // Limit step to not overshoot tend
      if (t + h > tend) {
        h = tend - t;
      }

      // RK45 with Dormand-Prince coefficients
      // Stage 1: k1 = f(t, y)
      status =
          fmi3_getContinuousStateDerivatives(_instance, k1.data(), numStates);
      check_status(status, "getContinuousStateDerivatives at stage 1");

      // Stage 2: k2 = f(t + (1/5)*h, y + (1/5)*h*k1)
      for (size_t i = 0; i < numStates; ++i) {
        ytemp[i] = y[i] + (h / 5.0) * k1[i];
      }
      status = fmi3_setContinuousStates(_instance, ytemp.data(), numStates);
      check_status(status, "setContinuousStates at stage 2");
      status =
          fmi3_getContinuousStateDerivatives(_instance, k2.data(), numStates);
      check_status(status, "getContinuousStateDerivatives at stage 2");

      // Stage 3: k3 = f(t + (3/10)*h, y + (3/40)*h*k1 + (9/40)*h*k2)
      for (size_t i = 0; i < numStates; ++i) {
        ytemp[i] = y[i] + (3.0 / 40.0) * h * k1[i] + (9.0 / 40.0) * h * k2[i];
      }
      status = fmi3_setContinuousStates(_instance, ytemp.data(), numStates);
      check_status(status, "setContinuousStates at stage 3");
      status =
          fmi3_getContinuousStateDerivatives(_instance, k3.data(), numStates);
      check_status(status, "getContinuousStateDerivatives at stage 3");

      // Stage 4: k4 = f(t + (4/5)*h, y + (44/45)*h*k1 - (56/15)*h*k2 +
      // (32/9)*h*k3)
      for (size_t i = 0; i < numStates; ++i) {
        ytemp[i] = y[i] + (44.0 / 45.0) * h * k1[i] - (56.0 / 15.0) * h * k2[i] +
                  (32.0 / 9.0) * h * k3[i];
      }
      status = fmi3_setContinuousStates(_instance, ytemp.data(), numStates);
      check_status(status, "setContinuousStates at stage 4");
      status =
          fmi3_getContinuousStateDerivatives(_instance, k4.data(), numStates);
      check_status(status, "getContinuousStateDerivatives at stage 4");

      // Stage 5: k5 = f(t + (8/9)*h, y + ...)
      for (size_t i = 0; i < numStates; ++i) {
        ytemp[i] = y[i] + (19372.0 / 6561.0) * h * k1[i] -
                  (25360.0 / 2187.0) * h * k2[i] +
                  (64448.0 / 6561.0) * h * k3[i] - (212.0 / 729.0) * h * k4[i];
      }
      status = fmi3_setContinuousStates(_instance, ytemp.data(), numStates);
      check_status(status, "setContinuousStates at stage 5");
      status =
          fmi3_getContinuousStateDerivatives(_instance, k5.data(), numStates);
      check_status(status, "getContinuousStateDerivatives at stage 5");

      // Stage 6: k6 = f(t + h, y + ...)
      for (size_t i = 0; i < numStates; ++i) {
        ytemp[i] = y[i] + (9017.0 / 3168.0) * h * k1[i] -
                  (355.0 / 33.0) * h * k2[i] + (46732.0 / 5247.0) * h * k3[i] +
                  (49.0 / 176.0) * h * k4[i] - (5103.0 / 18656.0) * h * k5[i];
      }
      status = fmi3_setContinuousStates(_instance, ytemp.data(), numStates);
      check_status(status, "setContinuousStates at stage 6");
      status =
          fmi3_getContinuousStateDerivatives(_instance, k6.data(), numStates);
      check_status(status, "getContinuousStateDerivatives at stage 6");

      // 5th order solution: y_new = y + h * (35/384*k1 + 500/1113*k3 + 125/192*k4
      // - 2187/6784*k5 + 11/84*k6) 4th order solution for error: y_hat = y + h *
      // (5179/57600*k1 + 7571/16695*k3 + 393/640*k4 - 92097/339200*k5 +
      // 187/2100*k6)
      std::vector<fmi3Float64> ynew(numStates);
      for (size_t i = 0; i < numStates; ++i) {
        ynew[i] = y[i] + h * (35.0 / 384.0 * k1[i] + 500.0 / 1113.0 * k3[i] +
                              125.0 / 192.0 * k4[i] - 2187.0 / 6784.0 * k5[i] +
                              11.0 / 84.0 * k6[i]);

        // Error estimate (difference between 5th and 4th order)
        double y4th =
            y[i] + h * (5179.0 / 57600.0 * k1[i] + 7571.0 / 16695.0 * k3[i] +
                        393.0 / 640.0 * k4[i] - 92097.0 / 339200.0 * k5[i] +
                        187.0 / 2100.0 * k6[i]);
        yerr[i] = std::abs(ynew[i] - y4th);
      }

      // Compute maximum relative error
      double maxError = 0.0;
      for (size_t i = 0; i < numStates; ++i) {
        double scale = absTol + relTol * std::abs(y[i]);
        double error = yerr[i] / scale;
        maxError = std::max(maxError, error);
      }

      // Adaptive step size control
      if (maxError <= 1.0) {
        // Step accepted: update y and t
        y = ynew;
        t += h;
        _current_time = t;

        // Set accepted state into FMU
        status = fmi3_setContinuousStates(_instance, y.data(), numStates);
        check_status(status, "setContinuousStates (accepted step)");

        // Increase step size for next iteration (but not too aggressively)
        h *= 0.9 * std::pow(1.0 / maxError, 0.2);
        h = std::min(h, 10.0 * (tend - t)); // Don't let h grow too much
      } else {
        // Step rejected: reduce step size
        h *= 0.9 * std::pow(1.0 / maxError, 0.25);
      }

      // Enforce minimum step size to prevent infinite loops
      if (h < hmin) {
        h = hmin;
      }
    }

    // Ensure we're exactly at tend
    _current_time = tend;

    // Handle events at the end of the step
    handle_events();
  }

  
}

void FmuWrapper::get_state_derivatives(fmi3Float64 *derivatives, size_t nx,
                                       const std::string &context) {
  // f evaluated at the PREVIOUS states. Getting the states first forces the refresh; for other FMUs it is just a harmless extra call

  _deriv_scratch.resize(nx);
  check_status(fmi3_getContinuousStates(_instance, _deriv_scratch.data(), nx),
               context);
  check_status(fmi3_getContinuousStateDerivatives(_instance, derivatives, nx),
               context);
}

void FmuWrapper::handle_events() {
  if (!_instance || !_initialized) {
    throw std::runtime_error("FMU instance not initialized");
  }
  // Check for events after stepping
  fmi3Boolean newDiscreteStatesNeeded = true;
  fmi3Boolean terminateSimulation = false;
  fmi3Boolean nominalsOfContinuousStatesChanged = false;
  fmi3Boolean valuesOfContinuousStatesChanged = false;
  fmi3Boolean nextEventTimeDefined = false;
  fmi3Float64 nextEventTime = 0.0;

  // Process events if necessary
  while (newDiscreteStatesNeeded) {
    fmi3Status status = fmi3_updateDiscreteStates(
        _instance, &newDiscreteStatesNeeded, &terminateSimulation,
        &nominalsOfContinuousStatesChanged, &valuesOfContinuousStatesChanged,
        &nextEventTimeDefined, &nextEventTime);
    check_status(status, "updateDiscreteStates");

    if (terminateSimulation) {
      throw std::runtime_error(
          "FMU requested termination during event handling");
    }
  }
}

void FmuWrapper::get_jacobian(json &jac) {
  if (!_instance || !_initialized) {
    throw std::runtime_error("FMU instance not initialized");
  }
  if (_type != FmuType::ModelExchange) {
    throw std::runtime_error("Jacobian is available for Model Exchange FMUs only");
  }

  size_t nx = 0;
  check_status(fmi3_getNumberOfContinuousStates(_instance, &nx), "getNumberOfContinuousStates");

  // Float64 inputs (scalars and arrays), flattened in declaration order
  if (!_jac_inputs_ready) {
    for (const auto &name : _in_vars) {
      const auto meta = parse_real_array_metadata(_model_description_xml, name);
      if (!meta._is_float64) {
        continue;
      }
      _jac_inputs.push_back(
          {name, resolve_var_ref(name), meta._is_array ? meta._length : 1});
    }
    _jac_inputs_ready = true;
  }
  size_t nu = 0;
  for (const auto &in : _jac_inputs) {
    nu += in.len;
  }

  auto get_derivatives = [&](std::vector<double> &f) {
    get_state_derivatives(f.data(), nx, "getContinuousStateDerivatives (jacobian)");
  };
  auto set_states = [&](const std::vector<double> &x) {
    check_status(fmi3_setContinuousStates(_instance, x.data(), nx),
                 "setContinuousStates (jacobian)");
  };
  auto set_inputs = [&](const std::vector<double> &u) {
    size_t k = 0;
    for (const auto &in : _jac_inputs) {
      check_status(fmi3_setFloat64(_instance, &in.vr, 1, u.data() + k, in.len),
                   "setFloat64 (jacobian) for " + in.name);
      k += in.len;
    }
  };

  // Linearization point
  std::vector<double> x0(nx), f0(nx), u0(nu);
  if (nx > 0) {
    check_status(fmi3_getContinuousStates(_instance, x0.data(), nx),
                 "getContinuousStates (jacobian)");
    get_derivatives(f0);
  }
  {
    size_t k = 0;
    for (const auto &in : _jac_inputs) {
      check_status(fmi3_getFloat64(_instance, &in.vr, 1, u0.data() + k, in.len),
                   "getFloat64 (jacobian) for " + in.name);
      k += in.len;
    }
  }

  std::vector<double> A(nx * nx, 0.0), B(nx * nu, 0.0);
  std::string method;

  // 1. Exact directional derivatives, if the FMU declares them.
  //    Some exporters (e.g. Simulink) ship an empty stub that returns fmi3OK
  //    without writing anything: the NaN pre-fill detects that, and we fall
  //    back to finite differences for the rest of the run.
  if (_provides_dir_der && nx > 0 && _der_vrs.size() == nx) {
    std::vector<fmi3ValueReference> knowns(_state_vrs);
    for (const auto &in : _jac_inputs) {
      knowns.push_back(in.vr);
    }
    const size_t nseed = nx + nu;
    std::vector<double> seed(nseed, 0.0), sens(nx);
    bool ok = true;
    for (size_t j = 0; j < nseed && ok; ++j) {
      std::fill(seed.begin(), seed.end(), 0.0);
      seed[j] = 1.0;
      std::fill(sens.begin(), sens.end(),
                std::numeric_limits<double>::quiet_NaN());
      const fmi3Status s = fmi3_getDirectionalDerivative(
          _instance, _der_vrs.data(), nx, knowns.data(), knowns.size(),
          seed.data(), nseed, sens.data(), nx);
      if (s != fmi3OK) {
        ok = false;
        break;
      }
      for (size_t i = 0; i < nx; ++i) {
        if (std::isnan(sens[i])) {
          ok = false;
          break;
        }
        if (j < nx) {
          A[i * nx + j] = sens[i];
        } else {
          B[i * nu + (j - nx)] = sens[i];
        }
      }
    }
    if (ok) {
      method = "directional_derivative";
    } else {
      _provides_dir_der = false; // unusable: don't try again
    }
  }

  // 2. Central finite differences on f(x, u)
  if (method.empty()) {
    method = "central_difference";
    const double eps = std::cbrt(std::numeric_limits<double>::epsilon());
    std::vector<double> fp(nx), fm(nx);

    std::vector<double> x(x0);
    for (size_t j = 0; j < nx; ++j) {
      const double h = eps * std::max(1.0, std::abs(x0[j]));
      x[j] = x0[j] + h;
      set_states(x);
      get_derivatives(fp);
      x[j] = x0[j] - h;
      set_states(x);
      get_derivatives(fm);
      x[j] = x0[j];
      for (size_t i = 0; i < nx; ++i) {
        A[i * nx + j] = (fp[i] - fm[i]) / (2.0 * h);
      }
    }
    if (nx > 0) {
      set_states(x0);
    }

    std::vector<double> u(u0);
    for (size_t j = 0; j < nu && nx > 0; ++j) {
      const double h = eps * std::max(1.0, std::abs(u0[j]));
      u[j] = u0[j] + h;
      set_inputs(u);
      get_derivatives(fp);
      u[j] = u0[j] - h;
      set_inputs(u);
      get_derivatives(fm);
      u[j] = u0[j];
      for (size_t i = 0; i < nx; ++i) {
        B[i * nu + j] = (fp[i] - fm[i]) / (2.0 * h);
      }
    }
    if (nu > 0) {
      set_inputs(u0);
    }
  }

  // Leave the FMU at the linearization point
  if (nx > 0) {
    set_states(x0);
    get_derivatives(f0);
  }

  // Output
  std::vector<std::string> state_names(_state_names);
  if (state_names.size() != nx) {
    state_names.clear();
    for (size_t i = 0; i < nx; ++i) {
      state_names.push_back("x" + std::to_string(i));
    }
  }
  std::vector<std::string> input_names;
  for (const auto &in : _jac_inputs) {
    if (in.len == 1) {
      input_names.push_back(in.name);
    } else {
      for (size_t e = 0; e < in.len; ++e) {
        input_names.push_back(in.name + "[" + std::to_string(e) + "]");
      }
    }
  }
  json jA = json::array(), jB = json::array();
  for (size_t i = 0; i < nx; ++i) {
    jA.push_back(std::vector<double>(A.begin() + i * nx, A.begin() + (i + 1) * nx));
    jB.push_back(std::vector<double>(B.begin() + i * nu, B.begin() + (i + 1) * nu));
  }
  jac.clear();
  jac["method"] = method;
  jac["states"] = state_names;
  jac["inputs"] = input_names;
  jac["x"] = x0;
  jac["xdot"] = f0;
  jac["A"] = jA;
  jac["B"] = jB;
}

void FmuWrapper::set_real(const std::string &name, double value) {
  if (!_instance || !_initialized) {
    throw std::runtime_error("FMU instance not initialized");
  }

  fmi3ValueReference vr = resolve_var_ref(name);

  fmi3Status status = fmi3_setFloat64(_instance, &vr, 1, &value, 1);
  check_status(status, std::string("setFloat64 for variable ") + name);
}

void FmuWrapper::set_bool(const std::string &name, bool value) {
  if (!_instance || !_initialized) {
    throw std::runtime_error("FMU instance not initialized");
  }

  fmi3ValueReference vr = resolve_var_ref(name);
  const fmi3Boolean fmi_value = value;

  fmi3Status status = fmi3_setBoolean(_instance, &vr, 1, &fmi_value, 1);
  check_status(status, std::string("setBoolean for variable ") + name);
}

void FmuWrapper::set_real(const std::string &name,
                          const std::vector<double> &values) {
  if (!_instance || !_initialized) {
    throw std::runtime_error("FMU instance not initialized");
  }
  const std::size_t expected = resolve_real_array_length(name);
  if (values.size() != expected) {
    throw std::runtime_error("Size mismatch for variable " + name +
                             ": expected " + std::to_string(expected) +
                             ", got " + std::to_string(values.size()));
  }
  if (values.empty()) {
    throw std::runtime_error("Cannot set empty array for variable " + name);
  }

  fmi3ValueReference vr = resolve_var_ref(name);
  fmi3Status status =
      fmi3_setFloat64(_instance, &vr, 1, values.data(), values.size());
  check_status(status, std::string("setFloat64 array for variable ") + name);
}

double FmuWrapper::get_real(const std::string &name) const {
  if (!_instance || !_initialized) {
    throw std::runtime_error("FMU instance not initialized");
  }

  fmi3ValueReference vr = resolve_var_ref(name);
  fmi3Float64 value = 0.0;

  fmi3Status status = fmi3_getFloat64(_instance, &vr, 1, &value, 1);
  check_status(status, std::string("getFloat64 for variable ") + name);

  return value;
}

bool FmuWrapper::get_bool(const std::string &name) const {
  if (!_instance || !_initialized) {
    throw std::runtime_error("FMU instance not initialized");
  }

  fmi3ValueReference vr = resolve_var_ref(name);
  fmi3Boolean value = fmi3False;

  fmi3Status status = fmi3_getBoolean(_instance, &vr, 1, &value, 1);
  check_status(status, std::string("getBoolean for variable ") + name);

  return value;
}

void FmuWrapper::get_real(const std::string &name,
                          std::vector<double> &values) const {
  if (!_instance || !_initialized) {
    throw std::runtime_error("FMU instance not initialized");
  }
  const std::size_t expected = resolve_real_array_length(name);
  values.resize(expected);
  if (values.empty()) {
    throw std::runtime_error("Cannot get empty array for variable " + name);
  }

  fmi3ValueReference vr = resolve_var_ref(name);
  fmi3Status status =
      fmi3_getFloat64(_instance, &vr, 1, values.data(), values.size());
  check_status(status, std::string("getFloat64 array for variable ") + name);
}

void FmuWrapper::list_variables(ostream &s) {
  if (!_instance || !_initialized) {
    throw std::runtime_error("FMU instance not initialized");
  }
  auto &fmu = _instance->fmu;
  int vnum = fmi3_getNumberOfVariables(fmu);
  string name = fmi3_modelName(fmu);
  const int fw = 15;
  const int tw = 18;
  s << style::bold << "Variables:" << endl
    << setw(fw) << "Name"
    << setw(fw) << "Description"
    << setw(tw) << "Data type"
    << setw(fw) << "Causality"
    << setw(fw) << "Initial"
    << style::reset << endl;
  for (int i = 0; i < vnum; ++i) {
    auto v = fmi3_getVariableByIndex(fmu, i + 1);
    const std::string var_name = fmi3_getVariableName(v);
    auto c = fmi3_getVariableCausality(v);
    if (c > fmi3Causality::fmi3CausalityCalculatedParameter)
      continue;
    std::string dtype = get_data_type_string(fmi3_getVariableDataType(v));
    const std::string dims = resolve_var_dimensions(var_name);
    if (!dims.empty()) {
      // Convert "[3x2]" to "(3x2)" for compact table output.
      dtype += "(" + dims.substr(1, dims.size() - 2) + ")";
    }

    s << setw(fw) << var_name
      << setw(fw) << fmi3_getVariableDescription(v)
      << setw(tw) << dtype
      << setw(fw) << get_causality_string(c)
      << setw(fw) << get_initial_kind_string(fmi3_getVariableInitial(v))
      << endl;
  }
}

size_t FmuWrapper::get_outputs(std::map<std::string, double> &outs) const {
  if (!_instance || !_initialized) {
    throw std::runtime_error("FMU instance not initialized");
  }
  size_t n = 0;
  for (auto const &name : _out_vars) {
    outs[name] = get_real(name);
    n++;
  }
  return n;
}

size_t FmuWrapper::get_inputs(std::map<std::string, double> &ins) const {
  if (!_instance || !_initialized) {
    throw std::runtime_error("FMU instance not initialized");
  }
  size_t n = 0;
  for (auto const &name : _in_vars) {
    ins[name] = get_real(name);
    n++;
  }
  return n;
}

size_t FmuWrapper::get_params(std::map<std::string, double> &params) const {
  if (!_instance || !_initialized) {
    throw std::runtime_error("FMU instance not initialized");
  }
  size_t n = 0;
  for (auto const &name : _param_vars) {
    params[name] = get_real(name);
    n++;
  }
  return n;
}

size_t FmuWrapper::get_indep(std::map<std::string, double> &indep) const {
  if (!_instance || !_initialized) {
    throw std::runtime_error("FMU instance not initialized");
  }
  size_t n = 0;
  for (auto const &name : _indep_vars) {
    indep[name] = get_real(name);
    n++;
  }
  return n;
}

json FmuWrapper::value(string const &name) {
  if (!_instance || !_initialized) {
    throw std::runtime_error("FMU instance not initialized");
  }
  fmi3ValueReference vref = resolve_var_ref(name);
  auto var = fmi3_getVariableByName(_instance->fmu, name.c_str());
  fmi3Status status;
  static json jval;
  data_type_t value;
  switch (fmi3_getVariableDataType(var)) {
  case fmi3DataType::fmi3DataTypeFloat64:
    status = fmi3_getFloat64(_instance, &vref, 1, &value.dbl, 1);
    check_status(status, std::string("getFloat64 for variable ") + name);
    jval = value.dbl;
    break;
  case fmi3DataType::fmi3DataTypeFloat32:
    status = fmi3_getFloat32(_instance, &vref, 1, &value.flt, 1);
    check_status(status, std::string("getFloat32 for variable ") + name);
    jval = value.flt;
    break;
  case fmi3DataType::fmi3DataTypeInt8:
    status = fmi3_getInt8(_instance, &vref, 1, &value.i8, 1);
    check_status(status, std::string("getInt8 for variable ") + name);
    jval = value.i8;
    break;
  case fmi3DataType::fmi3DataTypeUInt8:
    status = fmi3_getUInt8(_instance, &vref, 1, &value.u8, 1);
    check_status(status, std::string("getUInt8 for variable ") + name);
    jval = value.u8;
    break;
  case fmi3DataType::fmi3DataTypeInt16:
    status = fmi3_getInt16(_instance, &vref, 1, &value.i16, 1);
    check_status(status, std::string("getInt16 for variable ") + name);
    jval = value.i16;
    break;
  case fmi3DataType::fmi3DataTypeUInt16:
    status = fmi3_getUInt16(_instance, &vref, 1, &value.u16, 1);
    check_status(status, std::string("getUInt16 for variable ") + name);
    jval = value.u16;
    break;
  case fmi3DataType::fmi3DataTypeInt32:
    status = fmi3_getInt32(_instance, &vref, 1, &value.i32, 1);
    check_status(status, std::string("getInt32 for variable ") + name);
    jval = value.i32;
    break;
  case fmi3DataType::fmi3DataTypeUInt32:
    status = fmi3_getUInt32(_instance, &vref, 1, &value.u32, 1);
    check_status(status, std::string("getUInt3264 for variable ") + name);
    jval = value.u32;
    break;
  case fmi3DataType::fmi3DataTypeInt64:
    status = fmi3_getInt64(_instance, &vref, 1, &value.i64, 1);
    check_status(status, std::string("getInt64 for variable ") + name);
    jval = value.i64;
    break;
  case fmi3DataType::fmi3DataTypeUInt64:
    status = fmi3_getUInt64(_instance, &vref, 1, &value.u64, 1);
    check_status(status, std::string("getUInt64 for variable ") + name);
    jval = value.u64;
    break;
  case fmi3DataType::fmi3DataTypeBoolean:
    status = fmi3_getBoolean(_instance, &vref, 1, &value.b, 1);
    check_status(status, std::string("getBoolean for variable ") + name);
    jval = value.b;
    break;
  default:
    throw std::runtime_error("Unsupported value data type " + get_data_type_string(fmi3_getVariableDataType(var)));
    break;
  }
  return jval;
}

void FmuWrapper::get_status(json &status) {
  if (!_instance || !_initialized) {
    throw std::runtime_error("FMU instance not initialized");
  }
  status.clear();
  auto &fmu = _instance->fmu;
  int vnum = fmi3_getNumberOfVariables(fmu);
  string name;
  for (int i = 0; i < vnum; ++i) {
    auto v = fmi3_getVariableByIndex(fmu, i + 1);
    name = fmi3_getVariableName(v);
    auto causality = fmi3_getVariableCausality(v);
    if (causality == fmi3Causality::fmi3CausalityLocal ||
        causality == fmi3Causality::fmi3CausalityCalculatedParameter) {
      continue;
    }
    const std::string dims = resolve_var_dimensions(name);
    const bool is_vector = !dims.empty() && dims.find('x') == std::string::npos;
    if (is_vector) {
      std::vector<double> values;
      get_real(name, values);
      status[get_causality_string(causality)][name] = values;
    } else {
      status[get_causality_string(causality)][name] = value(name);
    }
  }
}

std::vector<std::string> FmuWrapper::get_input_names() const {
  return _in_vars;
}

std::vector<std::string> FmuWrapper::get_param_names() const {
  return _param_vars;
}

std::vector<std::string> FmuWrapper::get_output_names() const {
  return _out_vars;
}

std::vector<std::string> FmuWrapper::get_indep_names() const {
  return _indep_vars;
}

std::vector<std::string> FmuWrapper::get_binary_dependencies() const {
  if (_fmu_path.empty()) {
    throw std::runtime_error("FMU path is empty");
  }

  const fs::path fmu_path(_fmu_path);
  const std::string platform_dir = get_platform_dir();
  const std::string ext = get_binary_extension();

  fs::path binary_path;
  fs::path cleanup_dir;
  std::string inspect_output;
  try {
    if (fs::is_directory(fmu_path)) {
      const fs::path model_xml = fmu_path / "modelDescription.xml";
      const std::string model_id =
          find_model_identifier(read_text_file(model_xml));
      const fs::path bin_dir = fmu_path / "binaries" / platform_dir;
      auto binaries = list_binaries_in_directory(bin_dir, ext);

      if (binaries.empty()) {
        throw std::runtime_error("No platform binary found in " +
                                 bin_dir.string());
      }

      if (!model_id.empty()) {
        const fs::path expected = bin_dir / (model_id + ext);
        if (fs::exists(expected)) {
          binary_path = expected;
        }
      }
      if (binary_path.empty()) {
        binary_path = binaries.front();
      }
    } else {
#if defined(_WIN32)
      const auto now = std::chrono::high_resolution_clock::now()
                           .time_since_epoch()
                           .count();
      cleanup_dir =
          fs::temp_directory_path() / ("fmu_dep_" + std::to_string(now));
      fs::create_directories(cleanup_dir);

      const std::string extract_cmd =
          "tar -xf \"" + fmu_path.string() + "\" -C \"" +
          cleanup_dir.string() + "\"";
      run_command_capture(extract_cmd);

      const fs::path model_xml = cleanup_dir / "modelDescription.xml";
      const std::string model_id =
          find_model_identifier(read_text_file(model_xml));
      const fs::path bin_dir = cleanup_dir / "binaries" / platform_dir;
      auto binaries = list_binaries_in_directory(bin_dir, ext);
      if (binaries.empty()) {
        throw std::runtime_error("No platform binary found inside FMU for " +
                                 platform_dir);
      }

      if (!model_id.empty()) {
        const fs::path expected = bin_dir / (model_id + ext);
        if (fs::exists(expected)) {
          binary_path = expected;
        }
      }
      if (binary_path.empty()) {
        binary_path = binaries.front();
      }
#else
      const std::string quoted_fmu = shell_quote_posix(fmu_path.string());
      const std::string model_xml =
          run_command_capture("unzip -p " + quoted_fmu + " modelDescription.xml");
      const std::string model_id = find_model_identifier(model_xml);

      auto binaries = parse_zip_listing_for_binaries(
          run_command_capture("unzip -Z1 " + quoted_fmu), platform_dir, ext);
      if (binaries.empty()) {
        throw std::runtime_error("No platform binary found inside FMU for " +
                                 platform_dir);
      }

      std::string selected_entry = binaries.front();
      if (!model_id.empty()) {
        const std::string expected =
            "binaries/" + platform_dir + "/" + model_id + ext;
        auto it = std::find(binaries.begin(), binaries.end(), expected);
        if (it != binaries.end()) {
          selected_entry = *it;
        }
      }

      const auto now = std::chrono::high_resolution_clock::now()
                           .time_since_epoch()
                           .count();
      cleanup_dir =
          fs::temp_directory_path() / ("fmu_dep_" + std::to_string(now));
      fs::create_directories(cleanup_dir);

      const std::string quoted_entry = shell_quote_posix(selected_entry);
      const std::string quoted_tmp = shell_quote_posix(cleanup_dir.string());
      run_command_capture("unzip -qq -o " + quoted_fmu + " " + quoted_entry +
                          " -d " + quoted_tmp);
      binary_path = cleanup_dir / selected_entry;
#endif
    }

#if defined(_WIN32)
    inspect_output =
        run_command_capture("dumpbin /DEPENDENTS \"" + binary_path.string() + "\"");
#elif defined(__APPLE__)
    inspect_output =
        run_command_capture("otool -L " + shell_quote_posix(binary_path.string()));
#else
    inspect_output =
        run_command_capture("ldd " + shell_quote_posix(binary_path.string()));
#endif
  } catch (runtime_error &e) {
    if (!cleanup_dir.empty()) {
      std::error_code ec;
      fs::remove_all(cleanup_dir, ec);
    }
    cout << "Error: " << e.what() << endl;
    throw;
  }

  if (!cleanup_dir.empty()) {
    std::error_code ec;
    fs::remove_all(cleanup_dir, ec);
  }

  return parse_dependency_output(inspect_output);
}

fmi3ValueReference FmuWrapper::resolve_var_ref(const std::string &name) const {
  if (!_instance || !_initialized) {
    throw std::runtime_error("FMU instance not initialized");
  }
  // Check cache first
  auto it = _var_cache.find(name);
  if (it != _var_cache.end()) {
    return it->second;
  }

  // Look up variable in FMU
  fmi3VariableHandle *var = fmi3_getVariableByName(_fmu, name.c_str());
  if (!var) {
    throw std::runtime_error("Variable not found: " + name);
  }

  fmi3ValueReference vr =
      static_cast<fmi3ValueReference>(fmi3_getVariableValueReference(var));

  // Cache for future use
  const_cast<FmuWrapper *>(this)->_var_cache[name] = vr;

  return vr;
}

size_t FmuWrapper::resolve_real_array_length(const std::string &name) const {
  if (!_instance || !_initialized) {
    throw std::runtime_error("FMU instance not initialized");
  }

  const auto cached = _real_array_len_cache.find(name);
  if (cached != _real_array_len_cache.end()) {
    return cached->second;
  }

  if (_model_description_xml.empty()) {
    _model_description_xml = read_model_description_xml(_fmu_path);
  }

  const RealArrayMetadata meta =
      parse_real_array_metadata(_model_description_xml, name);
  if (!meta._is_float64) {
    throw std::runtime_error("Variable " + name + " is not Float64");
  }
  if (!meta._is_array) {
    throw std::runtime_error("Variable " + name + " is not an array");
  }
  if (meta._rank != 1) {
    throw std::runtime_error("Variable " + name +
                             " is not a 1-D array");
  }

  _real_array_len_cache[name] = meta._length;
  return meta._length;
}

std::string FmuWrapper::resolve_var_dimensions(const std::string &name) const {
  if (!_instance || !_initialized) {
    throw std::runtime_error("FMU instance not initialized");
  }

  const auto cached = _var_dimensions_cache.find(name);
  if (cached != _var_dimensions_cache.end()) {
    return cached->second;
  }

  if (_model_description_xml.empty()) {
    _model_description_xml = read_model_description_xml(_fmu_path);
  }

  const std::string dims =
      parse_variable_dimensions_string(_model_description_xml, name);
  _var_dimensions_cache[name] = dims;
  return dims;
}

void FmuWrapper::check_status(fmi3Status status,
                              const std::string &context) const {
  if (!_instance) {
    throw std::runtime_error("FMU instance not initialized");
  }
  if (status == fmi3OK || status == fmi3Warning) {
    return;
  }

  std::ostringstream oss;
  oss << "FMI error in " << context << ": ";

  switch (status) {
  case fmi3Discard:
    oss << "fmi3Discard";
    break;
  case fmi3Error:
    oss << "fmi3Error";
    break;
  case fmi3Fatal:
    oss << "fmi3Fatal";
    break;
  default:
    oss << "Unknown error code";
  }

  throw std::runtime_error(oss.str());
}
